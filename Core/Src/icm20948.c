/* =============================================================================
 * icm20948.c
 * -----------------------------------------------------------------------------
 * MỤC ĐÍCH:
 *   Triển khai (implementation) driver cho cảm biến IMU 9 trục ICM-20948.
 *   Bao gồm:
 *     - Giao tiếp SPI cấp thấp (đọc/ghi thanh ghi)
 *     - Khởi tạo chip (ICM_Init)
 *     - Hiệu chuẩn gyro (ICM_CalibrateGyro)
 *     - Đọc 9 trục đồng thời (ICM_Read9Axis)
 *
 * TỔNG QUAN GIAO TIẾP:
 *   STM32 ──SPI2──► ICM-20948 ──I2C nội bộ──► AK09916 (mag)
 *   - SPI2 chạy Mode 3 (CPOL=1, CPHA=1), prescaler 64 → ~1MHz @ 64MHz SYSCLK
 *   - Chân CS = PA8, điều khiển bằng software (NSS_SOFT)
 *   - Đọc 23 byte liên tục từ ACCEL_XOUT_H (0x2D) mỗi chu kỳ 20ms
 *
 * CẤU TRÚC FILE:
 *   1. Định nghĩa biến toàn cục (dữ liệu cảm biến)
 *   2. Hàm giao tiếp SPI cấp thấp (4 hàm)
 *   3. Hàm hiệu chuẩn gyro (1 hàm)
 *   4. Hàm khởi tạo chip (1 hàm)
 *   5. Hàm đọc 9 trục (1 hàm)
 * =============================================================================
 */

/* =============================================================================
 * PHẦN 1: INCLUDE & ĐỊNH NGHĨA BIẾN TOÀN CỤC
 * =============================================================================
 */

#include "icm20948.h"        /* Header của chính module này */
#include "mag_ak09916.h"     /* Cần cho AK09916_Init() và Mag_SetCalibrated() */
#include "led_direction.h"   /* Cần cho LED_Heartbeat_On/Off() trong calib gyro */

/* --- GIA TỐC KẾ 3 TRỤC (đơn vị: g, 1g = 9.81 m/s²) ---
 * Được cập nhật trong ICM_Read9Axis() mỗi 20ms.
 * Khi board nằm phẳng: ax≈0, ay≈0, az≈+1.00 (hoặc -1.00 tùy chiều gắn chip). */
volatile float ax, ay, az;

/* --- GYRO 3 TRỤC (đơn vị: dps — degrees per second) ---
 * Đã TRỪ BIAS sau khi ICM_CalibrateGyro() chạy xong.
 * Khi board đứng yên: cả 3 giá trị ≈ 0.00. */
volatile float gx, gy, gz;

/* --- BIAS GYRO (offset tĩnh sau calib) ---
 * Được tính trong ICM_CalibrateGyro() bằng trung bình 500 mẫu khi đứng yên.
 * Khởi tạo = 0 để lần đọc đầu tiên không bị sai (trước khi calib). */
volatile float gx_bias = 0.0f, gy_bias = 0.0f, gz_bias = 0.0f;

/* --- DỮ LIỆU THÔ 9 BYTE TỪ MAG AK09916 ---
 * ICM-20948 tự động đọc AK09916 qua I2C nội bộ và lưu vào EXT_SLV_SENS_DATA.
 * Cấu trúc:
 *   [0] ST1   — trạng thái 1 (bit0 = DRDY)
 *   [1..6] HXL..HZH — mag X/Y/Z (little-endian)
 *   [7] TMPS  — dummy
 *   [8] ST2   — trạng thái 2 (bit3 = overflow)
 * Dùng để debug + kiểm tra trạng thái mag. */
volatile uint8_t mag_raw[9] = {0};

/* --- BIẾN DEBUG TRẠNG THÁI MAG ---
 * debug_st1 = mag_raw[0] — trạng thái 1
 * debug_st2 = mag_raw[8] — trạng thái 2 (kiểm tra bit3 = overflow)
 * Nếu (debug_st2 & 0x08) != 0 → dữ liệu mag bị tràn, không dùng để calib. */
volatile uint8_t debug_st1 = 0, debug_st2 = 0;

/* =============================================================================
 * PHẦN 2: GIAO TIẾP SPI CẤP THẤP
 * -----------------------------------------------------------------------------
 * 4 hàm giao tiếp với thanh ghi ICM-20948 qua SPI2.
 * Tất cả đều:
 *   - Tự động chọn bank (ICM_SelectBank)
 *   - Kéo CS LOW trước, HIGH sau khi xong
 *   - Timeout = 100ms (đủ rộng cho SPI @ 1MHz truyền 23 byte ≈ 0.2ms)
 * =============================================================================
 */

/* -----------------------------------------------------------------------------
 * ICM_SelectBank — Chọn bank thanh ghi cho lần đọc/ghi tiếp theo.
 *
 * ICM-20948 có 4 bank (0..3). Phải ghi giá trị bank vào REG_BANK_SEL (0x7F)
 * trước khi truy cập thanh ghi trong bank đó.
 *
 * ĐẦU VÀO: bank — enum _b0/_b1/_b2/_b3 (đã dịch trái 4 bit)
 * GHI CHÚ: Ghi bank là thao tác đơn giản: chỉ 2 byte (reg + value), CS LOW→HIGH.
 * ----------------------------------------------------------------------------- */
void ICM_SelectBank(user_bank bank) {
    uint8_t tx[2] = {REG_BANK_SEL & 0x7F, (uint8_t)bank};
    ICM_CS_LOW();
    HAL_SPI_Transmit(&hspi2, tx, 2, 100);
    ICM_CS_HIGH();
}

/* -----------------------------------------------------------------------------
 * ICM_WriteReg — Ghi 1 byte vào thanh ghi trong bank chỉ định.
 *
 * QUY TRÌNH:
 *   1. Chọn bank (ICM_SelectBank)
 *   2. CS LOW
 *   3. Gửi 2 byte: [địa chỉ thanh ghi (bit7=0 = ghi)] + [giá trị cần ghi]
 *   4. CS HIGH
 *
 * ĐẦU VÀO:
 *   bank  — bank chứa thanh ghi
 *   reg   — địa chỉ thanh ghi (0x00-0x7F)
 *   value — giá trị cần ghi (0x00-0xFF)
 * ----------------------------------------------------------------------------- */
void ICM_WriteReg(user_bank bank, uint8_t reg, uint8_t value) {
    uint8_t tx[2] = {reg & 0x7F, value};   /* & 0x7F để đảm bảo bit7 = 0 (write) */
    ICM_SelectBank(bank);
    ICM_CS_LOW();
    HAL_SPI_Transmit(&hspi2, tx, 2, 100);
    ICM_CS_HIGH();
}

/* -----------------------------------------------------------------------------
 * ICM_ReadReg — Đọc 1 byte từ thanh ghi trong bank chỉ định.
 *
 * QUY TRÌNH:
 *   1. Chọn bank
 *   2. CS LOW
 *   3. Gửi 2 byte: [địa chỉ thanh ghi | 0x80] + [dummy 0x00]
 *      → ICM sẽ trả về 1 byte ở vị trí nhận thứ 2 (rx[1])
 *   4. CS HIGH
 *   5. Trả về rx[1]
 *
 * ĐẦU VÀO: bank, reg
 * TRẢ VỀ: giá trị byte đọc được
 * ----------------------------------------------------------------------------- */
uint8_t ICM_ReadReg(user_bank bank, uint8_t reg) {
    uint8_t tx[2] = {reg | 0x80, 0x00};   /* | 0x80 để đảm bảo bit7 = 1 (read) */
    uint8_t rx[2] = {0};
    ICM_SelectBank(bank);
    ICM_CS_LOW();
    HAL_SPI_TransmitReceive(&hspi2, tx, rx, 2, 100);
    ICM_CS_HIGH();
    return rx[1];   /* rx[0] là dummy, rx[1] mới là giá trị thật */
}

/* -----------------------------------------------------------------------------
 * ICM_ReadRegs — Đọc nhiều byte liên tiếp từ vùng nhớ bắt đầu tại reg.
 *
 * QUY TRÌNH:
 *   1. Chọn bank
 *   2. CS LOW
 *   3. Gửi len+1 byte: [địa chỉ | 0x80] + [dummy × len]
 *      → ICM trả về len byte dữ liệu ở vị trí rx[1..len]
 *   4. CS HIGH
 *   5. Copy rx[1..len] vào buffer data
 *
 * ĐẦU VÀO:
 *   bank — bank chứa thanh ghi
 *   reg  — địa chỉ thanh ghi bắt đầu
 *   data — con trỏ buffer nhận dữ liệu (phải đủ len byte)
 *   len  — số byte cần đọc (tối đa 63)
 *
 * LƯU Ý QUAN TRỌNG:
 *   - Giới hạn len ≤ 63 vì buffer nội bộ tx[64]/rx[64].
 *   - Nếu len > 63 → return ngay (không đọc gì) để tránh tràn buffer.
 *   - ICM-20948 tự động tăng địa chỉ sau mỗi byte đọc (burst read).
 *
 * ỨNG DỤNG CHÍNH: Đọc 23 byte từ ACCEL_XOUT_H để lấy toàn bộ accel + gyro
 *                 + temp + 9 byte mag trong 1 lần giao dịch SPI duy nhất.
 * ----------------------------------------------------------------------------- */
void ICM_ReadRegs(user_bank bank, uint8_t reg, uint8_t *data, uint16_t len) {
    uint8_t tx[64] = {0};
    uint8_t rx[64] = {0};
    if (len > 63) return;   /* Bảo vệ: tránh tràn buffer nội bộ */
    tx[0] = reg | 0x80;     /* Byte đầu: địa chỉ thanh ghi với bit7=1 (read) */
    ICM_SelectBank(bank);
    ICM_CS_LOW();
    HAL_SPI_TransmitReceive(&hspi2, tx, rx, len + 1, 100);
    ICM_CS_HIGH();
    /* Copy dữ liệu thật (bỏ byte dummy ở rx[0]) */
    for (uint16_t i = 0; i < len; i++) data[i] = rx[i + 1];
}

/* =============================================================================
 * PHẦN 3: HIỆU CHUẨN GYRO
 * =============================================================================
 */

/* -----------------------------------------------------------------------------
 * ICM_CalibrateGyro — Hiệu chuẩn gyro, tính offset tĩnh (bias).
 *
 * NGUYÊN LÝ:
 *   Khi board đứng yên, gyro lý tưởng phải đọc (0, 0, 0) dps. Nhưng thực tế
 *   gyro luôn có offset nhỏ (zero-rate offset) do sai số chế tạo + nhiệt độ.
 *   → Cần đo offset này khi biết chắc board đứng yên, rồi trừ đi sau này.
 *
 * QUY TRÌNH:
 *   1. Bật LED PC13 báo hiệu bắt đầu calib (sáng liên tục)
 *   2. Đợi 50 mẫu × 2ms = 100ms để gyro ổn định (loại bỏ giao động ban đầu)
 *   3. Đọc 500 mẫu × 2ms = 1000ms, cộng dồn giá trị gyro thô
 *   4. Tính trung bình → ghi vào gx_bias, gy_bias, gz_bias
 *   5. Tắt LED báo hiệu xong
 *
 * YÊU CẦU: Board phải ĐỨNG YÊN HOÀN TOÀN trong suốt ~1.5 giây.
 *          Nếu board rung → bias sai → yaw sẽ drift nhanh sau này.
 *
 * ĐƠN VỊ: dps (degrees per second)
 * ĐỘ PHÂN GIẢI: 65.536 LSB/dps (dải đo ±500 dps, cấu hình trong GYRO_CONFIG_1)
 * CÔNG THỨC: raw / 65.536 = giá trị dps
 * ----------------------------------------------------------------------------- */
void ICM_CalibrateGyro(void) {
    float gx_sum = 0, gy_sum = 0, gz_sum = 0;
    uint8_t raw[12];               /* Chỉ đọc 12 byte (accel + gyro), bỏ mag */
    int samples = 500;

    LED_Heartbeat_On();

    /* Bước 1: Warm-up — đợi 50 mẫu để gyro ổn định sau khi cấp nguồn */
    for (int i = 0; i < 50; i++) {
        ICM_ReadRegs(_b0, ACCEL_XOUT_H, raw, 12);
        HAL_Delay(2);
    }

    /* Bước 2: Thu thập 500 mẫu, cộng dồn giá trị gyro */
    for (int i = 0; i < samples; i++) {
        ICM_ReadRegs(_b0, ACCEL_XOUT_H, raw, 12);

        /* Giải mã gyro từ 12 byte: raw[6..11] là gyro X/Y/Z (2 byte mỗi trục) */
        int16_t raw_gx = (int16_t)((raw[6]  << 8) | raw[7]);
        int16_t raw_gy = (int16_t)((raw[8]  << 8) | raw[9]);
        int16_t raw_gz = (int16_t)((raw[10] << 8) | raw[11]);

        /* Chuyển raw → dps và cộng dồn */
        gx_sum += raw_gx / 65.536f;
        gy_sum += raw_gy / 65.536f;
        gz_sum += raw_gz / 65.536f;
        HAL_Delay(2);
    }

    /* Bước 3: Tính trung bình → bias */
    gx_bias = gx_sum / samples;
    gy_bias = gy_sum / samples;
    gz_bias = gz_sum / samples;

    LED_Heartbeat_Off();
}

/* =============================================================================
 * PHẦN 4: KHỞI TẠO CHIP
 * =============================================================================
 */

/* -----------------------------------------------------------------------------
 * ICM_Init — Khởi tạo toàn bộ ICM-20948.
 *
 * QUY TRÌNH:
 *   1. Kéo CS HIGH (idle)
 *   2. Đợi 20ms cho chip ổn định sau khi cấp nguồn
 *   3. Đọc WHO_AM_I (0x00, bank 0) — phải = 0xEA
 *      Nếu khác → return ngay, không cấu hình tiếp
 *   4. Reset chip: PWR_MGMT_1 = 0xC1 (bit7 = DEVICE_RESET, bit6 = SLEEP, bit0 = CLKSEL)
 *   5. Đợi 50ms, sau đó wake-up: PWR_MGMT_1 = 0x01 (chọn auto clock)
 *   6. Cấu hình accel:
 *      - ACCEL_SMPLRT_DIV_1/2 = 0x00 → ODR = 1.125 kHz (max)
 *      - ACCEL_CONFIG = (0x01<<1)|0x01 → FSR = ±4g, DLPF bật
 *   7. Cấu hình gyro:
 *      - GYRO_SMPLRT_DIV = 0x00 → ODR = 1.125 kHz (max)
 *      - GYRO_CONFIG_1 = (0x01<<1)|0x01 → FSR = ±500 dps, DLPF bật
 *   8. Gọi AK09916_Init() để khởi tạo magnetometer qua I2C nội bộ
 *
 * TRẢ VỀ: giá trị WHO_AM_I đọc được (0xEA = OK, khác = lỗi)
 *
 * CÁCH DÙNG TRONG main():
 *   if (ICM_Init() != 0xEA) {
 *       // Xử lý lỗi: nháy LED báo động
 *       while (1) { ... }
 *   }
 *
 * LƯU Ý VỀ FSR (Full Scale Range):
 *   - Accel ±4g: độ phân giải 8192 LSB/g → chia raw cho 8192.0f
 *   - Gyro ±500 dps: độ phân giải 65.536 LSB/dps → chia raw cho 65.536f
 *   Nếu đổi FSR trong tương lai → phải đổi hệ số chia tương ứng.
 * ----------------------------------------------------------------------------- */
uint8_t ICM_Init(void) {
    ICM_CS_HIGH();
    HAL_Delay(20);

    /* Bước 1: Kiểm tra WHO_AM_I */
    uint8_t whoami = ICM_ReadReg(_b0, WHO_AM_I_REG);
    if (whoami != 0xEA) return whoami;   /* Trả về giá trị sai để main xử lý */

    /* Bước 2: Reset chip */
    ICM_WriteReg(_b0, PWR_MGMT_1, 0xC1);   /* DEVICE_RESET=1, SLEEP=1, CLKSEL=1 (auto) */
    HAL_Delay(50);

    /* Bước 3: Wake-up, chọn clock source = auto (dùng gyro X làm ref) */
    ICM_WriteReg(_b0, PWR_MGMT_1, 0x01);
    HAL_Delay(20);

    /* Bước 4: Cấu hình Accel — ODR max, FSR ±4g, DLPF bật */
    ICM_WriteReg(_b2, ACCEL_SMPLRT_DIV_1, 0x00);
    ICM_WriteReg(_b2, ACCEL_SMPLRT_DIV_2, 0x00);
    ICM_WriteReg(_b2, ACCEL_CONFIG, (0x01 << 1) | 0x01);

    /* Bước 5: Cấu hình Gyro — ODR max, FSR ±500 dps, DLPF bật */
    ICM_WriteReg(_b2, GYRO_SMPLRT_DIV_REG, 0x00);
    ICM_WriteReg(_b2, GYRO_CONFIG_1, (0x01 << 1) | 0x01);

    /* Bước 6: Khởi tạo magnetometer AK09916 */
    AK09916_Init();

    return whoami;   /* Trả về 0xEA để main biết init thành công */
}

/* =============================================================================
 * PHẦN 5: ĐỌC 9 TRỤC
 * =============================================================================
 */

/* -----------------------------------------------------------------------------
 * ICM_Read9Axis — Đọc đồng thời 9 trục (accel + gyro + mag) trong 1 giao dịch SPI.
 *
 * NGUYÊN LÝ:
 *   ICM-20948 có vùng dữ liệu liên tục bắt đầu từ ACCEL_XOUT_H (0x2D, bank 0):
 *     0x2D..0x2E : Accel X (2 byte, big-endian)
 *     0x2F..0x30 : Accel Y
 *     0x31..0x32 : Accel Z
 *     0x33..0x34 : Gyro X
 *     0x35..0x36 : Gyro Y
 *     0x37..0x38 : Gyro Z
 *     0x39..0x3A : Temp (2 byte, không dùng)
 *     0x3B..0x43 : EXT_SLV_SENS_DATA (9 byte, chứa mag từ AK09916)
 *   → Đọc 23 byte liên tục là có đủ 9 trục + đồng bộ thời gian.
 *
 * QUY TRÌNH:
 *   1. Đọc 23 byte từ ACCEL_XOUT_H vào buffer raw[]
 *   2. Giải mã accel: raw[0..5] → ax, ay, az (chia 8192.0f)
 *   3. Giải mã gyro:  raw[6..11] → gx, gy, gz (chia 65.536f, trừ bias)
 *   4. Lưu 9 byte mag raw[14..22] vào mag_raw[] + debug_st1/st2
 *   5. Giải mã mag:  raw[15..20] → mx, my, mz (little-endian)
 *   6. Hoán vị trục mag + đảo dấu Z (khớp hướng gắn chip trên board)
 *   7. Gọi Mag_SetCalibrated() để áp dụng hiệu chuẩn hard/soft-iron
 *
 * LƯU Ý VỀ BIT ORDER:
 *   - Accel/Gyro: BIG-ENDIAN (byte cao trước) → (raw[i] << 8) | raw[i+1]
 *   - Mag:        LITTLE-ENDIAN (byte thấp trước) → (raw[i+1] << 8) | raw[i]
 *   Đây là đặc thù của AK09916 (little-endian) khác với ICM (big-endian).
 *
 * LƯU Ý VỀ HOÁN VỊ TRỤC MAG:
 *   Code đọc raw_mx từ raw[16]/raw[15], raw_my từ raw[18]/raw[17]...
 *   nhưng khi gán: uncal_mx = raw_my, uncal_my = raw_mx.
 *   → Đây là HOÁN VỊ X↔Y + ĐẢO DẤU Z để khớp hệ trục của board.
 *   Nếu board lắp khác → phải sửa lại cho đúng.
 *
 * ĐƠN VỊ SAU KHI ĐỌC:
 *   ax/ay/az — đơn vị g (1g = 9.81 m/s²)
 *   gx/gy/gz — đơn vị dps (đã trừ bias)
 *   mx/my/mz — đơn vị µT (đã hiệu chuẩn hard/soft-iron)
 * ----------------------------------------------------------------------------- */
void ICM_Read9Axis(void) {
    uint8_t raw[23];
    ICM_ReadRegs(_b0, ACCEL_XOUT_H, raw, 23);

    int16_t raw_ax = (int16_t)((raw[0] << 8) | raw[1]);
    int16_t raw_ay = (int16_t)((raw[2] << 8) | raw[3]);
    int16_t raw_az = (int16_t)((raw[4] << 8) | raw[5]);
    ax = raw_ax / 8192.0f;
    ay = raw_ay / 8192.0f;
    az = raw_az / 8192.0f;

    /* --- GIẢI MÃ GYRO (raw[6..11], big-endian, chia 65.536 vì FSR ±500 dps) ---
     * Trừ bias đã hiệu chuẩn để loại bỏ zero-rate offset. */
    int16_t raw_gx = (int16_t)((raw[6]  << 8) | raw[7]);
    int16_t raw_gy = (int16_t)((raw[8]  << 8) | raw[9]);
    int16_t raw_gz = (int16_t)((raw[10] << 8) | raw[11]);
    gx = (raw_gx / 65.536f) - gx_bias;
    gy = (raw_gy / 65.536f) - gy_bias;
    gz = (raw_gz / 65.536f) - gz_bias;

    /* --- LƯU 9 BYTE MAG RAW (raw[14..22]) ---
     * Bao gồm cả 2 byte trạng thái ST1/ST2 để debug. */
    for (int i = 0; i < 9; i++) mag_raw[i] = raw[14 + i];
    debug_st1 = mag_raw[0];   /* ST1: bit0 = DRDY (data ready) */
    debug_st2 = mag_raw[8];   /* ST2: bit3 = overflow */

    /* --- GIẢI MÃ MAG (little-endian, 0.15 µT/LSB) ---
     * AK09916 trả về little-endian: byte thấp trước, byte cao sau.
     * Công thức: giá trị = (byte_cao << 8) | byte_thấp
     * Nhưng vì little-endian, byte thấp ở vị trí raw[15], cao ở raw[16].
     * → (raw[16] << 8) | raw[15] */
    int16_t raw_mx = (int16_t)((raw[16] << 8) | raw[15]);
    int16_t raw_my = (int16_t)((raw[18] << 8) | raw[17]);
    int16_t raw_mz = (int16_t)((raw[20] << 8) | raw[19]);

    /* --- HOÁN VỊ TRỤC + ĐẢO DẤU Z (khớp hệ trục board) ---
     * Đây là đặc thù của board cụ thể — nếu lắp chip xoay 90° so với trục
     * mong muốn, phải hoán vị X↔Y và/hoặc đảo dấu trục.
     * 0.15f là độ phân giải của AK09916: 0.15 µT/LSB. */
    float uncal_mx =  (float)raw_my * 0.15f;
    float uncal_my =  (float)raw_mx * 0.15f;
    float uncal_mz = -(float)raw_mz * 0.15f;

    /* --- ÁP DỤNG HIỆU CHUẨN HARD/SOFT-IRON ---
     * Mag_SetCalibrated() sẽ:
     *   1. Trừ mag_x/y/z_bias (hard-iron)
     *   2. Nhân mag_x/y/z_scale (soft-iron)
     *   3. Ghi kết quả vào mx/my/mz toàn cục */
    Mag_SetCalibrated(uncal_mx, uncal_my, uncal_mz);
}
