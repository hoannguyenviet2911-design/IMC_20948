/* =============================================================================
 * mag_ak09916.c
 * -----------------------------------------------------------------------------
 * MỤC ĐÍCH:
 *   Triển khai driver cho magnetometer AK09916 — cảm biến từ trường 3 trục
 *   được tích hợp BÊN TRONG chip ICM-20948.
 *
 *   Cung cấp:
 *     - Giao tiếp I2C nội bộ (qua ICM-20948 làm I2C Master)
 *     - Khởi tạo AK09916 (bật chế độ đo liên tục)
 *     - Hiệu chuẩn mag (hard-iron + soft-iron)
 *     - Áp dụng hiệu chuẩn vào dữ liệu thô
 *
 * SƠ ĐỒ GIAO TIẾP:
 *   STM32 ──SPI2──► ICM-20948 ──I2C nội bộ──► AK09916
 *                      │
 *                      └── Tự động đọc AK09916, lưu vào EXT_SLV_SENS_DATA
 *                      └── STM32 đọc 9 byte mag từ ICM_Read9Axis()
 *
 * TẠI SAO PHẢI HIỆU CHUẨN MAG?
 *   1. Hard-iron offset  — do vật liệu sắt từ gần chip (loa, ốc vít...)
 *                          → dịch tâm vòng tròn từ trường khỏi gốc tọa độ
 *   2. Soft-iron distortion — do vật liệu dẫn từ → méo vòng tròn thành ellipse
 *
 *   Hiệu chuẩn = xoay board 360° theo CẢ 3 TRỤC để tìm min/max của mỗi trục,
 *   từ đó tính bias (tâm) và scale (bán kính) để "kéo" dữ liệu về vòng tròn đơn vị.
 *
 * CẤU TRÚC FILE:
 *   1. Định nghĩa biến toàn cục (dữ liệu mag + hệ số hiệu chuẩn)
 *   2. Hàm helper: áp dụng hiệu chuẩn vào dữ liệu thô
 *   3. Hàm giao tiếp I2C nội bộ (2 hàm)
 *   4. Hàm khởi tạo AK09916 (1 hàm)
 *   5. Hàm hiệu chuẩn mag (1 hàm)
 * =============================================================================
 */

/* =============================================================================
 * PHẦN 1: INCLUDE & ĐỊNH NGHĨA BIẾN TOÀN CỤC
 * =============================================================================
 */

#include "mag_ak09916.h"     /* Header của chính module này */
#include "led_direction.h"   /* Cần cho LED_Heartbeat_On/Off() trong calib mag */

/* --- TỪ TRƯỜNG 3 TRỤC (đơn vị: µT — micro Tesla) ---
 * Đã qua hiệu chuẩn hard-iron (trừ bias) và soft-iron (nhân scale).
 * Được cập nhật bởi Mag_SetCalibrated() — gọi từ ICM_Read9Axis() mỗi 20ms.
 *
 * Khi xoay board 360° quanh trục Z: mx/my dao động đối xứng quanh 0, mz gần như
 * không đổi (chỉ phụ thuộc vĩ độ địa lý).
 * Khi board hướng Bắc: mx > 0 (hoặc theo quy ước trục), my ≈ 0. */
volatile float mx, my, mz;

/* --- BIAS HARD-IRON (tâm vòng tròn từ trường) ---
 * Được tính trong ICM_CalibrateMag() bằng công thức:
 *   bias = (max + min) / 2
 * Đây là offset cần TRỪ khỏi giá trị thô để dịch tâm về gốc tọa độ.
 * Kỳ vọng: |bias| < 100 µT. Nếu lớn hơn → có nam châm mạnh gần chip.
 * Khởi tạo = 0 để lần đọc đầu tiên không bị sai (trước khi calib). */
volatile float mag_x_bias = 0.0f, mag_y_bias = 0.0f, mag_z_bias = 0.0f;

/* --- SCALE SOFT-IRON (bán kính ellipse → chuẩn hóa về hình tròn) ---
 * Được tính trong ICM_CalibrateMag() bằng công thức:
 *   scale = avg_delta / delta_trục
 * Trong đó avg_delta là trung bình bán kính 3 trục, delta_trục là bán kính trục đó.
 * Kỳ vọng: 0.5 < scale < 2.0. Nếu scale quá lớn/nhỏ → dữ liệu calib kém.
 * Khởi tạo = 1.0 (không thay đổi gì) trước khi calib. */
volatile float mag_x_scale = 1.0f, mag_y_scale = 1.0f, mag_z_scale = 1.0f;

/* =============================================================================
 * PHẦN 2: HÀM HELPER — ÁP DỤNG HIỆU CHUẨN VÀO DỮ LIỆU THÔ
 * =============================================================================
 */

/* -----------------------------------------------------------------------------
 * Mag_SetCalibrated — Nhận 3 giá trị mag thô (đã hoán vị trục) và áp dụng
 *                     công thức hiệu chuẩn, lưu kết quả vào mx/my/mz.
 *
 * CÔNG THỨC:
 *   mx = (ux - mag_x_bias) * mag_x_scale
 *   my = (uy - mag_y_bias) * mag_y_scale
 *   mz = (uz - mag_z_bias) * mag_z_scale
 *
 *   Trong đó:
 *     ux, uy, uz      = giá trị thô sau hoán vị trục (từ ICM_Read9Axis)
 *     mag_*_bias      = offset hard-iron (tâm vòng tròn)
 *     mag_*_scale     = hệ số scale soft-iron (sửa méo ellipse)
 *
 * QUY TRÌNH 2 BƯỚC:
 *   Bước 1 (bias): trừ offset → dịch tâm ellipse về gốc tọa độ
 *   Bước 2 (scale): nhân hệ số → biến ellipse thành hình tròn
 *
 * ĐẦU VÀO:  ux, uy, uz — giá trị thô (µT, sau hoán vị trục)
 * ĐẦU RA:   mx, my, mz — giá trị đã hiệu chuẩn (µT)
 * ĐƯỢC GỌI: từ ICM_Read9Axis() — không gọi trực tiếp từ main loop.
 * ----------------------------------------------------------------------------- */
void Mag_SetCalibrated(float ux, float uy, float uz) {
    mx = (ux - mag_x_bias) * mag_x_scale;
    my = (uy - mag_y_bias) * mag_y_scale;
    mz = (uz - mag_z_bias) * mag_z_scale;
}

/* =============================================================================
 * PHẦN 3: GIAO TIẾP I2C NỘI BỘ (QUA ICM-20948 LÀM I2C MASTER)
 * -----------------------------------------------------------------------------
 * AK09916 KHÔNG kết nối trực tiếp với STM32. Nó kết nối với ICM-20948 qua
 * I2C nội bộ, và ICM-20948 đóng vai trò I2C Master.
 *
 * Để ghi/đọc AK09916, ta phải:
 *   1. Cấu hình ICM ở bank 3 (I2C Master)
 *   2. Ghi địa chỉ slave + thanh ghi đích vào I2C_SLV0_ADDR / I2C_SLV0_REG
 *   3. Kích hoạt đọc/ghi qua I2C_SLV0_CTRL
 *   4. Chờ ICM thực hiện xong giao dịch I2C nội bộ
 *   5. (Nếu đọc) Lấy dữ liệu từ thanh ghi EXT_SLV_SENS_DATA (0x3B) ở bank 0
 * =============================================================================
 */

/* -----------------------------------------------------------------------------
 * AK09916_WriteReg — Ghi 1 byte vào thanh ghi AK09916 (qua I2C Master của ICM).
 *
 * QUY TRÌNH:
 *   1. Đặt địa chỉ slave (không có bit 0x80 = chế độ GHI) vào I2C_SLV0_ADDR
 *   2. Đặt thanh ghi đích vào I2C_SLV0_REG
 *   3. Đặt dữ liệu cần ghi vào I2C_SLV0_DO (Data Out)
 *   4. Đợi 10ms để các thanh ghi trên ổn định
 *   5. Kích hoạt: I2C_SLV0_CTRL = 0x80 | 0x01
 *      - 0x80: ENABLE (bật slave 0)
 *      - 0x01: LENG = 1 (ghi/đọc 1 byte)
 *   6. Đợi 20ms để ICM thực hiện xong giao dịch I2C
 *
 * ĐẦU VÀO: reg, value
 * LƯU Ý: Không có return value (không kiểm tra lỗi).
 *
 * VÍ DỤ: AK09916_WriteReg(MAG_CTRL2, 0x08); // Bật continuous 100Hz
 * ----------------------------------------------------------------------------- */
void AK09916_WriteReg(uint8_t reg, uint8_t value) {
    ICM_WriteReg(_b3, I2C_SLV0_ADDR, AK09916_ADDRESS);   /* Chế độ GHI (bit7=0) */
    ICM_WriteReg(_b3, I2C_SLV0_REG,  reg);               /* Thanh ghi đích */
    ICM_WriteReg(_b3, I2C_SLV0_DO,   value);             /* Dữ liệu cần ghi */
    HAL_Delay(10);
    ICM_WriteReg(_b3, I2C_SLV0_CTRL, 0x80 | 0x01);       /* Enable + 1 byte */
    HAL_Delay(20);                                       /* Chờ ICM thực thi */
}

/* -----------------------------------------------------------------------------
 * AK09916_ReadReg — Đọc 1 byte từ thanh ghi AK09916 (qua I2C Master của ICM).
 *
 * QUY TRÌNH:
 *   1. Đặt địa chỉ slave (CÓ bit 0x80 = chế độ ĐỌC) vào I2C_SLV0_ADDR
 *   2. Đặt thanh ghi đích vào I2C_SLV0_REG
 *   3. Kích hoạt: I2C_SLV0_CTRL = 0x80 | 0x01 (đọc 1 byte)
 *   4. Đợi 20ms để ICM đọc xong AK09916 và lưu vào EXT_SLV_SENS_DATA
 *   5. Đọc kết quả từ ICM qua SPI: ICM_ReadReg(_b0, 0x3B)
 *      - 0x3B là byte đầu tiên của vùng EXT_SLV_SENS_DATA
 *      - Khi đọc 1 byte, ICM trả về byte đầu tiên trong vùng này
 *
 * ĐẦU VÀO: reg — thanh ghi cần đọc
 * TRẢ VỀ: giá trị byte đọc được (0x00-0xFF)
 *
 * LƯU Ý: Hàm này chỉ dùng để kiểm tra/debug AK09916. Trong vận hành bình
 * thường, dữ liệu mag được đọc tự động qua ICM_Read9Axis() (đọc 9 byte liên
 * tục từ EXT_SLV_SENS_DATA). */
uint8_t AK09916_ReadReg(uint8_t reg) {
    ICM_WriteReg(_b3, I2C_SLV0_ADDR, 0x80 | AK09916_ADDRESS);  /* Chế độ ĐỌC (bit7=1) */
    ICM_WriteReg(_b3, I2C_SLV0_REG,  reg);                     /* Thanh ghi đích */
    ICM_WriteReg(_b3, I2C_SLV0_CTRL, 0x80 | 0x01);             /* Enable + 1 byte */
    HAL_Delay(20);                                             /* Chờ ICM đọc xong */
    return ICM_ReadReg(_b0, 0x3B);                             /* Lấy byte từ bank 0 */
}

/* =============================================================================
 * PHẦN 4: KHỞI TẠO AK09916
 * =============================================================================
 */

/* -----------------------------------------------------------------------------
 * AK09916_Init — Khởi tạo magnetometer AK09916.
 *
 * QUY TRÌNH:
 *   1. Bật I2C Master trong ICM:
 *      - Đọc USER_CTRL_REG (bank 0)
 *      - Set bit 5 (I2C_MST_EN) = 1
 *      - Ghi lại USER_CTRL_REG
 *   2. Cấu hình I2C Master:
 *      - I2C_MST_CTRL = 0x07 → clock ~400kHz (fast mode)
 *      - I2C_MST_ODR_CONFIG = 0x03 → ODR = 1.1kHz / 2^3 ≈ 137 Hz
 *   3. Reset AK09916 (soft reset):
 *      - Ghi MAG_CTRL3 = 0x01 (bit0 = SRST)
 *      - Đợi 100ms để AK09916 khởi động lại
 *   4. Bật chế độ đo liên tục 100Hz:
 *      - Ghi MAG_CTRL2 = 0x08 (continuous mode 4)
 *      - Đợi 20ms
 *   5. Cấu hình ICM tự động đọc AK09916 mỗi chu kỳ:
 *      - Địa chỉ slave (ĐỌC) → I2C_SLV0_ADDR
 *      - Thanh ghi bắt đầu = MAG_ST1 (0x10) → I2C_SLV0_REG
 *      - Đọc 9 byte liên tục (0x09) → I2C_SLV0_CTRL = 0x80 | 0x09
 *      → ICM sẽ tự động đọc 9 byte (ST1 + HXL..HZH + TMPS + ST2) từ AK09916
 *        mỗi chu kỳ và lưu vào EXT_SLV_SENS_DATA
 *
 * ĐƯỢC GỌI: tự động bên trong ICM_Init() — không cần gọi riêng.
 *
 * THANH GHI ĐIỀU KHIỂN AK09916:
 *   MAG_CTRL2 (0x31): chọn mode đo
 *     0x00 = power down
 *     0x01 = single measurement
 *     0x02 = continuous 10Hz
 *     0x04 = continuous 20Hz
 *     0x08 = continuous 100Hz  ← code dùng
 *   MAG_CTRL3 (0x32): soft reset
 *     0x01 = SRST (reset)
 * ----------------------------------------------------------------------------- */
void AK09916_Init(void) {
    /* Bước 1: Bật I2C Master trong ICM-20948 */
    uint8_t temp = ICM_ReadReg(_b0, USER_CTRL_REG);
    temp |= 0x20;                               /* Set bit5 = I2C_MST_EN */
    ICM_WriteReg(_b0, USER_CTRL_REG, temp);
    HAL_Delay(20);

    /* Bước 2: Cấu hình I2C Master (clock 400kHz) và ODR (~137Hz) */
    ICM_WriteReg(_b3, I2C_MST_CTRL, 0x07);
    HAL_Delay(10);
    ICM_WriteReg(_b3, I2C_MST_ODR_CONFIG, 0x03);
    HAL_Delay(10);

    /* Bước 3: Soft reset AK09916 */
    AK09916_WriteReg(MAG_CTRL3, 0x01);          /* SRST = 1 */
    HAL_Delay(100);                             /* Chờ 100ms để chip reset */

    /* Bước 4: Bật chế độ đo liên tục 100Hz */
    AK09916_WriteReg(MAG_CTRL2, 0x08);          /* Continuous mode 4 */
    HAL_Delay(20);

    /* Bước 5: Cấu hình ICM tự động đọc 9 byte mag mỗi chu kỳ
     * - Địa chỉ slave: 0x80 | 0x0C = 0x8C (ĐỌC từ AK09916)
     * - Thanh ghi bắt đầu: MAG_ST1 = 0x10
     * - Số byte: 9 (ST1 + HXL/HXH + HYL/HYH + HZL/HZH + TMPS + ST2)
     * - Cờ 0x80: ENABLE slave 0 */
    ICM_WriteReg(_b3, I2C_SLV0_ADDR, 0x80 | AK09916_ADDRESS);
    ICM_WriteReg(_b3, I2C_SLV0_REG,  MAG_ST1);
    ICM_WriteReg(_b3, I2C_SLV0_CTRL, 0x80 | 0x09);
    HAL_Delay(50);
}

/* =============================================================================
 * PHẦN 5: HIỆU CHUẨN MAG (HARD-IRON + SOFT-IRON)
 * =============================================================================
 */

/* -----------------------------------------------------------------------------
 * ICM_CalibrateMag — Hiệu chuẩn magnetometer (tìm bias + scale).
 *
 * NGUYÊN LÝ:
 *   Khi xoay board 360° theo mọi trục, đầu mút vector từ trường (mx, my, mz)
 *   vẽ nên 1 hình cầu (sphere) trong không gian 3D. Nhưng do sai số:
 *     - Hard-iron: tâm hình cầu KHÔNG ở gốc tọa độ → cần trừ bias
 *     - Soft-iron: hình cầu bị MÉO thành ellipsoid → cần nhân scale
 *   Mục tiêu: sau calib, đầu mút vector nằm trên mặt cầu đơn vị (bán kính không đổi).
 *
 * QUY TRÌNH (20 giây):
 *   1. Khởi tạo min/max ban đầu (số rất lớn/nhỏ để chắc chắn bị ghi đè)
 *   2. Trong 20s, liên tục:
 *      a. Đọc 9 trục (ICM_Read9Axis → cập nhật mx/my/mz)
 *      b. Nháy LED PC13 (150ms on/off) báo hiệu đang calib
 *      c. Cập nhật min/max của mx, my, mz (nếu dữ liệu hợp lệ)
 *      d. Delay 20ms (tần số ~50Hz)
 *   3. Sau 20s, tính toán:
 *      a. bias = (max + min) / 2  cho mỗi trục
 *      b. delta = (max - min) / 2  (bán kính thực của trục đó)
 *      c. avg_delta = trung bình 3 delta
 *      d. scale = avg_delta / delta  (chuẩn hóa bán kính các trục về bằng nhau)
 *
 * YÊU CẦU NGƯỜI DÙNG:
 *   - Xoay board 360° theo CẢ 3 TRỤC (X, Y, Z) trong 20s
 *   - Tránh xa nam châm, loa, vật liệu sắt từ
 *   - LED PC13 nháy chậm trong suốt quá trình calib (báo hiệu)
 *
 * KIỂM TRA DỮ LIỆU HỢP LỆ:
 *   if (!(debug_st2 & 0x08)) { ... }
 *   - debug_st2 = mag_raw[8] = thanh ghi ST2 của AK09916
 *   - Bit 3 (0x08) = OVERFLOW: nếu = 1 → dữ liệu bị tràn, không dùng để calib
 *   → Chỉ cập nhật min/max khi dữ liệu KHÔNG bị tràn.
 *
 * BẢO VỆ CHIA CHO 0:
 *   if (dx > 0.001f) mag_x_scale = avg / dx;
 *   - Nếu không xoay đủ 360° trên 1 trục → delta trục đó = 0 → chia cho 0
 *   - Kiểm tra dx > 0.001 để tránh lỗi
 *   - Nếu trục đó không đủ dữ liệu → giữ scale = 1.0 (không sửa méo)
 *
 * ĐƯỢC GỌI: từ main() sau ICM_CalibrateGyro() — chạy 1 lần lúc khởi động.
 * ----------------------------------------------------------------------------- */
void ICM_CalibrateMag(void) {
    /* Khởi tạo bias = 0, scale = 1 (không thay đổi gì) */
    mag_x_bias = 0.0f; mag_y_bias = 0.0f; mag_z_bias = 0.0f;
    mag_x_scale = 1.0f; mag_y_scale = 1.0f; mag_z_scale = 1.0f;

    /* Khởi tạo min = rất lớn, max = rất nhỏ để chắc chắn bị ghi đè lần đầu.
     * 32767 / -32768 là giới hạn của int16 (giá trị raw tối đa của mag). */
    float mag_min[3] = { 32767.0f,  32767.0f,  32767.0f};
    float mag_max[3] = {-32768.0f, -32768.0f, -32768.0f};

    uint32_t start_tick = HAL_GetTick();

    /* Vòng lặp calib 20 giây */
    while ((HAL_GetTick() - start_tick) < 20000) {
        /* Nháy LED: 150ms ON, 150ms OFF → chu kỳ 300ms */
        if (((HAL_GetTick() / 150) % 2) == 0) LED_Heartbeat_On();
        else                                   LED_Heartbeat_Off();

        /* Đọc dữ liệu mag mới nhất (cập nhật mx/my/mz) */
        ICM_Read9Axis();

        /* Chỉ cập nhật min/max nếu dữ liệu mag KHÔNG bị tràn.
         * debug_st2 & 0x08 = bit overflow của AK09916 */
        if (!(debug_st2 & 0x08)) {
            if (mx < mag_min[0]) mag_min[0] = mx;
            if (mx > mag_max[0]) mag_max[0] = mx;
            if (my < mag_min[1]) mag_min[1] = my;
            if (my > mag_max[1]) mag_max[1] = my;
            if (mz < mag_min[2]) mag_min[2] = mz;
            if (mz > mag_max[2]) mag_max[2] = mz;
        }
        HAL_Delay(20);   /* 50Hz sampling */
    }
    LED_Heartbeat_Off();

    /* === TÍNH BIAS (HARD-IRON) ===
     * Tâm của hình cầu = trung bình của min và max */
    mag_x_bias = (mag_max[0] + mag_min[0]) / 2.0f;
    mag_y_bias = (mag_max[1] + mag_min[1]) / 2.0f;
    mag_z_bias = (mag_max[2] + mag_min[2]) / 2.0f;

    /* === TÍNH SCALE (SOFT-IRON) ===
     * Bán kính của mỗi trục = (max - min) / 2 */
    float dx = (mag_max[0] - mag_min[0]) / 2.0f;
    float dy = (mag_max[1] - mag_min[1]) / 2.0f;
    float dz = (mag_max[2] - mag_min[2]) / 2.0f;

    /* Bán kính trung bình (mong muốn tất cả trục có cùng bán kính này) */
    float avg = (dx + dy + dz) / 3.0f;

    /* scale = avg / delta_trục → chuẩn hóa bán kính các trục về bằng nhau.
     * Kiểm tra delta > 0.001 để tránh chia cho 0. */
    if (dx > 0.001f) mag_x_scale = avg / dx;
    if (dy > 0.001f) mag_y_scale = avg / dy;
    if (dz > 0.001f) mag_z_scale = avg / dz;
}
