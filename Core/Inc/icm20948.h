/* =============================================================================
 * icm20948.h
 * -----------------------------------------------------------------------------
 * MỤC ĐÍCH:
 *   Khai báo driver cho cảm biến IMU 9 trục ICM-20948 (TDK InvenSense).
 *   Bao gồm:
 *     - Gia tốc kế 3 trục (Accelerometer) — đo gia tốc, phát hiện nghiêng
 *     - Gyroscope 3 trục   (Gyro)         — đo vận tốc góc, tích hợp yaw
 *     - Từ trường kế 3 trục (Magnetometer AK09916 gắn trong chip) — đo hướng
 *
 * GIAO TIẾP:
 *   - ICM-20948 kết nối với STM32 qua SPI2 (chế độ Mode 3: CPOL=1, CPHA=1)
 *   - Chân CS điều khiển bằng phần mềm (NSS_SOFT) — PA8
 *   - Magnetometer AK09916 nằm BÊN TRONG ICM-20948, kết nối với ICM qua I2C
 *     nội bộ (I2C Master), không phải qua STM32. STM32 đọc mag gián tiếp qua
 *     thanh ghi EXT_SLV_SENS_DATA của ICM-20948.
 *
 * CẤU TRÚC FILE:
 *   1. Định nghĩa thanh ghi (register map)
 *   2. Macro điều khiển chân CS
 *   3. Enum chọn bank
 *   4. Biến toàn cục (dữ liệu cảm biến)
 *   5. Prototype các hàm
 * =============================================================================
 */

#ifndef ICM20948_H
#define ICM20948_H

#include "main.h"
#include <stdint.h>

/* =============================================================================
 * PHẦN 1: ĐỊA CHỈ THANH GHI (REGISTER MAP)
 * -----------------------------------------------------------------------------
 * ICM-20948 có 4 BANK thanh ghi (0 → 3). Mỗi bank có không gian địa chỉ 0x00-0x7F.
 * Để truy cập 1 thanh ghi, phải:
 *   1. Ghi bank cần dùng vào thanh ghi REG_BANK_SEL (0x7F)
 *   2. Sau đó đọc/ghi thanh ghi đích
 *
 * LƯU Ý: Bit 7 của địa chỉ thanh ghi quy định chiều đọc/ghi:
 *   - Bit 7 = 0 → GHI (Write)
 *   - Bit 7 = 1 → ĐỌC  (Read)
 * Code xử lý việc này tự động trong ICM_ReadReg / ICM_WriteReg.
 * =============================================================================
 */

/* --- Thanh ghi điều khiển BANK --- */
#define REG_BANK_SEL            0x7F   /* Chọn bank hiện hành (BANK 0..3) */

/* --- Thanh ghi BANK 0 (cấu hình cơ bản + dữ liệu cảm biến) --- */
#define WHO_AM_I_REG            0x00   /* Đọc ID chip — phải trả về 0xEA */
#define PWR_MGMT_1              0x06   /* Quản lý nguồn, chọn clock source */
#define USER_CTRL_REG           0x03   /* Bật/tắt I2C Master, FIFO, DMP... */
#define ACCEL_XOUT_H            0x2D   /* Bắt đầu vùng dữ liệu accel (6 byte) + gyro (6 byte) + temp (2) + mag (9). Đọc liên tục 23 byte từ đây. */

/* --- Thanh ghi BANK 2 (cấu hình accel & gyro chi tiết) --- */
#define GYRO_SMPLRT_DIV_REG     0x00   /* Chia tần số lấy mẫu gyro: ODR = 1.1kHz / (1 + DIV) */
#define GYRO_CONFIG_1           0x01   /* Chọn dải đo gyro (FSR) + bộ lọc DLPF */
#define ACCEL_SMPLRT_DIV_1      0x10   /* Byte cao của thanh ghi chia tần số accel */
#define ACCEL_SMPLRT_DIV_2      0x11   /* Byte thấp của thanh ghi chia tần số accel */
#define ACCEL_CONFIG            0x14   /* Chọn dải đo accel (FSR) + bộ lọc DLPF */

/* --- Thanh ghi BANK 3 (cấu hình I2C Master — dùng để giao tiếp với AK09916) --- */
#define I2C_MST_CTRL            0x01   /* Cấu hình I2C Master (tốc độ clock) */
#define I2C_SLV0_ADDR           0x03   /* Địa chỉ slave 0 (bit7 = chiều đọc/ghi, bit6-0 = addr) */
#define I2C_SLV0_REG            0x04   /* Thanh ghi đích trên slave 0 cần đọc/ghi */
#define I2C_SLV0_CTRL           0x05   /* Bật/tắt slave 0, số byte đọc/ghi */
#define I2C_SLV0_DO             0x06   /* Dữ liệu ghi ra slave 0 (khi ở chế độ write) */
#define I2C_MST_ODR_CONFIG      0x00   /* Tần số lấy mẫu của I2C Master */

/* =============================================================================
 * PHẦN 2: MACRO ĐIỀU KHIỂN CHÂN CHIP SELECT (CS)
 * -----------------------------------------------------------------------------
 * Chân CS (Chip Select) dùng để "chọn" ICM-20948 trên bus SPI.
 *   - CS = LOW  → ICM-20948 lắng nghe bus SPI (active)
 *   - CS = HIGH → ICM-20948 bỏ qua bus SPI (idle)
 *
 * Macro dùng ICM_CS_GPIO_Port và ICM_CS_Pin được CubeMX sinh ra trong main.h
 * (đã cấu hình = PA8). Nhờ đó nếu đổi chân trong CubeMX, code tự động đúng.
 *
 * TẠI SAO PHẢI DÙNG MACRO?
 *   - Tránh gõ lặp lại HAL_GPIO_WritePin(...) ở nhiều nơi.
 *   - Nếu sau này đổi chân CS, chỉ cần sửa 1 chỗ trong CubeMX.
 * =============================================================================
 */
#define ICM_CS_LOW()   HAL_GPIO_WritePin(ICM_CS_GPIO_Port, ICM_CS_Pin, GPIO_PIN_RESET)
#define ICM_CS_HIGH()  HAL_GPIO_WritePin(ICM_CS_GPIO_Port, ICM_CS_Pin, GPIO_PIN_SET)

/* =============================================================================
 * PHẦN 3: ENUM CHỌN BANK
 * -----------------------------------------------------------------------------
 * ICM-20948 có 4 bank (0..3). Mỗi bank có thanh ghi riêng.
 * Giá trị của enum chính là giá trị cần ghi vào REG_BANK_SEL để chuyển bank:
 *   _b0 = 0<<4 = 0x00 → BANK 0
 *   _b1 = 1<<4 = 0x10 → BANK 1
 *   _b2 = 2<<4 = 0x20 → BANK 2
 *   _b3 = 3<<4 = 0x30 → BANK 3
 *
 * TẠI SAO DỊCH TRÁI 4 BIT?
 *   Theo datasheet ICM-20948, thanh ghi BANK_SEL[4:0] nằm ở bit 4-0 của
 *   REG_BANK_SEL. Bit 5-7 dành riêng (reserved). Việc dịch trái 4 bit đảm bảo
 *   giá trị bank nằm đúng vị trí, đồng thời để dễ đọc code (_b2 thay vì 0x20).
 *
 * CÁCH DÙNG:
 *   ICM_WriteReg(_b2, ACCEL_CONFIG, 0x01);  // Ghi vào bank 2
 * =============================================================================
 */
typedef enum {
    _b0 = 0 << 4,   /* BANK 0 — thanh ghi cơ bản */
    _b1 = 1 << 4,   /* BANK 1 — thanh ghi ít dùng */
    _b2 = 2 << 4,   /* BANK 2 — cấu hình accel/gyro */
    _b3 = 3 << 4    /* BANK 3 — cấu hình I2C Master */
} user_bank;

/* =============================================================================
 * PHẦN 4: BIẾN TOÀN CỤC (DỮ LIỆU CẢM BIẾN)
 * -----------------------------------------------------------------------------
 * Các biến này được định nghĩa thực sự trong icm20948.c và được cập nhật bởi
 * hàm ICM_Read9Axis(). Các module khác (yaw_fusion, stationary, debug_vars)
 * đọc trực tiếp giá trị này thông qua từ khóa "extern".
 *
 * Từ khóa "volatile":
 *   Bắt buộc phải có vì các biến này được cập nhật liên tục trong vòng lặp
 *   chính và có thể được đọc từ debugger. Trình biên dịch KHÔNG được tối ưu
 *   hóa (cache vào thanh ghi) mà phải đọc lại từ RAM mỗi lần truy cập.
 * =============================================================================
 */

/* --- Gia tốc kế 3 trục (đơn vị: g, 1g = 9.81 m/s²) ---
 * Được tính bằng: raw / 8192.0f  (với dải đo ±4g, độ phân giải 8192 LSB/g)
 * Khi board nằm phẳng: ax≈0, ay≈0, az≈+1.00 (hoặc -1.00 tùy chiều gắn chip) */
extern volatile float ax, ay, az;

/* --- Gyro 3 trục (đơn vị: dps — degrees per second) ---
 * Được tính bằng: (raw / 65.536f) - bias  (với dải đo ±500 dps, 65.536 LSB/dps)
 * Khi board đứng yên: cả 3 giá trị ≈ 0.00 sau khi trừ bias */
extern volatile float gx, gy, gz;

/* --- Bias gyro (offset tĩnh sau calib) ---
 * Được tính 1 lần trong ICM_CalibrateGyro() bằng cách lấy trung bình 500 mẫu
 * khi board đứng yên. Sau đó mọi giá trị gx/gy/gz đọc được đều trừ đi bias này
 * để loại bỏ sai số zero-rate offset của phần cứng.
 * Kỳ vọng: |bias| < 2.0 dps. Nếu lớn hơn → calib sai (board bị rung). */
extern volatile float gx_bias, gy_bias, gz_bias;

/* --- Dữ liệu thô 9 byte từ magnetometer AK09916 ---
 * ICM-20948 tự động đọc AK09916 qua I2C nội bộ và lưu vào vùng EXT_SLV_SENS_DATA.
 * Cấu trúc 9 byte (đọc liên tục từ MAG_ST1):
 *   [0] ST1   — trạng thái 1 (bit0 = DRDY, bit1 = DOR)
 *   [1] HXL   — mag X low byte
 *   [2] HXH   — mag X high byte
 *   [3] HYL   — mag Y low byte
 *   [4] HYH   — mag Y high byte
 *   [5] HZL   — mag Z low byte
 *   [6] HZH   — mag Z high byte
 *   [7] TMPS  — dummy (không dùng)
 *   [8] ST2   — trạng thái 2 (bit3 = overflow)
 * Dùng để debug + kiểm tra trạng thái mag có sẵn sàng không. */
extern volatile uint8_t mag_raw[9];

/* --- Biến debug trạng thái mag (tiện cho Live Expressions) ---
 * debug_st1 = mag_raw[0] — trạng thái 1
 * debug_st2 = mag_raw[8] — trạng thái 2 (kiểm tra bit3 = overflow)
 * Nếu (debug_st2 & 0x08) != 0 → dữ liệu mag bị tràn, không nên dùng để calib. */
extern volatile uint8_t debug_st1, debug_st2;

/* =============================================================================
 * PHẦN 5: PROTOTYPE CÁC HÀM
 * =============================================================================
 */

/* -----------------------------------------------------------------------------
 * NHÓM HÀM GIAO TIẾP SPI CẤP THẤP
 * -----------------------------------------------------------------------------
 * Các hàm này thao tác trực tiếp với thanh ghi ICM-20948 qua SPI2.
 * Tất cả đều tự động điều khiển chân CS (LOW trước, HIGH sau khi xong).
 */

/* Chọn bank cho lần đọc/ghi tiếp theo.
 * Phải gọi trước mỗi lần đọc/ghi nếu bank khác với lần trước.
 * Ví dụ: ICM_SelectBank(_b2); */
void ICM_SelectBank(user_bank bank);

/* Ghi 1 byte vào thanh ghi trong bank chỉ định.
 * Tự động chọn bank trước khi ghi.
 * Ví dụ: ICM_WriteReg(_b2, ACCEL_CONFIG, 0x01); */
void ICM_WriteReg(user_bank bank, uint8_t reg, uint8_t value);

/* Đọc 1 byte từ thanh ghi trong bank chỉ định.
 * Trả về giá trị đọc được (0x00-0xFF).
 * Ví dụ: uint8_t id = ICM_ReadReg(_b0, WHO_AM_I_REG); */
uint8_t ICM_ReadReg(user_bank bank, uint8_t reg);

/* Đọc nhiều byte liên tiếp từ vùng nhớ bắt đầu tại reg.
 * - data: con trỏ buffer nhận dữ liệu (phải đủ len byte)
 * - len : số byte cần đọc (tối đa 63 để tránh tràn buffer nội bộ)
 * Ví dụ: uint8_t raw[23]; ICM_ReadRegs(_b0, ACCEL_XOUT_H, raw, 23); */
void ICM_ReadRegs(user_bank bank, uint8_t reg, uint8_t *data, uint16_t len);

/* -----------------------------------------------------------------------------
 * NHÓM HÀM KHỞI TẠO & HIỆU CHUẨN
 * -----------------------------------------------------------------------------
 */

/* Khởi tạo ICM-20948:
 *   1. Reset, đọc WHO_AM_I (phải = 0xEA)
 *   2. Đánh thức chip, chọn clock source
 *   3. Cấu hình accel (±4g), gyro (±500 dps)
 *   4. Khởi tạo AK09916 (mag) qua I2C nội bộ
 * TRẢ VỀ: giá trị WHO_AM_I đọc được (0xEA = OK, khác = lỗi)
 * CÁCH DÙNG: if (ICM_Init() != 0xEA) {  xử lý lỗi } */

uint8_t ICM_Init(void);

/* Hiệu chuẩn gyro — TỰ ĐỘNG, không cần người dùng can thiệp.
 * - Điều kiện: board phải ĐỨNG YÊN HOÀN TOÀN trong suốt quá trình (~1.5s)
 * - Thời gian: 500 mẫu × 2ms = 1 giây
 * - Kết quả: ghi vào gx_bias, gy_bias, gz_bias
 * - LED PC13 sáng liên tục trong lúc calib (báo hiệu cho người dùng)
 * GỌI 1 LẦN DUY NHẤT trong main() sau ICM_Init(). */
void ICM_CalibrateGyro(void);

/* Đọc đồng thời 9 trục (accel + gyro + mag) trong 1 lần đọc SPI.
 * - Đọc 23 byte liên tục từ ACCEL_XOUT_H (bank 0)
 * - Tự động trừ bias gyro, hiệu chuẩn mag (hard/soft-iron)
 * - Cập nhật các biến toàn cục: ax/ay/az, gx/gy/gz, mx/my/mz, mag_raw
 * Thời gian thực thi: ~1ms (SPI @ 1MHz).
 * GỌI trong vòng lặp chính mỗi 20ms. */
void ICM_Read9Axis(void);

#endif /* ICM20948_H */
