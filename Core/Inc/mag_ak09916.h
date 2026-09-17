/* =============================================================================
 * mag_ak09916.h
 * -----------------------------------------------------------------------------
 * MỤC ĐÍCH:
 *   Khai báo driver cho magnetometer AK09916 — cảm biến từ trường 3 trục
 *   được tích hợp BÊN TRONG chip ICM-20948 (không phải chip rời).
 *
 *   AK09916 đo từ trường Trái Đất theo 3 trục X/Y/Z → dùng để xác định
 *   hướng tuyệt đối của board so với Bắc từ (la bàn điện tử).
 *
 * ĐẶC ĐIỂM GIAO TIẾP (RẤT QUAN TRỌNG):
 *   - AK09916 KHÔNG kết nối trực tiếp với STM32.
 *   - Nó kết nối với ICM-20948 qua I2C nội bộ (ICM đóng vai trò I2C Master).
 *   - STM32 đọc dữ liệu mag GIÁN TIẾP qua thanh ghi EXT_SLV_SENS_DATA (0x3B)
 *     của ICM-20948, sau khi ICM tự động đọc AK09916.
 *
 *   Sơ đồ:
 *     STM32 ──SPI2──► ICM-20948 ──I2C nội bộ──► AK09916
 *                        │
 *                        └── Tự động đọc AK09916, lưu vào EXT_SLV_SENS_DATA
 *                        └── STM32 đọc 9 byte mag từ ICM_Read9Axis()
 *
 * TẠI SAO PHẢI HIỆU CHUẨN MAG?
 *   Magnetometer thực tế bị 2 loại sai số:
 *     1. Hard-iron offset  — do vật liệu sắt từ gần chip (loa, ốc vít...)
 *                            → dịch tâm vòng tròn từ trường khỏi gốc tọa độ
 *     2. Soft-iron distortion — do vật liệu dẫn từ → méo vòng tròn thành ellipse
 *
 *   Hiệu chuẩn = xoay board 360° theo CẢ 3 TRỤC để tìm min/max của mỗi trục,
 *   từ đó tính bias (tâm) và scale (bán kính) để "kéo" dữ liệu về vòng tròn đơn vị.
 *
 * CẤU TRÚC FILE:
 *   1. Địa chỉ & thanh ghi AK09916
 *   2. Biến toàn cục (dữ liệu mag + hệ số hiệu chuẩn)
 *   3. Prototype các hàm
 * =============================================================================
 */

#ifndef MAG_AK09916_H
#define MAG_AK09916_H

#include "icm20948.h"

/* =============================================================================
 * PHẦN 1: ĐỊA CHỈ I2C & THANH GHI CỦA AK09916
 * -----------------------------------------------------------------------------
 * AK09916 có địa chỉ I2C cố định 0x0C (7-bit).
 * Các thanh ghi quan trọng được liệt kê dưới đây (theo datasheet AK09916).
 * =============================================================================
 */

/* Địa chỉ I2C 7-bit của AK09916 trên bus nội bộ ICM-20948.
 * Khi giao tiếp, ICM tự động thêm bit R/W vào bit 7 (0x80 = đọc, 0x00 = ghi). */
#define AK09916_ADDRESS     0x0C

/* --- Thanh ghi trạng thái & điều khiển ---
 * MAG_ST1   : Status 1 — kiểm tra dữ liệu sẵn sàng (bit0 = DRDY)
 * MAG_CTRL2 : Control 2 — chọn chế độ đo (continuous/single) & tần số
 * MAG_CTRL3 : Control 3 — soft reset (bit0 = SRST)
 */
#define MAG_ST1             0x10   /* Trạng thái 1: bit0=DRDY, bit1=DOR */
#define MAG_CTRL2           0x31   /* Điều khiển 2: chọn mode đo */
#define MAG_CTRL3           0x32   /* Điều khiển 3: soft reset */

/* Giá trị thường dùng cho MAG_CTRL2:
 *   0x00 = Power down
 *   0x01 = Single measurement
 *   0x02 = Continuous 10 Hz
 *   0x04 = Continuous 20 Hz
 *   0x08 = Continuous 100 Hz  ← code này dùng
 */

/* =============================================================================
 * PHẦN 2: BIẾN TOÀN CỤC
 * -----------------------------------------------------------------------------
 * Các biến này được cập nhật bởi Mag_SetCalibrated() — gọi từ ICM_Read9Axis().
 * Các module khác (yaw_fusion, debug_vars) đọc giá trị qua "extern".
 * =============================================================================
 */

/* --- Từ trường 3 trục (đơn vị: µT — micro Tesla) ---
 * Đây là giá trị ĐÃ QUA hiệu chuẩn hard-iron (bias) và soft-iron (scale).
 * Khi xoay board 360° quanh trục Z: mx/my dao động đối xứng quanh 0, mz gần như không đổi.
 * Khi board hướng Bắc: mx > 0 (hoặc theo quy ước trục), my ≈ 0. */
extern volatile float mx, my, mz;

/* --- Bias hard-iron (tâm vòng tròn từ trường) ---
 * Được tính trong ICM_CalibrateMag() bằng công thức:
 *   bias = (max + min) / 2
 * Đây là offset cần TRỪ khỏi giá trị thô để dịch tâm về gốc tọa độ.
 * Kỳ vọng: |bias| < 100 µT. Nếu lớn hơn → có nam châm mạnh gần chip. */
extern volatile float mag_x_bias, mag_y_bias, mag_z_bias;

/* --- Scale soft-iron (bán kính ellipse → chuẩn hóa về hình tròn) ---
 * Được tính trong ICM_CalibrateMag() bằng công thức:
 *   scale = avg_delta / delta_trục
 * Trong đó avg_delta là trung bình bán kính 3 trục, delta_trục là bán kính trục đó.
 * Kỳ vọng: 0.5 < scale < 2.0. Nếu scale quá lớn/nhỏ → dữ liệu calib kém. */
extern volatile float mag_x_scale, mag_y_scale, mag_z_scale;

/* =============================================================================
 * PHẦN 3: PROTOTYPE CÁC HÀM
 * =============================================================================
 */

/* -----------------------------------------------------------------------------
 * NHÓM HÀM GIAO TIẾP VỚI AK09916 QUA I2C MASTER CỦA ICM-20948
 * -----------------------------------------------------------------------------
 * Các hàm này KHÔNG giao tiếp trực tiếp với AK09916, mà thông qua chuỗi:
 *   1. Cấu hình ICM-20948 ở bank 3 (I2C Master)
 *   2. Ghi địa chỉ slave + thanh ghi đích vào I2C_SLV0_ADDR / I2C_SLV0_REG
 *   3. Kích hoạt đọc/ghi qua I2C_SLV0_CTRL
 *   4. Chờ ICM thực hiện xong giao dịch I2C nội bộ
 *   5. (Nếu đọc) Lấy dữ liệu từ thanh ghi EXT_SLV_SENS_DATA (0x3B) ở bank 0
 */

/* Ghi 1 byte vào thanh ghi AK09916.
 * Ví dụ: AK09916_WriteReg(MAG_CTRL2, 0x08); // Bật continuous 100Hz */
void AK09916_WriteReg(uint8_t reg, uint8_t value);

/* Đọc 1 byte từ thanh ghi AK09916.
 * Ví dụ: uint8_t ctrl = AK09916_ReadReg(MAG_CTRL2); */
uint8_t AK09916_ReadReg(uint8_t reg);

/* Khởi tạo AK09916:
 *   1. Bật I2C Master trong USER_CTRL
 *   2. Cấu hình I2C_MST_CTRL (clock ~400kHz) và ODR
 *   3. Soft reset AK09916 (MAG_CTRL3 = 0x01)
 *   4. Bật chế độ đo liên tục 100Hz (MAG_CTRL2 = 0x08)
 *   5. Cấu hình ICM tự động đọc 9 byte từ MAG_ST1 mỗi chu kỳ
 * Được gọi tự động bên trong ICM_Init() — KHÔNG cần gọi riêng. */
void AK09916_Init(void);

/* -----------------------------------------------------------------------------
 * HÀM HIỆU CHUẨN MAG (chạy 1 lần lúc khởi động, do người dùng xoay board)
 * -----------------------------------------------------------------------------
 * Quy trình:
 *   1. Trong 20 giây, liên tục đọc dữ liệu mag thô
 *   2. Theo dõi giá trị min/max của mx, my, mz
 *   3. Tính bias = (max+min)/2 cho mỗi trục
 *   4. Tính scale = avg_delta / delta_trục cho mỗi trục
 *   5. Lưu vào các biến mag_x/y/z_bias và mag_x/y/z_scale
 *
 * YÊU CẦU NGƯỜI DÙNG:
 *   - Xoay board 360° theo CẢ 3 TRỤC (X, Y, Z) trong 20s
 *   - Tránh xa nam châm, loa, vật liệu sắt từ
 *   - LED PC13 nháy chậm trong suốt quá trình calib
 *
 * KẾT QUẢ:
 *   - mag_x/y/z_bias và mag_x/y/z_scale được cập nhật
 *   - Từ đó mọi giá trị mag đọc được sẽ tự động được hiệu chuẩn
 * GỌI 1 LẦN trong main() sau ICM_CalibrateGyro(). */
void ICM_CalibrateMag(void);

/* -----------------------------------------------------------------------------
 * HÀM HELPER — ÁP DỤNG HIỆU CHUẨN VÀO DỮ LIỆU THÔ
 * -----------------------------------------------------------------------------
 * Nhận 3 giá trị mag thô (đã được hoán vị trục cho khớp board) và áp dụng
 * công thức hiệu chuẩn để ra giá trị đã hiệu chuẩn, lưu vào mx/my/mz.
 *
 * Công thức:
 *   mx = (ux - mag_x_bias) * mag_x_scale
 *   my = (uy - mag_y_bias) * mag_y_scale
 *   mz = (uz - mag_z_bias) * mag_z_scale
 *
 * Trong đó:
 *   ux, uy, uz = giá trị thô sau hoán vị trục (từ ICM_Read9Axis)
 *   mag_*_bias = offset hard-iron
 *   mag_*_scale = hệ số scale soft-iron
 *
 * Được gọi trong ICM_Read9Axis() sau khi đọc 9 byte mag từ ICM.
 * KHÔNG gọi trực tiếp từ main loop. */
void Mag_SetCalibrated(float ux, float uy, float uz);

#endif /* MAG_AK09916_H */
