/* =============================================================================
 * debug_vars.h
 * -----------------------------------------------------------------------------
 * MỤC ĐÍCH:
 *   Khai báo các biến DEBUG toàn cục để theo dõi giá trị cảm biến / thuật toán
 *   trực tiếp trong STM32CubeIDE qua tính năng "Live Expressions".
 *
 * TẠI SAO CẦN FILE NÀY?
 *   - Các biến gốc (ax, gx, mx, yaw, ...) là số thực float 32-bit với nhiều
 *     chữ số thập phân (ví dụ 0.123456). Nhìn trong Live Expressions rất khó
 *     đọc và khó so sánh.
 *   - File này tạo ra các biến "bản sao" đã được CẮT BỚT 2 CHỮ SỐ THẬP PHÂN
 *     (không làm tròn) thông qua macro TRUNC2() — giúp hiển thị gọn, dễ quan sát.
 *   - Các biến dbg_* CHỈ ĐỂ XEM, KHÔNG dùng để tính toán (tránh sai số tích lũy).
 *
 * =============================================================================
 */

#ifndef DEBUG_VARS_H
#define DEBUG_VARS_H
#include "main.h"

/* =============================================================================
 * NHÓM 1: DỮ LIỆU CẢM BIẾN THÔ (đã qua hiệu chuẩn bias/scale)
 * -----------------------------------------------------------------------------
 * Đây là giá trị đọc từ ICM-20948 sau khi trừ bias gyro và hiệu chuẩn mag.
 * Đơn vị:
 *   - Gia tốc (ax, ay, az): đơn vị "g" (1g = 9.81 m/s²)
 *   - Gyro    (gx, gy, gz): đơn vị "dps" (degrees per second)
 *   - Mag     (mx, my, mz): đơn vị µT (micro Tesla) — đã qua hard/soft-iron calib
 * =============================================================================
 */

/* Gia tốc kế 3 trục — dùng để tính roll, pitch và phát hiện đứng yên.
 * Khi board nằm phẳng, kỳ vọng: ax≈0, ay≈0, az≈+1.00 (hoặc -1.00 tùy chiều) */
extern volatile float dbg_ax, dbg_ay, dbg_az;

/* Gyro 3 trục — dùng để tích hợp yaw (trục Z) và phát hiện chuyển động.
 * Khi board đứng yên, cả 3 giá trị phải gần 0.00 */
extern volatile float dbg_gx, dbg_gy, dbg_gz;

/* Từ trường kế 3 trục — dùng để tính yaw tuyệt đối (la bàn).
 * Khi xoay board 360°, mx/my dao động quanh 0, mz gần như không đổi */
extern volatile float dbg_mx, dbg_my, dbg_mz;

/* =============================================================================
 * NHÓM 2: BIAS GYRO (offset tĩnh của gyro sau khi calib)
 * -----------------------------------------------------------------------------
 * Được tính 1 lần lúc khởi động bởi ICM_CalibrateGyro() — lấy trung bình 500 mẫu
 * khi board đứng yên. Sau đó mọi giá trị gx/gy/gz đọc được đều bị trừ đi bias
 * này để loại bỏ sai số phần cứng (zero-rate offset).
 *
 * Kỳ vọng: |dbg_gx_bias|, |dbg_gy_bias|, |dbg_gz_bias| < 2.0 dps
 * Nếu lớn hơn → gyro bị lỗi hoặc calib sai (board bị rung lúc calib).
 * =============================================================================
 */
extern volatile float dbg_gx_bias, dbg_gy_bias, dbg_gz_bias;

/* =============================================================================
 * NHÓM 3: GÓC YAW (3 phiên bản khác nhau để so sánh)
 * -----------------------------------------------------------------------------
 * Yaw là góc quay quanh trục Z (trục thẳng đứng), đơn vị ĐỘ (0° → 360°).
 * Có 3 biến yaw song song để debug quá trình fusion:
 * =============================================================================
 */

/* dbg_yaw — góc yaw CUỐI CÙNG được dùng để điều khiển LED hướng.
 * Đây là giá trị "output" chính của thuật toán. Nếu đứng yên, giá trị này
 * được giữ nguyên (Stationary Hold) để tránh drift. */
extern volatile float dbg_yaw;

/* dbg_yaw_comp — góc yaw TÍNH RIÊNG TỪ MAG (compass), không qua fusion.
 * Đây là giá trị "tham chiếu tuyệt đối" — luôn đúng hướng Bắc thật (sau khi
 * đã bù offset). Tuy nhiên giá trị này có thể NHẢY khi có nhiễu từ trường.
 * Dùng để so sánh với dbg_yaw để biết fusion đang lệch bao nhiêu. */
extern volatile float dbg_yaw_comp;

/* dbg_yaw_fused — góc yaw SAU KHI FUSION gyro + mag nhưng TRƯỚC KHI áp dụng
 * Stationary Hold. Trong điều kiện bình thường (đang chuyển động), giá trị này
 * gần bằng dbg_yaw. Khi đứng yên, dbg_yaw_fused vẫn có thể drift nhẹ (do gyro
 * tích lũy) nhưng dbg_yaw thì bị "đóng băng".
 * Dùng để debug xem Stationary Hold có hoạt động đúng không. */
extern volatile float dbg_yaw_fused;

/* =============================================================================
 * NHÓM 4: GÓC NGHIÊNG (roll & pitch) — tính từ gia tốc kế
 * -----------------------------------------------------------------------------
 * Đơn vị: ĐỘ (degrees).
 *   - Roll  = góc nghiêng quanh trục X (trái/phải) — ký hiệu φ (phi)
 *   - Pitch = góc nghiêng quanh trục Y (trước/sau) — ký hiệu θ (theta)
 *
 * 2 góc này được dùng để BÙ NGHIÊNG cho mag (tilt compensation), giúp yaw
 * chính xác kể cả khi board không nằm phẳng.
 *
 * Khi board nằm phẳng: cả 2 phải ≈ 0.00
 * =============================================================================
 */
extern volatile float dbg_roll, dbg_pitch;

/* =============================================================================
 * NHÓM 5: ĐỘ LỚN VECTOR (dùng để phát hiện trạng thái đứng yên)
 * -----------------------------------------------------------------------------
 * Đây là "độ dài" của vector cảm biến, không phải giá trị từng trục.
 * =============================================================================
 */

/* dbg_gyro_mag — độ lớn vector gyro: sqrt(gx² + gy² + gz²)
 * Đơn vị: dps. Khi board đứng yên: gần 0.00 (thường < 1.5 dps)
 * Dùng để phát hiện chuyển động quay. */
extern volatile float dbg_gyro_mag;

/* dbg_accel_mag — độ lớn vector gia tốc: sqrt(ax² + ay² + az²)
 * Đơn vị: g. Khi board đứng yên: gần 1.00 (do trọng trường Trái Đất)
 * Nếu giá trị này KHÁC 1.00 nhiều → board đang tăng tốc (rơi, rung, di chuyển).
 * Kết hợp với dbg_gyro_mag để xác định trạng thái "đứng yên" (is_stationary). */
extern volatile float dbg_accel_mag;

/* =============================================================================
 * HÀM CẬP NHẬT
 * -----------------------------------------------------------------------------
 * UpdateDebugVariables() được gọi trong vòng lặp chính mỗi 20ms.
 * Nhiệm vụ: copy giá trị từ biến gốc (ax, gx, ...) sang biến dbg_* và
 *           CẮT 2 CHỮ SỐ THẬP PHÂN bằng macro TRUNC2().
 *
 * LƯU Ý QUAN TRỌNG:
 *   Các biến dbg_* CHỈ để hiển thị. KHÔNG được dùng trong tính toán
 *   (vì đã bị cắt bớt độ chính xác → sẽ gây sai số tích lũy).
 * =============================================================================
 */
void UpdateDebugVariables(void);

#endif /* DEBUG_VARS_H */
