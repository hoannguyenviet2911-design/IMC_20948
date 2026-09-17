/* =============================================================================
 * stationary.h
 * -----------------------------------------------------------------------------
 * MỤC ĐÍCH:
 *   Phát hiện trạng thái "đứng yên" (stationary) của board dựa trên dữ liệu
 *   từ gia tốc kế và gyro. Kết quả được dùng trong thuật toán fusion yaw để
 *   kích hoạt chế độ "Stationary Hold" — đóng băng góc yaw khi board không
 *   chuyển động, tránh hiện tượng drift do tích hợp gyro.
 *
 * TẠI SAO CẦN PHÁT HIỆN ĐỨNG YÊN?
 *   Thuật toán fusion yaw hoạt động như sau:
 *     yaw_fused = yaw_fused - gz*dt + alpha*(yaw_compass - yaw_fused)
 *   Trong đó phần "- gz*dt" là tích hợp gyro. Khi board đứng yên, gz không
 *   hoàn toàn bằng 0 (do nhiễu, nhiệt độ, bias chưa bù hết) → sau vài phút,
 *   yaw sẽ bị trôi (drift) dù board không quay.
 *
 *   Giải pháp: khi phát hiện đứng yên → KHÔNG tích hợp gyro nữa, giữ nguyên
 *   yaw hiện tại. Khi board chuyển động trở lại → tích hợp gyro bình thường.
 *
 * NGUYÊN LÝ PHÁT HIỆN:
 *   Board được coi là đứng yên khi THỎA MÃN ĐỒNG THỜI 2 điều kiện:
 *     1. Gia tốc kế chỉ đo trọng trường (không có gia tốc chuyển động)
 *        → |accel_mag - 1g| < ngưỡng
 *     2. Gyro không đo vận tốc góc (không có chuyển động quay)
 *        → gyro_mag < ngưỡng
 *
 *   Điều kiện phải đúng LIÊN TỤC trong 300ms (15 mẫu × 20ms) mới xác nhận
 *   đứng yên — tránh nhầm với các rung lắc tức thời.
 *
 * CẤU TRÚC FILE:
 *   1. Ngưỡng cấu hình (threshold)
 *   2. Biến toàn cục (trạng thái + giá trị debug)
 *   3. Prototype hàm
 * =============================================================================
 */

#ifndef STATIONARY_H
#define STATIONARY_H

#include "main.h"     /* Cần cho các macro chung và HAL */
#include <stdint.h>   /* Cần cho uint8_t, uint16_t */

/* =============================================================================
 * PHẦN 1: NGƯỠNG CẤU HÌNH
 * -----------------------------------------------------------------------------
 * Các hằng số này quyết định độ nhạy của việc phát hiện đứng yên.
 * Chỉnh nhỏ hơn → nhạy hơn (dễ nhận đứng yên) nhưng dễ nhầm khi có rung nhẹ.
 * Chỉnh lớn hơn → khắt khe hơn (chỉ nhận khi thật sự yên).
 * =============================================================================
 */

/* Ngưỡng gyro (đơn vị: dps — degrees per second).
 * Nếu độ lớn vector gyro < 1.5 dps → coi như không quay.
 * Khi board đứng yên hoàn toàn, gyro_mag thường < 0.5 dps sau calib tốt.
 * Ngưỡng 1.5 dps là giá trị cân bằng giữa độ nhạy và chống nhiễu. */
#define STATIONARY_GYRO_THRESHOLD   1.5f

/* Dung sai gia tốc (đơn vị: g).
 * Khi board đứng yên, accel_mag phải ≈ 1.0g (chỉ có trọng trường).
 * Nếu |accel_mag - 1.0| > 0.08 → board đang tăng tốc (rơi, rung, di chuyển).
 * 0.08g ≈ 0.78 m/s² — đủ nhạy để phát hiện di chuyển nhẹ nhưng không quá
 * khắt khe với nhiễu cảm biến. */
#define STATIONARY_ACCEL_TOLERANCE  0.08f

/* Số mẫu liên tiếp cần thỏa mãn để xác nhận đứng yên.
 * Mỗi mẫu cách nhau 20ms (chu kỳ main loop) → 15 mẫu = 300ms.
 * Cần 300ms để tránh nhầm với các rung lắc tức thời (gõ bàn, va chạm nhẹ).
 * Nếu ngưỡng này quá nhỏ → dễ nhầm; quá lớn → phản hồi chậm khi board dừng. */
#define STATIONARY_COUNT_REQUIRED   15

/* =============================================================================
 * PHẦN 2: BIẾN TOÀN CỤC
 * -----------------------------------------------------------------------------
 * Các biến này được cập nhật bởi UpdateStationaryState() mỗi 20ms.
 * Các module khác (yaw_fusion, debug_vars) đọc giá trị qua "extern".
 * =============================================================================
 */

/* --- Cờ trạng thái đứng yên (0 hoặc 1) ---
 *   0 = board đang chuyển động (hoặc chưa đủ 300ms đứng yên)
 *   1 = board đã đứng yên liên tục ≥ 300ms
 *
 * Được dùng trong UpdateYaw() để quyết định có tích hợp gyro hay không.
 * Khi = 1 → yaw được giữ nguyên, không cộng thêm gz*dt. */
extern volatile uint8_t is_stationary;

/* --- Bộ đếm số mẫu liên tiếp thỏa mãn điều kiện đứng yên ---
 * Tăng lên 1 mỗi 20ms nếu điều kiện đúng, reset về 0 ngay khi điều kiện sai.
 * Khi đạt STATIONARY_COUNT_REQUIRED (15) → is_stationary chuyển sang 1.
 * Dùng để debug: nếu giá trị cứ dao động quanh 5-10 mà không đạt 15 → board
 * bị rung nhẹ hoặc ngưỡng quá khắt khe. */
extern volatile uint16_t stationary_counter;

/* --- Độ lớn vector gyro (đơn vị: dps) ---
 * Được tính bằng: sqrt(gx² + gy² + gz²)
 * Đây là "tốc độ quay tổng hợp" của board theo mọi trục.
 * Khi đứng yên: < STATIONARY_GYRO_THRESHOLD (1.5 dps)
 * Khi quay: tăng vọt (có thể lên hàng trăm dps nếu quay nhanh). */
extern volatile float gyro_magnitude;

/* --- Độ lớn vector gia tốc (đơn vị: g) ---
 * Được tính bằng: sqrt(ax² + ay² + az²)
 * Khi đứng yên: ≈ 1.00g (chỉ có trọng trường Trái Đất)
 * Khi rơi tự do: ≈ 0g
 * Khi tăng tốc mạnh: > 1g (ví dụ lắc mạnh có thể lên 2-3g).
 * Điều kiện đứng yên: |accel_magnitude - 1.0| < STATIONARY_ACCEL_TOLERANCE */
extern volatile float accel_magnitude;

/* =============================================================================
 * PHẦN 3: PROTOTYPE HÀM
 * =============================================================================
 */

/* -----------------------------------------------------------------------------
 * Cập nhật trạng thái đứng yên dựa trên dữ liệu cảm biến mới nhất.
 *
 * Thuật toán:
 *   1. Tính accel_magnitude và gyro_magnitude từ ax/ay/az, gx/gy/gz
 *      (các biến này đã được cập nhật bởi ICM_Read9Axis trước đó)
 *   2. Kiểm tra điều kiện:
 *        cond = (|accel_magnitude - 1.0| < 0.08) AND (gyro_magnitude < 1.5)
 *   3. Nếu cond đúng:
 *        - Tăng stationary_counter (tối đa 15)
 *        - Nếu counter >= 15 → is_stationary = 1
 *      Nếu cond sai:
 *        - Reset counter = 0
 *        - is_stationary = 0 (thoát trạng thái đứng yên NGAY LẬP TỨC)
 *
 * LƯU Ý QUAN TRỌNG:
 *   - Thời gian VÀO trạng thái đứng yên: 300ms (cần 15 mẫu liên tiếp)
 *   - Thời gian THOÁT trạng thái đứng yên: 0ms (thoát ngay khi có chuyển động)
 *   → Bất đối xứng có chủ đích: vào chậm (chống nhiễu), ra nhanh (phản hồi tức thời).
 *
 * Được gọi trong main loop mỗi 20ms, NGAY SAU ICM_Read9Axis() và TRƯỚC
 * UpdateYaw() để đảm bảo yaw fusion dùng đúng trạng thái mới nhất.
 *
 * Ví dụ sử dụng trong main.c:
 *   while (1) {
 *       if (elapsed >= 20) {
 *           ICM_Read9Axis();              // Đọc cảm biến
 *           UpdateStationaryState();      // ← Cập nhật trạng thái
 *           UpdateYaw(dt);                // Fusion yaw (dùng is_stationary)
 *           UpdateDebugVariables();       // Cập nhật biến debug
 *           LED_Update_Direction();       // Cập nhật LED
 *       }
 *   } */
void UpdateStationaryState(void);

#endif /* STATIONARY_H */
