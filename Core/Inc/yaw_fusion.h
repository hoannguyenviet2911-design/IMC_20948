/* =============================================================================
 * yaw_fusion.h
 * -----------------------------------------------------------------------------
 * MỤC ĐÍCH:
 *   Khai báo module fusion góc YAW — "trái tim" của hệ thống định hướng.
 *   Module này kết hợp (fusion) dữ liệu từ 2 nguồn:
 *     1. MAGNETOMETER (la bàn) — cho yaw tuyệt đối, không drift, nhưng nhiễu
 *     2. GYROSCOPE              — cho yaw tương đối, mượt, nhưng bị drift
 *   → Kết quả: yaw vừa mượt vừa chính xác, không drift theo thời gian.
 *
 * TẠI SAO CẦN FUSION?
 *   - Chỉ dùng MAG:  yaw nhảy lung tung khi có nhiễu từ trường (động cơ, loa)
 *   - Chỉ dùng GYRO: yaw trôi dần (drift) do tích lũy sai số bias theo thời gian
 *   → Fusion kết hợp ưu điểm của cả 2: gyro làm mượt, mag kéo về đúng hướng.
 *
 * THUẬT TOÁN (Complementary Filter):
 *   yaw_fused = yaw_fused - gz*dt + (1-α)*(yaw_compass - yaw_fused)
 *   Trong đó:
 *     - gz*dt                 : tích hợp gyro (dự đoán yaw mới)
 *     - (1-α)*(yaw_mag - yaw) : kéo yaw về hướng mag (α = 0.98 → kéo 2%)
 *   α càng lớn → tin gyro càng nhiều (mượt nhưng chậm bám mag)
 *   α càng nhỏ → tin mag càng nhiều (bám mag nhanh nhưng nhiễu)
 *
 * CÁC KHÁI NIỆM GÓC:
 *   - Roll  (φ): góc nghiêng quanh trục X (trái/phải) — dùng bù nghiêng
 *   - Pitch (θ): góc nghiêng quanh trục Y (trước/sau) — dùng bù nghiêng
 *   - Yaw   (ψ): góc quay quanh trục Z (ngang) — góc cần xác định
 *
 *   Quy ước góc: 0° = Bắc, 90° = Đông, 180° = Nam, 270° = Tây
 *   Phạm vi: [0°, 360°) — đã chuẩn hóa (wrap-around)
 *
 * CẤU TRÚC FILE:
 *   1. Biến toàn cục (góc + các phiên bản yaw trung gian)
 *   2. Prototype các hàm
 * =============================================================================
 */

#ifndef YAW_FUSION_H
#define YAW_FUSION_H

#include "main.h"

/* =============================================================================
 * PHẦN 1: BIẾN TOÀN CỤC
 * -----------------------------------------------------------------------------
 * Các biến này được cập nhật bởi CalculateYaw() và UpdateYaw().
 * Các module khác (led_direction, debug_vars) đọc giá trị qua "extern".
 * =============================================================================
 */

/* --- Góc nghiêng (đơn vị: độ) ---
 * Được tính từ gia tốc kế trong CalculateYaw().
 *   roll  = atan2(ay, az)              — nghiêng trái/phải
 *   pitch = atan2(-ax, √(ay² + az²))   — nghiêng trước/sau
 * Khi board nằm phẳng: cả 2 ≈ 0.00°.
 * Được dùng để bù nghiêng cho mag (tilt compensation). */
extern volatile float roll;
extern volatile float pitch;

/* --- Góc YAW CUỐI CÙNG (đơn vị: độ, 0-360) ---
 * Đây là giá trị OUTPUT CHÍNH của toàn bộ hệ thống.
 * Sau khi trải qua: calculate → fusion → stationary hold.
 * Được dùng để:
 *   - Điều khiển LED báo hướng (Bắc/Đông/Nam/Tây) trong LED_Update_Direction()
 *   - Hiển thị lên debug (dbg_yaw)
 *
 * ĐẶC ĐIỂM:
 *   - Khi board đứng yên (is_stationary=1) → giữ nguyên, không đổi
 *   - Khi board chuyển động → cập nhật theo fusion gyro + mag */
extern volatile float yaw;

/* --- Góc YAW TÍNH RIÊNG TỪ MAG (đơn vị: độ, 0-360) ---
 * Đây là giá trị "tham chiếu tuyệt đối" — luôn phản ánh hướng thật so với Bắc từ.
 * Được tính trong CalculateYaw() mỗi chu kỳ, KHÔNG qua fusion, KHÔNG qua hold.
 *
 * ĐẶC ĐIỂM:
 *   - Chính xác về hướng tuyệt đối (không drift theo thời gian)
 *   - Có thể NHẢY khi có nhiễu từ trường (động cơ, loa, nam châm)
 *   - Dùng làm "đích" để fusion kéo yaw về
 *
 * DÙNG ĐỂ DEBUG:
 *   So sánh dbg_yaw vs dbg_yaw_comp:
 *     - Nếu lệch nhau nhiều (> 10°) → fusion đang bám mag chậm, hoặc mag bị nhiễu
 *     - Nếu gần nhau (< 5°) → fusion hoạt động tốt */
extern volatile float yaw_compass;

/* --- Góc YAW SAU FUSION NHƯNG TRƯỚC STATIONARY HOLD (đơn vị: độ, 0-360) ---
 * Đây là giá trị trung gian trong quá trình fusion:
 *   yaw_compass → [fusion với gyro] → yaw_fused → [hold nếu đứng yên] → yaw
 *
 * ĐẶC ĐIỂM:
 *   - Khi board chuyển động: yaw_fused = yaw (giống nhau)
 *   - Khi board đứng yên: yaw_fused vẫn có thể drift nhẹ (do gyro bias)
 *                         nhưng yaw thì bị "đóng băng" ở giá trị cuối cùng
 *
 * DÙNG ĐỂ DEBUG:
 *   Khi đứng yên:
 *     - dbg_yaw_fused thay đổi → chứng tỏ Stationary Hold đang hoạt động
 *     - dbg_yaw không đổi → đúng (đã bị hold)
 *   Nếu cả 2 đều không đổi → Stationary Hold không cần thiết (đã ổn định)
 *   Nếu cả 2 đều thay đổi → is_stationary = 0 (board không được nhận là đứng yên) */
extern volatile float yaw_fused;

/* --- Cờ "lần chạy đầu tiên" (0 hoặc 1) ---
 *   1 = Lần đầu UpdateYaw() được gọi → CHƯA có giá trị yaw trước đó
 *   0 = Đã chạy qua lần đầu → có thể bắt đầu fusion bình thường
 *
 * TẠI SAO CẦN?
 *   Ở lần chạy đầu tiên, yaw_fused = 0 (khởi tạo). Nếu fusion bình thường,
 *   yaw sẽ bắt đầu từ 0 và từ từ kéo về yaw_compass (có thể mất vài giây).
 *   → Khởi tạo trực tiếp yaw_fused = yaw_compass ngay lần đầu để yaw đúng ngay.
 *
 * Trong UpdateYaw():
 *   if (yaw_first_run) {
 *       yaw_fused = yaw_compass;   // ← Khởi tạo trực tiếp
 *       yaw_first_run = 0;
 *       yaw = yaw_fused;
 *       return;                     // ← Bỏ qua fusion lần này
 *   } */
extern volatile uint8_t yaw_first_run;

/* =============================================================================
 * PHẦN 2: PROTOTYPE CÁC HÀM
 * =============================================================================
 */

/* -----------------------------------------------------------------------------
 * Tính góc roll, pitch (từ accel) và yaw_compass (từ mag).
 *
 * CÔNG THỨC:
 *   roll_rad  = atan2(ay, az)
 *   pitch_rad = atan2(-ax, √(ay² + az²))
 *   roll      = roll_rad * 180/π
 *   pitch     = pitch_rad * 180/π
 *
 *   BÙ NGHIÊNG (Tilt Compensation) — RẤT QUAN TRỌNG:
 *   Mag đo từ trường trong HỆ TRỤC CỦA BOARD (gắn liền với chip).
 *   Nhưng để tính yaw (góc so với Bắc), ta cần từ trường trong HỆ TRỤC NGANG
 *   (vuông góc với trọng lực). Nếu board nghiêng, phải "xoay" vector mag về
 *   hệ ngang bằng ma trận xoay dựa trên roll/pitch.
 *
 *   mag_x_horiz = mx*cos_pitch + my*sin_roll*sin_pitch + mz*cos_roll*sin_pitch
 *   mag_y_horiz = my*cos_roll  - mz*sin_roll
 *
 *   yaw_compass = atan2(mag_y_horiz, mag_x_horiz) * 180/π + OFFSET
 *
 * ĐẦU VÀO:  ax/ay/az, mx/my/mz (đã đọc từ ICM_Read9Axis)
 * ĐẦU RA:   roll, pitch, yaw_compass (biến toàn cục)
 *
 * CHÚ Ý:
 *   - Không có hiệu chuẩn thêm ở đây — mag đã được hiệu chuẩn trong Mag_SetCalibrated()
 *   - Offset TOTAL_YAW_OFFSET (declination + mounting) được cộng vào cuối
 *
 * Được gọi tự động bên trong UpdateYaw() — KHÔNG cần gọi riêng. */
void CalculateYaw(void);

/* -----------------------------------------------------------------------------
 * Cập nhật yaw bằng thuật toán Complementary Filter.
 *
 * THUẬT TOÁN:
 *   Bước 1: Gọi CalculateYaw() để có yaw_compass mới nhất
 *
 *   Bước 2: Nếu là lần chạy đầu (yaw_first_run = 1):
 *             yaw_fused = yaw_compass     ← Khởi tạo trực tiếp
 *             yaw_first_run = 0
 *             yaw = yaw_fused
 *             return                       ← Thoát sớm
 *
 *   Bước 3: Nếu đang đứng yên (is_stationary = 1):
 *             yaw = yaw_fused              ← ĐÓNG BĂNG, không tích hợp gyro
 *             return                       ← Thoát sớm
 *           (Đây là cơ chế "Stationary Hold" chống drift)
 *
 *   Bước 4: Fusion bình thường (đang chuyển động):
 *             yaw_fused -= gz * dt                        ← Tích hợp gyro
 *             diff = yaw_compass - yaw_fused              ← Sai lệch
 *             chuẩn hóa diff về [-180°, +180°]            ← Tránh nhảy 359→0
 *             yaw_fused += (1 - α) * diff                 ← Kéo về mag (α = 0.98)
 *             chuẩn hóa yaw_fused về [0, 360)
 *             yaw = yaw_fused
 *
 * THAM SỐ:
 *   dt — khoảng thời gian giữa 2 lần gọi (đơn vị: giây).
 *        Trong main loop, dt ≈ 0.02s (20ms).
 *        Nếu dt > 0.05s → clamp về 0.05s để tránh sai số khi debug/delay.
 *
 * TẠI SAO CẦN CHUẨN HÓA DIFF VỀ [-180, +180]?
 *   Ví dụ: yaw_compass = 5°, yaw_fused = 355°.
 *   diff thô = 5 - 355 = -350° → nếu cộng trực tiếp, yaw_fused nhảy về -345° (sai).
 *   Sau chuẩn hóa: -350 + 360 = +10° → đúng (yaw cần quay +10° để đến 5°).
 *   → Tránh hiện tượng "quay ngược vòng" khi qua mốc 0°/360°.
 *
 * Được gọi trong main loop mỗi 20ms, NGAY SAU UpdateStationaryState(). */
void UpdateYaw(float dt);

#endif /* YAW_FUSION_H */
