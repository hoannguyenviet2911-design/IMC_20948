/* =============================================================================
 * yaw_fusion.c
 * -----------------------------------------------------------------------------
 * MỤC ĐÍCH:
 *   Triển khai thuật toán fusion góc YAW — "trái tim" của hệ thống định hướng.
 *   Kết hợp (fusion) dữ liệu từ 2 nguồn:
 *     1. MAGNETOMETER (la bàn) — cho yaw tuyệt đối, không drift, nhưng nhiễu
 *     2. GYROSCOPE              — cho yaw tương đối, mượt, nhưng bị drift
 *   → Kết quả: yaw vừa mượt vừa chính xác, không drift theo thời gian.
 *
 * THUẬT TOÁN (Complementary Filter):
 *   yaw_fused = yaw_fused - gz*dt + (1-α)*(yaw_compass - yaw_fused)
 *   Trong đó:
 *     - gz*dt                 : tích hợp gyro (dự đoán yaw mới)
 *     - (1-α)*(yaw_mag - yaw) : kéo yaw về hướng mag (α = 0.98 → kéo 2%)
 *   α càng lớn → tin gyro càng nhiều (mượt nhưng chậm bám mag)
 *   α càng nhỏ → tin mag càng nhiều (bám mag nhanh nhưng nhiễu)
 *
 * CẤU TRÚC FILE:
 *   1. Include & định nghĩa biến toàn cục
 *   2. Hàm CalculateYaw()  — tính yaw từ mag (có bù nghiêng)
 *   3. Hàm UpdateYaw(dt)   — fusion gyro + mag + stationary hold
 * =============================================================================
 */

/* =============================================================================
 * PHẦN 1: INCLUDE & ĐỊNH NGHĨA BIẾN TOÀN CỤC
 * =============================================================================
 */

#include "yaw_fusion.h"      /* Header của chính module */
#include "icm20948.h"        /* Cần ax/ay/az, gx/gy/gz */
#include "mag_ak09916.h"     /* Cần mx/my/mz */
#include "stationary.h"      /* Cần is_stationary */
#include <math.h>            /* Cần atan2f, sinf, cosf, sqrtf */

/* --- GÓC NGHIÊNG (đơn vị: độ) ---
 * Được tính từ gia tốc kế trong CalculateYaw().
 *   roll  = atan2(ay, az)              — nghiêng trái/phải
 *   pitch = atan2(-ax, √(ay² + az²))   — nghiêng trước/sau
 * Khi board nằm phẳng: cả 2 ≈ 0.00°.
 * Được dùng để bù nghiêng cho mag (tilt compensation). */
volatile float roll  = 0.0f;
volatile float pitch = 0.0f;

/* --- GÓC YAW CUỐI CÙNG (đơn vị: độ, 0-360) ---
 * Đây là giá trị OUTPUT CHÍNH của toàn bộ hệ thống.
 * Sau khi trải qua: calculate → fusion → stationary hold.
 * Được dùng để điều khiển LED báo hướng (Bắc/Đông/Nam/Tây). */
volatile float yaw   = 0.0f;

/* --- GÓC YAW TÍNH RIÊNG TỪ MAG (đơn vị: độ, 0-360) ---
 * Đây là giá trị "tham chiếu tuyệt đối" — luôn phản ánh hướng thật so với Bắc từ.
 * Được tính trong CalculateYaw() mỗi chu kỳ, KHÔNG qua fusion, KHÔNG qua hold.
 *
 * ĐẶC ĐIỂM:
 *   - Chính xác về hướng tuyệt đối (không drift theo thời gian)
 *   - Có thể NHẢY khi có nhiễu từ trường (động cơ, loa, nam châm)
 *   - Dùng làm "đích" để fusion kéo yaw về */
volatile float yaw_compass = 0.0f;

/* --- GÓC YAW SAU FUSION NHƯNG TRƯỚC STATIONARY HOLD (đơn vị: độ, 0-360) ---
 * Đây là giá trị trung gian trong quá trình fusion:
 *   yaw_compass → [fusion với gyro] → yaw_fused → [hold nếu đứng yên] → yaw
 *
 * ĐẶC ĐIỂM:
 *   - Khi board chuyển động: yaw_fused = yaw (giống nhau)
 *   - Khi board đứng yên: yaw_fused vẫn có thể drift nhẹ (do gyro bias)
 *                         nhưng yaw thì bị "đóng băng" ở giá trị cuối cùng */
volatile float yaw_fused   = 0.0f;

/* --- CỜ "LẦN CHẠY ĐẦU TIÊN" (0 hoặc 1) ---
 *   1 = Lần đầu UpdateYaw() được gọi → CHƯA có giá trị yaw trước đó
 *   0 = Đã chạy qua lần đầu → có thể bắt đầu fusion bình thường
 *
 * TẠI SAO CẦN?
 *   Ở lần chạy đầu tiên, yaw_fused = 0 (khởi tạo). Nếu fusion bình thường,
 *   yaw sẽ bắt đầu từ 0 và từ từ kéo về yaw_compass (có thể mất vài giây).
 *   → Khởi tạo trực tiếp yaw_fused = yaw_compass ngay lần đầu để yaw đúng ngay.
 *
 * Khởi tạo = 1 (đánh dấu chưa chạy lần nào). */
volatile uint8_t yaw_first_run = 1;

/* =============================================================================
 * PHẦN 2: TÍNH YAW TỪ MAG + BÙ NGHIÊNG (TILT COMPENSATION)
 * =============================================================================
 */

/* -----------------------------------------------------------------------------
 * CalculateYaw — Tính roll, pitch từ accel và yaw_compass từ mag.
 *
 * ĐẦU VÀO:  ax/ay/az (accel), mx/my/mz (mag đã hiệu chuẩn)
 * ĐẦU RA:   roll, pitch, yaw_compass (biến toàn cục)
 *
 * === BƯỚC 1: TÍNH ROLL VÀ PITCH TỪ ACCEL ===
 *
 * CÔNG THỨC:
 *   roll_rad  = atan2(ay, az)
 *   pitch_rad = atan2(-ax, √(ay² + az²))
 *
 * GIẢI THÍCH:
 *   Khi board nằm phẳng, trọng trường chỉ tác động lên trục Z (az ≈ 1g).
 *   Khi nghiêng board, trọng trường "phân bố" sang các trục X/Y.
 *   Từ tỉ lệ giữa các trục → tính được góc nghiêng.
 *
 *   atan2(y, x) trả về góc trong khoảng [-π, +π] — chính xác hơn atan(y/x)
 *   vì không bị chia cho 0 khi x = 0.
 *
 * === BƯỚC 2: TÍNH SIN/COS CỦA ROLL VÀ PITCH ===
 *
 *   Cần sin/cos của roll_rad và pitch_rad để dùng trong công thức bù nghiêng.
 *   Tính 1 lần, lưu vào biến → tránh gọi lại nhiều lần (tăng tốc).
 *
 * === BƯỚC 3: BÙ NGHIÊNG CHO MAG (TILT COMPENSATION) ===
 *
 * TẠI SAO CẦN BÙ NGHIÊNG?
 *   Mag đo từ trường trong HỆ TRỤC CỦA BOARD (gắn liền với chip).
 *   Nhưng để tính yaw (góc so với Bắc), ta cần từ trường trong
 *   HỆ TRỤC NGANG (vuông góc với trọng lực).
 *   Nếu board nghiêng, phải "xoay" vector mag về hệ ngang bằng ma trận xoay
 *   dựa trên roll/pitch.
 *
 * CÔNG THỨC:
 *   mag_x_horiz = mx*cos_pitch + my*sin_roll*sin_pitch + mz*cos_roll*sin_pitch
 *   mag_y_horiz = my*cos_roll  - mz*sin_roll
 *
 * Đây là kết quả của phép nhân ma trận xoay R_y(pitch) × R_x(roll) với vector mag.
 *
 * === BƯỚC 4: TÍNH YAW TỪ MAG NGANG ===
 *
 *   yaw_raw = atan2(mag_y_horiz, mag_x_horiz) * RAD_TO_DEG + TOTAL_YAW_OFFSET
 *
 *   - atan2(my, mx) cho góc so với trục X của hệ ngang.
 *   - Cộng TOTAL_YAW_OFFSET (declination + mounting) để chuyển sang Bắc thật.
 *   - Trừ 120.0f là OFFSET HIỆU CHỈNH THỰC NGHIỆM (xem phần cảnh báo bên dưới).
 *
 * === BƯỚC 5: CHUẨN HÓA GÓC VỀ [0, 360) ===
 *
 *   Sau khi cộng offset, góc có thể âm hoặc > 360.
 *   Chuẩn hóa để yaw luôn trong [0, 360).
 *
 * ⚠️ CẢNH BÁO VỀ OFFSET:
 *   Code hiện tại có 3 offset cộng dồn:
 *     1. TOTAL_YAW_OFFSET (declination + mounting) — trong macro
 *     2. -120.0f — HARD-CODE trong hàm, không rõ nguồn gốc
 *   Tổng offset có thể lên đến -196° → yaw sẽ lệch.
 *
 *   KHUYẾN NGHỊ: Bỏ -120.0f, chỉ dùng 1 offset duy nhất.
 *   Xem thêm comment ở phần cuối file.
 * ----------------------------------------------------------------------------- */
void CalculateYaw(void)
{
    /* === BƯỚC 1: Tính roll, pitch từ accel (đơn vị radian) === */
    float roll_rad  = atan2f(ay, az);
    float pitch_rad = atan2f(-ax, sqrtf(ay*ay + az*az));

    /* === BƯỚC 2: Chuyển sang độ để lưu vào biến toàn cục === */
    roll  = roll_rad  * RAD_TO_DEG;
    pitch = pitch_rad * RAD_TO_DEG;

    /* === BƯỚC 3: Tính sin/cos (dùng cho công thức bù nghiêng) === */
    float sin_roll  = sinf(roll_rad);
    float cos_roll  = cosf(roll_rad);
    float sin_pitch = sinf(pitch_rad);
    float cos_pitch = cosf(pitch_rad);

    /* === BƯỚC 4: Bù nghiêng — chiếu mag về hệ trục ngang ===
     * Đây là phép nhân ma trận xoay R_y(pitch) × R_x(roll) với vector (mx, my, mz).
     * Kết quả là vector mag trong hệ trục ngang (vuông góc với trọng lực). */
    float mag_x_horiz = mx*cos_pitch + my*sin_roll*sin_pitch + mz*cos_roll*sin_pitch;
    float mag_y_horiz = my*cos_roll  - mz*sin_roll;

    /* === BƯỚC 5: Tính yaw từ mag ngang + áp dụng offset ===
     * atan2(my, mx): góc so với trục X (hướng Đông theo quy ước)
     * Cộng TOTAL_YAW_OFFSET để chuyển sang Bắc thật.
     * Trừ 120.0f: OFFSET HIỆU CHỈNH THỰC NGHIỆM (cần xem lại). */
    float yaw_raw = atan2f(mag_y_horiz, mag_x_horiz) * RAD_TO_DEG + TOTAL_YAW_OFFSET;
    yaw_raw -= 120.0f;

    /* === BƯỚC 6: Chuẩn hóa góc về [0, 360) ===
     * Sau khi cộng offset, yaw_raw có thể âm (nếu offset âm lớn) hoặc > 360.
     * Cộng/trừ 360 để đưa về khoảng [0, 360). */
    if (yaw_raw < 0.0f)    yaw_raw += 360.0f;
    if (yaw_raw >= 360.0f) yaw_raw -= 360.0f;

    /* === LƯU KẾT QUẢ === */
    yaw_compass = yaw_raw;
}

/* =============================================================================
 * PHẦN 3: FUSION — KẾT HỢP GYRO + MAG + STATIONARY HOLD
 * =============================================================================
 */

/* -----------------------------------------------------------------------------
 * UpdateYaw — Cập nhật yaw bằng thuật toán Complementary Filter.
 *
 * ĐẦU VÀO:  dt — khoảng thời gian giữa 2 lần gọi (đơn vị: giây)
 *                Trong main loop, dt ≈ 0.02s (20ms).
 *                Nếu dt > 0.05s → nên clamp về 0.05s (xem main.c).
 * ĐẦU RA:   yaw (biến toàn cục)
 *
 * === LUỒNG XỬ LÝ ===
 *
 *   BƯỚC 1: Gọi CalculateYaw() để có yaw_compass mới nhất
 *
 *   BƯỚC 2: Nếu là lần chạy đầu (yaw_first_run = 1):
 *             yaw_fused = yaw_compass     ← Khởi tạo trực tiếp
 *             yaw_first_run = 0
 *             yaw = yaw_fused
 *             return                       ← Thoát sớm
 *
 *   BƯỚC 3: Nếu đang đứng yên (is_stationary = 1):
 *             yaw = yaw_fused              ← ĐÓNG BĂNG, không tích hợp gyro
 *             return                       ← Thoát sớm
 *
 *   BƯỚC 4: Fusion bình thường (đang chuyển động):
 *             yaw_fused -= gz * dt                        ← Tích hợp gyro
 *             diff = yaw_compass - yaw_fused              ← Sai lệch
 *             chuẩn hóa diff về [-180°, +180°]            ← Tránh nhảy 359→0
 *             yaw_fused += (1 - α) * diff                 ← Kéo về mag
 *             chuẩn hóa yaw_fused về [0, 360)
 *             yaw = yaw_fused
 *
 * === CHI TIẾT TỪNG BƯỚC ===
 * ----------------------------------------------------------------------------- */
void UpdateYaw(float dt)
{
    /* === BƯỚC 1: Tính yaw từ mag (cập nhật yaw_compass) ===
     * Hàm này cũng cập nhật roll, pitch từ accel. */
    CalculateYaw();

    /* === BƯỚC 2: Khởi tạo lần đầu ===
     * Nếu là lần chạy đầu tiên, gán trực tiếp yaw_fused = yaw_compass.
     * Tránh hiện tượng yaw bắt đầu từ 0 và từ từ kéo về giá trị đúng
     * (mất vài giây, LED báo sai hướng trong thời gian đó). */
    if (yaw_first_run) {
        yaw_fused = yaw_compass;
        yaw_first_run = 0;
        yaw = yaw_fused;
        return;   /* Thoát sớm, không fusion lần này */
    }

    /* === BƯỚC 3: STATIONARY HOLD ===
     * Nếu board đang đứng yên (is_stationary = 1):
     *   - Giữ nguyên yaw = yaw_fused (không tích hợp gyro)
     *   - Thoát sớm
     *
     * TẠI SAO CẦN BƯỚC NÀY?
     *   Khi board đứng yên, gz không hoàn toàn = 0 (do nhiễu nhiệt, bias chưa
     *   bù hết). Nếu vẫn tích hợp gyro, yaw sẽ drift dần dù board không quay.
     *   → Đóng băng yaw để tránh drift. */
    if (is_stationary) { yaw = yaw_fused; return; }

    /* === BƯỚC 4: FUSION BÌNH THƯỜNG (đang chuyển động) === */

    /* 4a. Tích hợp gyro:
     *   - gz là vận tốc góc quanh trục Z (dps).
     *   - Nhân với dt (giây) → góc quay trong khoảng dt.
     *   - TRỪ vì: gyro Z dương khi quay NGƯỢC chiều kim đồng hồ (theo quy ước
     *     của chip), nhưng yaw trong hệ tọa độ địa lý dương khi quay CÙNG chiều
     *     kim đồng hồ (Bắc → Đông → Nam → Tây).
     *   - Nếu board quay theo chiều dương của gyro → yaw giảm → cần trừ. */
    yaw_fused -= gz * dt;

    /* 4b. Tính sai lệch giữa yaw_compass (mag) và yaw_fused (fusion).
     *   diff = "cần quay thêm bao nhiêu để khớp mag". */
    float diff = yaw_compass - yaw_fused;

    /* 4c. CHUẨN HÓA DIFF VỀ [-180°, +180°]:
     *   Đây là bước CỰC KỲ QUAN TRỌNG để tránh hiện tượng "quay ngược vòng".
     *
     *   Ví dụ: yaw_compass = 5°, yaw_fused = 355°.
     *   diff thô = 5 - 355 = -350° → nếu cộng trực tiếp, yaw_fused nhảy về -345° (sai).
     *   Sau chuẩn hóa: -350 + 360 = +10° → đúng (yaw cần quay +10° để đến 5°).
     *
     *   Tương tự: yaw_compass = 355°, yaw_fused = 5°.
     *   diff thô = 355 - 5 = +350° → chuẩn hóa: 350 - 360 = -10° → đúng. */
    if (diff >  180.0f) diff -= 360.0f;
    if (diff < -180.0f) diff += 360.0f;

    /* 4d. KÉO YAW VỀ MAG:
     *   yaw_fused += (1 - α) * diff
     *   Với α = YAW_ALPHA = 0.98 → (1-α) = 0.02 → kéo 2% mỗi chu kỳ.
     *
     *   TẠI SAO CHỈ KÉO 2%?
     *   - Nếu kéo nhiều (α nhỏ) → bám mag nhanh nhưng nhiễu theo mag.
     *   - Nếu kéo ít (α lớn) → mượt, nhưng chậm bám khi mag thay đổi.
     *   - α = 0.98 là giá trị cân bằng tốt cho hầu hết ứng dụng. */
    yaw_fused += (1.0f - YAW_ALPHA) * diff;

    /* 4e. CHUẨN HÓA YAW_FUSED VỀ [0, 360):
     *   Sau khi cộng/trừ, yaw_fused có thể âm hoặc > 360. */
    if (yaw_fused < 0.0f)    yaw_fused += 360.0f;
    if (yaw_fused >= 360.0f) yaw_fused -= 360.0f;

    /* 4f. Gán yaw cuối cùng = yaw_fused (khi đang chuyển động). */
    yaw = yaw_fused;
}
