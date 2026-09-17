/* =============================================================================
 * stationary.c
 * -----------------------------------------------------------------------------
 * MỤC ĐÍCH:
 *   Triển khai module phát hiện trạng thái "đứng yên" của board.
 *   Kết quả (is_stationary) được dùng trong yaw_fusion để đóng băng yaw
 *   khi board không chuyển động — tránh hiện tượng drift do tích hợp gyro.
 *
 * TẠI SAO CẦN PHÁT HIỆN ĐỨNG YÊN?
 *   Thuật toán fusion yaw có phần tích hợp gyro: yaw -= gz*dt.
 *   Khi board đứng yên, gz không hoàn toàn = 0 (do nhiễu nhiệt, bias chưa bù
 *   hết, sai số ADC). Sau vài phút, yaw sẽ bị "trôi" (drift) dù board không quay.
 *
 *   Giải pháp: khi phát hiện đứng yên → KHÔNG tích hợp gyro → yaw giữ nguyên.
 *              khi chuyển động trở lại → tích hợp gyro bình thường.
 *
 * NGUYÊN LÝ PHÁT HIỆN:
 *   Board được coi là đứng yên khi THỎA MÃN ĐỒNG THỜI 2 điều kiện:
 *     1. Gia tốc ≈ 1g (chỉ có trọng trường, không có gia tốc chuyển động)
 *     2. Gyro ≈ 0    (không có vận tốc góc, không quay)
 *
 *   Điều kiện phải đúng LIÊN TỤC trong 300ms (15 mẫu × 20ms) mới xác nhận.
 *
 * CẤU TRÚC FILE:
 *   1. Include & định nghĩa biến toàn cục
 *   2. Hàm UpdateStationaryState() — cập nhật trạng thái mỗi 20ms
 * =============================================================================
 */

/* =============================================================================
 * PHẦN 1: INCLUDE & ĐỊNH NGHĨA BIẾN TOÀN CỤC
 * =============================================================================
 */

#include "stationary.h"    /* Header của chính module: macro ngưỡng + prototype */
#include "icm20948.h"      /* Cần ax/ay/az, gx/gy/gz từ cảm biến */
#include <math.h>          /* Cần sqrtf() và fabsf() */

/* --- CỜ TRẠNG THÁI ĐỨNG YÊN ---
 *   0 = board đang chuyển động (hoặc chưa đủ 300ms đứng yên)
 *   1 = board đã đứng yên liên tục ≥ 300ms
 *
 * Được đọc trong UpdateYaw() để quyết định có tích hợp gyro hay không.
 * Khởi tạo = 0 (mặc định là đang chuyển động). */
volatile uint8_t  is_stationary      = 0;

/* --- BỘ ĐẾM SỐ MẪU LIÊN TIẾP THỎA ĐIỀU KIỆN ĐỨNG YÊN ---
 * Tăng lên 1 mỗi 20ms nếu điều kiện đúng (tối đa = STATIONARY_COUNT_REQUIRED).
 * Reset về 0 ngay khi điều kiện sai (có chuyển động).
 *
 * Dùng để debug:
 *   - Nếu counter cứ dao động quanh 5-10 mà không đạt 15 → board bị rung nhẹ
 *   - Nếu counter luôn = 15 → board thật sự đứng yên
 *   - Nếu counter luôn = 0 → board đang chuyển động liên tục
 *
 * Kiểu uint16_t (2 byte) vì counter tối đa = 15, nhưng để dư phòng nếu
 * sau này tăng STATIONARY_COUNT_REQUIRED lên hàng ngàn. */
volatile uint16_t stationary_counter = 0;

/* --- ĐỘ LỚN VECTOR GYRO (đơn vị: dps) ---
 * Được tính bằng: sqrt(gx² + gy² + gz²)
 * Đây là "tốc độ quay tổng hợp" của board theo mọi trục.
 *   - Khi đứng yên: < STATIONARY_GYRO_THRESHOLD (1.5 dps)
 *   - Khi quay tay: 10-50 dps
 *   - Khi lắc mạnh: > 100 dps
 * Dùng để so sánh ngưỡng phát hiện quay. */
volatile float    gyro_magnitude     = 0.0f;

/* --- ĐỘ LỚN VECTOR GIA TỐC (đơn vị: g) ---
 * Được tính bằng: sqrt(ax² + ay² + az²)
 *   - Khi đứng yên: ≈ 1.00g (chỉ có trọng trường Trái Đất)
 *   - Khi rơi tự do: ≈ 0g
 *   - Khi lắc mạnh: 1.5 - 3.0g
 * Điều kiện đứng yên: |accel_magnitude - 1.0| < STATIONARY_ACCEL_TOLERANCE */
volatile float    accel_magnitude    = 0.0f;

/* =============================================================================
 * PHẦN 2: HÀM CẬP NHẬT TRẠNG THÁI ĐỨNG YÊN
 * =============================================================================
 */

/* -----------------------------------------------------------------------------
 * UpdateStationaryState — Cập nhật trạng thái đứng yên dựa trên cảm biến.
 *
 * THUẬT TOÁN (từng bước):
 *
 *   BƯỚC 1: Tính độ lớn vector gia tốc và gyro
 *      accel_magnitude = sqrt(ax² + ay² + az²)
 *      gyro_magnitude  = sqrt(gx² + gy² + gz²)
 *
 *   BƯỚC 2: Kiểm tra điều kiện đứng yên
 *      cond = (|accel_magnitude - 1.0| < 0.08)   ← gia tốc ≈ 1g
 *          && (gyro_magnitude < 1.5)              ← gyro ≈ 0
 *
 *   BƯỚC 3: Cập nhật counter và is_stationary
 *      Nếu cond đúng:
 *         - Tăng counter (tối đa 15)
 *         - Nếu counter >= 15 → is_stationary = 1
 *      Nếu cond sai:
 *         - counter = 0
 *         - is_stationary = 0 (thoát NGAY LẬP TỨC, không cần đếm ngược)
 *
 * ĐẶC ĐIỂM BẤT ĐỐI XỨNG (CỐ Ý):
 *   - VÀO trạng thái đứng yên: CHẬM (300ms)
 *     → Chống nhầm với nhiễu tức thời (gõ bàn, rung nhẹ)
 *   - RA trạng thái đứng yên: NHANH (0ms)
 *     → Ngay khi có chuyển động, thoát ngay để yaw fusion hoạt động bình thường
 *     → Tránh "đóng băng" yaw khi board thực sự đang quay
 *
 * THỜI GIAN THỰC THI: rất nhanh (~2 µs) — chỉ 2 phép sqrtf() + so sánh.
 *
 * ĐƯỢC GỌI TRONG MAIN LOOP:
 *   while (1) {
 *       if (elapsed >= 20) {
 *           ICM_Read9Axis();              // 1. Đọc cảm biến
 *           UpdateStationaryState();      // 2. ← Cập nhật trạng thái
 *           UpdateYaw(dt);                // 3. Fusion yaw (dùng is_stationary)
 *           UpdateDebugVariables();       // 4. Cập nhật biến debug
 *           LED_Update_Direction();       // 5. Cập nhật LED
 *       }
 *   }
 *
 * THỨ TỰ GỌI QUAN TRỌNG:
 *   - Phải gọi SAU ICM_Read9Axis() (để có ax/ay/az, gx/gy/gz mới nhất)
 *   - Phải gọi TRƯỚC UpdateYaw() (để yaw_fusion dùng is_stationary mới nhất)
 * ----------------------------------------------------------------------------- */
void UpdateStationaryState(void)
{
    /* -----------------------------------------------------------------------
     * BƯỚC 1: Tính độ lớn vector gia tốc và gyro
     *
     * Công thức: |v| = sqrt(x² + y² + z²)
     * Đây là "độ dài" của vector 3D, không quan tâm hướng.
     *
     * Ví dụ gia tốc:
     *   Board nằm phẳng: (0, 0, 1) → |v| = sqrt(0+0+1) = 1.00 g
     *   Board rơi tự do: (0, 0, 0) → |v| = 0.00 g
     *   Board lắc mạnh:  (0.5, 0.5, 1.2) → |v| = sqrt(0.25+0.25+1.44) = 1.39 g
     *
     * Sử dụng sqrtf() (float version) thay vì sqrt() (double) để nhanh hơn.
     * ----------------------------------------------------------------------- */
    accel_magnitude = sqrtf(ax*ax + ay*ay + az*az);
    gyro_magnitude  = sqrtf(gx*gx + gy*gy + gz*gz);

    /* -----------------------------------------------------------------------
     * BƯỚC 2: Kiểm tra điều kiện đứng yên
     *
     * ĐIỀU KIỆN 1: Gia tốc ≈ 1g
     *   |accel_magnitude - 1.0| < 0.08
     *   - Khi board yên: accel_magnitude = 1.00 → |1.00-1| = 0 < 0.08 ✅
     *   - Khi board rung: accel_magnitude = 1.20 → |1.20-1| = 0.20 > 0.08 ❌
     *   - Khi board rơi: accel_magnitude = 0.00 → |0.00-1| = 1.00 > 0.08 ❌
     *
     *   fabsf() là hàm giá trị tuyệt đối cho float (fast math).
     *
     * ĐIỀU KIỆN 2: Gyro ≈ 0
     *   gyro_magnitude < 1.5
     *   - Khi board yên: gyro_magnitude ≈ 0.2 → 0.2 < 1.5 ✅
     *   - Khi quay tay: gyro_magnitude ≈ 30 → 30 > 1.5 ❌
     *   - Khi lắc mạnh: gyro_magnitude ≈ 200 → 200 > 1.5 ❌
     *
     * TOÁN TỬ && (AND):
     *   Cả 2 điều kiện phải đúng CÙNG LÚC mới tính là đứng yên.
     *
     * Biến "cond" là uint8_t (0 hoặc 1) — kết quả của biểu thức logic.
     * ----------------------------------------------------------------------- */
    uint8_t cond = (fabsf(accel_magnitude - 1.0f) < STATIONARY_ACCEL_TOLERANCE)
                && (gyro_magnitude < STATIONARY_GYRO_THRESHOLD);

    /* -----------------------------------------------------------------------
     * BƯỚC 3: Cập nhật counter và is_stationary
     * ----------------------------------------------------------------------- */
    if (cond) {
        /* === TRƯỜNG HỢP: ĐỨNG YÊN ===
         * Tăng counter (nhưng không vượt quá STATIONARY_COUNT_REQUIRED).
         * Kiểm tra "if (counter < REQUIRED)" để tránh tràn số khi counter
         * tiếp tục tăng sau khi đã đạt ngưỡng. */
        if (stationary_counter < STATIONARY_COUNT_REQUIRED) stationary_counter++;

        /* Khi counter đạt ngưỡng (15 mẫu = 300ms liên tục) → xác nhận đứng yên.
         * Lưu ý: is_stationary chỉ được set = 1 khi counter ĐÃ ĐẠT ngưỡng.
         * Trong 300ms đầu tiên, is_stationary vẫn = 0 (đang trong giai đoạn "nghi ngờ"). */
        if (stationary_counter >= STATIONARY_COUNT_REQUIRED) is_stationary = 1;
    } else {
        /* === TRƯỜNG HỢP: CÓ CHUYỂN ĐỘNG ===
         * Reset counter về 0 (phải đếm lại từ đầu).
         * Set is_stationary = 0 NGAY LẬP TỨC (không cần đếm ngược).
         *
         * ĐẶC ĐIỂM BẤT ĐỐI XỨNG:
         *   - Vào trạng thái đứng yên: cần 300ms
         *   - Ra trạng thái đứng yên: 0ms (thoát ngay)
         *
         * Lý do: khi board bắt đầu quay, ta muốn yaw fusion hoạt động ngay để
         * bám kịp chuyển động. Nếu trì hoãn thoát, yaw sẽ bị "đóng băng" trong
         * lúc board đang quay → LED báo sai hướng tạm thời. */
        stationary_counter = 0;
        is_stationary = 0;
    }
}
