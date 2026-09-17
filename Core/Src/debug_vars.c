/* =============================================================================
 * PHẦN 1:
 * -----------------------------------------------------------------------------
 *
 *   - debug_vars.h  : khai báo biến dbg_* + prototype UpdateDebugVariables
 *   - icm20948.h    : cho ax/ay/az, gx/gy/gz, gx/y/z_bias
 *   - mag_ak09916.h : cho mx/my/mz
 *   - yaw_fusion.h  : cho yaw, yaw_compass, yaw_fused, roll, pitch
 *   - stationary.h  : cho gyro_magnitude, accel_magnitude
 * =============================================================================
 */
#include "debug_vars.h"
#include "icm20948.h"
#include "mag_ak09916.h"
#include "yaw_fusion.h"
#include "stationary.h"

/* =============================================================================
 * PHẦN 2: ĐỊNH NGHĨA (CẤP PHÁT) CÁC BIẾN DEBUG
 * -----------------------------------------------------------------------------
 * Đây là nơi DUY NHẤT các biến dbg_* được cấp phát bộ nhớ.
 * Tất cả khởi tạo = 0 để giá trị ban đầu rõ ràng (không phải giá trị rác).
 *
 * Từ khóa "volatile":
 *   Bắt buộc — vì các biến này được cập nhật liên tục trong main loop và
 *   được đọc từ debugger. Trình biên dịch KHÔNG được cache vào thanh ghi CPU
 *   (nếu cache, debugger đọc sẽ ra giá trị cũ, không real-time).
 *
 * NHÓM 1: CẢM BIẾN THÔ (accel + gyro)
 *   dbg_ax, dbg_ay, dbg_az — gia tốc 3 trục (đơn vị: g)
 *   dbg_gx, dbg_gy, dbg_gz — gyro 3 trục (đơn vị: dps), đã trừ bias
 * =============================================================================
 */
volatile float dbg_ax = 0, dbg_ay = 0, dbg_az = 0;
volatile float dbg_gx = 0, dbg_gy = 0, dbg_gz = 0;

/* NHÓM 2: CẢM BIẾN MAG (đã hiệu chuẩn)
 *   dbg_mx, dbg_my, dbg_mz — từ trường 3 trục (đơn vị: µT)
 *   Khi xoay board 360° quanh trục Z: mx/my dao động quanh 0, mz gần như không đổi */
volatile float dbg_mx = 0, dbg_my = 0, dbg_mz = 0;

/* NHÓM 3: BIAS GYRO (offset tĩnh sau calib)
 *   dbg_gx_bias, dbg_gy_bias, dbg_gz_bias — đơn vị: dps
 *   Kỳ vọng: |bias| < 2.0 dps. Nếu lớn hơn → calib sai (board rung lúc calib). */
volatile float dbg_gx_bias = 0, dbg_gy_bias = 0, dbg_gz_bias = 0;

/* NHÓM 4: CÁC PHIÊN BẢN GÓC YAW (đơn vị: độ, 0-360)
 *   dbg_yaw       — yaw cuối cùng (output chính)
 *   dbg_yaw_comp  — yaw tính từ mag (tham chiếu tuyệt đối)
 *   dbg_yaw_fused — yaw sau fusion nhưng trước hold (giá trị trung gian)
 *   Dùng để so sánh 3 giá trị → biết fusion có hoạt động đúng không. */
volatile float dbg_yaw = 0, dbg_yaw_comp = 0, dbg_yaw_fused = 0;

/* NHÓM 5: GÓC NGHIÊNG (đơn vị: độ)
 *   dbg_roll  — nghiêng quanh trục X (trái/phải)
 *   dbg_pitch — nghiêng quanh trục Y (trước/sau)
 *   Khi board nằm phẳng: cả 2 ≈ 0.00° */
volatile float dbg_roll = 0, dbg_pitch = 0;

/* NHÓM 6: ĐỘ LỚN VECTOR (dùng để phát hiện đứng yên)
 *   dbg_gyro_mag  — √(gx² + gy² + gz²), đơn vị: dps
 *   dbg_accel_mag — √(ax² + ay² + az²), đơn vị: g
 *   Khi board đứng yên: gyro_mag < 1.5, accel_mag ≈ 1.00 */
volatile float dbg_gyro_mag = 0, dbg_accel_mag = 0;

void UpdateDebugVariables(void)
{
    dbg_ax = TRUNC2(ax);   dbg_ay = TRUNC2(ay);   dbg_az = TRUNC2(az);
    dbg_gx = TRUNC2(gx);   dbg_gy = TRUNC2(gy);   dbg_gz = TRUNC2(gz);
    dbg_mx = TRUNC2(mx);   dbg_my = TRUNC2(my);   dbg_mz = TRUNC2(mz);

    dbg_gx_bias = TRUNC2(gx_bias);
    dbg_gy_bias = TRUNC2(gy_bias);
    dbg_gz_bias = TRUNC2(gz_bias);

    dbg_yaw       = TRUNC2(yaw);
    dbg_yaw_comp  = TRUNC2(yaw_compass);
    dbg_yaw_fused = TRUNC2(yaw_fused);

    dbg_roll      = TRUNC2(roll);
    dbg_pitch     = TRUNC2(pitch);


    dbg_gyro_mag  = TRUNC2(gyro_magnitude);
    dbg_accel_mag = TRUNC2(accel_magnitude);
}
