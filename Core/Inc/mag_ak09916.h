#ifndef MAG_AK09916_H
#define MAG_AK09916_H

#include "main.h"

/* ===== GYRO BIAS CỐ ĐỊNH =====
 * Đo 1 lần bằng code đo bias, sau đó điền giá trị thực tế.
 * Giá trị dưới đây là trung bình 3 lần đo của module ICM-20948 hiện tại.
 */
#define GYRO_BIAS_X   ( 1.6189f)
#define GYRO_BIAS_Y   ( 1.1058f)
#define GYRO_BIAS_Z   ( 0.6862f)

/* ===== Global IMU + MAG ===== */
extern volatile float ax, ay, az;
extern volatile float gx, gy, gz;
extern volatile float gx_bias, gy_bias, gz_bias;

extern volatile float mx, my, mz;

/* ===== Mag calibration ===== */
extern volatile float mag_x_bias, mag_y_bias, mag_z_bias;
extern volatile float mag_x_scale, mag_y_scale, mag_z_scale;

/* ===== API ===== */
void Mag_SetCalibrated(float ux, float uy, float uz);
void ICM_CalibrateMag(void);

#endif /* MAG_AK09916_H */
