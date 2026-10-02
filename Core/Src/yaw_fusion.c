#include "yaw_fusion.h"
#include "icm20948.h"
#include "mag_ak09916.h"
#include "Madgwick_filter.h"
#include <math.h>

#define GYRO_DPS_TO_RAD   0.01745329252f
#define RAD_TO_DEG_F      57.29577951308232f

volatile float roll  = 0.0f;
volatile float pitch = 0.0f;
volatile float yaw   = 0.0f;

volatile float yaw_compass = 0.0f;
volatile float yaw_fused   = 0.0f;   /* giữ để debug_vars không phải sửa */
volatile uint8_t yaw_first_run = 0;  /* giữ để debug_vars không phải sửa */
volatile float yaw_quat = 0.0f;

void CalculateYaw(void)
{
    roll = atan2f(q0*q1 + q2*q3, 0.5f - q1*q1 - q2*q2) * RAD_TO_DEG_F;

    float sinp = -2.0f * (q1*q3 - q0*q2);
    if (sinp >=  1.0f) pitch =  90.0f;
    else if (sinp <= -1.0f) pitch = -90.0f;
    else pitch = asinf(sinp) * RAD_TO_DEG_F;

    float yq = atan2f(q1*q2 + q0*q3, 0.5f - q2*q2 - q3*q3) * RAD_TO_DEG_F;

    yq += TOTAL_YAW_OFFSET;
    if (yq < 0.0f)    yq += 360.0f;
    if (yq >= 360.0f) yq -= 360.0f;

    yaw_quat    = yq;
    yaw_compass = yq;
}

void UpdateYaw(float dt)
{
    float gx_r = gx * GYRO_DPS_TO_RAD;
    float gy_r = gy * GYRO_DPS_TO_RAD;
    float gz_r = gz * GYRO_DPS_TO_RAD;

    MadgwickAHRSupdate(gx_r, gy_r, gz_r, ax, ay, az, mx, my, mz, dt);

    CalculateYaw();

    yaw_fused = yaw_compass;
    yaw       = yaw_compass;
}
