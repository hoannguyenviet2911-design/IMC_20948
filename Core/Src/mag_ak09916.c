#include "mag_ak09916.h"
#include "icm20948.h"

/* ===== Định nghĩa biến toàn cục ===== */
volatile float ax, ay, az;
volatile float gx, gy, gz;

/* ===== GYRO BIAS CỐ ĐỊNH (lấy từ #define trong .h) ===== */
volatile float gx_bias = GYRO_BIAS_X;
volatile float gy_bias = GYRO_BIAS_Y;
volatile float gz_bias = GYRO_BIAS_Z;

volatile float mx, my, mz;

volatile float mag_x_bias = 0.0f, mag_y_bias = 0.0f, mag_z_bias = 0.0f;
volatile float mag_x_scale = 1.0f, mag_y_scale = 1.0f, mag_z_scale = 1.0f;

/* ===== Áp dụng hiệu chuẩn ===== */
void Mag_SetCalibrated(float ux, float uy, float uz)
{
    mx = (ux - mag_x_bias) * mag_x_scale;
    my = (uy - mag_y_bias) * mag_y_scale;
    mz = (uz - mag_z_bias) * mag_z_scale;
}

/* ===== Calib mag (user xoay board 20s) ===== */
void ICM_CalibrateMag(void)
{
    mag_x_bias = mag_y_bias = mag_z_bias = 0.0f;
    mag_x_scale = mag_y_scale = mag_z_scale = 1.0f;

    float mag_min[3] = { 32767.0f,  32767.0f,  32767.0f};
    float mag_max[3] = {-32768.0f, -32768.0f, -32768.0f};

    uint32_t t0 = HAL_GetTick();
    while ((HAL_GetTick() - t0) < 20000)
    {
        axises m;
        if (ak09916_mag_read_uT(&m))
        {
            /* PHẢI GIỐNG HỆT main loop: AK09916 → ICM body frame
             * Xoay -90° quanh Z: (x,y) → (-y, x). */
            float ux = -(float)m.y;
            float uy =  -(float)m.x;
            float uz =  -(float)m.z;

            if (ux < mag_min[0]) mag_min[0] = ux;
            if (ux > mag_max[0]) mag_max[0] = ux;
            if (uy < mag_min[1]) mag_min[1] = uy;
            if (uy > mag_max[1]) mag_max[1] = uy;
            if (uz < mag_min[2]) mag_min[2] = uz;
            if (uz > mag_max[2]) mag_max[2] = uz;
        }
        HAL_Delay(20);
    }

    mag_x_bias = (mag_max[0] + mag_min[0]) / 2.0f;
    mag_y_bias = (mag_max[1] + mag_min[1]) / 2.0f;
    mag_z_bias = (mag_max[2] + mag_min[2]) / 2.0f;

    float dx = (mag_max[0] - mag_min[0]) / 2.0f;
    float dy = (mag_max[1] - mag_min[1]) / 2.0f;
    float dz = (mag_max[2] - mag_min[2]) / 2.0f;
    float avg = (dx + dy + dz) / 3.0f;

    if (dx > 0.001f) mag_x_scale = avg / dx;
    if (dy > 0.001f) mag_y_scale = avg / dy;
    if (dz > 0.001f) mag_z_scale = avg / dz;
}
