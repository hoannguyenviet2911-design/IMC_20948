#ifndef MADGWICK_FILTER_H
#define MADGWICK_FILTER_H

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ===== Tham số & tứ nguyên toàn cục ===== */
extern volatile float beta;
extern volatile float q0, q1, q2, q3;

/* ===== API lọc Madgwick ===== */
void MadgwickAHRSupdate(float gx, float gy, float gz,
                        float ax, float ay, float az,
                        float mx, float my, float mz,
                        float dt);

void MadgwickAHRSupdateIMU(float gx, float gy, float gz,
                           float ax, float ay, float az,
                           float dt);

float invSqrt(float x);

#ifdef __cplusplus
}
#endif

#endif /* MADGWICK_FILTER_H */
