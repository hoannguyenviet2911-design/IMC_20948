#ifndef MADGWICK_FILTER_H
#define MADGWICK_FILTER_H

#include <stdint.h>

/* Quaternion toàn cục — output của filter */
extern volatile float q0, q1, q2, q3;

/* Gain — điều chỉnh tốc độ bám của accel/mag */
extern volatile float beta;

/* Update với dt truyền vào (không hard-code sampleFreq) */
void MadgwickAHRSupdate(float gx, float gy, float gz,
                        float ax, float ay, float az,
                        float mx, float my, float mz,
                        float dt);

void MadgwickAHRSupdateIMU(float gx, float gy, float gz,
                           float ax, float ay, float az,
                           float dt);

float invSqrt(float x);

#endif
