

#ifndef YAW_FUSION_H
#define YAW_FUSION_H

#include "main.h"


extern volatile float roll;
extern volatile float pitch;

extern volatile float yaw_quat;

extern volatile float yaw;


extern volatile float yaw_compass;


extern volatile float yaw_fused;


extern volatile uint8_t yaw_first_run;


void CalculateYaw(void);


void UpdateYaw(float dt);

#endif /* YAW_FUSION_H */
