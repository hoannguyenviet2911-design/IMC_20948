#ifndef DEBUG_VARS_H
#define DEBUG_VARS_H
#include "main.h"

extern volatile float dbg_ax, dbg_ay, dbg_az;

extern volatile float dbg_gx, dbg_gy, dbg_gz;


extern volatile float dbg_mx, dbg_my, dbg_mz;


extern volatile float dbg_gx_bias, dbg_gy_bias, dbg_gz_bias;


extern volatile float dbg_yaw;


extern volatile float dbg_yaw_comp;


extern volatile float dbg_yaw_fused;


extern volatile float dbg_roll, dbg_pitch;


void UpdateDebugVariables(void);

#endif /* DEBUG_VARS_H */
