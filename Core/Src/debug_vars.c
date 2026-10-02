
#include "debug_vars.h"
#include "icm20948.h"
#include "mag_ak09916.h"
#include "yaw_fusion.h"


volatile float dbg_ax = 0, dbg_ay = 0, dbg_az = 0;
volatile float dbg_gx = 0, dbg_gy = 0, dbg_gz = 0;

volatile float dbg_mx = 0, dbg_my = 0, dbg_mz = 0;


volatile float dbg_gx_bias = 0, dbg_gy_bias = 0, dbg_gz_bias = 0;

volatile float dbg_yaw = 0, dbg_yaw_comp = 0, dbg_yaw_fused = 0;


volatile float dbg_roll = 0, dbg_pitch = 0;


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


}
