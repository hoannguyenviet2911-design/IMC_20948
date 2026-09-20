/* =============================================================================
 *
 * PHÂN VÙNG GÓC:
 *   ┌───────┬──────────────────────────┐
 *   │ BẮC   │ [315°, 360°) ∪ [0°, 45°) │
 *   │ ĐÔNG  │ [45°, 135°)              │
 *   │ NAM   │ [135°, 225°)             │
 *   │ TÂY   │ [225°, 315°)             │
 *   └───────┴──────────────────────────┘
 * =============================================================================
 */

#include "led_direction.h"
#include "yaw_fusion.h"

void LED_Update_Direction(void)
{
    LED_BAC_OFF();
    LED_DONG_OFF();
    LED_NAM_OFF();
    LED_TAY_OFF();

    if (yaw >= 315.0f || yaw < 45.0f) {
        LED_BAC_ON();
    }
    else if (yaw >= 45.0f && yaw < 135.0f) {
        LED_DONG_ON();
    }
    else if (yaw >= 135.0f && yaw < 225.0f) {
        LED_NAM_ON();
    }
    else {
        LED_TAY_ON();
    }
}
