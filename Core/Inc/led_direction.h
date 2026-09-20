/* =============================================================================
 * ÁNH XẠ PHẦN CỨNG (theo main.h):
 *   ┌────────┬─────────┬──────────────┬──────────────────────────┐
 *   │ Hướng  │ Chân    │ Active       │ Ghi chú                  │
 *   ├────────┼─────────┼──────────────┼──────────────────────────┤
 *   │ BẮC    │ PB11    │ HIGH         │ LED ngoài                │
 *   │ ĐÔNG   │ PA5     │ HIGH         │ LED ngoài                │
 *   │ NAM    │ PC13    │ LOW          │ LED onboard Blue Pill    │
 *   │ TÂY    │ PB8     │ HIGH         │ LED ngoài                │
 *   └────────┴─────────┴──────────────┴──────────────────────────┘

 * =============================================================================
 */

#ifndef LED_DIRECTION_H
#define LED_DIRECTION_H

#include "main.h"


#define LED_BAC_ON()    HAL_GPIO_WritePin(BAC_GPIO_Port,  BAC_Pin,  GPIO_PIN_SET)
#define LED_BAC_OFF()   HAL_GPIO_WritePin(BAC_GPIO_Port,  BAC_Pin,  GPIO_PIN_RESET)

#define LED_DONG_ON()   HAL_GPIO_WritePin(DONG_GPIO_Port, DONG_Pin, GPIO_PIN_SET)
#define LED_DONG_OFF()  HAL_GPIO_WritePin(DONG_GPIO_Port, DONG_Pin, GPIO_PIN_RESET)

#define LED_NAM_ON()    HAL_GPIO_WritePin(NAM_GPIO_Port,  NAM_Pin,  GPIO_PIN_RESET)
#define LED_NAM_OFF()   HAL_GPIO_WritePin(NAM_GPIO_Port,  NAM_Pin,  GPIO_PIN_SET)

#define LED_TAY_ON()    HAL_GPIO_WritePin(TAY_GPIO_Port,  TAY_Pin,  GPIO_PIN_SET)
#define LED_TAY_OFF()   HAL_GPIO_WritePin(TAY_GPIO_Port,  TAY_Pin,  GPIO_PIN_RESET)

#define LED_Heartbeat_On()   LED_NAM_ON()
#define LED_Heartbeat_Off()  LED_NAM_OFF()

void LED_Update_Direction(void);

#endif
