/* led_direction.h */
#ifndef LED_DIRECTION_H
#define LED_DIRECTION_H

#include "main.h"

/*
 * Ánh xạ theo CubeMX (tên macro đã được đặt theo nhãn trên board):
 *   BAC  (Bắc)  = PB11  — active HIGH
 *   TAY  (Đông) = PA5   — active HIGH
 *   DONG (Nam)  = PC13  — active LOW  (LED onboard Blue Pill)
 *   NAM  (Tây)  = PB8   — active HIGH
 */

/* LED heartbeat dùng chung PC13 (DONG) — báo hiệu khi calib */
#define LED_Heartbeat_On()   HAL_GPIO_WritePin(DONG_GPIO_Port, DONG_Pin, GPIO_PIN_RESET)
#define LED_Heartbeat_Off()  HAL_GPIO_WritePin(DONG_GPIO_Port, DONG_Pin, GPIO_PIN_SET)

void LED_Update_Direction(void);

#endif /* LED_DIRECTION_H */
