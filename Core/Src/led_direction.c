/* led_direction.c */
#include "led_direction.h"
#include "yaw_fusion.h"

void LED_Update_Direction(void)
{
    /* Tắt tất cả LED */
    HAL_GPIO_WritePin(BAC_GPIO_Port,  BAC_Pin,  GPIO_PIN_RESET);   /* Bắc */
    HAL_GPIO_WritePin(TAY_GPIO_Port,  TAY_Pin,  GPIO_PIN_RESET);   /* Đông */
    HAL_GPIO_WritePin(DONG_GPIO_Port, DONG_Pin, GPIO_PIN_SET);     /* Nam  (active LOW) */
    HAL_GPIO_WritePin(NAM_GPIO_Port,  NAM_Pin,  GPIO_PIN_RESET);   /* Tây */

    if (yaw >= 315.0f || yaw < 45.0f) {
        /* BẮC — PB11 */
        HAL_GPIO_WritePin(BAC_GPIO_Port, BAC_Pin, GPIO_PIN_SET);
    }
    else if (yaw >= 45.0f && yaw < 135.0f) {
        /* ĐÔNG — PA5 */
        HAL_GPIO_WritePin(TAY_GPIO_Port, TAY_Pin, GPIO_PIN_SET);
    }
    else if (yaw >= 135.0f && yaw < 225.0f) {
        /* NAM — PC13 (active LOW) */
        HAL_GPIO_WritePin(DONG_GPIO_Port, DONG_Pin, GPIO_PIN_RESET);
    }
    else {
        /* TÂY — PB8 */
        HAL_GPIO_WritePin(NAM_GPIO_Port, NAM_Pin, GPIO_PIN_SET);
    }
}
