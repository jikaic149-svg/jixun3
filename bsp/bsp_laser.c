#include "bsp_laser.h"

#include "tim.h"

static uint8_t laser_ready = 0U;
static uint8_t laser_on = 0U;

uint8_t BSP_LaserInit(void)
{
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0U);
    if (HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3) != HAL_OK) {
        laser_ready = 0U;
        laser_on = 0U;
        return 0U;
    }

    laser_ready = 1U;
    laser_on = 0U;
    return 1U;
}

void BSP_LaserOn(void)
{
    if (laser_ready == 0U) {
        return;
    }

    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3,
                          __HAL_TIM_GET_AUTORELOAD(&htim3));
    laser_on = 1U;
}

void BSP_LaserOff(void)
{
    if (laser_ready == 0U) {
        return;
    }

    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0U);
    laser_on = 0U;
}

uint8_t BSP_LaserIsOn(void)
{
    return laser_on;
}
