#include "gimbal_task.h"
#include "FreeRTOS.h"
#include "task.h"
#include "ins_task.h"

volatile GimbalState_e gimbal_state = GIMBAL_STATE_RELAX;
float ext_target_yaw = 0.0f;
uint8_t flag_ext_yaw = 0;
volatile uint8_t emergency_stop = 1U;

void Gimbal_Task(void *argument)
{
    (void)argument;

    for (;;)
    {
        emergency_stop = 1U;
        gimbal_state = GIMBAL_STATE_RELAX;
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}
