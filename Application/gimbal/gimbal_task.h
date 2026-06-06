#ifndef GIMBAL_TASK_H
#define GIMBAL_TASK_H

#include "stdint.h"

typedef enum {
    GIMBAL_STATE_RELAX = 0,
    GIMBAL_STATE_ENC_LOCK,
    GIMBAL_STATE_CALIBRATING,
    GIMBAL_STATE_IMU_STAB
} GimbalState_e;

extern volatile GimbalState_e gimbal_state;

extern void Gimbal_Task(void *argument);

#endif /* GIMBAL_TASK_H */
