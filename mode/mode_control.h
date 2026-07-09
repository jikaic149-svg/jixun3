#ifndef MODE_CONTROL_H
#define MODE_CONTROL_H

#include <stdint.h>
#include "gimbal_task.h"

typedef struct {
    float velocity_mps;
    float bias_deg;
    float turn_rpm;
    uint8_t drive_enabled;
    uint8_t tracking_active;
} ModeCommand_t;

void Mode_Init(void);
void Mode_Set(ChassisMode_e mode);
ChassisMode_e Mode_Get(void);
void Mode_Update(uint32_t now_ms);
void Mode_GetCommand(ModeCommand_t *command);
void Mode_TrackingSetError(float error, uint8_t black_count, uint32_t now_ms);

#endif /* MODE_CONTROL_H */
