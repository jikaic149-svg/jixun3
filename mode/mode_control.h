#ifndef MODE_CONTROL_H
#define MODE_CONTROL_H

#include <stdint.h>

typedef enum {
    CHASSIS_MODE_STAND = 0,
    CHASSIS_MODE_FORWARD,
    CHASSIS_MODE_BACKWARD,
    CHASSIS_MODE_TURN_LEFT,
    CHASSIS_MODE_TURN_RIGHT,
    CHASSIS_MODE_TRACKING
} ChassisMode_e;

typedef struct {
    float velocity_mps;
    float bias_deg;
    float turn_rpm;
    uint8_t drive_enabled;
    uint8_t tracking_active;
    uint8_t laser_enabled;
} ModeCommand_t;

void Mode_Init(void);
void Mode_Set(ChassisMode_e mode);
ChassisMode_e Mode_Get(void);
void Mode_Update(uint32_t now_ms);
void Mode_GetCommand(ModeCommand_t *command);
void Mode_TrackingSetError(float error, uint8_t black_count, uint32_t now_ms);

#endif /* MODE_CONTROL_H */
