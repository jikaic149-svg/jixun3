#ifndef MODE_GIMBAL_H
#define MODE_GIMBAL_H

#include <stdint.h>

typedef enum {
    GIMBAL_MODE_RELAX = 0,
    GIMBAL_MODE_HOLD,
    GIMBAL_MODE_AIM
} GimbalMode_e;

typedef struct {
    float target_offset_deg;
    float target_angle_deg;
    float error_deg;
    float yaw_current_a;
    uint8_t drive_enabled;
} GimbalCommand_t;

void Mode_GimbalInit(void);
void Mode_GimbalSet(GimbalMode_e mode);
GimbalMode_e Mode_GimbalGet(void);
void Mode_GimbalSetTargetOffsetDeg(float offset_deg);
void Mode_GimbalAddTargetOffsetDeg(float delta_deg);
void Mode_GimbalUpdate(float encoder_angle_deg,
                       float encoder_speed_rpm,
                       uint8_t feedback_ready,
                       float dt_sec);
void Mode_GimbalGetCommand(GimbalCommand_t *command);

#endif /* MODE_GIMBAL_H */
