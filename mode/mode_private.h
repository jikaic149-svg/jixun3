#ifndef MODE_PRIVATE_H
#define MODE_PRIVATE_H

#include <stdint.h>
#include "mode_control.h"

void Mode_CommandClear(ModeCommand_t *command);
void Mode_MotionBuild(ModeCommand_t *command,
                      float velocity_mps,
                      float bias_deg,
                      float turn_rpm);
void Mode_StopBuild(ModeCommand_t *command);
void Mode_ForwardBuild(ModeCommand_t *command);
void Mode_BackwardBuild(ModeCommand_t *command);
void Mode_TurnLeftBuild(ModeCommand_t *command);
void Mode_TurnRightBuild(ModeCommand_t *command);
void Mode_TrackingBuild(ModeCommand_t *command,
                        float error,
                        uint8_t black_count,
                        uint8_t sensor_flags,
                        uint8_t motion_settled,
                        float actual_velocity_mps,
                        float dt_sec);
void Mode_TrackingReset(void);
void Mode_SquareTrackingBuild(ModeCommand_t *command,
                              float error,
                              uint8_t black_count,
                              uint8_t black_mask,
                              uint8_t sensor_flags,
                              uint8_t motion_settled,
                              float actual_velocity_mps,
                              float dt_sec);
void Mode_SquareTrackingReset(void);

#endif /* MODE_PRIVATE_H */
