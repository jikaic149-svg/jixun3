#ifndef MODE_CONTROL_H
#define MODE_CONTROL_H

#include <stdint.h>

#define MODE_TRACKING_SENSOR_FRESH     0x01U
#define MODE_TRACKING_SENSOR_NEW_FRAME 0x02U

typedef enum {
    CHASSIS_MODE_STAND = 0,
    CHASSIS_MODE_FORWARD,
    CHASSIS_MODE_BACKWARD,
    CHASSIS_MODE_TURN_LEFT,
    CHASSIS_MODE_TURN_RIGHT,
    CHASSIS_MODE_TRACKING,
    CHASSIS_MODE_SQUARE_TRACKING,
    CHASSIS_MODE_TARGET_AIM,
    CHASSIS_MODE_COUNT
} ChassisMode_e;

typedef enum {
    MODE_TRACKING_PHASE_WAIT_START_MARKER = 0,
    MODE_TRACKING_PHASE_WAIT_START_EXIT,
    MODE_TRACKING_PHASE_FOLLOW_TO_TARGET,
    MODE_TRACKING_PHASE_TARGET_RUNOUT,
    MODE_TRACKING_PHASE_TARGET_WAIT_SETTLE,
    MODE_TRACKING_PHASE_TARGET_LASER_DWELL,
    MODE_TRACKING_PHASE_TARGET_DEPART,
    MODE_TRACKING_PHASE_WAIT_TARGET_EXIT,
    MODE_TRACKING_PHASE_FOLLOW_TO_FINISH,
    MODE_TRACKING_PHASE_FINISH_RUNOUT,
    MODE_TRACKING_PHASE_FINISHED
} ModeTrackingPhase_e;

typedef enum {
    MODE_SQUARE_PHASE_WAIT_LINE = 0,
    MODE_SQUARE_PHASE_FOLLOW,
    MODE_SQUARE_PHASE_CORNER_RUNOUT,
    MODE_SQUARE_PHASE_CORNER_BRAKE,
    MODE_SQUARE_PHASE_CORNER_TURN,
    MODE_SQUARE_PHASE_CORNER_ALIGN,
    MODE_SQUARE_PHASE_LOST_HOLD,
    MODE_SQUARE_PHASE_LOST_BRAKE,
    MODE_SQUARE_PHASE_LOST_SEARCH,
    MODE_SQUARE_PHASE_FINISHED,
    MODE_SQUARE_PHASE_FAULT_HOLD
} ModeSquarePhase_e;

typedef enum {
    MODE_TARGET_AIM_PHASE_WAIT_CALIBRATION = 0,
    MODE_TARGET_AIM_PHASE_STATIC_AIM,
    MODE_TARGET_AIM_PHASE_WAIT_LINE,
    MODE_TARGET_AIM_PHASE_FOLLOW_LINE,
    MODE_TARGET_AIM_PHASE_AIM_HOLD
} ModeTargetAimPhase_e;

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
void Mode_SetMotionSettled(uint8_t settled);
void Mode_SetActualVelocity(float velocity_mps);
void Mode_SetChassisYaw(float yaw_deg);
void Mode_SetLaserSafe(uint8_t safe);
void Mode_SetTargetVisionReady(uint8_t ready);
void Mode_SetTargetDriveEnabled(uint8_t enabled);
void Mode_TrackingSetSample(float error, uint8_t black_count,
                            uint8_t black_mask, uint32_t now_ms);
void Mode_TrackingSetError(float error, uint8_t black_count, uint32_t now_ms);
ModeTrackingPhase_e Mode_TrackingGetPhase(void);
ModeSquarePhase_e Mode_SquareTrackingGetPhase(void);
uint8_t Mode_SquareTrackingGetCornerCount(void);
ModeTargetAimPhase_e Mode_TargetAimGetPhase(void);

#endif /* MODE_CONTROL_H */
