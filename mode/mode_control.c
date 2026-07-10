#include "mode_control.h"

#include <stddef.h>
#include "config.h"
#include "mode_private.h"
#include "track.h"

static ChassisMode_e mode_active = CHASSIS_MODE_STAND;
static ModeCommand_t mode_command;
static float tracking_error = TRACK_LOST_LINE;
static uint8_t tracking_black_count = 0U;
static uint8_t tracking_seen = 0U;
static uint32_t tracking_update_ms = 0U;
static uint32_t mode_last_update_ms = 0U;

void Mode_CommandClear(ModeCommand_t *command)
{
    if (command == NULL) {
        return;
    }
    command->velocity_mps = 0.0f;
    command->bias_deg = BALANCE_MODE_STAND_BIAS_DEG;
    command->turn_rpm = 0.0f;
    command->drive_enabled = 0U;
    command->tracking_active = 0U;
    command->laser_enabled = 0U;
}

void Mode_Init(void)
{
    Mode_CommandClear(&mode_command);
    mode_active = CHASSIS_MODE_STAND;
    tracking_error = TRACK_LOST_LINE;
    tracking_black_count = 0U;
    tracking_seen = 0U;
    tracking_update_ms = 0U;
    mode_last_update_ms = 0U;
    Mode_TrackingReset();
}

void Mode_Set(ChassisMode_e mode)
{
    if (mode > CHASSIS_MODE_TRACKING) {
        mode = CHASSIS_MODE_STAND;
    }

    if (mode_active != mode) {
        mode_active = mode;
        if (mode == CHASSIS_MODE_TRACKING) {
            Mode_TrackingReset();
        }
    }
}

ChassisMode_e Mode_Get(void)
{
    return mode_active;
}

void Mode_Update(uint32_t now_ms)
{
    ModeCommand_t command;
    uint32_t elapsed_ms;
    float dt_sec;
    float error;
    uint8_t sensor_fresh;

    Mode_CommandClear(&command);

    if (mode_last_update_ms == 0U) {
        dt_sec = BALANCE_PID_DT_SEC;
    } else {
        elapsed_ms = now_ms - mode_last_update_ms;
        if (elapsed_ms == 0U) {
            dt_sec = BALANCE_PID_DT_SEC;
        } else if (elapsed_ms > 100U) {
            dt_sec = 0.1f;
        } else {
            dt_sec = 0.001f * (float)elapsed_ms;
        }
    }
    mode_last_update_ms = now_ms;

    switch (mode_active) {
    case CHASSIS_MODE_FORWARD:
        Mode_ForwardBuild(&command);
        break;
    case CHASSIS_MODE_BACKWARD:
        Mode_BackwardBuild(&command);
        break;
    case CHASSIS_MODE_TURN_LEFT:
        Mode_TurnLeftBuild(&command);
        break;
    case CHASSIS_MODE_TURN_RIGHT:
        Mode_TurnRightBuild(&command);
        break;
    case CHASSIS_MODE_TRACKING:
        sensor_fresh =
            ((tracking_seen != 0U) &&
             ((uint32_t)(now_ms - tracking_update_ms) <=
              TRACKING_DATA_TIMEOUT_MS)) ? 1U : 0U;
        error = sensor_fresh ? tracking_error : TRACK_LOST_LINE;
        Mode_TrackingBuild(&command, error, tracking_black_count,
                           sensor_fresh, dt_sec);
        break;
    case CHASSIS_MODE_STAND:
    default:
        Mode_StopBuild(&command);
        break;
    }

    mode_command = command;
}

void Mode_GetCommand(ModeCommand_t *command)
{
    if (command == NULL) {
        return;
    }
    *command = mode_command;
}

void Mode_TrackingSetError(float error, uint8_t black_count, uint32_t now_ms)
{
    tracking_error = error;
    tracking_black_count = black_count;
    tracking_update_ms = now_ms;
    tracking_seen = 1U;
}
