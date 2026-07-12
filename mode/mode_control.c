#include "mode_control.h"

#include <stddef.h>
#include "config.h"
#include "mode_private.h"
#include "track.h"

typedef struct {
    float error;
    uint8_t black_count;
    uint8_t black_mask;
    uint32_t update_ms;
    uint32_t generation;
} ModeTrackingInput_t;

static volatile ChassisMode_e mode_active = CHASSIS_MODE_STAND;
static ModeCommand_t mode_command;
static volatile ModeTrackingInput_t tracking_input_slots[2];
static volatile uint8_t tracking_input_published = 0U;
static uint32_t tracking_input_next_generation = 0U;
static uint32_t tracking_consumed_generation = 0U;
static uint8_t tracking_wait_for_new_frame = 0U;
static volatile uint8_t mode_motion_settled = 0U;
static volatile float mode_actual_velocity_mps = 0.0f;
static volatile float mode_chassis_yaw_deg = 0.0f;
static volatile uint8_t mode_laser_safe = 0U;
static uint32_t mode_last_update_ms = 0U;

static ModeTrackingInput_t mode_tracking_input_snapshot(void)
{
    ModeTrackingInput_t snapshot;
    uint8_t index = tracking_input_published;

    snapshot.error = tracking_input_slots[index].error;
    snapshot.black_count = tracking_input_slots[index].black_count;
    snapshot.black_mask = tracking_input_slots[index].black_mask;
    snapshot.update_ms = tracking_input_slots[index].update_ms;
    snapshot.generation = tracking_input_slots[index].generation;
    return snapshot;
}

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

void Mode_MotionBuild(ModeCommand_t *command,
                      float velocity_mps,
                      float bias_deg,
                      float turn_rpm)
{
    if (command == NULL) {
        return;
    }

    Mode_CommandClear(command);
    command->velocity_mps = velocity_mps;
    command->bias_deg = bias_deg;
    command->turn_rpm = turn_rpm;
    command->drive_enabled =
        ((velocity_mps > 0.001f) || (velocity_mps < -0.001f)) ? 1U : 0U;
}

void Mode_Init(void)
{
    Mode_CommandClear(&mode_command);
    mode_active = CHASSIS_MODE_STAND;
    for (uint8_t i = 0U; i < 2U; i++) {
        tracking_input_slots[i].error = TRACK_LOST_LINE;
        tracking_input_slots[i].black_count = 0U;
        tracking_input_slots[i].black_mask = 0U;
        tracking_input_slots[i].update_ms = 0U;
        tracking_input_slots[i].generation = 0U;
    }
    tracking_input_published = 0U;
    tracking_input_next_generation = 0U;
    tracking_consumed_generation = 0U;
    tracking_wait_for_new_frame = 0U;
    mode_motion_settled = 0U;
    mode_actual_velocity_mps = 0.0f;
    mode_chassis_yaw_deg = 0.0f;
    mode_laser_safe = 0U;
    mode_last_update_ms = 0U;
    Mode_TrackingReset();
    Mode_SquareTrackingReset();
}

void Mode_Set(ChassisMode_e mode)
{
    if (mode >= CHASSIS_MODE_COUNT) {
        mode = CHASSIS_MODE_STAND;
    }

    if (mode_active != mode) {
        mode_motion_settled = 0U;
        mode_laser_safe = 0U;
        if ((mode == CHASSIS_MODE_TRACKING) ||
            (mode == CHASSIS_MODE_SQUARE_TRACKING)) {
            ModeTrackingInput_t input = mode_tracking_input_snapshot();

            if (mode == CHASSIS_MODE_TRACKING) {
                Mode_TrackingReset();
            } else {
                Mode_SquareTrackingReset();
            }
            tracking_consumed_generation = input.generation;
            tracking_wait_for_new_frame = 1U;
        }
        mode_active = mode;
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
    uint8_t sensor_flags;

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
    case CHASSIS_MODE_SQUARE_TRACKING:
    {
        ModeTrackingInput_t input = mode_tracking_input_snapshot();
        uint8_t new_frame =
            (input.generation != tracking_consumed_generation) ? 1U : 0U;

        if (new_frame != 0U) {
            tracking_consumed_generation = input.generation;
            tracking_wait_for_new_frame = 0U;
        }

        sensor_flags = 0U;
        if ((tracking_wait_for_new_frame == 0U) &&
            (input.generation != 0U) &&
            ((uint32_t)(now_ms - input.update_ms) <=
             TRACKING_DATA_TIMEOUT_MS)) {
            sensor_flags = MODE_TRACKING_SENSOR_FRESH;
            if (new_frame != 0U) {
                sensor_flags |= MODE_TRACKING_SENSOR_NEW_FRAME;
            }
        }
        error = ((sensor_flags & MODE_TRACKING_SENSOR_FRESH) != 0U) ?
                input.error : TRACK_LOST_LINE;
        if (mode_active == CHASSIS_MODE_TRACKING) {
            Mode_TrackingBuild(&command, error, input.black_count,
                               sensor_flags, mode_laser_safe,
                               mode_actual_velocity_mps, dt_sec);
        } else {
            Mode_SquareTrackingBuild(&command, error, input.black_count,
                                     input.black_mask,
                                     sensor_flags, mode_motion_settled,
                                     mode_actual_velocity_mps,
                                     mode_chassis_yaw_deg,
                                     dt_sec);
        }
        break;
    }
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

void Mode_SetMotionSettled(uint8_t settled)
{
    mode_motion_settled = (settled != 0U) ? 1U : 0U;
}

void Mode_SetActualVelocity(float velocity_mps)
{
    mode_actual_velocity_mps = velocity_mps;
}

void Mode_SetChassisYaw(float yaw_deg)
{
    mode_chassis_yaw_deg = yaw_deg;
}

void Mode_SetLaserSafe(uint8_t safe)
{
    mode_laser_safe = (safe != 0U) ? 1U : 0U;
}

void Mode_TrackingSetSample(float error, uint8_t black_count,
                            uint8_t black_mask, uint32_t now_ms)
{
    uint8_t publish_index = (uint8_t)(tracking_input_published ^ 1U);

    tracking_input_next_generation++;
    if (tracking_input_next_generation == 0U) {
        tracking_input_next_generation = 1U;
    }
    if (black_count > TRACK_SENSOR_COUNT) {
        black_count = TRACK_SENSOR_COUNT;
    }

    tracking_input_slots[publish_index].error = error;
    tracking_input_slots[publish_index].black_count = black_count;
    tracking_input_slots[publish_index].black_mask = black_mask;
    tracking_input_slots[publish_index].update_ms = now_ms;
    tracking_input_slots[publish_index].generation =
        tracking_input_next_generation;
    tracking_input_published = publish_index;
}

void Mode_TrackingSetError(float error, uint8_t black_count, uint32_t now_ms)
{
    Mode_TrackingSetSample(error, black_count, 0U, now_ms);
}
