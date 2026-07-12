#include "mode_private.h"

#include <stddef.h>

#include "config.h"
#include "track.h"

static volatile ModeTargetAimPhase_e target_aim_phase =
    MODE_TARGET_AIM_PHASE_WAIT_CALIBRATION;
static float target_aim_clock_sec = 0.0f;
static float target_aim_line_start_sec = 0.0f;
static float target_aim_white_start_sec = 0.0f;
static float target_aim_probe_elapsed_sec = 0.0f;
static float target_aim_follow_wide_elapsed_sec = 0.0f;
static uint8_t target_aim_line_confirming = 0U;
static uint8_t target_aim_white_confirming = 0U;

static void target_aim_build_hold(ModeCommand_t *command)
{
    Mode_StopBuild(command);
    command->tracking_active = 1U;
    command->laser_enabled = 1U;
}

static void target_aim_force_task_flags(ModeCommand_t *command)
{
    command->tracking_active = 1U;
    command->laser_enabled = 1U;
}

static void target_aim_reset_line_confirmation(void)
{
    target_aim_line_confirming = 0U;
    target_aim_line_start_sec = target_aim_clock_sec;
}

static void target_aim_reset_white_confirmation(void)
{
    target_aim_white_confirming = 0U;
    target_aim_white_start_sec = target_aim_clock_sec;
}

static void target_aim_reset_wide_probe(void)
{
    target_aim_follow_wide_elapsed_sec = 0.0f;
}

static void target_aim_reset_follow_state(void)
{
    Mode_TrackingReset();
}

static uint8_t target_aim_confirm_on_new_frame(uint8_t condition,
                                               uint8_t sensor_new_frame,
                                               uint32_t confirm_ms,
                                               uint8_t *confirming,
                                               float *start_sec)
{
    if (sensor_new_frame == 0U) {
        return 0U;
    }
    if (condition == 0U) {
        *confirming = 0U;
        *start_sec = target_aim_clock_sec;
        return 0U;
    }
    if (confirm_ms == 0U) {
        *confirming = 0U;
        return 1U;
    }
    if (*confirming == 0U) {
        *confirming = 1U;
        *start_sec = target_aim_clock_sec;
        return 0U;
    }
    if ((target_aim_clock_sec - *start_sec) <
        (0.001f * (float)confirm_ms)) {
        return 0U;
    }

    *confirming = 0U;
    return 1U;
}

static void target_aim_build_line_follow(ModeCommand_t *command,
                                         float error,
                                         uint8_t sensor_flags,
                                         float dt_sec)
{
    Mode_TrackingBuildLineFollow(command, error, sensor_flags, dt_sec);
    if (command->turn_rpm > TARGET_AIM_TURN_LIMIT_RPM) {
        command->turn_rpm = TARGET_AIM_TURN_LIMIT_RPM;
    } else if (command->turn_rpm < -TARGET_AIM_TURN_LIMIT_RPM) {
        command->turn_rpm = -TARGET_AIM_TURN_LIMIT_RPM;
    }
    target_aim_force_task_flags(command);
}

static void target_aim_build_wide_start_probe(ModeCommand_t *command)
{
    command->velocity_mps = TARGET_AIM_WIDE_START_PROBE_VELOCITY_MPS;
    command->bias_deg = BALANCE_MODE_STAND_BIAS_DEG;
    command->turn_rpm = 0.0f;
    command->drive_enabled = 1U;
    target_aim_force_task_flags(command);
}

void Mode_TargetAimReset(void)
{
    target_aim_phase = MODE_TARGET_AIM_PHASE_WAIT_CALIBRATION;
    target_aim_clock_sec = 0.0f;
    target_aim_line_start_sec = 0.0f;
    target_aim_white_start_sec = 0.0f;
    target_aim_probe_elapsed_sec = 0.0f;
    target_aim_follow_wide_elapsed_sec = 0.0f;
    target_aim_line_confirming = 0U;
    target_aim_white_confirming = 0U;
}

ModeTargetAimPhase_e Mode_TargetAimGetPhase(void)
{
    return target_aim_phase;
}

void Mode_TargetAimBuild(ModeCommand_t *command,
                         float error,
                         uint8_t black_count,
                         uint8_t black_mask,
                         uint8_t sensor_flags,
                         uint8_t vision_ready,
                         uint8_t drive_enabled,
                         float dt_sec)
{
    uint8_t sensor_fresh;
    uint8_t sensor_new_frame;
    uint8_t line_visible;
    uint8_t all_white;
    uint8_t wide_black;

    if (command == NULL) {
        return;
    }
    if (dt_sec <= 0.0f) {
        dt_sec = BALANCE_PID_DT_SEC;
    } else if (dt_sec > 0.1f) {
        dt_sec = 0.1f;
    }
    target_aim_clock_sec += dt_sec;

    sensor_fresh =
        ((sensor_flags & MODE_TRACKING_SENSOR_FRESH) != 0U) ? 1U : 0U;
    sensor_new_frame =
        ((sensor_flags & MODE_TRACKING_SENSOR_NEW_FRAME) != 0U) ? 1U : 0U;
    line_visible =
        ((error != TRACK_LOST_LINE) &&
         (black_count > 0U) &&
         (black_count <= TARGET_AIM_LINE_BLACK_COUNT_MAX)) ? 1U : 0U;
    all_white =
        ((error == TRACK_LOST_LINE) &&
         (black_count == 0U) &&
         (black_mask == 0U)) ? 1U : 0U;
    wide_black =
        ((error != TRACK_LOST_LINE) &&
         (black_count > TARGET_AIM_LINE_BLACK_COUNT_MAX) &&
         (black_mask != 0U)) ? 1U : 0U;

    target_aim_build_hold(command);

    switch (target_aim_phase) {
    case MODE_TARGET_AIM_PHASE_WAIT_CALIBRATION:
        if (vision_ready != 0U) {
            if (drive_enabled != 0U) {
                target_aim_reset_follow_state();
                target_aim_phase = MODE_TARGET_AIM_PHASE_WAIT_LINE;
                target_aim_reset_line_confirmation();
                target_aim_probe_elapsed_sec = 0.0f;
            } else {
                target_aim_phase = MODE_TARGET_AIM_PHASE_STATIC_AIM;
            }
        }
        return;

    case MODE_TARGET_AIM_PHASE_STATIC_AIM:
        if ((drive_enabled != 0U) && (vision_ready != 0U)) {
            target_aim_reset_follow_state();
            target_aim_phase = MODE_TARGET_AIM_PHASE_WAIT_LINE;
            target_aim_reset_line_confirmation();
            target_aim_probe_elapsed_sec = 0.0f;
        }
        return;

    case MODE_TARGET_AIM_PHASE_WAIT_LINE:
        if (drive_enabled == 0U) {
            target_aim_reset_follow_state();
            target_aim_phase = MODE_TARGET_AIM_PHASE_STATIC_AIM;
            target_aim_reset_line_confirmation();
            return;
        }
        if (sensor_fresh == 0U) {
            target_aim_reset_line_confirmation();
            return;
        }
        if (all_white != 0U) {
            target_aim_reset_line_confirmation();
            return;
        }
        if (line_visible != 0U) {
            uint8_t line_confirmed = target_aim_confirm_on_new_frame(
                1U,
                sensor_new_frame,
                TARGET_AIM_LINE_CONFIRM_MS,
                &target_aim_line_confirming,
                &target_aim_line_start_sec);

            target_aim_build_line_follow(command, error, sensor_flags, dt_sec);
            if (line_confirmed != 0U) {
                target_aim_phase = MODE_TARGET_AIM_PHASE_FOLLOW_LINE;
                target_aim_reset_white_confirmation();
                target_aim_reset_wide_probe();
            }
            return;
        }
        (void)target_aim_confirm_on_new_frame(
            0U,
            sensor_new_frame,
            TARGET_AIM_LINE_CONFIRM_MS,
            &target_aim_line_confirming,
            &target_aim_line_start_sec);
        if ((wide_black != 0U) &&
            (target_aim_probe_elapsed_sec <
             (0.001f * (float)TARGET_AIM_WIDE_START_PROBE_MAX_MS))) {
            target_aim_probe_elapsed_sec += dt_sec;
            target_aim_build_wide_start_probe(command);
        }
        return;

    case MODE_TARGET_AIM_PHASE_FOLLOW_LINE:
        if (drive_enabled == 0U) {
            target_aim_reset_follow_state();
            target_aim_phase = MODE_TARGET_AIM_PHASE_STATIC_AIM;
            target_aim_reset_white_confirmation();
            return;
        }
        if (sensor_fresh == 0U) {
            target_aim_reset_white_confirmation();
            target_aim_reset_wide_probe();
            return;
        }

        if (target_aim_confirm_on_new_frame(
                all_white,
                sensor_new_frame,
                TARGET_AIM_WHITE_CONFIRM_MS,
                &target_aim_white_confirming,
                &target_aim_white_start_sec) != 0U) {
            target_aim_phase = MODE_TARGET_AIM_PHASE_AIM_HOLD;
            target_aim_build_hold(command);
            return;
        }

        if (target_aim_white_confirming != 0U) {
            /* Stop on the first all-white frame. Confirmation happens while
             * stationary so an old turn command cannot carry the car away. */
            target_aim_build_hold(command);
            return;
        }

        if (line_visible != 0U) {
            target_aim_reset_wide_probe();
            target_aim_build_line_follow(command, error, sensor_flags, dt_sec);
            return;
        }

        if (wide_black != 0U) {
            if (target_aim_follow_wide_elapsed_sec <
                (0.001f * (float)TARGET_AIM_WIDE_FOLLOW_PROBE_MAX_MS)) {
                target_aim_follow_wide_elapsed_sec += dt_sec;
                target_aim_build_wide_start_probe(command);
            }
            return;
        }

        target_aim_reset_wide_probe();
        return;

    case MODE_TARGET_AIM_PHASE_AIM_HOLD:
        return;

    default:
        Mode_TargetAimReset();
        target_aim_build_hold(command);
        return;
    }
}
