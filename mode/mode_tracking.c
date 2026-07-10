#include "mode_private.h"

#include "config.h"
#include "track.h"

typedef enum {
    TRACKING_BLOCK_WAIT_START = 0,
    TRACKING_BLOCK_WAIT_START_EXIT,
    TRACKING_BLOCK_TARGET_ARMED,
    TRACKING_BLOCK_TARGET_RUNOUT,
    TRACKING_BLOCK_TARGET_STOPPED,
    TRACKING_BLOCK_TARGET_WAIT_EXIT,
    TRACKING_BLOCK_FINISH_ARMED,
    TRACKING_BLOCK_FINISH_RUNOUT,
    TRACKING_BLOCK_FINISH_DONE
} TrackingBlockState_t;

static float tracking_filtered_error = 0.0f;
static float tracking_last_error = 0.0f;
static float tracking_integral = 0.0f;
static float tracking_last_turn_rpm = 0.0f;
static float tracking_last_turn_dir = 0.0f;
static float tracking_lost_search_dir = 0.0f;
static float tracking_last_velocity_mps = 0.0f;
static float tracking_last_bias_deg = BALANCE_MODE_STAND_BIAS_DEG;
static float tracking_lost_elapsed_sec = 0.0f;
static float tracking_block_wide_sec = 0.0f;
static float tracking_block_clear_sec = 0.0f;
static float tracking_target_runout_sec = 0.0f;
static float tracking_target_laser_sec = 0.0f;
static float tracking_finish_runout_sec = 0.0f;
static TrackingBlockState_t tracking_block_state = TRACKING_BLOCK_WAIT_START;
static uint8_t tracking_ready = 0U;
static uint8_t tracking_line_seen = 0U;
static uint8_t tracking_lost_active = 0U;
static uint8_t tracking_sharp_turn_active = 0U;

static float mode_absf(float value)
{
    return (value >= 0.0f) ? value : -value;
}

static float mode_limit_abs(float value, float limit)
{
    float abs_limit = mode_absf(limit);

    if (value > abs_limit) {
        return abs_limit;
    }
    if (value < -abs_limit) {
        return -abs_limit;
    }
    return value;
}

static float tracking_default_lost_dir(void)
{
    return (TRACKING_LOST_DEFAULT_TURN_DIR >= 0.0f) ? 1.0f : -1.0f;
}

static void tracking_begin_lost_search(void)
{
    if (tracking_lost_active != 0U) {
        return;
    }

    tracking_lost_search_dir = tracking_last_turn_dir;
    if (tracking_lost_search_dir == 0.0f) {
        if (tracking_last_turn_rpm > 0.001f) {
            tracking_lost_search_dir = 1.0f;
        } else if (tracking_last_turn_rpm < -0.001f) {
            tracking_lost_search_dir = -1.0f;
        } else {
            tracking_lost_search_dir = tracking_default_lost_dir();
        }
    }
    tracking_lost_active = 1U;
}

static float tracking_build_lost_turn(float min_turn_rpm)
{
    float turn_dir = (tracking_lost_active != 0U) ?
                     tracking_lost_search_dir : tracking_last_turn_dir;

    if (turn_dir == 0.0f) {
        turn_dir = tracking_default_lost_dir();
    }

    return mode_limit_abs(turn_dir * mode_absf(min_turn_rpm),
                          TRACKING_TURN_LIMIT_RPM);
}

static void tracking_update_block_state(uint8_t wide_block,
                                        uint8_t target_block,
                                        float dt_sec)
{
    switch (tracking_block_state) {
    case TRACKING_BLOCK_WAIT_START:
        if (wide_block != 0U) {
            tracking_block_wide_sec += dt_sec;
            if (tracking_block_wide_sec >=
                (0.001f * (float)TRACKING_START_BLOCK_CONFIRM_MS)) {
                tracking_block_state = TRACKING_BLOCK_WAIT_START_EXIT;
                tracking_block_clear_sec = 0.0f;
            }
        } else {
            tracking_block_wide_sec = 0.0f;
        }
        break;

    case TRACKING_BLOCK_WAIT_START_EXIT:
        if (wide_block != 0U) {
            tracking_block_clear_sec = 0.0f;
        } else {
            tracking_block_clear_sec += dt_sec;
            if (tracking_block_clear_sec >=
                (0.001f * (float)TRACKING_START_BLOCK_EXIT_MS)) {
                tracking_block_state = TRACKING_BLOCK_TARGET_ARMED;
                tracking_block_wide_sec = 0.0f;
            }
        }
        break;

    case TRACKING_BLOCK_TARGET_ARMED:
        if (target_block != 0U) {
            tracking_block_wide_sec += dt_sec;
            if (tracking_block_wide_sec >=
                (0.001f * (float)TRACKING_TARGET_BLOCK_CONFIRM_MS)) {
                tracking_block_state = TRACKING_BLOCK_TARGET_RUNOUT;
                tracking_target_runout_sec = 0.0f;
            }
        } else {
            tracking_block_wide_sec = 0.0f;
        }
        break;

    case TRACKING_BLOCK_TARGET_WAIT_EXIT:
        if (target_block != 0U) {
            tracking_block_clear_sec = 0.0f;
        } else {
            tracking_block_clear_sec += dt_sec;
            if (tracking_block_clear_sec >=
                (0.001f * (float)TRACKING_TARGET_BLOCK_EXIT_MS)) {
                tracking_block_state = TRACKING_BLOCK_FINISH_ARMED;
                tracking_block_wide_sec = 0.0f;
            }
        }
        break;

    case TRACKING_BLOCK_FINISH_ARMED:
        if (wide_block != 0U) {
            tracking_block_wide_sec += dt_sec;
            if (tracking_block_wide_sec >=
                (0.001f * (float)TRACKING_FINISH_BLOCK_CONFIRM_MS)) {
                tracking_block_state = TRACKING_BLOCK_FINISH_RUNOUT;
                tracking_finish_runout_sec = 0.0f;
            }
        } else {
            tracking_block_wide_sec = 0.0f;
        }
        break;

    case TRACKING_BLOCK_TARGET_RUNOUT:
    case TRACKING_BLOCK_TARGET_STOPPED:
    case TRACKING_BLOCK_FINISH_RUNOUT:
    case TRACKING_BLOCK_FINISH_DONE:
    default:
        break;
    }
}

void Mode_TrackingReset(void)
{
    tracking_filtered_error = 0.0f;
    tracking_last_error = 0.0f;
    tracking_integral = 0.0f;
    tracking_last_turn_rpm = 0.0f;
    tracking_last_turn_dir = 0.0f;
    tracking_lost_search_dir = 0.0f;
    tracking_last_velocity_mps = 0.0f;
    tracking_last_bias_deg = BALANCE_MODE_STAND_BIAS_DEG;
    tracking_lost_elapsed_sec = 0.0f;
    tracking_block_wide_sec = 0.0f;
    tracking_block_clear_sec = 0.0f;
    tracking_target_runout_sec = 0.0f;
    tracking_target_laser_sec = 0.0f;
    tracking_finish_runout_sec = 0.0f;
    tracking_block_state = TRACKING_BLOCK_WAIT_START;
    tracking_ready = 0U;
    tracking_line_seen = 0U;
    tracking_lost_active = 0U;
    tracking_sharp_turn_active = 0U;
}

void Mode_TrackingBuild(ModeCommand_t *command,
                        float error,
                        uint8_t black_count,
                        uint8_t sensor_fresh,
                        float dt_sec)
{
    float derivative;
    float abs_error;
    float speed_scale = 1.0f;
    float turn_rpm;
    uint8_t wide_block;
    uint8_t target_block;

    Mode_CommandClear(command);
    command->tracking_active = 1U;

    if (dt_sec <= 0.0f) {
        dt_sec = BALANCE_PID_DT_SEC;
    }

    wide_block = ((sensor_fresh != 0U) &&
                  (black_count >= TRACKING_BLOCK_BLACK_COUNT_MIN)) ? 1U : 0U;
    target_block = ((sensor_fresh != 0U) &&
                    (black_count >=
                     TRACKING_TARGET_BLOCK_BLACK_COUNT_MIN)) ? 1U : 0U;
    tracking_update_block_state(wide_block, target_block, dt_sec);

    if (tracking_block_state == TRACKING_BLOCK_WAIT_START) {
        tracking_ready = 0U;
        tracking_line_seen = 0U;
        tracking_lost_elapsed_sec = 0.0f;
        tracking_lost_active = 0U;
        tracking_lost_search_dir = 0.0f;
        tracking_last_turn_rpm = 0.0f;
        command->turn_rpm = 0.0f;
        command->laser_enabled = 0U;

        if (sensor_fresh != 0U) {
            command->velocity_mps = TRACKING_START_SEARCH_VELOCITY_MPS;
            command->bias_deg = TRACKING_FORWARD_BIAS_DEG;
            command->drive_enabled = 1U;
        } else {
            command->velocity_mps = 0.0f;
            command->bias_deg = BALANCE_MODE_STAND_BIAS_DEG;
            command->drive_enabled = 0U;
        }

        tracking_last_velocity_mps = command->velocity_mps;
        tracking_last_bias_deg = command->bias_deg;
        return;
    }

    if (tracking_block_state == TRACKING_BLOCK_TARGET_RUNOUT) {
        tracking_target_runout_sec += dt_sec;
        if (tracking_target_runout_sec >=
            (0.001f * (float)TRACKING_TARGET_BLOCK_RUNOUT_MS)) {
            tracking_block_state = TRACKING_BLOCK_TARGET_STOPPED;
            tracking_target_laser_sec = 0.0f;
        } else {
            command->velocity_mps = TRACKING_TARGET_BLOCK_RUNOUT_VELOCITY_MPS;
            command->bias_deg = TRACKING_FORWARD_BIAS_DEG;
            command->turn_rpm = 0.0f;
            command->drive_enabled = 1U;
            tracking_last_velocity_mps = command->velocity_mps;
            tracking_last_bias_deg = command->bias_deg;
            return;
        }
    }

    if (tracking_block_state == TRACKING_BLOCK_TARGET_STOPPED) {
        tracking_target_laser_sec += dt_sec;
        if (tracking_target_laser_sec >=
            (0.001f * (float)TRACKING_TARGET_BLOCK_LASER_MS)) {
            tracking_block_state = TRACKING_BLOCK_TARGET_WAIT_EXIT;
            tracking_block_clear_sec = 0.0f;
            tracking_ready = 0U;
            tracking_integral = 0.0f;
            tracking_lost_elapsed_sec = 0.0f;
            tracking_lost_active = 0U;
            tracking_lost_search_dir = 0.0f;
        } else {
            tracking_last_velocity_mps = 0.0f;
            tracking_last_bias_deg = BALANCE_MODE_STAND_BIAS_DEG;
            tracking_last_turn_rpm = 0.0f;
            tracking_lost_elapsed_sec = 0.0f;
            tracking_lost_active = 0U;
            tracking_lost_search_dir = 0.0f;
            command->velocity_mps = 0.0f;
            command->bias_deg = BALANCE_MODE_STAND_BIAS_DEG;
            command->turn_rpm = 0.0f;
            command->drive_enabled = 0U;
            command->laser_enabled = 1U;
            return;
        }
    }

    if (tracking_block_state == TRACKING_BLOCK_FINISH_RUNOUT) {
        tracking_finish_runout_sec += dt_sec;
        if (tracking_finish_runout_sec >=
            (0.001f * (float)TRACKING_FINISH_BLOCK_RUNOUT_MS)) {
            tracking_block_state = TRACKING_BLOCK_FINISH_DONE;
            tracking_ready = 0U;
            tracking_integral = 0.0f;
            tracking_lost_elapsed_sec = 0.0f;
            tracking_lost_active = 0U;
            tracking_lost_search_dir = 0.0f;
        } else {
            command->velocity_mps =
                TRACKING_FINISH_BLOCK_RUNOUT_VELOCITY_MPS;
            command->bias_deg = TRACKING_FORWARD_BIAS_DEG;
            command->turn_rpm = 0.0f;
            command->drive_enabled = 1U;
            tracking_last_velocity_mps = command->velocity_mps;
            tracking_last_bias_deg = command->bias_deg;
            return;
        }
    }

    if (tracking_block_state == TRACKING_BLOCK_FINISH_DONE) {
        tracking_last_velocity_mps = 0.0f;
        tracking_last_bias_deg = BALANCE_MODE_STAND_BIAS_DEG;
        tracking_last_turn_rpm = 0.0f;
        command->velocity_mps = 0.0f;
        command->bias_deg = BALANCE_MODE_STAND_BIAS_DEG;
        command->turn_rpm = 0.0f;
        command->drive_enabled = 0U;
        command->laser_enabled = 0U;
        return;
    }

    if (sensor_fresh == 0U) {
        tracking_lost_elapsed_sec = 0.0f;
        tracking_lost_active = 0U;
        tracking_lost_search_dir = 0.0f;
        tracking_last_turn_rpm = 0.0f;
        command->velocity_mps = 0.0f;
        command->bias_deg = BALANCE_MODE_STAND_BIAS_DEG;
        command->turn_rpm = 0.0f;
        command->drive_enabled = 0U;
        return;
    }

    if (error == TRACK_LOST_LINE) {
        float lost_hold_sec = 0.001f * (float)TRACKING_LOST_HOLD_MS;
        float lost_refind_sec =
            lost_hold_sec + 0.001f * (float)TRACKING_LOST_REFIND_MS;

        tracking_begin_lost_search();
        tracking_lost_elapsed_sec += dt_sec;

        if ((tracking_line_seen != 0U) &&
            (tracking_lost_elapsed_sec <= lost_hold_sec)) {
            float hold_turn_rpm =
                tracking_build_lost_turn(TRACKING_LOST_HOLD_TURN_RPM);

            command->turn_rpm = hold_turn_rpm;
            tracking_last_turn_rpm = hold_turn_rpm;
            command->velocity_mps = TRACKING_LOST_HOLD_VELOCITY_MPS;
            command->bias_deg = BALANCE_MODE_STAND_BIAS_DEG;
        } else if ((tracking_line_seen != 0U) &&
                   (tracking_lost_elapsed_sec <= lost_refind_sec)) {
            float refind_turn_rpm =
                tracking_build_lost_turn(TRACKING_LOST_REFIND_TURN_RPM);

            tracking_ready = 0U;
            tracking_integral = 0.0f;
            tracking_sharp_turn_active = 1U;
            command->turn_rpm = refind_turn_rpm;
            tracking_last_turn_rpm = refind_turn_rpm;
            command->velocity_mps = TRACKING_LOST_REFIND_VELOCITY_MPS;
            command->bias_deg = BALANCE_MODE_STAND_BIAS_DEG;
        } else {
            float search_turn_rpm =
                tracking_build_lost_turn(TRACKING_LOST_SEARCH_TURN_RPM);

            tracking_ready = 0U;
            tracking_integral = 0.0f;
            tracking_sharp_turn_active = 0U;
            command->turn_rpm = search_turn_rpm;
            tracking_last_turn_rpm = search_turn_rpm;
            command->velocity_mps = TRACKING_LOST_VELOCITY_MPS;
            command->bias_deg = BALANCE_MODE_STAND_BIAS_DEG;
        }
        command->drive_enabled =
            (mode_absf(command->velocity_mps) > 0.001f) ? 1U : 0U;
        return;
    }

    tracking_lost_elapsed_sec = 0.0f;
    tracking_line_seen = 1U;
    tracking_lost_active = 0U;
    tracking_lost_search_dir = 0.0f;

    if (tracking_ready == 0U) {
        tracking_filtered_error = error;
        tracking_last_error = error;
        tracking_integral = 0.0f;
        tracking_last_turn_rpm = 0.0f;
        tracking_ready = 1U;
    } else {
        tracking_filtered_error =
            TRACKING_ERROR_LPF_ALPHA * tracking_filtered_error +
            (1.0f - TRACKING_ERROR_LPF_ALPHA) * error;
    }

    abs_error = mode_absf(tracking_filtered_error);
    if (abs_error <= TRACKING_INTEGRAL_ZONE_ERROR) {
        tracking_integral += tracking_filtered_error * dt_sec;
        tracking_integral =
            mode_limit_abs(tracking_integral,
                           TRACKING_INTEGRAL_LIMIT_ERROR_S);
    } else {
        tracking_integral *= TRACKING_INTEGRAL_DECAY;
    }

    derivative = (tracking_filtered_error - tracking_last_error) / dt_sec;
    tracking_last_error = tracking_filtered_error;

    turn_rpm =
        TRACKING_TURN_KP_RPM_PER_ERROR * tracking_filtered_error +
        TRACKING_TURN_KI_RPM_PER_ERROR_S * tracking_integral +
        TRACKING_TURN_KD_RPM_PER_ERROR_RATE * derivative;
    turn_rpm = mode_limit_abs(turn_rpm, TRACKING_TURN_LIMIT_RPM);
    tracking_last_turn_rpm = turn_rpm;
    if (mode_absf(turn_rpm) >= TRACKING_TURN_MEMORY_MIN_RPM) {
        tracking_last_turn_dir = (turn_rpm >= 0.0f) ? 1.0f : -1.0f;
    }

    if ((abs_error >= TRACKING_SHARP_TURN_ENTER_ERROR) ||
        (mode_absf(turn_rpm) >= TRACKING_SHARP_TURN_GATE_RPM)) {
        tracking_sharp_turn_active = 1U;
    } else if (abs_error <= TRACKING_SHARP_TURN_EXIT_ERROR) {
        tracking_sharp_turn_active = 0U;
    }

    if (abs_error > TRACKING_CURVE_SLOWDOWN_ERROR) {
        float span = TRACKING_LOST_ERROR - TRACKING_CURVE_SLOWDOWN_ERROR;
        float norm = (span > 0.001f) ?
                     ((abs_error - TRACKING_CURVE_SLOWDOWN_ERROR) / span) :
                     1.0f;
        if (norm > 1.0f) {
            norm = 1.0f;
        }
        speed_scale =
            1.0f - norm * norm * (1.0f - TRACKING_MIN_SPEED_RATIO);
        if (speed_scale < TRACKING_MIN_SPEED_RATIO) {
            speed_scale = TRACKING_MIN_SPEED_RATIO;
        }
    }

    command->velocity_mps = TRACKING_BASE_VELOCITY_MPS * speed_scale;
    if ((tracking_sharp_turn_active != 0U) &&
        (command->velocity_mps > TRACKING_SHARP_TURN_VELOCITY_MPS)) {
        command->velocity_mps = TRACKING_SHARP_TURN_VELOCITY_MPS;
    }
    command->bias_deg = TRACKING_FORWARD_BIAS_DEG;
    command->turn_rpm = turn_rpm;
    command->drive_enabled = 1U;
    tracking_last_velocity_mps = command->velocity_mps;
    tracking_last_bias_deg = command->bias_deg;
}
