#include "mode_private.h"

#include "config.h"
#include "track.h"

static float tracking_filtered_error = 0.0f;
static float tracking_last_error = 0.0f;
static float tracking_integral = 0.0f;
static float tracking_last_turn_rpm = 0.0f;
static float tracking_last_turn_dir = 0.0f;
static float tracking_lost_search_dir = 0.0f;
static float tracking_last_velocity_mps = 0.0f;
static float tracking_last_bias_deg = BALANCE_MODE_STAND_BIAS_DEG;
static float tracking_lost_elapsed_sec = 0.0f;
static float tracking_target_runout_sec = 0.0f;
static float tracking_target_laser_sec = 0.0f;
static float tracking_target_depart_sec = 0.0f;
static float tracking_finish_runout_sec = 0.0f;
static float tracking_sensor_clock_sec = 0.0f;
static float tracking_last_frame_clock_sec = 0.0f;
static float tracking_condition_start_sec = 0.0f;
static float tracking_refind_start_sec = 0.0f;
static volatile ModeTrackingPhase_e tracking_phase =
    MODE_TRACKING_PHASE_WAIT_START_MARKER;
static uint8_t tracking_ready = 0U;
static uint8_t tracking_line_seen = 0U;
static uint8_t tracking_lost_active = 0U;
static uint8_t tracking_sharp_turn_active = 0U;
static uint8_t tracking_condition_active = 0U;
static uint8_t tracking_refind_active = 0U;
static uint8_t tracking_refind_required = 0U;
static uint8_t tracking_refind_requires_line = 0U;

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

static void tracking_build_motion(ModeCommand_t *command,
                                  float velocity_mps,
                                  float bias_deg,
                                  float turn_rpm)
{
    Mode_MotionBuild(command, velocity_mps, bias_deg, turn_rpm);
    command->tracking_active = 1U;
}

static void tracking_build_hold(ModeCommand_t *command, uint8_t laser_enabled)
{
    Mode_StopBuild(command);
    command->tracking_active = 1U;
    command->laser_enabled = (laser_enabled != 0U) ? 1U : 0U;
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
    tracking_refind_active = 0U;
}

static uint8_t tracking_refind_confirmed(uint8_t sensor_new_frame)
{
    if (TRACKING_REFIND_CONFIRM_MS == 0U) {
        tracking_refind_active = 0U;
        return 1U;
    }
    if (sensor_new_frame == 0U) {
        return 0U;
    }
    if (tracking_refind_active == 0U) {
        tracking_refind_active = 1U;
        tracking_refind_start_sec = tracking_sensor_clock_sec;
        return 0U;
    }
    if ((tracking_sensor_clock_sec - tracking_refind_start_sec) <
        (0.001f * (float)TRACKING_REFIND_CONFIRM_MS)) {
        return 0U;
    }

    tracking_refind_active = 0U;
    return 1U;
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

static void tracking_reset_condition(void)
{
    tracking_condition_start_sec = tracking_sensor_clock_sec;
    tracking_condition_active = 0U;
}

static uint8_t tracking_condition_confirmed(uint8_t condition,
                                            uint32_t confirm_ms)
{
    if (condition == 0U) {
        tracking_reset_condition();
        return 0U;
    }

    if (confirm_ms == 0U) {
        return 1U;
    }
    if (tracking_condition_active == 0U) {
        tracking_condition_active = 1U;
        tracking_condition_start_sec = tracking_sensor_clock_sec;
        return 0U;
    }

    return ((tracking_sensor_clock_sec - tracking_condition_start_sec) >=
            (0.001f * (float)confirm_ms)) ? 1U : 0U;
}

static void tracking_reset_sensor_history(void)
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
    tracking_last_frame_clock_sec = 0.0f;
    tracking_ready = 0U;
    tracking_line_seen = 0U;
    tracking_lost_active = 0U;
    tracking_sharp_turn_active = 0U;
    tracking_refind_active = 0U;
}

static void tracking_update_block_state(uint8_t wide_block,
                                        uint8_t target_block,
                                        uint8_t forward_motion)
{
    switch (tracking_phase) {
    case MODE_TRACKING_PHASE_WAIT_START_MARKER:
        if (tracking_condition_confirmed(
                ((wide_block != 0U) && (forward_motion != 0U)) ? 1U : 0U,
                TRACKING_START_BLOCK_CONFIRM_MS) != 0U) {
            tracking_phase = MODE_TRACKING_PHASE_WAIT_START_EXIT;
            tracking_reset_condition();
        }
        break;

    case MODE_TRACKING_PHASE_WAIT_START_EXIT:
        if (tracking_condition_confirmed(
                ((wide_block == 0U) && (forward_motion != 0U)) ? 1U : 0U,
                TRACKING_START_BLOCK_EXIT_MS) != 0U) {
            tracking_phase = MODE_TRACKING_PHASE_FOLLOW_TO_TARGET;
            tracking_reset_condition();
        }
        break;

    case MODE_TRACKING_PHASE_FOLLOW_TO_TARGET:
        if (tracking_condition_confirmed(
                ((target_block != 0U) && (forward_motion != 0U)) ? 1U : 0U,
                TRACKING_TARGET_BLOCK_CONFIRM_MS) != 0U) {
            tracking_phase = MODE_TRACKING_PHASE_TARGET_RUNOUT;
            tracking_target_runout_sec = 0.0f;
            tracking_reset_condition();
        }
        break;

    case MODE_TRACKING_PHASE_WAIT_TARGET_EXIT:
        if (tracking_condition_confirmed(
                ((target_block == 0U) && (forward_motion != 0U)) ? 1U : 0U,
                TRACKING_TARGET_BLOCK_EXIT_MS) != 0U) {
            tracking_phase = MODE_TRACKING_PHASE_FOLLOW_TO_FINISH;
            tracking_reset_condition();
        }
        break;

    case MODE_TRACKING_PHASE_FOLLOW_TO_FINISH:
        if (tracking_condition_confirmed(
                ((wide_block != 0U) && (forward_motion != 0U)) ? 1U : 0U,
                TRACKING_FINISH_BLOCK_CONFIRM_MS) != 0U) {
            tracking_phase = MODE_TRACKING_PHASE_FINISH_RUNOUT;
            tracking_finish_runout_sec = 0.0f;
            tracking_reset_condition();
        }
        break;

    case MODE_TRACKING_PHASE_TARGET_RUNOUT:
    case MODE_TRACKING_PHASE_TARGET_WAIT_SETTLE:
    case MODE_TRACKING_PHASE_TARGET_LASER_DWELL:
    case MODE_TRACKING_PHASE_TARGET_DEPART:
    case MODE_TRACKING_PHASE_FINISH_RUNOUT:
    case MODE_TRACKING_PHASE_FINISHED:
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
    tracking_target_runout_sec = 0.0f;
    tracking_target_laser_sec = 0.0f;
    tracking_target_depart_sec = 0.0f;
    tracking_finish_runout_sec = 0.0f;
    tracking_sensor_clock_sec = 0.0f;
    tracking_last_frame_clock_sec = 0.0f;
    tracking_condition_start_sec = 0.0f;
    tracking_refind_start_sec = 0.0f;
    tracking_phase = MODE_TRACKING_PHASE_WAIT_START_MARKER;
    tracking_ready = 0U;
    tracking_line_seen = 0U;
    tracking_lost_active = 0U;
    tracking_sharp_turn_active = 0U;
    tracking_condition_active = 0U;
    tracking_refind_active = 0U;
    /* Entering tracking must always span the refind window on new frames.
     * Do not rely on Mode_Update observing a stale cycle before the first
     * fresh frame, because the producer can publish between Set and Update. */
    tracking_refind_required = 1U;
    tracking_refind_requires_line = 0U;
}

ModeTrackingPhase_e Mode_TrackingGetPhase(void)
{
    return tracking_phase;
}

void Mode_TrackingBuild(ModeCommand_t *command,
                        float error,
                        uint8_t black_count,
                        uint8_t sensor_flags,
                        uint8_t motion_settled,
                        float actual_velocity_mps,
                        float dt_sec)
{
    float derivative;
    float frame_dt_sec;
    float abs_error;
    float speed_scale = 1.0f;
    float turn_rpm;
    uint8_t wide_block;
    uint8_t target_block;
    uint8_t sensor_fresh;
    uint8_t sensor_new_frame;
    uint8_t forward_motion;

    tracking_build_hold(command, 0U);

    if (dt_sec <= 0.0f) {
        dt_sec = BALANCE_PID_DT_SEC;
    }
    frame_dt_sec = dt_sec;

    sensor_new_frame =
        ((sensor_flags & MODE_TRACKING_SENSOR_NEW_FRAME) != 0U) ? 1U : 0U;
    sensor_fresh =
        ((sensor_flags & MODE_TRACKING_SENSOR_FRESH) != 0U) ? 1U : 0U;
    forward_motion =
        (actual_velocity_mps >= TRACKING_MARKER_MIN_FORWARD_MPS) ? 1U : 0U;

    if (sensor_fresh == 0U) {
        if (tracking_phase == MODE_TRACKING_PHASE_TARGET_LASER_DWELL) {
            tracking_phase = MODE_TRACKING_PHASE_TARGET_WAIT_SETTLE;
            tracking_target_laser_sec = 0.0f;
        }
        tracking_reset_sensor_history();
        tracking_reset_condition();
        tracking_refind_required = 1U;
        tracking_refind_requires_line = 0U;
        return;
    }

    tracking_sensor_clock_sec += dt_sec;
    if (sensor_new_frame != 0U) {
        if (tracking_last_frame_clock_sec > 0.0f) {
            frame_dt_sec = tracking_sensor_clock_sec -
                           tracking_last_frame_clock_sec;
            if ((frame_dt_sec <= 0.0f) || (frame_dt_sec > 0.1f)) {
                frame_dt_sec = dt_sec;
            }
        }
        tracking_last_frame_clock_sec = tracking_sensor_clock_sec;
    }

    if ((tracking_lost_active != 0U) &&
        (error != TRACK_LOST_LINE) &&
        (tracking_refind_required == 0U)) {
        tracking_refind_required = 1U;
        tracking_refind_requires_line = 1U;
        tracking_refind_active = 0U;
    }
    if (tracking_refind_required != 0U) {
        if ((tracking_refind_requires_line != 0U) &&
            (error == TRACK_LOST_LINE)) {
            tracking_refind_active = 0U;
            return;
        }
        if (tracking_refind_confirmed(sensor_new_frame) == 0U) {
            return;
        }
        if (tracking_refind_requires_line != 0U) {
            tracking_lost_active = 0U;
            tracking_lost_elapsed_sec = 0.0f;
        }
        tracking_refind_required = 0U;
        tracking_refind_requires_line = 0U;
    }

    wide_block = (black_count >= TRACKING_BLOCK_BLACK_COUNT_MIN) ? 1U : 0U;
    target_block =
        (black_count >= TRACKING_TARGET_BLOCK_BLACK_COUNT_MIN) ? 1U : 0U;
    if (sensor_new_frame != 0U) {
        tracking_update_block_state(wide_block, target_block,
                                    forward_motion);
    }

    if (tracking_phase == MODE_TRACKING_PHASE_WAIT_START_MARKER) {
        tracking_ready = 0U;
        tracking_line_seen = 0U;
        tracking_lost_elapsed_sec = 0.0f;
        tracking_lost_active = 0U;
        tracking_lost_search_dir = 0.0f;
        tracking_last_turn_rpm = 0.0f;
        tracking_build_motion(command,
                              TRACKING_START_SEARCH_VELOCITY_MPS,
                              TRACKING_FORWARD_BIAS_DEG,
                              0.0f);

        tracking_last_velocity_mps = command->velocity_mps;
        tracking_last_bias_deg = command->bias_deg;
        return;
    }

    if (tracking_phase == MODE_TRACKING_PHASE_TARGET_RUNOUT) {
        if (actual_velocity_mps < -TRACKING_MARKER_MIN_FORWARD_MPS) {
            tracking_phase = MODE_TRACKING_PHASE_FOLLOW_TO_TARGET;
            tracking_target_runout_sec = 0.0f;
            tracking_reset_condition();
        } else if (forward_motion != 0U) {
            tracking_target_runout_sec += dt_sec;
        }
        if (tracking_target_runout_sec >=
            (0.001f * (float)TRACKING_TARGET_BLOCK_RUNOUT_MS)) {
            tracking_phase = MODE_TRACKING_PHASE_TARGET_WAIT_SETTLE;
            tracking_target_laser_sec = 0.0f;
        } else {
            tracking_build_motion(
                command,
                TRACKING_TARGET_BLOCK_RUNOUT_VELOCITY_MPS,
                TRACKING_FORWARD_BIAS_DEG,
                0.0f);
            tracking_last_velocity_mps = command->velocity_mps;
            tracking_last_bias_deg = command->bias_deg;
            return;
        }
    }

    if (tracking_phase == MODE_TRACKING_PHASE_TARGET_WAIT_SETTLE) {
        tracking_build_hold(command, 0U);
        tracking_target_laser_sec = 0.0f;
        if (motion_settled == 0U) {
            return;
        }
        tracking_phase = MODE_TRACKING_PHASE_TARGET_LASER_DWELL;
    }

    if (tracking_phase == MODE_TRACKING_PHASE_TARGET_LASER_DWELL) {
        if (motion_settled == 0U) {
            tracking_phase = MODE_TRACKING_PHASE_TARGET_WAIT_SETTLE;
            tracking_target_laser_sec = 0.0f;
            tracking_build_hold(command, 0U);
            return;
        }

        if (tracking_target_laser_sec >=
            (0.001f * (float)TRACKING_TARGET_BLOCK_LASER_MS)) {
            tracking_phase = MODE_TRACKING_PHASE_TARGET_DEPART;
            tracking_target_depart_sec = 0.0f;
            tracking_reset_condition();
            tracking_ready = 0U;
            tracking_integral = 0.0f;
            tracking_lost_elapsed_sec = 0.0f;
            tracking_lost_active = 0U;
            tracking_lost_search_dir = 0.0f;
        } else {
            tracking_target_laser_sec += dt_sec;
            tracking_last_velocity_mps = 0.0f;
            tracking_last_bias_deg = BALANCE_MODE_STAND_BIAS_DEG;
            tracking_last_turn_rpm = 0.0f;
            tracking_lost_elapsed_sec = 0.0f;
            tracking_lost_active = 0U;
            tracking_lost_search_dir = 0.0f;
            tracking_build_hold(command, 1U);
            return;
        }
    }

    if (tracking_phase == MODE_TRACKING_PHASE_TARGET_DEPART) {
        if (forward_motion != 0U) {
            tracking_target_depart_sec += dt_sec;
        }
        if (tracking_target_depart_sec >=
            (0.001f * (float)TRACKING_TARGET_BLOCK_DEPART_MS)) {
            tracking_phase = MODE_TRACKING_PHASE_WAIT_TARGET_EXIT;
            tracking_reset_condition();
        }
    }

    if (tracking_phase == MODE_TRACKING_PHASE_FINISH_RUNOUT) {
        if (actual_velocity_mps < -TRACKING_MARKER_MIN_FORWARD_MPS) {
            tracking_phase = MODE_TRACKING_PHASE_FOLLOW_TO_FINISH;
            tracking_finish_runout_sec = 0.0f;
            tracking_reset_condition();
        } else if (forward_motion != 0U) {
            tracking_finish_runout_sec += dt_sec;
        }
        if (tracking_finish_runout_sec >=
            (0.001f * (float)TRACKING_FINISH_BLOCK_RUNOUT_MS)) {
            tracking_phase = MODE_TRACKING_PHASE_FINISHED;
            tracking_ready = 0U;
            tracking_integral = 0.0f;
            tracking_lost_elapsed_sec = 0.0f;
            tracking_lost_active = 0U;
            tracking_lost_search_dir = 0.0f;
        } else {
            tracking_build_motion(
                command,
                TRACKING_FINISH_BLOCK_RUNOUT_VELOCITY_MPS,
                TRACKING_FORWARD_BIAS_DEG,
                0.0f);
            tracking_last_velocity_mps = command->velocity_mps;
            tracking_last_bias_deg = command->bias_deg;
            return;
        }
    }

    if (tracking_phase == MODE_TRACKING_PHASE_FINISHED) {
        tracking_last_velocity_mps = 0.0f;
        tracking_last_bias_deg = BALANCE_MODE_STAND_BIAS_DEG;
        tracking_last_turn_rpm = 0.0f;
        tracking_build_hold(command, 0U);
        return;
    }

    if ((error != TRACK_LOST_LINE) && (sensor_new_frame == 0U)) {
        tracking_build_motion(command,
                              tracking_last_velocity_mps,
                              tracking_last_bias_deg,
                              tracking_last_turn_rpm);
        return;
    }

    if (error == TRACK_LOST_LINE) {
        float lost_hold_sec = 0.001f * (float)TRACKING_LOST_HOLD_MS;
        float lost_refind_sec =
            lost_hold_sec + 0.001f * (float)TRACKING_LOST_REFIND_MS;
        float lost_velocity_mps;
        float lost_turn_rpm;

        tracking_begin_lost_search();
        tracking_lost_elapsed_sec += dt_sec;

        if ((tracking_line_seen != 0U) &&
            (tracking_lost_elapsed_sec <= lost_hold_sec)) {
            lost_turn_rpm =
                tracking_build_lost_turn(TRACKING_LOST_HOLD_TURN_RPM);
            lost_velocity_mps = TRACKING_LOST_HOLD_VELOCITY_MPS;
        } else if ((tracking_line_seen != 0U) &&
                   (tracking_lost_elapsed_sec <= lost_refind_sec)) {
            tracking_ready = 0U;
            tracking_integral = 0.0f;
            tracking_sharp_turn_active = 1U;
            lost_turn_rpm =
                tracking_build_lost_turn(TRACKING_LOST_REFIND_TURN_RPM);
            lost_velocity_mps = TRACKING_LOST_REFIND_VELOCITY_MPS;
        } else {
            tracking_ready = 0U;
            tracking_integral = 0.0f;
            tracking_sharp_turn_active = 0U;
            lost_turn_rpm =
                tracking_build_lost_turn(TRACKING_LOST_SEARCH_TURN_RPM);
            lost_velocity_mps = TRACKING_LOST_VELOCITY_MPS;
        }
        tracking_build_motion(command,
                              lost_velocity_mps,
                              BALANCE_MODE_STAND_BIAS_DEG,
                              lost_turn_rpm);
        tracking_last_turn_rpm = lost_turn_rpm;
        tracking_last_velocity_mps = command->velocity_mps;
        tracking_last_bias_deg = command->bias_deg;
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
        tracking_integral += tracking_filtered_error * frame_dt_sec;
        tracking_integral =
            mode_limit_abs(tracking_integral,
                           TRACKING_INTEGRAL_LIMIT_ERROR_S);
    } else {
        tracking_integral *= TRACKING_INTEGRAL_DECAY;
    }

    derivative = (tracking_filtered_error - tracking_last_error) /
                 frame_dt_sec;
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

    tracking_last_velocity_mps = TRACKING_BASE_VELOCITY_MPS * speed_scale;
    if ((tracking_sharp_turn_active != 0U) &&
        (tracking_last_velocity_mps > TRACKING_SHARP_TURN_VELOCITY_MPS)) {
        tracking_last_velocity_mps = TRACKING_SHARP_TURN_VELOCITY_MPS;
    }
    tracking_build_motion(command,
                          tracking_last_velocity_mps,
                          TRACKING_FORWARD_BIAS_DEG,
                          turn_rpm);
    tracking_last_bias_deg = TRACKING_FORWARD_BIAS_DEG;
}
