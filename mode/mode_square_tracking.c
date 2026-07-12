#include "mode_private.h"

#include "config.h"
#include "track.h"

static volatile ModeSquarePhase_e square_phase = MODE_SQUARE_PHASE_WAIT_LINE;
static volatile uint8_t square_corner_count = 0U;
static float square_sensor_clock_sec = 0.0f;
static float square_phase_elapsed_sec = 0.0f;
static float square_condition_start_sec = 0.0f;
static float square_refind_start_sec = 0.0f;
static float square_approach_start_sec = 0.0f;
static float square_last_new_frame_clock_sec = 0.0f;
static float square_filtered_error = 0.0f;
static float square_last_turn_rpm = 0.0f;
static float square_last_velocity_mps = 0.0f;
static float square_turn_start_yaw_deg = 0.0f;
static float square_turn_yaw_progress_deg = 0.0f;
static uint8_t square_condition_active = 0U;
static uint8_t square_refind_active = 0U;
static uint8_t square_approach_active = 0U;
static uint8_t square_approach_confirmed = 0U;
static uint8_t square_filter_ready = 0U;
static uint8_t square_corner_armed = 1U;
static uint8_t square_lost_corner_candidate = 0U;

static uint8_t square_bit_count(uint8_t value)
{
    uint8_t count = 0U;

    while (value != 0U) {
        count += (uint8_t)(value & 1U);
        value >>= 1U;
    }
    return count;
}

static uint8_t square_corner_pattern(uint8_t black_mask)
{
    uint8_t low_side_black = square_bit_count(
        (uint8_t)(black_mask & 0x0FU));
    uint8_t high_side_black = square_bit_count(
        (uint8_t)(black_mask & 0xF0U));

    return (((low_side_black >= SQUARE_TRACKING_CORNER_SIDE_BLACK_MIN) &&
             (high_side_black <= SQUARE_TRACKING_CORNER_OTHER_BLACK_MAX)) ||
            ((high_side_black >= SQUARE_TRACKING_CORNER_SIDE_BLACK_MIN) &&
             (low_side_black <= SQUARE_TRACKING_CORNER_OTHER_BLACK_MAX))) ?
           1U : 0U;
}

static float square_absf(float value)
{
    return (value >= 0.0f) ? value : -value;
}

static float square_wrap_delta_deg(float delta_deg)
{
    while (delta_deg > 180.0f) {
        delta_deg -= 360.0f;
    }
    while (delta_deg < -180.0f) {
        delta_deg += 360.0f;
    }
    return delta_deg;
}

static void square_capture_turn_yaw(float chassis_yaw_deg)
{
    square_turn_start_yaw_deg = chassis_yaw_deg;
    square_turn_yaw_progress_deg = 0.0f;
}

static void square_update_turn_yaw(float chassis_yaw_deg)
{
    square_turn_yaw_progress_deg = square_absf(square_wrap_delta_deg(
        chassis_yaw_deg - square_turn_start_yaw_deg));
}

static float square_limit_abs(float value, float limit)
{
    if (value > limit) {
        return limit;
    }
    if (value < -limit) {
        return -limit;
    }
    return value;
}

static float square_apply_continuous_deadband(float value, float deadband)
{
    float abs_deadband = square_absf(deadband);

    if (value > abs_deadband) {
        return value - abs_deadband;
    }
    if (value < -abs_deadband) {
        return value + abs_deadband;
    }
    return 0.0f;
}

static void square_build_motion(ModeCommand_t *command,
                                float velocity_mps,
                                float turn_rpm)
{
    Mode_MotionBuild(command,
                     velocity_mps,
                     SQUARE_TRACKING_FORWARD_BIAS_DEG,
                     turn_rpm);
    command->tracking_active = 1U;
}

static void square_build_hold(ModeCommand_t *command)
{
    Mode_StopBuild(command);
    command->tracking_active = 1U;
}

static void square_build_fixed_turn(ModeCommand_t *command)
{
    if (SQUARE_TRACKING_TURN_DIRECTION >= 0.0f) {
        Mode_TurnLeftBuild(command);
    } else {
        Mode_TurnRightBuild(command);
    }
    command->tracking_active = 1U;
}

static void square_build_corner_search_turn(ModeCommand_t *command)
{
    float search_rpm = square_absf(SQUARE_TRACKING_CORNER_SEARCH_RPM);

    if (SQUARE_TRACKING_TURN_DIRECTION < 0.0f) {
        search_rpm = -search_rpm;
    }
    Mode_MotionBuild(command,
                     0.0f,
                     BALANCE_MODE_STAND_BIAS_DEG,
                     search_rpm);
    command->tracking_active = 1U;
}

static void square_reset_condition(void)
{
    square_condition_active = 0U;
    square_condition_start_sec = square_sensor_clock_sec;
}

static uint8_t square_condition_confirmed(uint8_t condition,
                                          uint32_t confirm_ms,
                                          uint8_t sensor_new_frame)
{
    if (condition == 0U) {
        square_reset_condition();
        return 0U;
    }
    if (sensor_new_frame == 0U) {
        return 0U;
    }
    if (confirm_ms == 0U) {
        return 1U;
    }
    if (square_condition_active == 0U) {
        square_condition_active = 1U;
        square_condition_start_sec = square_sensor_clock_sec;
        return 0U;
    }
    return ((square_sensor_clock_sec - square_condition_start_sec) >=
            (0.001f * (float)confirm_ms)) ? 1U : 0U;
}

static void square_reset_refind(void)
{
    square_refind_active = 0U;
    square_refind_start_sec = square_sensor_clock_sec;
}

static uint8_t square_refind_confirmed(uint8_t line_valid,
                                       uint8_t sensor_new_frame)
{
    if (line_valid == 0U) {
        square_reset_refind();
        return 0U;
    }
    if (sensor_new_frame == 0U) {
        return 0U;
    }
    if (SQUARE_TRACKING_REFIND_CONFIRM_MS == 0U) {
        square_refind_active = 0U;
        return 1U;
    }
    if (square_refind_active == 0U) {
        square_refind_active = 1U;
        square_refind_start_sec = square_sensor_clock_sec;
        return 0U;
    }
    if ((square_sensor_clock_sec - square_refind_start_sec) <
        (0.001f * (float)SQUARE_TRACKING_REFIND_CONFIRM_MS)) {
        return 0U;
    }
    square_refind_active = 0U;
    return 1U;
}

static void square_reset_corner_approach(void)
{
    square_approach_active = 0U;
    square_approach_confirmed = 0U;
    square_approach_start_sec = square_sensor_clock_sec;
}

static void square_update_corner_approach(uint8_t black_mask,
                                          float actual_velocity_mps,
                                          uint8_t sensor_new_frame)
{
    uint8_t approach_seen =
        ((square_corner_armed != 0U) &&
         (actual_velocity_mps >=
          -SQUARE_TRACKING_CORNER_REVERSE_REJECT_MPS) &&
         (square_corner_pattern(black_mask) != 0U)) ? 1U : 0U;

    if (approach_seen == 0U) {
        square_reset_corner_approach();
        return;
    }
    if (sensor_new_frame == 0U) {
        return;
    }
    if (square_approach_active == 0U) {
        square_approach_active = 1U;
        square_approach_start_sec = square_sensor_clock_sec;
        return;
    }
    if ((square_sensor_clock_sec - square_approach_start_sec) >=
        (0.001f *
         (float)SQUARE_TRACKING_CORNER_CONFIRM_MS)) {
        square_approach_confirmed = 1U;
    }
}

static void square_filter_error(float error)
{
    if (square_filter_ready == 0U) {
        square_filtered_error = error;
        square_filter_ready = 1U;
    } else {
        square_filtered_error =
            SQUARE_TRACKING_ERROR_LPF_ALPHA * square_filtered_error +
            (1.0f - SQUARE_TRACKING_ERROR_LPF_ALPHA) * error;
    }
}

static float square_alignment_turn(void)
{
    return square_limit_abs(
        SQUARE_TRACKING_TURN_KP_RPM_PER_ERROR * square_filtered_error,
        BALANCE_MODE_TURN_SPEED_RPM);
}

static void square_build_follow_command(ModeCommand_t *command, float error)
{
    float abs_error;
    float velocity_mps;
    float turn_rpm;

    square_filter_error(error);
    abs_error = square_absf(square_filtered_error);
    turn_rpm = SQUARE_TRACKING_TURN_KP_RPM_PER_ERROR *
               square_filtered_error;
    turn_rpm = square_apply_continuous_deadband(
        turn_rpm, SQUARE_TRACKING_TURN_DEADBAND_RPM);
    turn_rpm = square_limit_abs(turn_rpm,
                                SQUARE_TRACKING_TURN_LIMIT_RPM);
    velocity_mps =
        (abs_error >= SQUARE_TRACKING_CURVE_SLOWDOWN_ERROR) ?
        SQUARE_TRACKING_SHARP_VELOCITY_MPS :
        SQUARE_TRACKING_BASE_VELOCITY_MPS;
    square_last_turn_rpm = turn_rpm;
    square_last_velocity_mps = velocity_mps;
    square_build_motion(command, velocity_mps, turn_rpm);
}

static void square_build_lost_hold(ModeCommand_t *command)
{
    float hold_turn_rpm = square_limit_abs(
        square_last_turn_rpm,
        SQUARE_TRACKING_LOST_HOLD_TURN_LIMIT_RPM);

    square_build_motion(command,
                        SQUARE_TRACKING_LOST_HOLD_VELOCITY_MPS,
                        hold_turn_rpm);
}

static void square_enter_wait_line(void)
{
    square_phase = MODE_SQUARE_PHASE_WAIT_LINE;
    square_phase_elapsed_sec = 0.0f;
    square_filter_ready = 0U;
    square_last_turn_rpm = 0.0f;
    square_last_velocity_mps = 0.0f;
    square_turn_yaw_progress_deg = 0.0f;
    square_lost_corner_candidate = 0U;
    square_reset_corner_approach();
    square_reset_condition();
    square_reset_refind();
}

static void square_enter_fault(void)
{
    square_phase = MODE_SQUARE_PHASE_FAULT_HOLD;
    square_phase_elapsed_sec = 0.0f;
    square_last_turn_rpm = 0.0f;
    square_last_velocity_mps = 0.0f;
    square_turn_yaw_progress_deg = 0.0f;
    square_reset_corner_approach();
    square_reset_condition();
    square_reset_refind();
}

static void square_complete_alignment(void)
{
    square_corner_count++;
    square_last_turn_rpm = 0.0f;
    square_last_velocity_mps = 0.0f;
    square_turn_start_yaw_deg = 0.0f;
    square_turn_yaw_progress_deg = 0.0f;
    square_filter_ready = 0U;
    square_corner_armed = 0U;
    square_lost_corner_candidate = 0U;
    square_reset_corner_approach();
    square_reset_condition();

    if ((SQUARE_TRACKING_TARGET_CORNER_COUNT != 0U) &&
        (square_corner_count >= SQUARE_TRACKING_TARGET_CORNER_COUNT)) {
        square_phase = MODE_SQUARE_PHASE_FINISHED;
    } else {
        square_phase = MODE_SQUARE_PHASE_FOLLOW;
    }
    square_phase_elapsed_sec = 0.0f;
}

void Mode_SquareTrackingReset(void)
{
    square_phase = MODE_SQUARE_PHASE_WAIT_LINE;
    square_corner_count = 0U;
    square_sensor_clock_sec = 0.0f;
    square_phase_elapsed_sec = 0.0f;
    square_condition_start_sec = 0.0f;
    square_refind_start_sec = 0.0f;
    square_approach_start_sec = 0.0f;
    square_last_new_frame_clock_sec = 0.0f;
    square_filtered_error = 0.0f;
    square_last_turn_rpm = 0.0f;
    square_last_velocity_mps = 0.0f;
    square_turn_start_yaw_deg = 0.0f;
    square_turn_yaw_progress_deg = 0.0f;
    square_condition_active = 0U;
    square_refind_active = 0U;
    square_approach_active = 0U;
    square_approach_confirmed = 0U;
    square_filter_ready = 0U;
    square_corner_armed = 1U;
    square_lost_corner_candidate = 0U;
}

ModeSquarePhase_e Mode_SquareTrackingGetPhase(void)
{
    return square_phase;
}

uint8_t Mode_SquareTrackingGetCornerCount(void)
{
    return square_corner_count;
}

void Mode_SquareTrackingBuild(ModeCommand_t *command,
                              float error,
                              uint8_t black_count,
                              uint8_t black_mask,
                              uint8_t sensor_flags,
                              uint8_t motion_settled,
                              float actual_velocity_mps,
                              float chassis_yaw_deg,
                              float dt_sec)
{
    uint8_t sensor_fresh;
    uint8_t sensor_new_frame;

    (void)black_count;
    square_build_hold(command);
    if (dt_sec <= 0.0f) {
        dt_sec = BALANCE_PID_DT_SEC;
    }

    sensor_fresh =
        ((sensor_flags & MODE_TRACKING_SENSOR_FRESH) != 0U) ? 1U : 0U;
    sensor_new_frame =
        ((sensor_flags & MODE_TRACKING_SENSOR_NEW_FRAME) != 0U) ? 1U : 0U;

    if ((square_phase == MODE_SQUARE_PHASE_FINISHED) ||
        (square_phase == MODE_SQUARE_PHASE_FAULT_HOLD)) {
        return;
    }

    if (sensor_fresh == 0U) {
        if ((square_phase == MODE_SQUARE_PHASE_CORNER_RUNOUT) ||
            (square_phase == MODE_SQUARE_PHASE_CORNER_BRAKE) ||
            (square_phase == MODE_SQUARE_PHASE_CORNER_TURN) ||
            (square_phase == MODE_SQUARE_PHASE_CORNER_ALIGN) ||
            (square_phase == MODE_SQUARE_PHASE_LOST_BRAKE) ||
            (square_phase == MODE_SQUARE_PHASE_LOST_SEARCH)) {
            square_enter_fault();
        } else {
            square_enter_wait_line();
        }
        square_last_new_frame_clock_sec = 0.0f;
        return;
    }

    square_sensor_clock_sec += dt_sec;
    if ((square_last_new_frame_clock_sec > 0.0f) &&
        ((square_sensor_clock_sec - square_last_new_frame_clock_sec) >
         (0.001f * (float)SQUARE_TRACKING_MAX_NEW_FRAME_GAP_MS)) &&
        (sensor_new_frame == 0U)) {
        if ((square_phase == MODE_SQUARE_PHASE_CORNER_RUNOUT) ||
            (square_phase == MODE_SQUARE_PHASE_CORNER_BRAKE) ||
            (square_phase == MODE_SQUARE_PHASE_CORNER_TURN) ||
            (square_phase == MODE_SQUARE_PHASE_CORNER_ALIGN) ||
            (square_phase == MODE_SQUARE_PHASE_LOST_BRAKE) ||
            (square_phase == MODE_SQUARE_PHASE_LOST_SEARCH)) {
            square_enter_fault();
        } else {
            square_enter_wait_line();
        }
        return;
    }
    if (sensor_new_frame != 0U) {
        uint8_t frame_gap_invalid = 0U;

        if ((square_last_new_frame_clock_sec > 0.0f) &&
            ((square_sensor_clock_sec - square_last_new_frame_clock_sec) >
             (0.001f *
              (float)SQUARE_TRACKING_MAX_NEW_FRAME_GAP_MS))) {
            frame_gap_invalid = 1U;
        }
        square_last_new_frame_clock_sec = square_sensor_clock_sec;
        if (frame_gap_invalid != 0U) {
            if ((square_phase == MODE_SQUARE_PHASE_CORNER_RUNOUT) ||
                (square_phase == MODE_SQUARE_PHASE_CORNER_BRAKE) ||
                (square_phase == MODE_SQUARE_PHASE_CORNER_TURN) ||
                (square_phase == MODE_SQUARE_PHASE_CORNER_ALIGN) ||
                (square_phase == MODE_SQUARE_PHASE_LOST_BRAKE) ||
                (square_phase == MODE_SQUARE_PHASE_LOST_SEARCH)) {
                square_enter_fault();
                return;
            }
            square_enter_wait_line();
        }
    }

    if (square_phase == MODE_SQUARE_PHASE_WAIT_LINE) {
        uint8_t valid_narrow_line =
            ((error != TRACK_LOST_LINE) &&
             (square_corner_pattern(black_mask) == 0U)) ?
            1U : 0U;

        if (square_refind_confirmed(valid_narrow_line,
                                    sensor_new_frame) == 0U) {
            return;
        }
        square_phase = MODE_SQUARE_PHASE_FOLLOW;
        square_filter_ready = 0U;
        square_phase_elapsed_sec = 0.0f;
        square_reset_condition();
    }

    if (square_phase == MODE_SQUARE_PHASE_FOLLOW) {
        if (sensor_new_frame == 0U) {
            square_build_motion(command,
                                square_last_velocity_mps,
                                square_last_turn_rpm);
            return;
        }

        if (error == TRACK_LOST_LINE) {
            square_lost_corner_candidate = square_approach_confirmed;
            square_reset_corner_approach();
            square_phase = MODE_SQUARE_PHASE_LOST_HOLD;
            square_phase_elapsed_sec = 0.0f;
            square_reset_refind();
            square_build_lost_hold(command);
            return;
        }

        square_update_corner_approach(black_mask,
                                      actual_velocity_mps,
                                      sensor_new_frame);

        if (square_corner_armed == 0U) {
            if (square_condition_confirmed(
                    (square_corner_pattern(black_mask) == 0U) ?
                    1U : 0U,
                    SQUARE_TRACKING_CORNER_REARM_MS,
                    sensor_new_frame) != 0U) {
                square_corner_armed = 1U;
                square_reset_condition();
            }
        } else if (square_condition_confirmed(
                       ((actual_velocity_mps >=
                         -SQUARE_TRACKING_CORNER_REVERSE_REJECT_MPS) &&
                        (square_corner_pattern(black_mask) != 0U)) ?
                       1U : 0U,
                       SQUARE_TRACKING_CORNER_CONFIRM_MS,
                       sensor_new_frame) != 0U) {
            square_corner_armed = 0U;
            square_phase = MODE_SQUARE_PHASE_CORNER_RUNOUT;
            square_phase_elapsed_sec = 0.0f;
            square_reset_corner_approach();
            square_reset_condition();
            square_last_velocity_mps =
                SQUARE_TRACKING_CORNER_RUNOUT_VELOCITY_MPS;
            square_last_turn_rpm = 0.0f;
            square_build_motion(
                command,
                SQUARE_TRACKING_CORNER_RUNOUT_VELOCITY_MPS,
                0.0f);
            return;
        }

        square_build_follow_command(command, error);
        return;
    }

    if (square_phase == MODE_SQUARE_PHASE_CORNER_RUNOUT) {
        square_phase_elapsed_sec += dt_sec;
        if (actual_velocity_mps <
            -SQUARE_TRACKING_CORNER_REVERSE_REJECT_MPS) {
            square_enter_wait_line();
            square_build_hold(command);
            return;
        }
        if (square_phase_elapsed_sec >=
            (0.001f * (float)SQUARE_TRACKING_CORNER_RUNOUT_MS)) {
            square_phase = MODE_SQUARE_PHASE_CORNER_BRAKE;
            square_phase_elapsed_sec = 0.0f;
            square_build_hold(command);
        } else {
            square_build_motion(
                command,
                SQUARE_TRACKING_CORNER_RUNOUT_VELOCITY_MPS,
                0.0f);
        }
        return;
    }

    if (square_phase == MODE_SQUARE_PHASE_CORNER_BRAKE) {
        square_phase_elapsed_sec += dt_sec;
        if (motion_settled != 0U) {
            square_phase = MODE_SQUARE_PHASE_CORNER_TURN;
            square_phase_elapsed_sec = 0.0f;
            square_capture_turn_yaw(chassis_yaw_deg);
            square_filter_ready = 0U;
            square_reset_condition();
        } else if (square_phase_elapsed_sec >=
                   (0.001f *
                    (float)SQUARE_TRACKING_CORNER_BRAKE_TIMEOUT_MS)) {
            square_enter_fault();
        }
        return;
    }

    if (square_phase == MODE_SQUARE_PHASE_CORNER_TURN) {
        square_phase_elapsed_sec += dt_sec;
        square_update_turn_yaw(chassis_yaw_deg);
        if (square_turn_yaw_progress_deg <
            SQUARE_TRACKING_CORNER_YAW_FULL_TURN_DEG) {
            square_build_fixed_turn(command);
        } else {
            square_build_corner_search_turn(command);
        }

        if (square_phase_elapsed_sec >=
            (0.001f * (float)SQUARE_TRACKING_CORNER_TURN_MAX_MS)) {
            square_enter_fault();
            square_build_hold(command);
            return;
        }
        if ((sensor_new_frame == 0U) ||
            (square_phase_elapsed_sec <
             (0.001f * (float)SQUARE_TRACKING_CORNER_TURN_MIN_MS)) ||
            (square_turn_yaw_progress_deg <
             SQUARE_TRACKING_CORNER_YAW_REACQUIRE_DEG)) {
            return;
        }

        if ((error != TRACK_LOST_LINE) &&
            (square_corner_pattern(black_mask) == 0U)) {
            square_phase = MODE_SQUARE_PHASE_CORNER_ALIGN;
            square_phase_elapsed_sec = 0.0f;
            square_filter_ready = 0U;
            square_reset_condition();
        }
        return;
    }

    if (square_phase == MODE_SQUARE_PHASE_CORNER_ALIGN) {
        float align_turn_rpm;

        square_phase_elapsed_sec += dt_sec;
        if (square_phase_elapsed_sec >=
            (0.001f * (float)SQUARE_TRACKING_CORNER_ALIGN_MAX_MS)) {
            square_enter_fault();
            return;
        }
        if (sensor_new_frame != 0U) {
            if ((error == TRACK_LOST_LINE) ||
                (square_corner_pattern(black_mask) != 0U)) {
                square_filter_ready = 0U;
                square_reset_condition();
                square_build_corner_search_turn(command);
                return;
            }
            square_filter_error(error);
        }
        if (square_filter_ready == 0U) {
            square_build_corner_search_turn(command);
            return;
        }

        align_turn_rpm = square_alignment_turn();
        square_last_turn_rpm = align_turn_rpm;
        square_last_velocity_mps =
            SQUARE_TRACKING_CORNER_ALIGN_VELOCITY_MPS;
        square_build_motion(command,
                            SQUARE_TRACKING_CORNER_ALIGN_VELOCITY_MPS,
                            align_turn_rpm);

        if ((square_absf(square_filtered_error) <=
             SQUARE_TRACKING_CORNER_ALIGN_ERROR) &&
            (square_absf(error) <= SQUARE_TRACKING_CORNER_ALIGN_ERROR) &&
            (actual_velocity_mps >=
             SQUARE_TRACKING_CORNER_ALIGN_MIN_FORWARD_MPS)) {
            if (square_condition_confirmed(1U,
                                           SQUARE_TRACKING_CORNER_ALIGN_CONFIRM_MS,
                                           sensor_new_frame) != 0U) {
                square_complete_alignment();
                if (square_phase == MODE_SQUARE_PHASE_FINISHED) {
                    square_build_hold(command);
                }
            }
            return;
        }

        square_reset_condition();
        return;
    }

    if (square_phase == MODE_SQUARE_PHASE_LOST_HOLD) {
        square_phase_elapsed_sec += dt_sec;
        if ((square_lost_corner_candidate == 0U) &&
            (error != TRACK_LOST_LINE) &&
            (square_corner_pattern(black_mask) == 0U) &&
            (square_refind_confirmed(1U, sensor_new_frame) != 0U)) {
            square_phase = MODE_SQUARE_PHASE_FOLLOW;
            square_phase_elapsed_sec = 0.0f;
            square_filter_ready = 0U;
            square_lost_corner_candidate = 0U;
            square_reset_condition();
            square_build_follow_command(command, error);
            return;
        }
        if ((sensor_new_frame != 0U) &&
            ((error == TRACK_LOST_LINE) ||
             (square_corner_pattern(black_mask) != 0U))) {
            square_reset_refind();
        }
        if (square_phase_elapsed_sec >=
            (0.001f * (float)SQUARE_TRACKING_LOST_HOLD_MS)) {
            square_phase = MODE_SQUARE_PHASE_LOST_BRAKE;
            square_phase_elapsed_sec = 0.0f;
            square_reset_refind();
            return;
        }

        square_build_lost_hold(command);
        return;
    }

    if (square_phase == MODE_SQUARE_PHASE_LOST_BRAKE) {
        uint8_t narrow_line_present =
            ((error != TRACK_LOST_LINE) &&
             (square_corner_pattern(black_mask) == 0U)) ?
            1U : 0U;

        square_phase_elapsed_sec += dt_sec;
        if (motion_settled != 0U) {
            if ((square_lost_corner_candidate != 0U) ||
                (narrow_line_present == 0U)) {
                square_phase = MODE_SQUARE_PHASE_LOST_SEARCH;
                square_phase_elapsed_sec = 0.0f;
                square_filter_ready = 0U;
                if (square_lost_corner_candidate != 0U) {
                    square_capture_turn_yaw(chassis_yaw_deg);
                }
                square_reset_condition();
            } else {
                square_enter_wait_line();
            }
        } else if (square_phase_elapsed_sec >=
                   (0.001f *
                    (float)SQUARE_TRACKING_LOST_BRAKE_TIMEOUT_MS)) {
            square_enter_fault();
        }
        return;
    }

    if (square_phase == MODE_SQUARE_PHASE_LOST_SEARCH) {
        float search_turn_rpm;

        square_phase_elapsed_sec += dt_sec;
        if (square_phase_elapsed_sec >=
            (0.001f * (float)SQUARE_TRACKING_LOST_SEARCH_TIMEOUT_MS)) {
            square_enter_fault();
            return;
        }
        if (square_lost_corner_candidate != 0U) {
            square_update_turn_yaw(chassis_yaw_deg);
            if ((square_phase_elapsed_sec <
                 (0.001f *
                  (float)SQUARE_TRACKING_CORNER_TURN_MIN_MS)) ||
                (square_turn_yaw_progress_deg <
                 SQUARE_TRACKING_CORNER_YAW_REACQUIRE_DEG)) {
                square_filter_ready = 0U;
                square_reset_condition();
                if (square_turn_yaw_progress_deg <
                    SQUARE_TRACKING_CORNER_YAW_FULL_TURN_DEG) {
                    square_build_fixed_turn(command);
                } else {
                    square_build_corner_search_turn(command);
                }
                return;
            }
            if ((sensor_new_frame != 0U) &&
                (error != TRACK_LOST_LINE) &&
                (square_corner_pattern(black_mask) == 0U)) {
                square_phase = MODE_SQUARE_PHASE_CORNER_ALIGN;
                square_phase_elapsed_sec = 0.0f;
                square_filter_ready = 0U;
                square_reset_condition();
            } else if (square_turn_yaw_progress_deg <
                       SQUARE_TRACKING_CORNER_YAW_FULL_TURN_DEG) {
                square_build_fixed_turn(command);
            } else {
                square_build_corner_search_turn(command);
            }
            return;
        }

        if (sensor_new_frame != 0U) {
            if ((error == TRACK_LOST_LINE) ||
                (square_corner_pattern(black_mask) != 0U)) {
                square_filter_ready = 0U;
                square_reset_condition();
                square_build_fixed_turn(command);
                return;
            }
            square_filter_error(error);
        }
        if (square_filter_ready == 0U) {
            square_build_fixed_turn(command);
            return;
        }

        if ((square_absf(square_filtered_error) <=
             SQUARE_TRACKING_CORNER_ALIGN_ERROR) &&
            (square_absf(error) <= SQUARE_TRACKING_CORNER_ALIGN_ERROR)) {
            square_build_hold(command);
            if (square_condition_confirmed(motion_settled,
                                           SQUARE_TRACKING_REFIND_CONFIRM_MS,
                                           sensor_new_frame) != 0U) {
                square_phase = MODE_SQUARE_PHASE_FOLLOW;
                square_phase_elapsed_sec = 0.0f;
                square_filter_ready = 0U;
                square_last_turn_rpm = 0.0f;
                square_last_velocity_mps = 0.0f;
                square_reset_condition();
                square_build_follow_command(command, error);
            }
            return;
        }

        square_reset_condition();
        search_turn_rpm = square_alignment_turn();
        square_build_motion(command, 0.0f, search_turn_rpm);
    }
}
