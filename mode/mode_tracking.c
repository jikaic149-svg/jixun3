#include "mode_private.h"

#include "config.h"
#include "track.h"

static float tracking_filtered_error = 0.0f;
static float tracking_last_error = 0.0f;
static float tracking_integral = 0.0f;
static float tracking_last_turn_rpm = 0.0f;
static uint8_t tracking_ready = 0U;

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

void Mode_TrackingReset(void)
{
    tracking_filtered_error = 0.0f;
    tracking_last_error = 0.0f;
    tracking_integral = 0.0f;
    tracking_last_turn_rpm = 0.0f;
    tracking_ready = 0U;
}

void Mode_TrackingBuild(ModeCommand_t *command,
                        float error,
                        uint8_t sensor_fresh,
                        float dt_sec)
{
    float derivative;
    float abs_error;
    float speed_scale = 1.0f;
    float turn_rpm;

    Mode_CommandClear(command);
    command->tracking_active = 1U;

    if ((sensor_fresh == 0U) || (error == TRACK_LOST_LINE)) {
        tracking_ready = 0U;
        tracking_integral = 0.0f;
        tracking_last_turn_rpm =
            mode_limit_abs(tracking_last_turn_rpm * TRACKING_LOST_TURN_DECAY,
                           TRACKING_TURN_LIMIT_RPM);
        command->turn_rpm = tracking_last_turn_rpm;
        command->velocity_mps = TRACKING_LOST_VELOCITY_MPS;
        command->bias_deg = BALANCE_MODE_STAND_BIAS_DEG;
        command->drive_enabled =
            (mode_absf(command->velocity_mps) > 0.001f) ? 1U : 0U;
        return;
    }

    if (dt_sec <= 0.0f) {
        dt_sec = BALANCE_PID_DT_SEC;
    }

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
    command->bias_deg = TRACKING_FORWARD_BIAS_DEG;
    command->turn_rpm = turn_rpm;
    command->drive_enabled = 1U;
}
