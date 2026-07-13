#include "mode_gimbal.h"

#include <stddef.h>

#include "config.h"

static GimbalMode_e gimbal_mode = GIMBAL_MODE_RELAX;
static GimbalCommand_t gimbal_command;
static float gimbal_center_angle_deg = 0.0f;
static float gimbal_target_offset_deg = 0.0f;
static float gimbal_chassis_speed_feedforward_deg = 0.0f;
static float gimbal_integral_a = 0.0f;
static float gimbal_filtered_speed_rpm = 0.0f;
static float gimbal_last_current_a = 0.0f;
static uint8_t gimbal_center_valid = 0U;
static uint8_t gimbal_capture_center = 0U;
static uint8_t gimbal_speed_filter_ready = 0U;

static float gimbal_absf(float value)
{
    return (value >= 0.0f) ? value : -value;
}

static float gimbal_limit(float value, float abs_limit)
{
    float limit = gimbal_absf(abs_limit);

    if (value > limit) {
        return limit;
    }
    if (value < -limit) {
        return -limit;
    }
    return value;
}

static float gimbal_wrap_angle(float angle_deg)
{
    while (angle_deg >= 360.0f) {
        angle_deg -= 360.0f;
    }
    while (angle_deg < 0.0f) {
        angle_deg += 360.0f;
    }
    return angle_deg;
}

static float gimbal_wrap_error(float error_deg)
{
    while (error_deg >= 180.0f) {
        error_deg -= 360.0f;
    }
    while (error_deg < -180.0f) {
        error_deg += 360.0f;
    }
    return error_deg;
}

static void gimbal_clear_command(void)
{
    gimbal_command.target_offset_deg =
        gimbal_target_offset_deg + gimbal_chassis_speed_feedforward_deg;
    gimbal_command.target_angle_deg =
        gimbal_wrap_angle(gimbal_center_angle_deg +
                          gimbal_command.target_offset_deg);
    gimbal_command.chassis_speed_feedforward_deg =
        gimbal_chassis_speed_feedforward_deg;
    gimbal_command.error_deg = 0.0f;
    gimbal_command.yaw_current_a = 0.0f;
    gimbal_command.drive_enabled = 0U;
}

void Mode_GimbalInit(void)
{
    gimbal_mode = GIMBAL_MODE_RELAX;
    gimbal_center_angle_deg = 0.0f;
    gimbal_target_offset_deg = 0.0f;
    gimbal_chassis_speed_feedforward_deg = 0.0f;
    gimbal_integral_a = 0.0f;
    gimbal_filtered_speed_rpm = 0.0f;
    gimbal_last_current_a = 0.0f;
    gimbal_center_valid = 0U;
    gimbal_capture_center = 0U;
    gimbal_speed_filter_ready = 0U;
    gimbal_clear_command();
}

void Mode_GimbalSet(GimbalMode_e mode)
{
    GimbalMode_e previous_mode = gimbal_mode;

    if (mode > GIMBAL_MODE_AIM) {
        mode = GIMBAL_MODE_RELAX;
    }
    if (gimbal_mode == mode) {
        return;
    }

    gimbal_mode = mode;
    gimbal_integral_a = 0.0f;
    gimbal_speed_filter_ready = 0U;
    gimbal_chassis_speed_feedforward_deg = 0.0f;

    if (mode == GIMBAL_MODE_RELAX) {
        gimbal_capture_center = 0U;
        gimbal_last_current_a = 0.0f;
        gimbal_clear_command();
    } else if (mode == GIMBAL_MODE_HOLD) {
        gimbal_target_offset_deg = 0.0f;
        gimbal_capture_center = 1U;
    } else if ((previous_mode == GIMBAL_MODE_RELAX) ||
               (gimbal_center_valid == 0U)) {
        gimbal_target_offset_deg = 0.0f;
        gimbal_capture_center = 1U;
    }
}

GimbalMode_e Mode_GimbalGet(void)
{
    return gimbal_mode;
}

void Mode_GimbalSetTargetOffsetDeg(float offset_deg)
{
    gimbal_target_offset_deg =
        gimbal_limit(offset_deg, GIMBAL_YAW_SOFT_LIMIT_DEG);
}

void Mode_GimbalAddTargetOffsetDeg(float delta_deg)
{
    Mode_GimbalSetTargetOffsetDeg(gimbal_target_offset_deg + delta_deg);
}

float Mode_GimbalShapeVisionErrorPx(int16_t error_px)
{
    float signed_error_px = (float)error_px;
    float abs_error_px = gimbal_absf(signed_error_px);
    float deadband_px = (float)VISION_YAW_DEADBAND_PX;
    float transition_px = VISION_YAW_FULL_RESPONSE_PX - deadband_px;
    float blend;

    if (abs_error_px <= deadband_px) {
        return 0.0f;
    }
    if ((transition_px <= 0.001f) ||
        (abs_error_px >= VISION_YAW_FULL_RESPONSE_PX)) {
        return signed_error_px;
    }

    blend = (abs_error_px - deadband_px) / transition_px;
    blend = blend * blend * (3.0f - 2.0f * blend);
    return signed_error_px * blend;
}

void Mode_GimbalSetTargetFromEncoderDeg(float encoder_angle_deg,
                                        float correction_deg)
{
    float target_angle_deg;
    float target_offset_deg;

    encoder_angle_deg = gimbal_wrap_angle(encoder_angle_deg);
    correction_deg =
        gimbal_limit(correction_deg, VISION_YAW_MAX_CORRECTION_DEG);
    if ((gimbal_capture_center != 0U) || (gimbal_center_valid == 0U)) {
        gimbal_center_angle_deg = encoder_angle_deg;
        gimbal_center_valid = 1U;
        gimbal_capture_center = 0U;
        gimbal_target_offset_deg = 0.0f;
    }
    if (correction_deg == 0.0f) {
        return;
    }

    target_angle_deg = gimbal_wrap_angle(encoder_angle_deg + correction_deg);
    target_offset_deg =
        gimbal_wrap_error(target_angle_deg - gimbal_center_angle_deg);
    target_offset_deg =
        gimbal_limit(target_offset_deg, VISION_YAW_CAPTURE_LIMIT_DEG);
    Mode_GimbalSetTargetOffsetDeg(target_offset_deg);
}

void Mode_GimbalResetIntegral(void)
{
    gimbal_integral_a = 0.0f;
}

void Mode_GimbalSetChassisSpeedFeedforward(float velocity_mps,
                                           uint8_t enabled)
{
    if ((enabled == 0U) || (gimbal_mode != GIMBAL_MODE_AIM)) {
        gimbal_chassis_speed_feedforward_deg = 0.0f;
        return;
    }
    gimbal_chassis_speed_feedforward_deg = gimbal_limit(
        velocity_mps * GIMBAL_YAW_CHASSIS_SPEED_FF_DEG_PER_MPS,
        GIMBAL_YAW_CHASSIS_SPEED_FF_LIMIT_DEG);
}

void Mode_GimbalUpdate(float encoder_angle_deg,
                       float encoder_speed_rpm,
                       uint8_t feedback_ready,
                       float dt_sec)
{
    float target_offset_deg;
    float target_angle_deg;
    float raw_error_deg;
    float control_error_deg;
    float static_feedforward_a;
    float output_current_a;

    gimbal_clear_command();

    if (dt_sec <= 0.0f) {
        dt_sec = GIMBAL_YAW_CONTROL_DT_SEC;
    } else if (dt_sec > 0.02f) {
        dt_sec = 0.02f;
    }

    if ((feedback_ready == 0U) || (gimbal_mode == GIMBAL_MODE_RELAX)) {
        gimbal_chassis_speed_feedforward_deg = 0.0f;
        gimbal_integral_a = 0.0f;
        gimbal_last_current_a = 0.0f;
        gimbal_speed_filter_ready = 0U;
        gimbal_clear_command();
        return;
    }

    encoder_angle_deg = gimbal_wrap_angle(encoder_angle_deg);
    if ((gimbal_capture_center != 0U) || (gimbal_center_valid == 0U)) {
        gimbal_center_angle_deg = encoder_angle_deg;
        gimbal_center_valid = 1U;
        gimbal_capture_center = 0U;
        gimbal_integral_a = 0.0f;
    }

    if (gimbal_speed_filter_ready == 0U) {
        gimbal_filtered_speed_rpm = encoder_speed_rpm;
        gimbal_speed_filter_ready = 1U;
    } else {
        gimbal_filtered_speed_rpm =
            GIMBAL_YAW_SPEED_LPF_ALPHA * gimbal_filtered_speed_rpm +
            (1.0f - GIMBAL_YAW_SPEED_LPF_ALPHA) * encoder_speed_rpm;
    }

    target_offset_deg = (gimbal_mode == GIMBAL_MODE_HOLD) ?
                        0.0f :
                        gimbal_target_offset_deg +
                        gimbal_chassis_speed_feedforward_deg;
    target_angle_deg =
        gimbal_wrap_angle(gimbal_center_angle_deg + target_offset_deg);
    raw_error_deg =
        gimbal_wrap_error(target_angle_deg - encoder_angle_deg);
    raw_error_deg = gimbal_limit(raw_error_deg, GIMBAL_YAW_MAX_ERROR_DEG);
    control_error_deg = raw_error_deg;

    if (control_error_deg > GIMBAL_YAW_DEADBAND_DEG) {
        control_error_deg -= GIMBAL_YAW_DEADBAND_DEG;
    } else if (control_error_deg < -GIMBAL_YAW_DEADBAND_DEG) {
        control_error_deg += GIMBAL_YAW_DEADBAND_DEG;
    } else {
        control_error_deg = 0.0f;
    }

    if (control_error_deg == 0.0f) {
        gimbal_integral_a *= 0.90f;
        if (gimbal_absf(gimbal_integral_a) < 0.0001f) {
            gimbal_integral_a = 0.0f;
        }
    } else if (gimbal_absf(raw_error_deg) >
               GIMBAL_YAW_INTEGRAL_ZONE_DEG) {
        gimbal_integral_a = 0.0f;
    } else {
        if ((gimbal_integral_a * control_error_deg) < 0.0f) {
            gimbal_integral_a = 0.0f;
        }
        if (gimbal_absf(gimbal_filtered_speed_rpm) <
            GIMBAL_YAW_INTEGRAL_MAX_SPEED_RPM) {
            gimbal_integral_a +=
                GIMBAL_YAW_KI_A_PER_DEG_S * control_error_deg * dt_sec;
        } else {
            gimbal_integral_a *= GIMBAL_YAW_INTEGRAL_MOVING_DECAY;
        }
        gimbal_integral_a =
            gimbal_limit(gimbal_integral_a, GIMBAL_YAW_INTEGRAL_LIMIT_A);
    }

    static_feedforward_a = 0.0f;
    if ((control_error_deg != 0.0f) &&
        (gimbal_absf(gimbal_filtered_speed_rpm) <
         GIMBAL_YAW_STATIC_FF_MAX_SPEED_RPM)) {
        float abs_control_error_deg = gimbal_absf(control_error_deg);
        float abs_speed_rpm = gimbal_absf(gimbal_filtered_speed_rpm);
        float error_scale = 1.0f;
        float speed_scale = 1.0f;

        if (GIMBAL_YAW_STATIC_FF_FULL_ERROR_DEG > 0.001f) {
            error_scale =
                abs_control_error_deg /
                GIMBAL_YAW_STATIC_FF_FULL_ERROR_DEG;
            if (error_scale > 1.0f) {
                error_scale = 1.0f;
            }
        }

        if (abs_speed_rpm > GIMBAL_YAW_STATIC_FF_FULL_SPEED_RPM) {
            float taper_span_rpm =
                GIMBAL_YAW_STATIC_FF_MAX_SPEED_RPM -
                GIMBAL_YAW_STATIC_FF_FULL_SPEED_RPM;

            if (taper_span_rpm > 0.001f) {
                speed_scale =
                    (GIMBAL_YAW_STATIC_FF_MAX_SPEED_RPM - abs_speed_rpm) /
                    taper_span_rpm;
            } else {
                speed_scale = 0.0f;
            }
            speed_scale = gimbal_limit(speed_scale, 1.0f);
            if (speed_scale < 0.0f) {
                speed_scale = 0.0f;
            }
        }
        static_feedforward_a =
            GIMBAL_YAW_STATIC_FF_A * error_scale * speed_scale;
        if (control_error_deg < 0.0f) {
            static_feedforward_a = -static_feedforward_a;
        }
    }

    output_current_a =
        (GIMBAL_YAW_KP_A_PER_DEG * control_error_deg +
         gimbal_integral_a -
         GIMBAL_YAW_KD_A_PER_RPM * gimbal_filtered_speed_rpm +
         static_feedforward_a) *
        GIMBAL_YAW_CONTROL_SIGN;
    output_current_a =
        gimbal_limit(output_current_a, GIMBAL_YAW_CURRENT_LIMIT_A);
    {
        float max_step_a = GIMBAL_YAW_CURRENT_SLEW_A_PER_SEC * dt_sec;
        float current_step_a = output_current_a - gimbal_last_current_a;

        current_step_a = gimbal_limit(current_step_a, max_step_a);
        output_current_a = gimbal_last_current_a + current_step_a;
        gimbal_last_current_a = output_current_a;
    }

    gimbal_command.target_offset_deg = target_offset_deg;
    gimbal_command.target_angle_deg = target_angle_deg;
    gimbal_command.chassis_speed_feedforward_deg =
        gimbal_chassis_speed_feedforward_deg;
    gimbal_command.error_deg = raw_error_deg;
    gimbal_command.yaw_current_a = output_current_a;
    gimbal_command.drive_enabled = 1U;
}

void Mode_GimbalGetCommand(GimbalCommand_t *command)
{
    if (command == NULL) {
        return;
    }
    *command = gimbal_command;
}
