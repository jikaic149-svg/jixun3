#include <stdint.h>
#include <stdio.h>

#include "config.h"
#include "mode_gimbal.h"

static int failures = 0;

#define CHECK(condition, message)                                      \
    do {                                                               \
        if (!(condition)) {                                            \
            printf("FAIL: %s (line %d)\n", (message), __LINE__);      \
            failures++;                                                \
        }                                                              \
    } while (0)

static float absf(float value)
{
    return (value >= 0.0f) ? value : -value;
}

static void check_near(float actual,
                       float expected,
                       float tolerance,
                       const char *message)
{
    CHECK(absf(actual - expected) <= tolerance, message);
}

static GimbalCommand_t get_command(void)
{
    GimbalCommand_t command;

    Mode_GimbalGetCommand(&command);
    return command;
}

static void capture_center(GimbalMode_e mode, float encoder_angle_deg)
{
    Mode_GimbalInit();
    Mode_GimbalSet(mode);
    Mode_GimbalUpdate(encoder_angle_deg, 0.0f, 1U,
                      GIMBAL_YAW_CONTROL_DT_SEC);
}

static void test_relax_and_feedback_failure_are_safe(void)
{
    GimbalCommand_t command;

    Mode_GimbalInit();
    Mode_GimbalUpdate(75.0f, 200.0f, 1U,
                      GIMBAL_YAW_CONTROL_DT_SEC);
    command = get_command();
    CHECK(Mode_GimbalGet() == GIMBAL_MODE_RELAX,
          "initial mode must be RELAX");
    CHECK(command.drive_enabled == 0U && command.yaw_current_a == 0.0f,
          "RELAX must always command zero current");

    Mode_GimbalSet(GIMBAL_MODE_AIM);
    Mode_GimbalSetTargetOffsetDeg(GIMBAL_YAW_SOFT_LIMIT_DEG);
    for (uint8_t i = 0U; i < 20U; i++) {
        Mode_GimbalUpdate(75.0f, 0.0f, 1U,
                          GIMBAL_YAW_CONTROL_DT_SEC);
    }
    command = get_command();
    CHECK(command.drive_enabled != 0U && command.yaw_current_a != 0.0f,
          "feedback-loss setup must establish a nonzero AIM current");

    Mode_GimbalUpdate(75.0f, -1000.0f, 0U,
                      GIMBAL_YAW_CONTROL_DT_SEC);
    command = get_command();
    CHECK(command.drive_enabled == 0U && command.yaw_current_a == 0.0f,
          "invalid feedback must suppress drive and current");
}

static void test_hold_captures_current_encoder_angle(void)
{
    GimbalCommand_t command;

    capture_center(GIMBAL_MODE_HOLD, 123.0f);
    command = get_command();
    CHECK(command.drive_enabled != 0U,
          "HOLD with valid feedback must enable the yaw drive");
    check_near(command.target_offset_deg, 0.0f, 0.0001f,
               "HOLD must use zero target offset");
    check_near(command.target_angle_deg, 123.0f, 0.0001f,
               "HOLD must capture the first valid encoder angle");
    check_near(command.error_deg, 0.0f, 0.0001f,
               "capturing HOLD must not create an angle step");
    check_near(command.yaw_current_a, 0.0f, 0.0001f,
               "capturing HOLD at zero speed must output no current");

    Mode_GimbalUpdate(124.0f, 0.0f, 1U,
                      GIMBAL_YAW_CONTROL_DT_SEC);
    command = get_command();
    check_near(command.target_angle_deg, 123.0f, 0.0001f,
               "HOLD target must remain at the captured position");
    check_near(command.error_deg, -1.0f, 0.0001f,
               "HOLD must correct displacement from the captured position");
}

static void test_signed_corrections_and_angle_wrap(void)
{
    GimbalCommand_t command;

    capture_center(GIMBAL_MODE_AIM, 358.0f);
    Mode_GimbalSetTargetFromEncoderDeg(358.0f, 5.0f);
    Mode_GimbalUpdate(358.0f, 0.0f, 1U,
                      GIMBAL_YAW_CONTROL_DT_SEC);
    command = get_command();
    check_near(command.target_offset_deg, 5.0f, 0.0001f,
               "positive correction must create a positive offset");
    check_near(command.target_angle_deg, 3.0f, 0.0001f,
               "positive correction must wrap through 360 degrees");
    check_near(command.error_deg, 5.0f, 0.0001f,
               "wrapped positive correction must keep the short-path error");
    CHECK(command.yaw_current_a * GIMBAL_YAW_CONTROL_SIGN > 0.0f,
          "positive correction must produce current in the configured direction");

    capture_center(GIMBAL_MODE_AIM, 2.0f);
    Mode_GimbalSetTargetFromEncoderDeg(2.0f, -5.0f);
    Mode_GimbalUpdate(2.0f, 0.0f, 1U,
                      GIMBAL_YAW_CONTROL_DT_SEC);
    command = get_command();
    check_near(command.target_offset_deg, -5.0f, 0.0001f,
               "negative correction must create a negative offset");
    check_near(command.target_angle_deg, 357.0f, 0.0001f,
               "negative correction must wrap below zero degrees");
    check_near(command.error_deg, -5.0f, 0.0001f,
               "wrapped negative correction must keep the short-path error");
    CHECK(command.yaw_current_a * GIMBAL_YAW_CONTROL_SIGN < 0.0f,
          "negative correction must produce current in the configured direction");
}

static void test_visual_capture_window(void)
{
    GimbalCommand_t command;

    capture_center(GIMBAL_MODE_AIM, 100.0f);
    Mode_GimbalSetTargetFromEncoderDeg(
        170.0f, VISION_YAW_MAX_CORRECTION_DEG);
    Mode_GimbalUpdate(100.0f, 0.0f, 1U,
                      GIMBAL_YAW_CONTROL_DT_SEC);
    command = get_command();
    check_near(command.target_offset_deg,
               VISION_YAW_CAPTURE_LIMIT_DEG, 0.0001f,
               "visual target must clamp at the positive capture limit");
    check_near(command.target_angle_deg,
               100.0f + VISION_YAW_CAPTURE_LIMIT_DEG, 0.0001f,
               "positive capture clamp must remain relative to the entry center");

    Mode_GimbalSetTargetFromEncoderDeg(
        30.0f, -VISION_YAW_MAX_CORRECTION_DEG);
    Mode_GimbalUpdate(100.0f, 0.0f, 1U,
                      GIMBAL_YAW_CONTROL_DT_SEC);
    command = get_command();
    check_near(command.target_offset_deg,
               -VISION_YAW_CAPTURE_LIMIT_DEG, 0.0001f,
               "visual target must clamp at the negative capture limit");
    check_near(command.target_angle_deg,
               100.0f - VISION_YAW_CAPTURE_LIMIT_DEG, 0.0001f,
               "negative capture clamp must remain relative to the entry center");
}

static void check_current_bounds(float current_a,
                                 float previous_current_a,
                                 const char *limit_message,
                                 const char *slew_message)
{
    const float max_step_a =
        GIMBAL_YAW_CURRENT_SLEW_A_PER_SEC *
        GIMBAL_YAW_CONTROL_DT_SEC;

    CHECK(absf(current_a) <= GIMBAL_YAW_CURRENT_LIMIT_A + 0.00001f,
          limit_message);
    CHECK(absf(current_a - previous_current_a) <= max_step_a + 0.00001f,
          slew_message);
}

static void test_pid_current_limit_and_slew_limit(void)
{
    GimbalCommand_t command;
    float previous_current_a = 0.0f;

    capture_center(GIMBAL_MODE_AIM, 0.0f);
    Mode_GimbalSetTargetOffsetDeg(GIMBAL_YAW_SOFT_LIMIT_DEG);
    for (uint16_t i = 0U; i < 400U; i++) {
        Mode_GimbalUpdate(0.0f, -1000.0f, 1U,
                          GIMBAL_YAW_CONTROL_DT_SEC);
        command = get_command();
        check_current_bounds(
            command.yaw_current_a, previous_current_a,
            "positive saturation must respect the absolute current limit",
            "positive saturation must respect the 1 ms current slew limit");
        previous_current_a = command.yaw_current_a;
    }
    check_near(absf(previous_current_a), GIMBAL_YAW_CURRENT_LIMIT_A,
               0.0001f,
               "sustained large positive demand must reach the current clamp");

    Mode_GimbalSetTargetOffsetDeg(-GIMBAL_YAW_SOFT_LIMIT_DEG);
    for (uint16_t i = 0U; i < 400U; i++) {
        Mode_GimbalUpdate(0.0f, 1000.0f, 1U,
                          GIMBAL_YAW_CONTROL_DT_SEC);
        command = get_command();
        check_current_bounds(
            command.yaw_current_a, previous_current_a,
            "negative saturation must respect the absolute current limit",
            "current reversal must respect the 1 ms current slew limit");
        previous_current_a = command.yaw_current_a;
    }
    check_near(absf(previous_current_a), GIMBAL_YAW_CURRENT_LIMIT_A,
               0.0001f,
               "sustained large negative demand must reach the current clamp");
}

static void test_deadband_does_not_accumulate_current(void)
{
    GimbalCommand_t command;
    const float inside_deadband_deg = GIMBAL_YAW_DEADBAND_DEG * 0.90f;

    capture_center(GIMBAL_MODE_AIM, 200.0f);
    Mode_GimbalSetTargetOffsetDeg(inside_deadband_deg);
    for (uint16_t i = 0U; i < 5000U; i++) {
        Mode_GimbalUpdate(200.0f, 0.0f, 1U,
                          GIMBAL_YAW_CONTROL_DT_SEC);
        command = get_command();
        CHECK(command.yaw_current_a == 0.0f,
              "positive deadband error must never accumulate current");
    }

    Mode_GimbalSetTargetOffsetDeg(-inside_deadband_deg);
    for (uint16_t i = 0U; i < 5000U; i++) {
        Mode_GimbalUpdate(200.0f, 0.0f, 1U,
                          GIMBAL_YAW_CONTROL_DT_SEC);
        command = get_command();
        CHECK(command.yaw_current_a == 0.0f,
              "negative deadband error must never accumulate current");
    }
}

static void test_static_friction_feedforward_is_gated(void)
{
    GimbalCommand_t command;
    const float small_error_deg = GIMBAL_YAW_DEADBAND_DEG + 0.05f;

    capture_center(GIMBAL_MODE_AIM, 200.0f);
    Mode_GimbalSetTargetOffsetDeg(small_error_deg);
    for (uint8_t i = 0U; i < 30U; i++) {
        Mode_GimbalUpdate(200.0f, 0.0f, 1U,
                          GIMBAL_YAW_CONTROL_DT_SEC);
    }
    command = get_command();
    CHECK(command.yaw_current_a * GIMBAL_YAW_CONTROL_SIGN >
              GIMBAL_YAW_STATIC_FF_A * 0.80f,
          "low-speed positive error must receive static-friction feedforward");

    capture_center(GIMBAL_MODE_AIM, 200.0f);
    Mode_GimbalSetTargetOffsetDeg(-small_error_deg);
    for (uint8_t i = 0U; i < 30U; i++) {
        Mode_GimbalUpdate(200.0f, 0.0f, 1U,
                          GIMBAL_YAW_CONTROL_DT_SEC);
    }
    command = get_command();
    CHECK(command.yaw_current_a * GIMBAL_YAW_CONTROL_SIGN <
              -GIMBAL_YAW_STATIC_FF_A * 0.80f,
          "low-speed negative error must reverse static-friction feedforward");

    capture_center(GIMBAL_MODE_AIM, 200.0f);
    Mode_GimbalSetTargetOffsetDeg(small_error_deg);
    for (uint8_t i = 0U; i < 40U; i++) {
        Mode_GimbalUpdate(
            200.0f,
            0.5f * (GIMBAL_YAW_STATIC_FF_FULL_SPEED_RPM +
                    GIMBAL_YAW_STATIC_FF_MAX_SPEED_RPM),
            1U,
            GIMBAL_YAW_CONTROL_DT_SEC);
    }
    command = get_command();
    CHECK(command.yaw_current_a * GIMBAL_YAW_CONTROL_SIGN > 0.0f &&
              command.yaw_current_a * GIMBAL_YAW_CONTROL_SIGN <
                  GIMBAL_YAW_STATIC_FF_A * 0.80f,
          "static-friction feedforward must taper smoothly with yaw speed");

    capture_center(GIMBAL_MODE_AIM, 200.0f);
    Mode_GimbalSetTargetOffsetDeg(small_error_deg);
    for (uint8_t i = 0U; i < 30U; i++) {
        Mode_GimbalUpdate(200.0f,
                          GIMBAL_YAW_STATIC_FF_MAX_SPEED_RPM + 1.0f,
                          1U,
                          GIMBAL_YAW_CONTROL_DT_SEC);
    }
    command = get_command();
    CHECK(absf(command.yaw_current_a) < GIMBAL_YAW_STATIC_FF_A,
          "moving yaw must not receive static-friction feedforward");
}

static void test_relax_exit_clears_current_immediately(void)
{
    GimbalCommand_t command;

    capture_center(GIMBAL_MODE_AIM, 45.0f);
    Mode_GimbalSetTargetOffsetDeg(20.0f);
    for (uint8_t i = 0U; i < 20U; i++) {
        Mode_GimbalUpdate(45.0f, 0.0f, 1U,
                          GIMBAL_YAW_CONTROL_DT_SEC);
    }
    command = get_command();
    CHECK(command.drive_enabled != 0U && command.yaw_current_a != 0.0f,
          "test setup must establish a nonzero AIM current");

    Mode_GimbalSet(GIMBAL_MODE_RELAX);
    command = get_command();
    CHECK(command.drive_enabled == 0U && command.yaw_current_a == 0.0f,
          "entering RELAX must synchronously clear the current command");

    Mode_GimbalUpdate(45.0f, 1000.0f, 1U,
                      GIMBAL_YAW_CONTROL_DT_SEC);
    command = get_command();
    CHECK(command.drive_enabled == 0U && command.yaw_current_a == 0.0f,
          "RELAX updates must remain at zero after a driven AIM state");
}

static void test_chassis_speed_feedforward_only_offsets_yaw_target(void)
{
    GimbalCommand_t command;

    capture_center(GIMBAL_MODE_AIM, 100.0f);
    Mode_GimbalSetTargetFromEncoderDeg(100.0f, 2.0f);
    Mode_GimbalSetChassisSpeedFeedforward(0.08f, 1U);
    Mode_GimbalUpdate(100.0f, 0.0f, 1U,
                      GIMBAL_YAW_CONTROL_DT_SEC);
    command = get_command();
    check_near(command.chassis_speed_feedforward_deg, 0.12f, 0.0001f,
               "0.08 m/s measured speed must add 0.12 deg to yaw target");
    check_near(command.target_offset_deg, 2.12f, 0.0001f,
               "yaw feedforward must add to the visual target offset");

    Mode_GimbalSetChassisSpeedFeedforward(-0.08f, 1U);
    Mode_GimbalUpdate(100.0f, 0.0f, 1U,
                      GIMBAL_YAW_CONTROL_DT_SEC);
    command = get_command();
    check_near(command.chassis_speed_feedforward_deg, -0.12f, 0.0001f,
               "reverse motion must reverse the yaw feedforward");
    check_near(command.target_offset_deg, 1.88f, 0.0001f,
               "negative yaw feedforward must subtract from visual offset");

    Mode_GimbalSetChassisSpeedFeedforward(0.08f, 0U);
    Mode_GimbalUpdate(100.0f, 0.0f, 1U,
                      GIMBAL_YAW_CONTROL_DT_SEC);
    command = get_command();
    check_near(command.chassis_speed_feedforward_deg, 0.0f, 0.0001f,
               "stopping or braking must clear yaw feedforward");
    check_near(command.target_offset_deg, 2.0f, 0.0001f,
               "clearing feedforward must preserve the visual target");

    Mode_GimbalSetChassisSpeedFeedforward(1.0f, 1U);
    Mode_GimbalUpdate(100.0f, 0.0f, 1U,
                      GIMBAL_YAW_CONTROL_DT_SEC);
    command = get_command();
    check_near(command.chassis_speed_feedforward_deg,
               GIMBAL_YAW_CHASSIS_SPEED_FF_LIMIT_DEG, 0.0001f,
               "yaw speed feedforward must respect its angle limit");

    Mode_GimbalUpdate(100.0f, 0.0f, 0U,
                      GIMBAL_YAW_CONTROL_DT_SEC);
    command = get_command();
    check_near(command.chassis_speed_feedforward_deg, 0.0f, 0.0001f,
               "feedback loss must clear yaw feedforward");

    Mode_GimbalSetChassisSpeedFeedforward(0.08f, 1U);
    Mode_GimbalSet(GIMBAL_MODE_RELAX);
    command = get_command();
    check_near(command.chassis_speed_feedforward_deg, 0.0f, 0.0001f,
               "RELAX must synchronously clear yaw feedforward");
}

int main(void)
{
    test_relax_and_feedback_failure_are_safe();
    test_hold_captures_current_encoder_angle();
    test_signed_corrections_and_angle_wrap();
    test_visual_capture_window();
    test_pid_current_limit_and_slew_limit();
    test_deadband_does_not_accumulate_current();
    test_static_friction_feedforward_is_gated();
    test_relax_exit_clears_current_immediately();
    test_chassis_speed_feedforward_only_offsets_yaw_target();

    if (failures != 0) {
        printf("%d gimbal aim assertion(s) failed\n", failures);
        return 1;
    }
    printf("gimbal aim tests passed\n");
    return 0;
}
