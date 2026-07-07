#include "gimbal_task.h"

#include <stddef.h>

#include "FreeRTOS.h"
#include "task.h"
#include "ins_task.h"
#include "bsp_can.h"
#include "config.h"
#include "main.h"

typedef struct {
    float kp;
    float ki;
    float kd;
    float integral;
    float integral_limit;
    float output_limit;
} ChassisPidController_t;

volatile GimbalState_e gimbal_state = GIMBAL_STATE_RELAX;
volatile ChassisMotorState_e chassis_motor_state = CHASSIS_MOTOR_ESTOP;
volatile ChassisMode_e chassis_mode = CHASSIS_MODE_STAND;
volatile uint8_t chassis_motor_feedback_ok = 0U;
volatile uint8_t chassis_motor_enabled_ok = 0U;
volatile uint8_t chassis_speed_test_armed = 0U;
volatile uint8_t chassis_fault_code = CHASSIS_FAULT_NONE;
volatile float wheel_speed_ref_left_rpm = 0.0f;
volatile float wheel_speed_actual_left_rpm = 0.0f;
volatile float wheel_speed_output_left_a = 0.0f;
volatile float wheel_speed_ref_right_rpm = 0.0f;
volatile float wheel_speed_actual_right_rpm = 0.0f;
volatile float wheel_speed_output_right_a = 0.0f;
volatile float wheel_speed_feedback_left_a = 0.0f;
volatile float wheel_speed_feedback_right_a = 0.0f;
volatile float wheel_speed_eso_z1_rpm = 0.0f;
volatile float wheel_speed_eso_z2_rpm_s = 0.0f;
volatile float wheel_speed_pi_integral = 0.0f;
volatile float wheel_speed_pi_integral_right = 0.0f;
volatile float wheel_speed_target_rpm = WHEEL_SPEED_TEST_REF_RPM;
volatile float balance_pid_theta_deg = 0.0f;
volatile float balance_pid_theta_ref_deg = 0.0f;
volatile float balance_pid_manual_bias_deg = 0.0f;
volatile float balance_pid_theta_rate_dps = 0.0f;
volatile float balance_pid_accel_angle_deg = 0.0f;
volatile float balance_pid_position_m = 0.0f;
volatile float balance_pid_position_ref_m = 0.0f;
volatile float balance_pid_velocity_mps = 0.0f;
volatile float balance_pid_velocity_ref_mps = 0.0f;
volatile float balance_pid_accel_mps2 = 0.0f;
volatile float balance_pid_accel_theta_ref_deg = 0.0f;
volatile float balance_pid_wheel_diff_mps = 0.0f;
volatile float balance_pid_speed_theta_ref_deg = 0.0f;
volatile float balance_pid_integral_theta_ref_deg = 0.0f;
volatile float balance_pid_zero_trim_deg = 0.0f;
volatile float balance_pid_angle_speed_ref_rpm = 0.0f;
volatile float balance_pid_rate_speed_ref_rpm = 0.0f;
volatile float balance_pid_balance_speed_ref_rpm = 0.0f;
volatile float balance_pid_current_target_a = 0.0f;
volatile float balance_pid_current_cmd_a = 0.0f;
volatile float balance_pid_current_feedback_a = 0.0f;
volatile float balance_pid_turn_speed_ref_rpm = 0.0f;
volatile float balance_pid_output_left_a = 0.0f;
volatile float balance_pid_output_right_a = 0.0f;
volatile float balance_pid_runtime_kp = BALANCE_PID_ANGLE_KP_RPM_PER_DEG;
volatile float balance_pid_runtime_ki = BALANCE_PID_ANGLE_KI_RPM_PER_DEG_S;
volatile float balance_pid_runtime_kd = BALANCE_PID_RATE_KD_RPM_PER_DPS;
volatile float balance_pid_runtime_speed_limit_rpm =
    BALANCE_PID_BALANCE_SPEED_LIMIT_RPM;
volatile float balance_pid_runtime_current_limit_a =
    BALANCE_PID_CURRENT_LIMIT_A;
volatile float balance_pid_runtime_current_slew_a_per_sec =
    BALANCE_PID_CURRENT_SLEW_A_PER_SEC;
volatile uint32_t chassis_serial_cmd_rx_count = 0U;
volatile uint32_t chassis_serial_cmd_ok_count = 0U;
volatile uint8_t chassis_serial_cmd_last_type = 0U;
volatile float chassis_serial_cmd_last_value = 0.0f;
volatile uint8_t balance_zero_bias_frozen = 0U;
volatile uint8_t emergency_stop = 1U;

static ChassisMotorState_e last_chassis_motor_state = CHASSIS_MOTOR_ESTOP;
static volatile uint8_t wheel_speed_target_changed = 0U;
static volatile uint8_t chassis_mode_changed = 0U;
static float left_speed_pid_integral = 0.0f;
static float right_speed_pid_integral = 0.0f;
static float left_speed_feedback_lpf = 0.0f;
static float right_speed_feedback_lpf = 0.0f;
static float left_speed_last_output = 0.0f;
static float right_speed_last_output = 0.0f;
static float wheel_speed_target_ramp_rpm = 0.0f;
static uint8_t wheel_speed_feedback_lpf_ready = 0U;
#if WHEEL_SPEED_TEST_ENABLE && WHEEL_SPEED_TEST_AUTO_SEQUENCE_ENABLE
static const float wheel_speed_test_auto_sequence_rpm[] = {
    50.0f, 0.0f,
    100.0f, 0.0f,
    150.0f, 0.0f,
    200.0f, 0.0f,
    -50.0f, 0.0f,
    -100.0f, 0.0f,
    -150.0f, 0.0f
};
static uint8_t wheel_speed_test_auto_index = 0U;
static uint32_t wheel_speed_test_auto_step_tick = 0U;
static uint8_t wheel_speed_test_auto_running = 0U;
#endif
static ChassisPidController_t balance_angle_pid;
static uint8_t balance_pid_initialized = 0U;
static float balance_manual_bias_deg = BALANCE_PID_THETA_REF_DEG;
static float balance_position_m = 0.0f;
static float balance_position_ref_m = 0.0f;
static float balance_velocity_lpf_mps = 0.0f;
static uint8_t balance_velocity_lpf_ready = 0U;
static float balance_velocity_ref_ramp_mps = 0.0f;
static float balance_last_measured_velocity_mps = 0.0f;
static float balance_accel_lpf_mps2 = 0.0f;
static uint8_t balance_accel_lpf_ready = 0U;
static float balance_speed_integral_m = 0.0f;
static uint8_t balance_speed_guard_active = 0U;
static ChassisMode_e balance_integral_last_mode = CHASSIS_MODE_STAND;
static float balance_drive_launch_timer_sec = 0.0f;
static float balance_stop_brake_timer_sec = 0.0f;
static float balance_drive_pulse_timer_sec = 0.0f;
static float balance_drive_pulse_brake_timer_sec = 0.0f;
static float balance_drive_pulse_hold_timer_sec = 0.0f;
/* 零点自学习 trim:整个会话保留,不随 fault/重新解锁清零 */
static float balance_zero_trim_deg = 0.0f;

static float Chassis_AbsF(float value)
{
    return (value >= 0.0f) ? value : -value;
}

static float Chassis_LimitAbs(float value, float abs_limit)
{
    float limit = Chassis_AbsF(abs_limit);

    if (value > limit) {
        return limit;
    }
    if (value < -limit) {
        return -limit;
    }
    return value;
}

static float Chassis_ClampF(float value, float min_value, float max_value)
{
    if (value > max_value) {
        return max_value;
    }
    if (value < min_value) {
        return min_value;
    }
    return value;
}

static float Chassis_StepTowards(float current, float target, float max_step)
{
    float delta = target - current;

    if (max_step <= 0.0f) {
        return target;
    }

    delta = Chassis_LimitAbs(delta, max_step);
    return current + delta;
}

static float Chassis_LimitSpeedTarget(float rpm)
{
    float abs_rpm = Chassis_AbsF(rpm);

    if (abs_rpm <= 0.001f) {
        return 0.0f;
    }

    if (abs_rpm < WHEEL_SPEED_TEST_REF_MIN_ABS_RPM) {
        abs_rpm = WHEEL_SPEED_TEST_REF_MIN_ABS_RPM;
    }
    if (abs_rpm > WHEEL_SPEED_TEST_REF_MAX_ABS_RPM) {
        abs_rpm = WHEEL_SPEED_TEST_REF_MAX_ABS_RPM;
    }

    return (rpm >= 0.0f) ? abs_rpm : -abs_rpm;
}

static float Chassis_SpeedFeedforward(float ref_rpm)
{
    float abs_ref = Chassis_AbsF(ref_rpm);
    float ff;

    if (abs_ref <= 0.001f) {
        return 0.0f;
    }

    ff = WHEEL_SPEED_FF_STATIC_A + WHEEL_SPEED_FF_KV_A_PER_RPM * abs_ref;
    return (ref_rpm >= 0.0f) ? ff : -ff;
}

static void ChassisPid_Init(ChassisPidController_t *pid,
                            float kp,
                            float ki,
                            float kd,
                            float integral_limit,
                            float output_limit)
{
    if (pid == NULL) {
        return;
    }

    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->integral = 0.0f;
    pid->integral_limit = Chassis_AbsF(integral_limit);
    pid->output_limit = Chassis_AbsF(output_limit);
}

static void ChassisPid_Reset(ChassisPidController_t *pid)
{
    if (pid == NULL) {
        return;
    }

    pid->integral = 0.0f;
}

static void ChassisPid_ApplyRuntime(ChassisPidController_t *pid)
{
    if (pid == NULL) {
        return;
    }

    pid->kp = balance_pid_runtime_kp;
    pid->ki = balance_pid_runtime_ki;
    pid->kd = balance_pid_runtime_kd;
    pid->integral_limit =
        Chassis_AbsF(BALANCE_PID_ANGLE_INTEGRAL_LIMIT_DEG_S);
    pid->output_limit = Chassis_AbsF(balance_pid_runtime_speed_limit_rpm);
}

#if BALANCE_PID_ENABLE
static float ChassisPid_UpdateAngle(ChassisPidController_t *pid,
                                    float theta_error_deg,
                                    float theta_rate_dps,
                                    float dt_sec)
{
    float old_integral;
    float output_unsat;
    float output;

    if (pid == NULL) {
        return 0.0f;
    }

    old_integral = pid->integral;
    if ((pid->ki != 0.0f) && (dt_sec > 0.0f)) {
        pid->integral += theta_error_deg * dt_sec;
        pid->integral = Chassis_LimitAbs(pid->integral,
                                         pid->integral_limit);
    } else {
        pid->integral = 0.0f;
    }

    output_unsat = (pid->kp * theta_error_deg +
                    pid->ki * pid->integral +
                    pid->kd * theta_rate_dps) *
                   BALANCE_PID_POLARITY;
    output = Chassis_LimitAbs(output_unsat, pid->output_limit);

    if ((pid->ki != 0.0f) && (output != output_unsat)) {
        pid->integral = old_integral;
    }

    return output;
}
#endif

static void Chassis_BalancePidInitOnce(void)
{
    if (balance_pid_initialized == 0U) {
        ChassisPid_Init(&balance_angle_pid,
                        balance_pid_runtime_kp,
                        balance_pid_runtime_ki,
                        balance_pid_runtime_kd,
                        BALANCE_PID_ANGLE_INTEGRAL_LIMIT_DEG_S,
                        balance_pid_runtime_speed_limit_rpm);
        balance_pid_initialized = 1U;
    } else {
        ChassisPid_ApplyRuntime(&balance_angle_pid);
    }
}

void WheelSpeed_SetTargetRpm(float rpm)
{
    wheel_speed_target_rpm = Chassis_LimitSpeedTarget(rpm);
    wheel_speed_target_changed = 1U;
}

void Balance_SetManualBiasDeg(float bias_deg)
{
    balance_manual_bias_deg = Chassis_LimitAbs(bias_deg,
                                               BALANCE_MANUAL_BIAS_LIMIT_DEG);
    balance_pid_manual_bias_deg = balance_manual_bias_deg;
}

void Balance_SetPidRuntimeParam(BalancePidRuntimeParam_e param, float value)
{
    switch (param) {
    case BALANCE_PID_RUNTIME_KP:
        balance_pid_runtime_kp = Chassis_ClampF(value, 0.0f, 80.0f);
        break;
    case BALANCE_PID_RUNTIME_KI:
        balance_pid_runtime_ki = Chassis_ClampF(value, 0.0f, 20.0f);
        break;
    case BALANCE_PID_RUNTIME_KD:
        balance_pid_runtime_kd = Chassis_ClampF(value, 0.0f, 5.0f);
        break;
    case BALANCE_PID_RUNTIME_SPEED_LIMIT_RPM:
        balance_pid_runtime_speed_limit_rpm =
            Chassis_ClampF(Chassis_AbsF(value),
                           BALANCE_PID_RUNTIME_SPEED_LIMIT_MIN_RPM,
                           500.0f);
        break;
    case BALANCE_PID_RUNTIME_CURRENT_LIMIT_A:
        balance_pid_runtime_current_limit_a =
            Chassis_ClampF(Chassis_AbsF(value), 0.05f, 1.00f);
        break;
    case BALANCE_PID_RUNTIME_CURRENT_SLEW_A_PER_SEC:
        balance_pid_runtime_current_slew_a_per_sec =
            Chassis_ClampF(Chassis_AbsF(value), 1.0f, 200.0f);
        break;
    default:
        return;
    }

    ChassisPid_ApplyRuntime(&balance_angle_pid);
    ChassisPid_Reset(&balance_angle_pid);
}

void Chassis_SetControlMode(ChassisMode_e mode)
{
    if (mode > CHASSIS_MODE_TURN_RIGHT) {
        mode = CHASSIS_MODE_STAND;
    }

    Balance_SetManualBiasDeg(0.0f);

    if (chassis_mode != mode) {
        chassis_mode = mode;
        chassis_mode_changed = 1U;
    }
}

static void Chassis_UpdateKey(void)
{
    static uint8_t key_stable_state = 1U;
    static uint8_t key_last_raw = 1U;
    static uint32_t key_raw_change_tick = 0U;
    uint32_t now_tick = (uint32_t)xTaskGetTickCount();
    uint8_t key_raw_state;

    key_raw_state = (HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin) == GPIO_PIN_RESET) ? 0U : 1U;
    if (key_raw_state != key_last_raw) {
        key_last_raw = key_raw_state;
        key_raw_change_tick = now_tick;
    }

    if (((uint32_t)(now_tick - key_raw_change_tick) >= pdMS_TO_TICKS(KEY_DEBOUNCE_MS)) &&
        key_stable_state != key_last_raw) {
        key_stable_state = key_last_raw;

        if (key_stable_state == 0U) {
            if (emergency_stop == 0U) {
                Chassis_SetControlMode(CHASSIS_MODE_STAND);
                chassis_speed_test_armed = 0U;
                emergency_stop = 1U;
            } else if ((chassis_motor_feedback_ok != 0U) &&
                       (chassis_motor_enabled_ok != 0U)) {
#if (BALANCE_PID_ENABLE || WHEEL_SPEED_TEST_ENABLE)
#if IMU_YAW_BIAS_KEY_CAL_ENABLE
                if (INS_GetYawCalibrationState() != INS_YAW_CAL_DONE) {
                    if (INS_GetYawCalibrationState() != INS_YAW_CAL_RUNNING) {
                        INS_RequestYawCalibration();
                    }
                    chassis_fault_code = CHASSIS_FAULT_NONE;
                    chassis_speed_test_armed = 0U;
                    emergency_stop = 1U;
                } else {
                    chassis_fault_code = CHASSIS_FAULT_NONE;
                    chassis_speed_test_armed = 1U;
                    INS_RequestBalanceAngleReseed();
                    emergency_stop = 0U;
                }
#else
                chassis_fault_code = CHASSIS_FAULT_NONE;
                chassis_speed_test_armed = 1U;
                INS_RequestBalanceAngleReseed();
                emergency_stop = 0U;
#endif
#endif
            }
        }
    }
}

static void Chassis_UpdateLed(void)
{
    GPIO_PinState blue = GPIO_PIN_RESET;
    GPIO_PinState green = GPIO_PIN_RESET;
    GPIO_PinState red = GPIO_PIN_RESET;
#if IMU_YAW_BIAS_KEY_CAL_ENABLE
    uint8_t yaw_cal_state = INS_GetYawCalibrationState();
    uint8_t blink_on = ((((uint32_t)xTaskGetTickCount()) /
                         pdMS_TO_TICKS(250U)) & 1U) ? 1U : 0U;
#endif

    if ((chassis_motor_feedback_ok == 0U) ||
        (chassis_motor_enabled_ok == 0U)) {
        red = GPIO_PIN_SET;
    } else if ((emergency_stop == 0U) && (chassis_speed_test_armed != 0U)) {
        green = GPIO_PIN_SET;
#if IMU_YAW_BIAS_KEY_CAL_ENABLE
    } else if (yaw_cal_state == INS_YAW_CAL_RUNNING) {
        blue = (blink_on != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET;
    } else if (yaw_cal_state == INS_YAW_CAL_DONE) {
        blue = GPIO_PIN_SET;
        green = GPIO_PIN_SET;
#endif
    } else {
        blue = GPIO_PIN_SET;
    }

    HAL_GPIO_WritePin(GPIOH, GPIO_PIN_10, blue);
    HAL_GPIO_WritePin(GPIOH, GPIO_PIN_11, green);
    HAL_GPIO_WritePin(GPIOH, GPIO_PIN_12, red);
}

static void Chassis_UpdateState(void)
{
    if (chassis_motor_feedback_ok == 0U) {
        chassis_motor_state = CHASSIS_MOTOR_WAIT_FEEDBACK;
        return;
    }

    if (chassis_motor_enabled_ok == 0U) {
        chassis_motor_state = CHASSIS_MOTOR_WAIT_ENABLE;
        return;
    }

    if ((emergency_stop != 0U) || (chassis_speed_test_armed == 0U)) {
        chassis_motor_state = CHASSIS_MOTOR_ESTOP;
        return;
    }

#if BALANCE_PID_ENABLE
    chassis_motor_state = CHASSIS_MOTOR_BALANCE_PID;
#elif WHEEL_SPEED_TEST_ENABLE
    chassis_motor_state = CHASSIS_MOTOR_SPEED_TEST;
#else
    chassis_motor_state = CHASSIS_MOTOR_READY;
#endif
}

static void Chassis_ResetSpeedCtrl(void)
{
    left_speed_pid_integral = 0.0f;
    right_speed_pid_integral = 0.0f;
    left_speed_feedback_lpf = left_wheel_motor.speed_can_rpm;
    right_speed_feedback_lpf = right_wheel_motor.speed_can_rpm;
    left_speed_last_output = 0.0f;
    right_speed_last_output = 0.0f;
    wheel_speed_target_ramp_rpm = 0.0f;
    wheel_speed_feedback_lpf_ready = 0U;
    wheel_speed_feedback_left_a = left_wheel_motor.current;
    wheel_speed_feedback_right_a = right_wheel_motor.current;
    wheel_speed_eso_z1_rpm = left_wheel_motor.speed_can_rpm;
    wheel_speed_eso_z2_rpm_s = 0.0f;
    wheel_speed_pi_integral = 0.0f;
    wheel_speed_pi_integral_right = 0.0f;
}

#if WHEEL_SPEED_TEST_ENABLE && WHEEL_SPEED_TEST_AUTO_SEQUENCE_ENABLE
static void Chassis_ResetSpeedTestAuto(void)
{
    wheel_speed_test_auto_index = 0U;
    wheel_speed_test_auto_step_tick = 0U;
    wheel_speed_test_auto_running = 0U;
    WheelSpeed_SetTargetRpm(0.0f);
}

static void Chassis_UpdateSpeedTestAuto(void)
{
    uint32_t now_tick = (uint32_t)xTaskGetTickCount();
    uint32_t step_ticks = pdMS_TO_TICKS(WHEEL_SPEED_TEST_AUTO_STEP_MS);
    uint8_t sequence_count =
        (uint8_t)(sizeof(wheel_speed_test_auto_sequence_rpm) /
                  sizeof(wheel_speed_test_auto_sequence_rpm[0]));

    if (step_ticks == 0U) {
        step_ticks = 1U;
    }

    if (wheel_speed_test_auto_running == 0U) {
        wheel_speed_test_auto_index = 0U;
        wheel_speed_test_auto_step_tick = now_tick;
        wheel_speed_test_auto_running = 1U;
        WheelSpeed_SetTargetRpm(
            wheel_speed_test_auto_sequence_rpm[wheel_speed_test_auto_index]);
        return;
    }

    if (((uint32_t)(now_tick - wheel_speed_test_auto_step_tick) >= step_ticks) &&
        (wheel_speed_test_auto_index < (uint8_t)(sequence_count - 1U))) {
        wheel_speed_test_auto_index++;
        wheel_speed_test_auto_step_tick = now_tick;
        WheelSpeed_SetTargetRpm(
            wheel_speed_test_auto_sequence_rpm[wheel_speed_test_auto_index]);
    }
}
#endif

static void Chassis_ResetBalancePid(void)
{
    Chassis_BalancePidInitOnce();
    ChassisPid_Reset(&balance_angle_pid);
    balance_position_m = 0.0f;
    balance_position_ref_m = 0.0f;
    balance_speed_integral_m = 0.0f;
    balance_speed_guard_active = 0U;
    balance_drive_launch_timer_sec = 0.0f;
    balance_stop_brake_timer_sec = 0.0f;
    balance_drive_pulse_timer_sec = 0.0f;
    balance_drive_pulse_brake_timer_sec = 0.0f;
    balance_drive_pulse_hold_timer_sec = 0.0f;
    balance_velocity_lpf_mps = 0.0f;
    balance_velocity_lpf_ready = 0U;
    balance_velocity_ref_ramp_mps = 0.0f;
    balance_last_measured_velocity_mps = 0.0f;
    balance_accel_lpf_mps2 = 0.0f;
    balance_accel_lpf_ready = 0U;
    balance_zero_bias_frozen = 0U;
    balance_pid_theta_deg = global_balance_angle;
    balance_pid_theta_ref_deg = 0.0f;
    balance_pid_manual_bias_deg = balance_manual_bias_deg;
    balance_pid_theta_rate_dps = global_balance_rate;
    balance_pid_accel_angle_deg = global_balance_accel_angle;
    balance_pid_position_m = 0.0f;
    balance_pid_position_ref_m = 0.0f;
    balance_pid_velocity_mps = 0.0f;
    balance_pid_velocity_ref_mps = 0.0f;
    balance_pid_accel_mps2 = 0.0f;
    balance_pid_accel_theta_ref_deg = 0.0f;
    balance_pid_wheel_diff_mps = 0.0f;
    balance_pid_speed_theta_ref_deg = 0.0f;
    balance_pid_integral_theta_ref_deg = 0.0f;
    balance_pid_angle_speed_ref_rpm = 0.0f;
    balance_pid_rate_speed_ref_rpm = 0.0f;
    balance_pid_balance_speed_ref_rpm = 0.0f;
    balance_pid_current_target_a = 0.0f;
    balance_pid_current_cmd_a = 0.0f;
    balance_pid_current_feedback_a = 0.0f;
    balance_pid_turn_speed_ref_rpm = 0.0f;
    balance_pid_output_left_a = 0.0f;
    balance_pid_output_right_a = 0.0f;
    Chassis_ResetSpeedCtrl();
}

static void Chassis_ResetModeIntegrator(void)
{
    balance_position_ref_m = BALANCE_PID_POSITION_SIGN * balance_position_m;
    balance_velocity_ref_ramp_mps = 0.0f;
    balance_speed_integral_m = 0.0f;
    balance_drive_launch_timer_sec = 0.0f;
    balance_stop_brake_timer_sec = 0.0f;
    balance_drive_pulse_timer_sec = 0.0f;
    balance_drive_pulse_brake_timer_sec = 0.0f;
    balance_drive_pulse_hold_timer_sec = 0.0f;
    Chassis_ResetSpeedCtrl();
}

static float Chassis_GetWheelSpeedRpm(const volatile Motor_Measure_t *motor)
{
    if (motor == NULL) {
        return 0.0f;
    }

    return motor->speed_can_rpm;
}

static float Chassis_GetBalanceAngleDeg(void)
{
    float kalman_weight = Chassis_ClampF(BALANCE_PID_KALMAN_ANGLE_WEIGHT,
                                         0.0f,
                                         1.0f);
    float accel_weight = Chassis_ClampF(BALANCE_PID_ACCEL_ANGLE_WEIGHT,
                                        0.0f,
                                        1.0f);
    float weight_sum = kalman_weight + accel_weight;

    if (weight_sum <= 0.0001f) {
        return global_balance_angle;
    }

    return (kalman_weight * global_balance_angle +
            accel_weight * global_balance_accel_angle) /
           weight_sum;
}

static void Chassis_UpdateBalanceViewZeroOutput(void)
{
    float left_rpm = Chassis_GetWheelSpeedRpm(&left_wheel_motor);
    float right_rpm = Chassis_GetWheelSpeedRpm(&right_wheel_motor);
    float velocity_mps = 0.5f * (left_rpm + right_rpm) *
                         BALANCE_RPM_TO_MPS;
    float wheel_diff_mps = (right_rpm - left_rpm) * BALANCE_RPM_TO_MPS;

    balance_pid_theta_deg = Chassis_GetBalanceAngleDeg();
    balance_pid_theta_ref_deg = 0.0f;
    balance_pid_manual_bias_deg = balance_manual_bias_deg;
    balance_pid_theta_rate_dps = global_balance_rate;
    balance_pid_accel_angle_deg = global_balance_accel_angle;
    balance_pid_position_m = BALANCE_PID_POSITION_SIGN * balance_position_m;
    balance_pid_position_ref_m = balance_position_ref_m;
    balance_pid_velocity_mps = BALANCE_PID_VELOCITY_SIGN * velocity_mps;
    balance_pid_velocity_ref_mps = 0.0f;
    balance_pid_accel_mps2 = 0.0f;
    balance_pid_accel_theta_ref_deg = 0.0f;
    balance_pid_wheel_diff_mps = wheel_diff_mps;
    balance_pid_speed_theta_ref_deg = 0.0f;
    balance_pid_integral_theta_ref_deg = 0.0f;
    balance_pid_angle_speed_ref_rpm = 0.0f;
    balance_pid_rate_speed_ref_rpm = 0.0f;
    balance_pid_balance_speed_ref_rpm = 0.0f;
    balance_pid_current_target_a = 0.0f;
    balance_pid_current_cmd_a = 0.0f;
    balance_pid_current_feedback_a = 0.5f * (left_wheel_motor.current +
                                             right_wheel_motor.current);
    balance_pid_turn_speed_ref_rpm = 0.0f;
    balance_pid_output_left_a = 0.0f;
    balance_pid_output_right_a = 0.0f;
    wheel_speed_ref_left_rpm = 0.0f;
    wheel_speed_actual_left_rpm = left_rpm;
    wheel_speed_output_left_a = 0.0f;
    wheel_speed_ref_right_rpm = 0.0f;
    wheel_speed_actual_right_rpm = right_rpm;
    wheel_speed_output_right_a = 0.0f;
}

static float Chassis_SpeedPidUpdate(float ref_rpm,
                                    float feedback_rpm,
                                    float polarity,
                                    float *integral,
                                    float *last_output,
                                    float dt_sec,
                                    float current_limit_a,
                                    float current_slew_a_per_sec)
{
    float speed_error;
    float proportional_error;
    float error_deadband;
    float speed_ff;
    float u_unsat;
    float output;
    float max_current_step;

    if ((integral == NULL) || (last_output == NULL)) {
        return 0.0f;
    }

    speed_error = ref_rpm - feedback_rpm;
    proportional_error = speed_error;
    error_deadband = WHEEL_SPEED_PID_ERROR_DEADBAND_RPM;
    if (error_deadband > 0.0f) {
        if (proportional_error > error_deadband) {
            proportional_error -= error_deadband;
        } else if (proportional_error < -error_deadband) {
            proportional_error += error_deadband;
        } else {
            proportional_error = 0.0f;
        }
    }

    if (WHEEL_SPEED_PID_KI_A_PER_RPM_S != 0.0f) {
        *integral += speed_error * dt_sec;
        *integral = Chassis_LimitAbs(*integral, WHEEL_SPEED_PID_INTEGRAL_LIMIT);
    } else {
        *integral = 0.0f;
    }

    speed_ff = Chassis_SpeedFeedforward(ref_rpm);
#if WHEEL_SPEED_FF_DISABLE_WHEN_BRAKING
    if ((ref_rpm * feedback_rpm > 0.0f) &&
        (Chassis_AbsF(ref_rpm) < Chassis_AbsF(feedback_rpm))) {
        speed_ff = 0.0f;
    }
#endif

    u_unsat = (speed_ff +
               WHEEL_SPEED_PID_KP_A_PER_RPM * proportional_error +
               WHEEL_SPEED_PID_KI_A_PER_RPM_S * (*integral) -
               WHEEL_SPEED_PID_KD_A_PER_RPM * feedback_rpm) *
              polarity;
    output = Chassis_LimitAbs(u_unsat, current_limit_a);
    max_current_step = current_slew_a_per_sec * dt_sec;
    output = Chassis_StepTowards(*last_output, output, max_current_step);

    if ((WHEEL_SPEED_PID_KI_A_PER_RPM_S != 0.0f) &&
        (output != u_unsat)) {
        *integral -= speed_error * dt_sec;
    }

    *last_output = output;
    return output;
}

static void Chassis_CalcSpeedTestOutput(float *left_current, float *right_current)
{
    float left_ref = 0.0f;
    float right_ref = 0.0f;
    float left_out = 0.0f;
    float right_out = 0.0f;
    float left_feedback_rpm = 0.0f;
    float right_feedback_rpm = 0.0f;

    if (chassis_motor_state == CHASSIS_MOTOR_SPEED_TEST) {
        float target_step = WHEEL_SPEED_TARGET_RAMP_RPM_PER_SEC *
                            WHEEL_SPEED_TEST_DT_SEC;
        float left_feedback_alpha = LEFT_WHEEL_SPEED_FEEDBACK_LPF_ALPHA;
        float right_feedback_alpha = RIGHT_WHEEL_SPEED_FEEDBACK_LPF_ALPHA;

#if WHEEL_SPEED_TEST_ENABLE && WHEEL_SPEED_TEST_AUTO_SEQUENCE_ENABLE
        Chassis_UpdateSpeedTestAuto();
#endif

        left_feedback_alpha = Chassis_ClampF(left_feedback_alpha, 0.0f, 1.0f);
        right_feedback_alpha = Chassis_ClampF(right_feedback_alpha, 0.0f, 1.0f);

        wheel_speed_target_ramp_rpm = Chassis_StepTowards(wheel_speed_target_ramp_rpm,
                                                          wheel_speed_target_rpm,
                                                          target_step);
        left_ref = wheel_speed_target_ramp_rpm;
        right_ref = wheel_speed_target_ramp_rpm;

        if (wheel_speed_feedback_lpf_ready == 0U) {
            left_speed_feedback_lpf = left_wheel_motor.speed_can_rpm;
            right_speed_feedback_lpf = right_wheel_motor.speed_can_rpm;
            wheel_speed_feedback_lpf_ready = 1U;
        } else {
            left_speed_feedback_lpf += left_feedback_alpha *
                                       (left_wheel_motor.speed_can_rpm -
                                        left_speed_feedback_lpf);
            right_speed_feedback_lpf += right_feedback_alpha *
                                        (right_wheel_motor.speed_can_rpm -
                                         right_speed_feedback_lpf);
        }
        left_feedback_rpm = left_speed_feedback_lpf;
        right_feedback_rpm = right_speed_feedback_lpf;

        left_out = Chassis_SpeedPidUpdate(left_ref,
                                          left_feedback_rpm,
                                          LEFT_WHEEL_SPEED_PID_POLARITY,
                                          &left_speed_pid_integral,
                                          &left_speed_last_output,
                                          WHEEL_SPEED_TEST_DT_SEC,
                                          WHEEL_SPEED_TEST_CURRENT_LIMIT_A,
                                          WHEEL_SPEED_CURRENT_SLEW_A_PER_SEC);
        right_out = Chassis_SpeedPidUpdate(right_ref,
                                           right_feedback_rpm,
                                           RIGHT_WHEEL_SPEED_PID_POLARITY,
                                           &right_speed_pid_integral,
                                           &right_speed_last_output,
                                           WHEEL_SPEED_TEST_DT_SEC,
                                           WHEEL_SPEED_TEST_CURRENT_LIMIT_A,
                                           WHEEL_SPEED_CURRENT_SLEW_A_PER_SEC);

#if WHEEL_SPEED_TEST_UNIDIRECTIONAL
        if (((left_ref > 0.0f) && (left_out < 0.0f)) ||
            ((left_ref < 0.0f) && (left_out > 0.0f))) {
            left_out = 0.0f;
        }
        if (((right_ref > 0.0f) && (right_out < 0.0f)) ||
            ((right_ref < 0.0f) && (right_out > 0.0f))) {
            right_out = 0.0f;
        }
#endif
    } else {
        Chassis_ResetSpeedCtrl();
    }

    wheel_speed_ref_left_rpm = left_ref;
    wheel_speed_actual_left_rpm = left_feedback_rpm;
    wheel_speed_output_left_a = left_out;
    wheel_speed_ref_right_rpm = right_ref;
    wheel_speed_actual_right_rpm = right_feedback_rpm;
    wheel_speed_output_right_a = right_out;
    wheel_speed_feedback_left_a = left_wheel_motor.current;
    wheel_speed_feedback_right_a = right_wheel_motor.current;
    wheel_speed_eso_z1_rpm = left_feedback_rpm;
    wheel_speed_eso_z2_rpm_s = right_feedback_rpm;
    wheel_speed_pi_integral = left_speed_pid_integral;
    wheel_speed_pi_integral_right = right_speed_pid_integral;

    *left_current = left_out;
    *right_current = right_out;
}

#if BALANCE_PID_ENABLE
static float Chassis_ModeTargetVelocityMps(ChassisMode_e mode)
{
    if (mode == CHASSIS_MODE_FORWARD) {
        return BALANCE_MODE_DRIVE_VELOCITY_MPS;
    }
    if (mode == CHASSIS_MODE_BACKWARD) {
        return -BALANCE_MODE_DRIVE_VELOCITY_MPS;
    }
    return 0.0f;
}

static float Chassis_ModeTargetBiasDeg(ChassisMode_e mode)
{
    if (mode == CHASSIS_MODE_FORWARD) {
        return BALANCE_MODE_FORWARD_BIAS_DEG;
    }
    if (mode == CHASSIS_MODE_BACKWARD) {
        return BALANCE_MODE_BACKWARD_BIAS_DEG;
    }
    return BALANCE_MODE_STAND_BIAS_DEG;
}

static float Chassis_ModeTurnSpeedRpm(ChassisMode_e mode)
{
    if (mode == CHASSIS_MODE_TURN_LEFT) {
        return BALANCE_MODE_TURN_SPEED_RPM;
    }
    if (mode == CHASSIS_MODE_TURN_RIGHT) {
        return -BALANCE_MODE_TURN_SPEED_RPM;
    }
    return 0.0f;
}
#endif

#if BALANCE_PID_ENABLE
static void Chassis_CalcBalancePidOutput(float *left_current, float *right_current)
{
    float left_rpm;
    float right_rpm;
    float velocity_raw_mps;
    float measured_velocity_mps;
    float measured_position_m;
    float velocity_alpha;
    float wheel_diff_mps;
    float theta_deg;
    float accel_angle_deg;
    float theta_ref_deg;
    float theta_rate_dps;
    float theta_error_deg;
    float target_velocity_mps;
    float mode_target_velocity_mps;
    float speed_error_mps;
    float accel_raw_mps2;
    float accel_error_mps2;
    float accel_theta_ref_deg;
    float position_error_m;
    float speed_theta_ref_deg;
    float mode_bias_deg;
    float drive_base_rpm;
    float stop_brake_rpm;
    float launch_bias_deg;
    float balance_speed_ref_rpm;
    float turn_speed_ref_rpm;
    float left_ref_rpm;
    float right_ref_rpm;
    float left_out;
    float right_out;
    ChassisMode_e previous_mode;
    uint8_t drive_bias_mode;
    uint8_t drive_launch_active;
    uint8_t drive_pulse_brake_active;
    uint8_t drive_pulse_hold_active;
    uint8_t stop_brake_active;

    if (chassis_motor_state != CHASSIS_MOTOR_BALANCE_PID) {
        Chassis_ResetBalancePid();
        Chassis_UpdateBalanceViewZeroOutput();
        *left_current = 0.0f;
        *right_current = 0.0f;
        return;
    }

    Chassis_BalancePidInitOnce();

    theta_deg = Chassis_GetBalanceAngleDeg();
#if BALANCE_ZERO_TRIM_ENABLE
    theta_deg -= balance_zero_trim_deg;
#endif
    accel_angle_deg = global_balance_accel_angle;
    theta_rate_dps = global_balance_rate;

    if (Chassis_AbsF(theta_deg) > BALANCE_PID_SAFE_ANGLE_DEG) {
        chassis_fault_code = CHASSIS_FAULT_PID_ANGLE;
        Chassis_SetControlMode(CHASSIS_MODE_STAND);
        chassis_speed_test_armed = 0U;
        emergency_stop = 1U;
        Chassis_ResetBalancePid();
        Chassis_UpdateBalanceViewZeroOutput();
        *left_current = 0.0f;
        *right_current = 0.0f;
        return;
    }

    if (Chassis_AbsF(theta_rate_dps) > BALANCE_PID_SAFE_RATE_DPS) {
        chassis_fault_code = CHASSIS_FAULT_PID_RATE;
        Chassis_SetControlMode(CHASSIS_MODE_STAND);
        chassis_speed_test_armed = 0U;
        emergency_stop = 1U;
        Chassis_ResetBalancePid();
        Chassis_UpdateBalanceViewZeroOutput();
        *left_current = 0.0f;
        *right_current = 0.0f;
        return;
    }

    left_rpm = Chassis_GetWheelSpeedRpm(&left_wheel_motor);
    right_rpm = Chassis_GetWheelSpeedRpm(&right_wheel_motor);
    velocity_raw_mps = 0.5f * (left_rpm + right_rpm) *
                       BALANCE_RPM_TO_MPS;
    velocity_raw_mps = Chassis_LimitAbs(velocity_raw_mps,
                                        BALANCE_PID_VELOCITY_LIMIT_MPS);

    velocity_alpha = Chassis_ClampF(BALANCE_PID_VELOCITY_LPF_ALPHA, 0.0f, 1.0f);
    if (balance_velocity_lpf_ready == 0U) {
        balance_velocity_lpf_mps = velocity_raw_mps;
        balance_velocity_lpf_ready = 1U;
    } else {
        balance_velocity_lpf_mps += velocity_alpha *
                                    (velocity_raw_mps -
                                     balance_velocity_lpf_mps);
    }

    balance_position_m += balance_velocity_lpf_mps * BALANCE_PID_DT_SEC;
    balance_position_m = Chassis_LimitAbs(balance_position_m,
                                          BALANCE_PID_POSITION_LIMIT_M);
    measured_velocity_mps = BALANCE_PID_VELOCITY_SIGN *
                            balance_velocity_lpf_mps;
    measured_position_m = BALANCE_PID_POSITION_SIGN * balance_position_m;
    wheel_diff_mps = (right_rpm - left_rpm) * BALANCE_RPM_TO_MPS;

    accel_raw_mps2 = (measured_velocity_mps -
                      balance_last_measured_velocity_mps) /
                     BALANCE_PID_DT_SEC;
    if (balance_accel_lpf_ready == 0U) {
        balance_accel_lpf_mps2 = 0.0f;
        balance_accel_lpf_ready = 1U;
    } else {
        balance_accel_lpf_mps2 += velocity_alpha *
                                  (accel_raw_mps2 -
                                   balance_accel_lpf_mps2);
    }
    balance_last_measured_velocity_mps = measured_velocity_mps;

#if BALANCE_ZERO_TRIM_ENABLE
    /*
     * 零点自学习:静止直立时 theta 均值就是真实零点。
     * 门控确保只在"stand + 车不动 + 轮不转 + 姿态安静"时学习,
     * 时间常数 1/0.02 = 50s,与直立环(~1s)动力学解耦。
     */
    if ((chassis_mode == CHASSIS_MODE_STAND) &&
        (Chassis_AbsF(theta_deg) < BALANCE_ZERO_TRIM_GATE_THETA_DEG) &&
        (Chassis_AbsF(theta_rate_dps) < BALANCE_ZERO_TRIM_GATE_RATE_DPS) &&
        (Chassis_AbsF(measured_velocity_mps) < BALANCE_ZERO_TRIM_GATE_V_MPS) &&
        (Chassis_AbsF(left_rpm) < BALANCE_ZERO_TRIM_GATE_WHEEL_RPM) &&
        (Chassis_AbsF(right_rpm) < BALANCE_ZERO_TRIM_GATE_WHEEL_RPM)) {
        balance_zero_trim_deg += BALANCE_ZERO_TRIM_RATE_PER_SEC *
                                 theta_deg * BALANCE_PID_DT_SEC;
        balance_zero_trim_deg =
            Chassis_LimitAbs(balance_zero_trim_deg,
                             BALANCE_ZERO_TRIM_LIMIT_DEG);
    }
#endif

    target_velocity_mps = Chassis_ModeTargetVelocityMps(chassis_mode);
    mode_target_velocity_mps = target_velocity_mps;
    mode_bias_deg = Chassis_ModeTargetBiasDeg(chassis_mode);
    drive_bias_mode = ((chassis_mode == CHASSIS_MODE_FORWARD) ||
                       (chassis_mode == CHASSIS_MODE_BACKWARD)) ? 1U : 0U;
    if (chassis_mode != balance_integral_last_mode) {
        previous_mode = balance_integral_last_mode;
        balance_speed_integral_m = 0.0f;
        balance_integral_last_mode = chassis_mode;
        if (drive_bias_mode != 0U) {
#if BALANCE_MODE_LAUNCH_ENABLE
            balance_drive_launch_timer_sec = BALANCE_MODE_LAUNCH_TIME_SEC;
#else
            balance_drive_launch_timer_sec = 0.0f;
#endif
            balance_stop_brake_timer_sec = 0.0f;
#if BALANCE_MODE_PULSE_DRIVE_ENABLE
            balance_drive_pulse_timer_sec =
                BALANCE_MODE_PULSE_DRIVE_TIME_SEC;
#else
            balance_drive_pulse_timer_sec = 0.0f;
#endif
            balance_drive_pulse_brake_timer_sec = 0.0f;
            balance_drive_pulse_hold_timer_sec = 0.0f;
        } else {
            balance_drive_launch_timer_sec = 0.0f;
            balance_drive_pulse_timer_sec = 0.0f;
            balance_drive_pulse_brake_timer_sec = 0.0f;
            balance_drive_pulse_hold_timer_sec = 0.0f;
#if BALANCE_MODE_STOP_BRAKE_ENABLE
            if ((chassis_mode == CHASSIS_MODE_STAND) &&
                ((previous_mode == CHASSIS_MODE_FORWARD) ||
                 (previous_mode == CHASSIS_MODE_BACKWARD))) {
                balance_stop_brake_timer_sec =
                    BALANCE_MODE_STOP_BRAKE_TIME_SEC;
            } else {
                balance_stop_brake_timer_sec = 0.0f;
            }
#else
            balance_stop_brake_timer_sec = 0.0f;
#endif
        }
    }
    /*
     * 闭环限速守卫(带迟滞):超速时撤销行走速度目标,让外环全部权限
     * 用于刹车。只改 target_velocity_mps,不触碰直立角度环。
     */
    if (Chassis_AbsF(measured_velocity_mps) > BALANCE_SPEED_GUARD_ON_MPS) {
        balance_speed_guard_active = 1U;
    } else if (Chassis_AbsF(measured_velocity_mps) <
               BALANCE_SPEED_GUARD_OFF_MPS) {
        balance_speed_guard_active = 0U;
    }
    if (balance_speed_guard_active != 0U) {
        target_velocity_mps = 0.0f;
        /* 不等斜坡,立刻撤销速度目标,让 Kv 项全额刹车 */
        balance_velocity_ref_ramp_mps = 0.0f;
    }
    drive_launch_active =
        ((drive_bias_mode != 0U) &&
         (balance_drive_launch_timer_sec > 0.0f)) ? 1U : 0U;
    if (drive_launch_active != 0U) {
        target_velocity_mps = 0.0f;
        balance_velocity_ref_ramp_mps = 0.0f;
        balance_drive_launch_timer_sec =
            Chassis_StepTowards(balance_drive_launch_timer_sec,
                                0.0f,
                                BALANCE_PID_DT_SEC);
    }
    drive_pulse_brake_active = 0U;
    drive_pulse_hold_active = 0U;
#if BALANCE_MODE_PULSE_DRIVE_ENABLE
    if (drive_bias_mode != 0U) {
        if (balance_drive_pulse_brake_timer_sec > 0.0f) {
            drive_pulse_brake_active = 1U;
        } else if (balance_drive_pulse_hold_timer_sec > 0.0f) {
            drive_pulse_hold_active = 1U;
        } else if ((drive_launch_active == 0U) &&
                   ((balance_drive_pulse_timer_sec <= 0.0f) ||
                    (Chassis_AbsF(measured_velocity_mps) >
                     BALANCE_MODE_PULSE_BRAKE_ON_MPS))) {
            balance_drive_pulse_brake_timer_sec =
                BALANCE_MODE_PULSE_BRAKE_TIME_SEC;
            drive_pulse_brake_active = 1U;
        } else if (drive_launch_active == 0U) {
            balance_drive_pulse_timer_sec =
                Chassis_StepTowards(balance_drive_pulse_timer_sec,
                                    0.0f,
                                    BALANCE_PID_DT_SEC);
        }

        if (drive_pulse_brake_active != 0U) {
            float resume_timer_sec = BALANCE_MODE_PULSE_BRAKE_TIME_SEC -
                                     BALANCE_MODE_PULSE_BRAKE_MIN_TIME_SEC;
            if (resume_timer_sec < 0.0f) {
                resume_timer_sec = 0.0f;
            }
            target_velocity_mps = 0.0f;
            balance_velocity_ref_ramp_mps = 0.0f;
            balance_drive_pulse_brake_timer_sec =
                Chassis_StepTowards(balance_drive_pulse_brake_timer_sec,
                                    0.0f,
                                    BALANCE_PID_DT_SEC);
            if ((balance_drive_pulse_brake_timer_sec <= 0.0f) ||
                ((balance_drive_pulse_brake_timer_sec <= resume_timer_sec) &&
                 (Chassis_AbsF(measured_velocity_mps) <
                 BALANCE_MODE_PULSE_RESUME_MPS))) {
                balance_drive_pulse_brake_timer_sec = 0.0f;
                if (BALANCE_MODE_PULSE_HOLD_TIME_SEC > 0.0f) {
                    balance_drive_pulse_hold_timer_sec =
                        BALANCE_MODE_PULSE_HOLD_TIME_SEC;
                    drive_pulse_hold_active = 1U;
                } else {
                    balance_drive_pulse_timer_sec =
                        BALANCE_MODE_PULSE_DRIVE_TIME_SEC;
                }
                drive_pulse_brake_active = 0U;
            }
        } else if (drive_pulse_hold_active != 0U) {
            target_velocity_mps = 0.0f;
            balance_velocity_ref_ramp_mps = 0.0f;
            balance_drive_pulse_hold_timer_sec =
                Chassis_StepTowards(balance_drive_pulse_hold_timer_sec,
                                    0.0f,
                                    BALANCE_PID_DT_SEC);
            if (balance_drive_pulse_hold_timer_sec <= 0.0f) {
                balance_drive_pulse_hold_timer_sec = 0.0f;
                balance_drive_pulse_timer_sec =
                    BALANCE_MODE_PULSE_DRIVE_TIME_SEC;
                drive_pulse_hold_active = 0U;
            }
        }
    }
#else
    (void)drive_pulse_brake_active;
    (void)drive_pulse_hold_active;
#endif
    stop_brake_active =
        ((((chassis_mode == CHASSIS_MODE_STAND) &&
           (balance_stop_brake_timer_sec > 0.0f) &&
           (Chassis_AbsF(measured_velocity_mps) >
            BALANCE_MODE_STOP_BRAKE_OFF_MPS)) ||
          (drive_pulse_brake_active != 0U)) ? 1U : 0U);
    if ((stop_brake_active != 0U) &&
        (Chassis_AbsF(theta_deg) > BALANCE_MODE_BRAKE_RECOVER_ANGLE_DEG)) {
        stop_brake_active = 0U;
    }
    if (stop_brake_active != 0U) {
        balance_stop_brake_timer_sec =
            Chassis_StepTowards(balance_stop_brake_timer_sec,
                                0.0f,
                                BALANCE_PID_DT_SEC);
    } else if (chassis_mode == CHASSIS_MODE_STAND) {
        balance_stop_brake_timer_sec = 0.0f;
    }
    balance_velocity_ref_ramp_mps =
        Chassis_StepTowards(balance_velocity_ref_ramp_mps,
                            target_velocity_mps,
                            BALANCE_MODE_VELOCITY_RAMP_MPS_PER_SEC *
                            BALANCE_PID_DT_SEC);
    if (drive_bias_mode != 0U) {
        balance_position_ref_m = measured_position_m;
    } else {
        balance_position_ref_m += balance_velocity_ref_ramp_mps *
                                  BALANCE_PID_DT_SEC;
    }
    balance_position_ref_m = Chassis_LimitAbs(balance_position_ref_m,
                                              BALANCE_PID_POSITION_LIMIT_M);

    speed_error_mps = balance_velocity_ref_ramp_mps - measured_velocity_mps;
    if (Chassis_AbsF(speed_error_mps) < BALANCE_PID_WHEEL_SPEED_DEADBAND_MPS) {
        speed_error_mps = 0.0f;
    }
    position_error_m = balance_position_ref_m - measured_position_m;
    if (drive_bias_mode != 0U) {
        position_error_m = 0.0f;
    }
    accel_error_mps2 = 0.0f - balance_accel_lpf_mps2;
    accel_theta_ref_deg = BALANCE_POSITION_KA_DEG_PER_MPS2 *
                          accel_error_mps2;

    /*
     * 速度积分只在行走模式工作,积分对象是速度误差(单位 m),
     * 收敛到维持目标速度所需倾角(克服静摩擦与残余零点误差)。
     * 守卫激活期间积分向 0 衰减而不是冻结:起步顶静摩擦时积分可能
     * 已攒满 +1.5deg,冻结会在超速时继续同向助推(v=0.4 时速度项
     * -1.0deg 压不过它);衰减 0.5s 内清完,之后刹车全额生效,
     * 且衰减到 0 即停,不会积成反向,windup 保护仍成立。
     * stand 模式不积分,保持原 P+V 行为不变。
     */
    if ((drive_launch_active != 0U) || (drive_pulse_brake_active != 0U) ||
        (stop_brake_active != 0U) ||
        (drive_pulse_hold_active != 0U)) {
        balance_speed_integral_m = 0.0f;
    } else if (drive_bias_mode != 0U) {
        if (balance_speed_guard_active == 0U) {
            balance_speed_integral_m += speed_error_mps * BALANCE_PID_DT_SEC;
        } else {
            balance_speed_integral_m = Chassis_StepTowards(
                balance_speed_integral_m,
                0.0f,
                BALANCE_SPEED_GUARD_INTEGRAL_DECAY_M_PER_S *
                BALANCE_PID_DT_SEC);
        }
    } else {
        balance_speed_integral_m = 0.0f;
    }
    balance_speed_integral_m =
        Chassis_LimitAbs(balance_speed_integral_m,
                         BALANCE_POSITION_INTEGRAL_LIMIT_M_S);
    speed_theta_ref_deg =
        BALANCE_POSITION_KP_DEG_PER_M * position_error_m +
        BALANCE_POSITION_KV_DEG_PER_MPS * speed_error_mps +
        accel_theta_ref_deg +
        BALANCE_POSITION_KI_DEG_PER_M_S * balance_speed_integral_m;
    speed_theta_ref_deg = Chassis_LimitAbs(speed_theta_ref_deg,
                                           BALANCE_PID_THETA_REF_LIMIT_DEG);
    if (drive_launch_active != 0U) {
        launch_bias_deg = (mode_target_velocity_mps >= 0.0f) ?
                          BALANCE_MODE_LAUNCH_BIAS_DEG :
                          -BALANCE_MODE_LAUNCH_BIAS_DEG;
        speed_theta_ref_deg = launch_bias_deg;
    }

    theta_ref_deg = balance_manual_bias_deg + mode_bias_deg +
                    speed_theta_ref_deg;
    theta_ref_deg = Chassis_LimitAbs(theta_ref_deg,
                                     BALANCE_PID_THETA_REF_LIMIT_DEG);
    theta_error_deg = theta_deg - theta_ref_deg;

    balance_speed_ref_rpm = ChassisPid_UpdateAngle(&balance_angle_pid,
                                                   theta_error_deg,
                                                   theta_rate_dps,
                                                   BALANCE_PID_DT_SEC);
    balance_speed_ref_rpm =
        Chassis_LimitAbs(balance_speed_ref_rpm,
                         balance_pid_runtime_speed_limit_rpm);
    if (stop_brake_active != 0U) {
        balance_speed_ref_rpm *= BALANCE_MODE_STOP_BRAKE_BALANCE_SCALE;
    }

    turn_speed_ref_rpm = Chassis_ModeTurnSpeedRpm(chassis_mode);
    turn_speed_ref_rpm = Chassis_LimitAbs(turn_speed_ref_rpm,
                                          balance_pid_runtime_speed_limit_rpm);
    /* Legacy fixed-feedforward note; active code below uses actual-base. */
    /*
     * 行走速度前馈:把 v_ref 直接换算成轮速(0.10m/s≈31rpm)加到左右轮
     * 目标上,由轮速内环(自带积分+静摩擦前馈)破静摩擦持续滚动;
     * 外环倾角只做平衡/加减速修正,不再靠攒倾角顶静摩擦。
     * 依据:纯倾角方案实测为 1.5s 周期 stick-slip 极限环(积分顶满
     * 1.5deg、theta 倾 1.36deg 车仍原地抖,v 均值仅 0.007m/s)。
     * 守卫触发时 ramp 已被置 0,前馈随之立即消失。
    */
    /*
     * Current drive structure: v_ref only shapes theta_ref. The speed
     * controller base follows measured wheel speed, so balance correction
     * cannot be cancelled by a fixed target-speed feedforward at standstill.
     */
    drive_base_rpm = 0.0f;
    if ((drive_bias_mode != 0U) &&
        (drive_launch_active == 0U) &&
        (drive_pulse_brake_active == 0U) &&
        (drive_pulse_hold_active == 0U) &&
        (stop_brake_active == 0U)) {
#if BALANCE_MODE_DRIVE_USE_ACTUAL_BASE
        drive_base_rpm = 0.5f * (left_rpm + right_rpm);
#if BALANCE_MODE_PULSE_DRIVE_ENABLE
        if (mode_target_velocity_mps >= 0.0f) {
            drive_base_rpm += BALANCE_MODE_PULSE_DRIVE_BASE_RPM;
        } else {
            drive_base_rpm -= BALANCE_MODE_PULSE_DRIVE_BASE_RPM;
        }
#endif
#else
        drive_base_rpm = BALANCE_PID_VELOCITY_SIGN *
                         balance_velocity_ref_ramp_mps /
                         BALANCE_RPM_TO_MPS;
#endif
    }
    stop_brake_rpm = 0.0f;
    if (stop_brake_active != 0U) {
        stop_brake_rpm = -BALANCE_MODE_STOP_BRAKE_GAIN *
                         measured_velocity_mps /
                         BALANCE_RPM_TO_MPS;
        stop_brake_rpm =
            Chassis_LimitAbs(stop_brake_rpm,
                             BALANCE_MODE_STOP_BRAKE_LIMIT_RPM);
    }
    left_ref_rpm = balance_speed_ref_rpm + drive_base_rpm + stop_brake_rpm +
                   BALANCE_PID_LEFT_YAW_SIGN * turn_speed_ref_rpm;
    right_ref_rpm = balance_speed_ref_rpm + drive_base_rpm + stop_brake_rpm +
                    BALANCE_PID_RIGHT_YAW_SIGN * turn_speed_ref_rpm;
    left_ref_rpm = Chassis_LimitAbs(left_ref_rpm,
                                    balance_pid_runtime_speed_limit_rpm);
    right_ref_rpm = Chassis_LimitAbs(right_ref_rpm,
                                     balance_pid_runtime_speed_limit_rpm);

    left_out = Chassis_SpeedPidUpdate(left_ref_rpm,
                                      left_rpm,
                                      LEFT_WHEEL_SPEED_PID_POLARITY,
                                      &left_speed_pid_integral,
                                      &left_speed_last_output,
                                      BALANCE_PID_DT_SEC,
                                      balance_pid_runtime_current_limit_a,
                                      balance_pid_runtime_current_slew_a_per_sec);
    right_out = Chassis_SpeedPidUpdate(right_ref_rpm,
                                       right_rpm,
                                       RIGHT_WHEEL_SPEED_PID_POLARITY,
                                       &right_speed_pid_integral,
                                       &right_speed_last_output,
                                       BALANCE_PID_DT_SEC,
                                       balance_pid_runtime_current_limit_a,
                                       balance_pid_runtime_current_slew_a_per_sec);

    balance_pid_theta_deg = theta_deg;
    balance_pid_theta_ref_deg = theta_ref_deg;
    balance_pid_manual_bias_deg = balance_manual_bias_deg;
    balance_pid_theta_rate_dps = theta_rate_dps;
    balance_pid_accel_angle_deg = accel_angle_deg;
    balance_pid_position_m = measured_position_m;
    balance_pid_position_ref_m = balance_position_ref_m;
    balance_pid_velocity_mps = measured_velocity_mps;
    balance_pid_velocity_ref_mps = balance_velocity_ref_ramp_mps;
    balance_pid_accel_mps2 = balance_accel_lpf_mps2;
    balance_pid_accel_theta_ref_deg = accel_theta_ref_deg;
    balance_pid_wheel_diff_mps = wheel_diff_mps;
    balance_pid_speed_theta_ref_deg = speed_theta_ref_deg;
    balance_pid_integral_theta_ref_deg =
        BALANCE_POSITION_KI_DEG_PER_M_S * balance_speed_integral_m;
    balance_pid_zero_trim_deg = balance_zero_trim_deg;
    balance_pid_angle_speed_ref_rpm =
        BALANCE_PID_POLARITY *
        balance_pid_runtime_kp * theta_error_deg;
    balance_pid_rate_speed_ref_rpm =
        BALANCE_PID_POLARITY *
        balance_pid_runtime_kd * theta_rate_dps;
    balance_pid_balance_speed_ref_rpm = balance_speed_ref_rpm;
    balance_pid_turn_speed_ref_rpm = turn_speed_ref_rpm;
    balance_pid_current_target_a = 0.5f * (left_out + right_out);
    balance_pid_current_cmd_a = 0.5f * (left_out + right_out);
    balance_pid_current_feedback_a = 0.5f * (left_wheel_motor.current +
                                             right_wheel_motor.current);
    balance_pid_output_left_a = left_out;
    balance_pid_output_right_a = right_out;

    wheel_speed_ref_left_rpm = left_ref_rpm;
    wheel_speed_actual_left_rpm = left_rpm;
    wheel_speed_output_left_a = left_out;
    wheel_speed_ref_right_rpm = right_ref_rpm;
    wheel_speed_actual_right_rpm = right_rpm;
    wheel_speed_output_right_a = right_out;
    wheel_speed_feedback_left_a = left_wheel_motor.current;
    wheel_speed_feedback_right_a = right_wheel_motor.current;
    wheel_speed_eso_z1_rpm = global_balance_accel_forward_ms2;
    wheel_speed_eso_z2_rpm_s = wheel_diff_mps;
    wheel_speed_pi_integral = balance_angle_pid.integral;
    wheel_speed_pi_integral_right = balance_speed_integral_m;

    *left_current = left_out;
    *right_current = right_out;
}
#endif

static void Chassis_CalcMotorOutput(float *left_current, float *right_current)
{
#if BALANCE_PID_ENABLE
    if (chassis_motor_state == CHASSIS_MOTOR_BALANCE_PID) {
        Chassis_CalcBalancePidOutput(left_current, right_current);
        return;
    }
#endif

#if WHEEL_SPEED_TEST_ENABLE
    if (chassis_motor_state == CHASSIS_MOTOR_SPEED_TEST) {
        Chassis_CalcSpeedTestOutput(left_current, right_current);
        return;
    }
#endif

    Chassis_ResetSpeedCtrl();
    Chassis_UpdateBalanceViewZeroOutput();
    *left_current = 0.0f;
    *right_current = 0.0f;
}

void Gimbal_Task(void *argument)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(WHEEL_SAFE_TASK_PERIOD_MS);

    (void)argument;

    for (;;) {
        uint32_t now_tick = (uint32_t)xTaskGetTickCount();
        float left_current = 0.0f;
        float right_current = 0.0f;
        uint8_t left_feedback_ok =
            Motor_FeedbackReady(&left_wheel_motor,
                                now_tick,
                                pdMS_TO_TICKS(WHEEL_FEEDBACK_TIMEOUT_MS));
        uint8_t right_feedback_ok =
            Motor_FeedbackReady(&right_wheel_motor,
                                now_tick,
                                pdMS_TO_TICKS(WHEEL_FEEDBACK_TIMEOUT_MS));

        chassis_motor_feedback_ok = (left_feedback_ok && right_feedback_ok) ? 1U : 0U;
        chassis_motor_enabled_ok =
            (left_wheel_motor.enabled && right_wheel_motor.enabled) ? 1U : 0U;

        if ((chassis_motor_feedback_ok == 0U) ||
            (chassis_motor_enabled_ok == 0U)) {
            chassis_fault_code = (chassis_motor_feedback_ok == 0U) ?
                                 CHASSIS_FAULT_FEEDBACK :
                                 CHASSIS_FAULT_ENABLE;
            Chassis_SetControlMode(CHASSIS_MODE_STAND);
            chassis_speed_test_armed = 0U;
            emergency_stop = 1U;
        }

        Chassis_UpdateKey();
#if IMU_YAW_BIAS_KEY_CAL_ENABLE
        if (INS_GetYawCalibrationState() == INS_YAW_CAL_RUNNING) {
            chassis_speed_test_armed = 0U;
            emergency_stop = 1U;
        }
#endif
        Chassis_UpdateState();

        if (chassis_motor_state == CHASSIS_MOTOR_BALANCE_PID) {
            gimbal_state = GIMBAL_STATE_IMU_STAB;
        } else if (chassis_motor_state == CHASSIS_MOTOR_SPEED_TEST) {
            gimbal_state = GIMBAL_STATE_ENC_LOCK;
        } else {
            gimbal_state = GIMBAL_STATE_RELAX;
        }

        if (last_chassis_motor_state != chassis_motor_state) {
            if (last_chassis_motor_state == CHASSIS_MOTOR_BALANCE_PID) {
                Chassis_ResetBalancePid();
            }
#if WHEEL_SPEED_TEST_ENABLE && WHEEL_SPEED_TEST_AUTO_SEQUENCE_ENABLE
            if (last_chassis_motor_state == CHASSIS_MOTOR_SPEED_TEST) {
                Chassis_ResetSpeedTestAuto();
            }
#endif

            if (chassis_motor_state == CHASSIS_MOTOR_SPEED_TEST) {
                Chassis_ResetSpeedCtrl();
#if WHEEL_SPEED_TEST_ENABLE && WHEEL_SPEED_TEST_AUTO_SEQUENCE_ENABLE
                Chassis_ResetSpeedTestAuto();
#endif
            } else if (chassis_motor_state == CHASSIS_MOTOR_BALANCE_PID) {
                Chassis_ResetBalancePid();
            }
        }
        last_chassis_motor_state = chassis_motor_state;

        if (chassis_mode_changed != 0U) {
            chassis_mode_changed = 0U;
            Chassis_ResetModeIntegrator();
        }

        if (wheel_speed_target_changed != 0U) {
            wheel_speed_target_changed = 0U;
            left_speed_pid_integral = 0.0f;
            right_speed_pid_integral = 0.0f;
        }

        Chassis_CalcMotorOutput(&left_current, &right_current);

        Motor_SetCurrent(LEFT_WHEEL_MOTOR_ID, left_current);
        Motor_SetCurrent(RIGHT_WHEEL_MOTOR_ID, right_current);

        Chassis_UpdateLed();

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}
