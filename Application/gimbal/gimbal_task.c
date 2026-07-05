#include "gimbal_task.h"
#include "FreeRTOS.h"
#include "task.h"
#include "ins_task.h"
#include "balance_outer_loop.h"
#include "balance_zero_bias_search.h"
#include "lqr_controller.h"
#include "bsp_can.h"
#include "config.h"
#include "main.h"

volatile GimbalState_e gimbal_state = GIMBAL_STATE_RELAX;
volatile ChassisMotorState_e chassis_motor_state = CHASSIS_MOTOR_ESTOP;
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
volatile float wheel_speed_encoder_left_rpm = 0.0f;
volatile float wheel_speed_encoder_right_rpm = 0.0f;
volatile float wheel_speed_feedback_left_a = 0.0f;
volatile float wheel_speed_feedback_right_a = 0.0f;
volatile float wheel_speed_eso_z1_rpm = 0.0f;
volatile float wheel_speed_eso_z2_rpm_s = 0.0f;
volatile float wheel_speed_pi_integral = 0.0f;
volatile float wheel_speed_pi_integral_right = 0.0f;
volatile float wheel_speed_target_rpm = WHEEL_SPEED_TEST_REF_RPM;
volatile float balance_lqr_theta_deg = 0.0f;
volatile float balance_lqr_theta_ref_deg = 0.0f;
volatile float balance_lqr_zero_bias_deg = 0.0f;
volatile float balance_lqr_theta_rate_dps = 0.0f;
volatile float balance_lqr_position_m = 0.0f;
volatile float balance_lqr_velocity_mps = 0.0f;
volatile float balance_lqr_force_n = 0.0f;
volatile float balance_lqr_force_x_n = 0.0f;
volatile float balance_lqr_force_v_n = 0.0f;
volatile float balance_lqr_force_theta_n = 0.0f;
volatile float balance_lqr_force_rate_n = 0.0f;
volatile float balance_lqr_current_target_a = 0.0f;
volatile float balance_lqr_current_cmd_a = 0.0f;
volatile float balance_lqr_current_feedback_a = 0.0f;
volatile float balance_lqr_output_left_a = 0.0f;
volatile float balance_lqr_output_right_a = 0.0f;
volatile uint8_t balance_zero_bias_frozen = 0U;
float ext_target_yaw = 0.0f;
uint8_t flag_ext_yaw = 0;
volatile uint8_t emergency_stop = 1U;

static ChassisMotorState_e last_chassis_motor_state = CHASSIS_MOTOR_ESTOP;
static volatile uint8_t wheel_speed_target_changed = 0U;
static float left_speed_pid_integral = 0.0f;
static float right_speed_pid_integral = 0.0f;
static float left_speed_feedback_lpf = 0.0f;
static float right_speed_feedback_lpf = 0.0f;
static float left_speed_last_output = 0.0f;
static float right_speed_last_output = 0.0f;
static float wheel_speed_target_ramp_rpm = 0.0f;
static uint8_t wheel_speed_feedback_lpf_ready = 0U;
static LQR_FixedGain_t balance_lqr;
static uint8_t balance_lqr_initialized = 0U;
static float balance_position_m = 0.0f;
static float balance_velocity_lpf_mps = 0.0f;
static float balance_lqr_last_left_a = 0.0f;
static float balance_lqr_last_right_a = 0.0f;
static uint8_t balance_velocity_lpf_ready = 0U;
static float balance_manual_bias_deg = BALANCE_LQR_THETA_REF_DEG;
static BalanceOuterLoop_t balance_outer_loop;
static uint8_t balance_outer_loop_initialized = 0U;
static BalanceZeroBiasSearch_t balance_zero_bias_search;
static uint8_t balance_zero_bias_initialized = 0U;

static float chassis_absf(float value)
{
    return (value >= 0.0f) ? value : -value;
}

static float chassis_limit_abs(float value, float abs_limit)
{
    float limit = chassis_absf(abs_limit);

    if (value > limit) {
        return limit;
    }
    if (value < -limit) {
        return -limit;
    }
    return value;
}

static float chassis_limit_speed_target(float rpm)
{
    float abs_rpm = chassis_absf(rpm);

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

static float chassis_step_towards(float current, float target, float max_step)
{
    float delta = target - current;

    if (max_step <= 0.0f) {
        return target;
    }

    delta = chassis_limit_abs(delta, max_step);
    return current + delta;
}

static float chassis_step_current_with_zero_unload(float current,
                                                   float target,
                                                   float normal_step,
                                                   float zero_step)
{
    if (((current > 0.0f) && (target < 0.0f)) ||
        ((current < 0.0f) && (target > 0.0f))) {
        return chassis_step_towards(current, 0.0f, zero_step);
    }

    return chassis_step_towards(current, target, normal_step);
}

static float chassis_apply_min_effective_current(float target,
                                                 float theta_deg,
                                                 float theta_rate_dps)
{
    float abs_target = chassis_absf(target);
    float state_active;

    if (BALANCE_LQR_MIN_EFFECTIVE_CURRENT_A <= 0.0f ||
        abs_target <= 0.0005f ||
        abs_target >= BALANCE_LQR_MIN_EFFECTIVE_CURRENT_A) {
        return target;
    }

    state_active =
        (chassis_absf(theta_deg) >= BALANCE_LQR_MIN_EFFECTIVE_THETA_DEG ||
         chassis_absf(theta_rate_dps) >= BALANCE_LQR_MIN_EFFECTIVE_RATE_DPS)
            ? 1.0f
            : 0.0f;

    if (state_active <= 0.0f) {
        return target;
    }

    return (target >= 0.0f) ? BALANCE_LQR_MIN_EFFECTIVE_CURRENT_A :
                              -BALANCE_LQR_MIN_EFFECTIVE_CURRENT_A;
}

static float chassis_speed_feedforward(float ref_rpm)
{
    float abs_ref = chassis_absf(ref_rpm);
    float ff;

    if (abs_ref <= 0.001f) {
        return 0.0f;
    }

    ff = WHEEL_SPEED_FF_STATIC_A +
         WHEEL_SPEED_FF_KV_A_PER_RPM * abs_ref;
    return (ref_rpm >= 0.0f) ? ff : -ff;
}

static float chassis_clampf(float value, float min_value, float max_value)
{
    if (value > max_value) {
        return max_value;
    }
    if (value < min_value) {
        return min_value;
    }
    return value;
}

static uint32_t chassis_ms_to_control_steps(uint32_t time_ms)
{
    uint32_t period_ms = WHEEL_SAFE_TASK_PERIOD_MS;
    uint32_t steps;

    if (period_ms == 0U) {
        period_ms = 1U;
    }

    steps = (time_ms + period_ms - 1U) / period_ms;
    return (steps == 0U) ? 1U : steps;
}

static void chassis_zero_bias_init_once(void)
{
    if (balance_zero_bias_initialized == 0U) {
        BalanceZeroBiasSearch_Init(&balance_zero_bias_search);
        balance_zero_bias_initialized = 1U;
    }
}

static BalanceZeroBiasSearch_Config_t chassis_zero_bias_config(void)
{
    BalanceZeroBiasSearch_Config_t config;

    config.enable = BALANCE_ZERO_BIAS_ENABLE;
    config.bias_limit_deg = BALANCE_ZERO_BIAS_LIMIT_DEG;
    config.active_theta_deg = BALANCE_ZERO_BIAS_ACTIVE_THETA_DEG;
    config.active_rate_dps = BALANCE_ZERO_BIAS_ACTIVE_RATE_DPS;
    config.active_current_a = BALANCE_ZERO_BIAS_ACTIVE_CURRENT_A;
    config.active_position_m = BALANCE_ZERO_BIAS_ACTIVE_POSITION_M;
    config.active_velocity_mps = BALANCE_ZERO_BIAS_ACTIVE_VELOCITY_MPS;
    config.v_deadband_mps = BALANCE_ZERO_BIAS_V_DEADBAND_MPS;
    config.x_deadband_m = BALANCE_ZERO_BIAS_X_DEADBAND_M;
    config.dither_deg = BALANCE_ZERO_BIAS_DITHER_DEG;
    config.update_step_deg = BALANCE_ZERO_BIAS_UPDATE_STEP_DEG;
    config.settle_steps = chassis_ms_to_control_steps(BALANCE_ZERO_BIAS_SETTLE_MS);
    config.eval_steps = chassis_ms_to_control_steps(BALANCE_ZERO_BIAS_EVAL_MS);
    config.freeze_enable = BALANCE_ZERO_BIAS_FREEZE_ENABLE;
    config.freeze_stable_rounds = BALANCE_ZERO_BIAS_FREEZE_ROUNDS;
    config.improve_eps = BALANCE_ZERO_BIAS_IMPROVE_EPS;
    config.v_scale_mps = BALANCE_ZERO_BIAS_V_SCALE_MPS;
    config.x_scale_m = BALANCE_ZERO_BIAS_X_SCALE_M;
    config.rate_scale_dps = BALANCE_ZERO_BIAS_RATE_SCALE_DPS;
    config.current_scale_a = BALANCE_ZERO_BIAS_CURRENT_SCALE_A;
    config.w_v = BALANCE_ZERO_BIAS_W_V;
    config.w_x = BALANCE_ZERO_BIAS_W_X;
    config.w_rate = BALANCE_ZERO_BIAS_W_RATE;
    config.w_current = BALANCE_ZERO_BIAS_W_CURRENT;

    return config;
}

static float chassis_zero_bias_current_abs(float left_current, float right_current)
{
    return 0.5f * (chassis_absf(left_current) + chassis_absf(right_current));
}

static void chassis_outer_loop_init_once(void)
{
    if (balance_outer_loop_initialized == 0U) {
        BalanceOuterLoop_Init(&balance_outer_loop);
        balance_outer_loop_initialized = 1U;
    }
}

static BalanceOuterLoop_Config_t chassis_outer_loop_config(void)
{
    BalanceOuterLoop_Config_t config;

    config.enable = BALANCE_OUTER_ENABLE;
    config.velocity_enable = BALANCE_OUTER_V_ENABLE;
    config.position_enable = BALANCE_OUTER_X_ENABLE;
    config.integral_enable = BALANCE_OUTER_I_ENABLE;
    config.theta_ref_limit_deg = BALANCE_OUTER_THETA_REF_LIMIT_DEG;
    config.k_v_deg_per_mps = BALANCE_OUTER_KV_DEG_PER_MPS;
    config.k_x_deg_per_m = BALANCE_OUTER_KX_DEG_PER_M;
    config.k_i_deg_per_m_s = BALANCE_OUTER_KI_DEG_PER_M_S;
    config.integral_limit_m_s = BALANCE_OUTER_INTEGRAL_LIMIT_M_S;
    config.active_theta_deg = BALANCE_OUTER_ACTIVE_THETA_DEG;
    config.active_current_a = BALANCE_OUTER_ACTIVE_CURRENT_A;

    return config;
}

static BalanceOuterLoop_Input_t chassis_outer_loop_input(float position_m,
                                                        float velocity_mps,
                                                        float theta_deg,
                                                        float current_abs_a)
{
    BalanceOuterLoop_Input_t input;

    input.position_m = position_m;
    input.velocity_mps = velocity_mps;
    input.theta_deg = theta_deg;
    input.current_abs_a = current_abs_a;
    input.dt_sec = BALANCE_LQR_DT_SEC;
    input.fault_ok = (chassis_fault_code == CHASSIS_FAULT_NONE &&
                      emergency_stop == 0U) ? 1U : 0U;

    return input;
}

static BalanceZeroBiasSearch_Input_t chassis_zero_bias_input(float theta_deg,
                                                            float theta_rate_dps,
                                                            float position_m,
                                                            float velocity_mps,
                                                            float current_abs_a)
{
    BalanceZeroBiasSearch_Input_t input;

    input.theta_deg = theta_deg;
    input.theta_rate_dps = theta_rate_dps;
    input.position_m = position_m;
    input.velocity_mps = velocity_mps;
    input.current_abs_a = current_abs_a;
    input.fault_ok = (chassis_fault_code == CHASSIS_FAULT_NONE &&
                      emergency_stop == 0U) ? 1U : 0U;

    return input;
}

static void chassis_balance_lqr_init_once(void)
{
    static const float k[BALANCE_LQR_STATE_DIM] = {
        BALANCE_LQR_K_X * BALANCE_LQR_K_X_SCALE,
        BALANCE_LQR_K_V * BALANCE_LQR_K_V_SCALE,
        BALANCE_LQR_K_THETA * BALANCE_LQR_K_THETA_SCALE,
        BALANCE_LQR_K_THETA_DOT * BALANCE_LQR_K_THETA_DOT_SCALE
    };

    if (balance_lqr_initialized == 0U) {
        LQR_FixedGain_Init(&balance_lqr,
                           k,
                           BALANCE_LQR_STATE_DIM,
                           -BALANCE_LQR_FORCE_LIMIT_N,
                           BALANCE_LQR_FORCE_LIMIT_N);
        balance_lqr_initialized = 1U;
    }
}

void WheelSpeed_SetTargetRpm(float rpm)
{
    wheel_speed_target_rpm = chassis_limit_speed_target(rpm);
    wheel_speed_target_changed = 1U;
}

void Balance_SetManualBiasDeg(float bias_deg)
{
    balance_manual_bias_deg = chassis_limit_abs(bias_deg,
                                                BALANCE_MANUAL_BIAS_LIMIT_DEG);
    chassis_zero_bias_init_once();
    BalanceZeroBiasSearch_Reset(&balance_zero_bias_search, 1U);
    balance_zero_bias_frozen = BalanceZeroBiasSearch_IsFrozen(&balance_zero_bias_search);
}

static void chassis_update_key(void)
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
                chassis_speed_test_armed = 0U;
                emergency_stop = 1U;
            } else if (chassis_motor_feedback_ok != 0U &&
                       chassis_motor_enabled_ok != 0U) {
#if (BALANCE_LQR_ENABLE || WHEEL_SPEED_TEST_ENABLE)
                chassis_fault_code = CHASSIS_FAULT_NONE;
                chassis_speed_test_armed = 1U;
                emergency_stop = 0U;
#endif
            }
        }
    }
}

static void chassis_update_led(void)
{
    GPIO_PinState blue = GPIO_PIN_RESET;
    GPIO_PinState green = GPIO_PIN_RESET;
    GPIO_PinState red = GPIO_PIN_RESET;

    if (emergency_stop == 0U && chassis_speed_test_armed != 0U) {
        green = GPIO_PIN_SET;
    } else if (chassis_motor_feedback_ok == 0U ||
               chassis_motor_enabled_ok == 0U) {
        red = GPIO_PIN_SET;
    } else {
        blue = GPIO_PIN_SET;
    }

    HAL_GPIO_WritePin(GPIOH, GPIO_PIN_10, blue);
    HAL_GPIO_WritePin(GPIOH, GPIO_PIN_11, green);
    HAL_GPIO_WritePin(GPIOH, GPIO_PIN_12, red);
}

static void chassis_update_state(void)
{
    if (chassis_motor_feedback_ok == 0U) {
        chassis_motor_state = CHASSIS_MOTOR_WAIT_FEEDBACK;
        return;
    }

    if (chassis_motor_enabled_ok == 0U) {
        chassis_motor_state = CHASSIS_MOTOR_WAIT_ENABLE;
        return;
    }

    if (emergency_stop != 0U || chassis_speed_test_armed == 0U) {
        chassis_motor_state = CHASSIS_MOTOR_ESTOP;
        return;
    }

#if BALANCE_LQR_ENABLE
    chassis_motor_state = CHASSIS_MOTOR_BALANCE_LQR;
#elif WHEEL_SPEED_TEST_ENABLE
    chassis_motor_state = CHASSIS_MOTOR_SPEED_TEST;
#else
    chassis_motor_state = CHASSIS_MOTOR_READY;
#endif
}

static void chassis_reset_speed_ctrl(void)
{
    left_speed_pid_integral = 0.0f;
    right_speed_pid_integral = 0.0f;
    left_speed_feedback_lpf = left_wheel_motor.speed_can_rpm;
    right_speed_feedback_lpf = right_wheel_motor.speed_can_rpm;
    left_speed_last_output = 0.0f;
    right_speed_last_output = 0.0f;
    wheel_speed_target_ramp_rpm = 0.0f;
    wheel_speed_feedback_lpf_ready = 0U;
    wheel_speed_encoder_left_rpm = left_wheel_motor.speed;
    wheel_speed_encoder_right_rpm = right_wheel_motor.speed;
    wheel_speed_feedback_left_a = left_wheel_motor.current;
    wheel_speed_feedback_right_a = right_wheel_motor.current;
    wheel_speed_eso_z1_rpm = left_wheel_motor.speed_can_rpm;
    wheel_speed_eso_z2_rpm_s = 0.0f;
    wheel_speed_pi_integral = 0.0f;
    wheel_speed_pi_integral_right = 0.0f;
}

static void chassis_reset_balance_lqr(void)
{
    chassis_balance_lqr_init_once();
    chassis_outer_loop_init_once();
    chassis_zero_bias_init_once();
    BalanceOuterLoop_Reset(&balance_outer_loop);
    BalanceZeroBiasSearch_Reset(&balance_zero_bias_search, 0U);
    balance_zero_bias_frozen = BalanceZeroBiasSearch_IsFrozen(&balance_zero_bias_search);
    left_speed_pid_integral = 0.0f;
    right_speed_pid_integral = 0.0f;
    left_speed_last_output = 0.0f;
    right_speed_last_output = 0.0f;
    balance_position_m = 0.0f;
    balance_velocity_lpf_mps = 0.0f;
    balance_lqr_last_left_a = 0.0f;
    balance_lqr_last_right_a = 0.0f;
    balance_velocity_lpf_ready = 0U;
    balance_lqr_theta_deg = global_balance_angle;
    balance_lqr_theta_ref_deg = 0.0f;
    balance_lqr_zero_bias_deg = balance_manual_bias_deg +
                                BalanceZeroBiasSearch_GetBias(&balance_zero_bias_search);
    balance_lqr_theta_rate_dps = global_balance_rate;
    balance_lqr_position_m = 0.0f;
    balance_lqr_velocity_mps = 0.0f;
    balance_lqr_force_n = 0.0f;
    balance_lqr_force_x_n = 0.0f;
    balance_lqr_force_v_n = 0.0f;
    balance_lqr_force_theta_n = 0.0f;
    balance_lqr_force_rate_n = 0.0f;
    balance_lqr_current_target_a = 0.0f;
    balance_lqr_current_cmd_a = 0.0f;
    balance_lqr_current_feedback_a = 0.0f;
    balance_lqr_output_left_a = 0.0f;
    balance_lqr_output_right_a = 0.0f;
}

static float chassis_get_wheel_speed_rpm(const volatile Motor_Measure_t *motor)
{
    if (motor == NULL) {
        return 0.0f;
    }

#if BALANCE_LQR_SPEED_SOURCE == BALANCE_LQR_SPEED_SOURCE_ENCODER
    return (motor->speed_ready != 0U) ? motor->speed : motor->speed_can_rpm;
#else
    return motor->speed_can_rpm;
#endif
}

static void chassis_update_balance_view_zero_output(void)
{
    float left_rpm = chassis_get_wheel_speed_rpm(&left_wheel_motor);
    float right_rpm = chassis_get_wheel_speed_rpm(&right_wheel_motor);
    float velocity_mps = 0.5f * (left_rpm + right_rpm) *
                         BALANCE_RPM_TO_MPS;

    balance_lqr_theta_deg = global_balance_angle;
    balance_lqr_theta_ref_deg = 0.0f;
    balance_lqr_zero_bias_deg = balance_manual_bias_deg +
                                (balance_zero_bias_initialized ?
                                 BalanceZeroBiasSearch_GetBias(&balance_zero_bias_search) :
                                 0.0f);
    balance_zero_bias_frozen = (balance_zero_bias_initialized ?
                                BalanceZeroBiasSearch_IsFrozen(&balance_zero_bias_search) :
                                0U);
    balance_lqr_theta_rate_dps = global_balance_rate;
    balance_lqr_position_m = BALANCE_LQR_POSITION_SIGN * balance_position_m;
    balance_lqr_velocity_mps = BALANCE_LQR_VELOCITY_SIGN * velocity_mps;
    balance_lqr_force_n = 0.0f;
    balance_lqr_force_x_n = 0.0f;
    balance_lqr_force_v_n = 0.0f;
    balance_lqr_force_theta_n = 0.0f;
    balance_lqr_force_rate_n = 0.0f;
    balance_lqr_current_target_a = 0.0f;
    balance_lqr_current_cmd_a = 0.0f;
    balance_lqr_current_feedback_a = 0.5f * (left_wheel_motor.current +
                                             right_wheel_motor.current);
    balance_lqr_output_left_a = 0.0f;
    balance_lqr_output_right_a = 0.0f;
    wheel_speed_ref_left_rpm = 0.0f;
    wheel_speed_actual_left_rpm = left_rpm;
    wheel_speed_output_left_a = 0.0f;
    wheel_speed_ref_right_rpm = 0.0f;
    wheel_speed_actual_right_rpm = right_rpm;
    wheel_speed_output_right_a = 0.0f;
}

static float chassis_speed_pid_update(float ref_rpm,
                                      float feedback_rpm,
                                      float polarity,
                                      float *integral,
                                      float *last_output,
                                      float dt_sec,
                                      float current_limit_a,
                                      float current_slew_a_per_sec)
{
    float speed_error;
    float u_unsat;
    float output;
    float max_current_step;

    if (integral == NULL || last_output == NULL) {
        return 0.0f;
    }

    speed_error = ref_rpm - feedback_rpm;
    if (WHEEL_SPEED_PID_KI_A_PER_RPM_S != 0.0f) {
        *integral += speed_error * dt_sec;
        *integral = chassis_limit_abs(*integral, WHEEL_SPEED_PID_INTEGRAL_LIMIT);
    } else {
        *integral = 0.0f;
    }

    u_unsat = (chassis_speed_feedforward(ref_rpm) +
               WHEEL_SPEED_PID_KP_A_PER_RPM * speed_error +
               WHEEL_SPEED_PID_KI_A_PER_RPM_S * (*integral) -
               WHEEL_SPEED_PID_KD_A_PER_RPM * feedback_rpm) *
              polarity;
    output = chassis_limit_abs(u_unsat, current_limit_a);
    max_current_step = current_slew_a_per_sec * dt_sec;
    output = chassis_step_towards(*last_output, output, max_current_step);

    if ((WHEEL_SPEED_PID_KI_A_PER_RPM_S != 0.0f) &&
        (output != u_unsat)) {
        *integral -= speed_error * dt_sec;
    }

    *last_output = output;
    return output;
}

static void chassis_calc_speed_test_output(float *left_current, float *right_current)
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

        if (left_feedback_alpha < 0.0f) {
            left_feedback_alpha = 0.0f;
        } else if (left_feedback_alpha > 1.0f) {
            left_feedback_alpha = 1.0f;
        }
        if (right_feedback_alpha < 0.0f) {
            right_feedback_alpha = 0.0f;
        } else if (right_feedback_alpha > 1.0f) {
            right_feedback_alpha = 1.0f;
        }

        wheel_speed_target_ramp_rpm = chassis_step_towards(wheel_speed_target_ramp_rpm,
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
                                       (left_wheel_motor.speed_can_rpm - left_speed_feedback_lpf);
            right_speed_feedback_lpf += right_feedback_alpha *
                                        (right_wheel_motor.speed_can_rpm - right_speed_feedback_lpf);
        }
        left_feedback_rpm = left_speed_feedback_lpf;
        right_feedback_rpm = right_speed_feedback_lpf;

        left_out = chassis_speed_pid_update(left_ref,
                                            left_feedback_rpm,
                                            LEFT_WHEEL_SPEED_PID_POLARITY,
                                            &left_speed_pid_integral,
                                            &left_speed_last_output,
                                            WHEEL_SPEED_TEST_DT_SEC,
                                            WHEEL_SPEED_TEST_CURRENT_LIMIT_A,
                                            WHEEL_SPEED_CURRENT_SLEW_A_PER_SEC);
        right_out = chassis_speed_pid_update(right_ref,
                                             right_feedback_rpm,
                                             RIGHT_WHEEL_SPEED_PID_POLARITY,
                                             &right_speed_pid_integral,
                                             &right_speed_last_output,
                                             WHEEL_SPEED_TEST_DT_SEC,
                                             WHEEL_SPEED_TEST_CURRENT_LIMIT_A,
                                             WHEEL_SPEED_CURRENT_SLEW_A_PER_SEC);

#if WHEEL_SPEED_TEST_UNIDIRECTIONAL
        if ((left_ref > 0.0f && left_out < 0.0f) ||
            (left_ref < 0.0f && left_out > 0.0f)) {
            left_out = 0.0f;
        }
        if ((right_ref > 0.0f && right_out < 0.0f) ||
            (right_ref < 0.0f && right_out > 0.0f)) {
            right_out = 0.0f;
        }
#endif
    } else {
        chassis_reset_speed_ctrl();
    }

    wheel_speed_ref_left_rpm = left_ref;
    wheel_speed_actual_left_rpm = left_feedback_rpm;
    wheel_speed_output_left_a = left_out;
    wheel_speed_ref_right_rpm = right_ref;
    wheel_speed_actual_right_rpm = right_feedback_rpm;
    wheel_speed_output_right_a = right_out;
    wheel_speed_encoder_left_rpm = left_wheel_motor.speed;
    wheel_speed_encoder_right_rpm = right_wheel_motor.speed;
    wheel_speed_feedback_left_a = left_wheel_motor.current;
    wheel_speed_feedback_right_a = right_wheel_motor.current;
    wheel_speed_eso_z1_rpm = left_feedback_rpm;
    wheel_speed_eso_z2_rpm_s = right_feedback_rpm;
    wheel_speed_pi_integral = left_speed_pid_integral;
    wheel_speed_pi_integral_right = right_speed_pid_integral;

    *left_current = left_out;
    *right_current = right_out;
}

static void chassis_calc_balance_lqr_output(float *left_current, float *right_current)
{
    float left_rpm;
    float right_rpm;
    float velocity_raw_mps;
    float velocity_state_mps;
    float position_state_m;
    float measured_velocity_mps;
    float measured_position_m;
    float velocity_alpha;
    float theta_deg;
    float theta_ref_deg;
    float theta_rate_dps;
    float theta_rad;
    float theta_rate_rad_s;
    float x[BALANCE_LQR_STATE_DIM];
    float force_n;
    float force_x_n;
    float force_v_n;
    float force_theta_n;
    float force_rate_n;
    float k_x_scale;
    float k_v_scale;
    float k_theta_scale;
    float k_theta_dot_scale;
    float current_limit_a;
    float current_slew_a_per_sec;
    float current_each_a = 0.0f;
    float speed_ref_rpm = 0.0f;
    float avg_rpm;
    float left_out;
    float right_out;
    float zero_bias_current_abs_a;
    BalanceOuterLoop_Config_t outer_config;
    BalanceOuterLoop_Input_t outer_input;
    BalanceZeroBiasSearch_Config_t zero_bias_config;
    BalanceZeroBiasSearch_Input_t zero_bias_input;

    if (chassis_motor_state != CHASSIS_MOTOR_BALANCE_LQR) {
        chassis_reset_balance_lqr();
        chassis_update_balance_view_zero_output();
        *left_current = 0.0f;
        *right_current = 0.0f;
        return;
    }

    chassis_balance_lqr_init_once();
    chassis_outer_loop_init_once();
    chassis_zero_bias_init_once();
    outer_config = chassis_outer_loop_config();
    zero_bias_config = chassis_zero_bias_config();

    theta_deg = global_balance_angle;
    theta_rate_dps = global_balance_rate;

    if (chassis_absf(theta_deg) > BALANCE_LQR_SAFE_ANGLE_DEG) {
        chassis_fault_code = CHASSIS_FAULT_LQR_ANGLE;
        chassis_speed_test_armed = 0U;
        emergency_stop = 1U;
        chassis_reset_balance_lqr();
        chassis_update_balance_view_zero_output();
        *left_current = 0.0f;
        *right_current = 0.0f;
        return;
    }

    if (chassis_absf(theta_rate_dps) > BALANCE_LQR_SAFE_RATE_DPS) {
        chassis_fault_code = CHASSIS_FAULT_LQR_RATE;
        chassis_speed_test_armed = 0U;
        emergency_stop = 1U;
        chassis_reset_balance_lqr();
        chassis_update_balance_view_zero_output();
        *left_current = 0.0f;
        *right_current = 0.0f;
        return;
    }

    left_rpm = chassis_get_wheel_speed_rpm(&left_wheel_motor);
    right_rpm = chassis_get_wheel_speed_rpm(&right_wheel_motor);
    avg_rpm = 0.5f * (left_rpm + right_rpm);
    velocity_raw_mps = 0.5f * (left_rpm + right_rpm) *
                       BALANCE_RPM_TO_MPS;
    velocity_raw_mps = chassis_limit_abs(velocity_raw_mps,
                                         BALANCE_LQR_VELOCITY_LIMIT_MPS);

    velocity_alpha = chassis_clampf(BALANCE_LQR_VELOCITY_LPF_ALPHA, 0.0f, 1.0f);
    if (balance_velocity_lpf_ready == 0U) {
        balance_velocity_lpf_mps = velocity_raw_mps;
        balance_velocity_lpf_ready = 1U;
    } else {
        balance_velocity_lpf_mps += velocity_alpha *
                                    (velocity_raw_mps - balance_velocity_lpf_mps);
    }

    balance_position_m += balance_velocity_lpf_mps * BALANCE_LQR_DT_SEC;
    balance_position_m = chassis_limit_abs(balance_position_m,
                                           BALANCE_LQR_POSITION_LIMIT_M);

    measured_velocity_mps = BALANCE_LQR_VELOCITY_SIGN * balance_velocity_lpf_mps;
    measured_position_m = BALANCE_LQR_POSITION_SIGN * balance_position_m;

#if BALANCE_LQR_USE_WHEEL_STATE
    velocity_state_mps = measured_velocity_mps;
    position_state_m = measured_position_m;
#else
    velocity_state_mps = 0.0f;
    position_state_m = 0.0f;
#endif

    zero_bias_current_abs_a = chassis_zero_bias_current_abs(balance_lqr_last_left_a,
                                                           balance_lqr_last_right_a);
    outer_input = chassis_outer_loop_input(measured_position_m,
                                           measured_velocity_mps,
                                           theta_deg,
                                           zero_bias_current_abs_a);
    zero_bias_input = chassis_zero_bias_input(theta_deg,
                                             theta_rate_dps,
                                             measured_position_m,
                                             measured_velocity_mps,
                                             zero_bias_current_abs_a);
    theta_ref_deg = balance_manual_bias_deg +
                    BalanceOuterLoop_Update(&balance_outer_loop,
                                            &outer_config,
                                            &outer_input) +
                    BalanceZeroBiasSearch_GetCandidate(&balance_zero_bias_search,
                                                       &zero_bias_config,
                                                       &zero_bias_input);

    theta_rad = theta_deg * BALANCE_DEG_TO_RAD;
    theta_rate_rad_s = theta_rate_dps * BALANCE_DEG_TO_RAD;

    BalanceLQR_BuildState(x,
                          position_state_m,
                          0.0f,
                          velocity_state_mps,
                          0.0f,
                          theta_rad,
                          theta_ref_deg * BALANCE_DEG_TO_RAD,
                          theta_rate_rad_s);

    k_x_scale = BALANCE_LQR_K_X_SCALE;
    k_v_scale = BALANCE_LQR_K_V_SCALE;
    k_theta_scale = BALANCE_LQR_K_THETA_SCALE;
    k_theta_dot_scale = BALANCE_LQR_K_THETA_DOT_SCALE;
    current_limit_a = BALANCE_LQR_CURRENT_LIMIT_A;
    current_slew_a_per_sec = BALANCE_LQR_CURRENT_SLEW_A_PER_SEC;

#if BALANCE_LQR_DISTURBANCE_SCHEDULE_ENABLE
    {
        float disturbance_level = 0.0f;
        float theta_level;
        float rate_level;
        float velocity_level;

        theta_level = chassis_absf(theta_deg) / BALANCE_LQR_DIST_THETA_DEG;
        rate_level = chassis_absf(theta_rate_dps) / BALANCE_LQR_DIST_RATE_DPS;
        velocity_level = chassis_absf(measured_velocity_mps) /
                         BALANCE_LQR_DIST_VELOCITY_MPS;

        if (theta_level > disturbance_level) {
            disturbance_level = theta_level;
        }
        if (rate_level > disturbance_level) {
            disturbance_level = rate_level;
        }
        if (velocity_level > disturbance_level) {
            disturbance_level = velocity_level;
        }

        disturbance_level = chassis_clampf(disturbance_level - 1.0f,
                                           0.0f,
                                           1.0f);

        k_v_scale *= 1.0f + disturbance_level *
                     (BALANCE_LQR_DIST_V_BOOST - 1.0f);
        k_theta_scale *= 1.0f + disturbance_level *
                         (BALANCE_LQR_DIST_THETA_BOOST - 1.0f);
        k_theta_dot_scale *= 1.0f + disturbance_level *
                             (BALANCE_LQR_DIST_RATE_BOOST - 1.0f);
        current_limit_a += disturbance_level *
                           (BALANCE_LQR_DIST_CURRENT_LIMIT_A -
                            BALANCE_LQR_CURRENT_LIMIT_A);
        current_slew_a_per_sec += disturbance_level *
                                  (BALANCE_LQR_DIST_CURRENT_SLEW_A_PER_SEC -
                                   BALANCE_LQR_CURRENT_SLEW_A_PER_SEC);
    }
#endif

#if BALANCE_LQR_X_RISK_GATE_ENABLE
    {
        float x_risk = 0.0f;
        float theta_risk;
        float rate_risk;

        theta_risk = chassis_absf(theta_deg) / BALANCE_LQR_X_GATE_THETA_DEG;
        rate_risk = chassis_absf(theta_rate_dps) / BALANCE_LQR_X_GATE_RATE_DPS;
        if (theta_risk > x_risk) {
            x_risk = theta_risk;
        }
        if (rate_risk > x_risk) {
            x_risk = rate_risk;
        }

        x_risk = chassis_clampf(x_risk - 1.0f, 0.0f, 1.0f);
        k_x_scale *= 1.0f - x_risk * (1.0f - BALANCE_LQR_X_GATE_MIN_SCALE);
    }
#endif

    force_x_n = (-(BALANCE_LQR_K_X * k_x_scale) *
                 x[BALANCE_LQR_STATE_X]) *
                BALANCE_LQR_FORCE_SIGN;
    force_v_n = (-(BALANCE_LQR_K_V * k_v_scale) *
                 x[BALANCE_LQR_STATE_V]) *
                BALANCE_LQR_FORCE_SIGN;
    force_theta_n = (-(BALANCE_LQR_K_THETA * k_theta_scale) *
                     x[BALANCE_LQR_STATE_THETA]) *
                    BALANCE_LQR_FORCE_SIGN;
    force_rate_n = (-(BALANCE_LQR_K_THETA_DOT * k_theta_dot_scale) *
                    x[BALANCE_LQR_STATE_THETA_DOT]) *
                   BALANCE_LQR_FORCE_SIGN;

    force_n = force_x_n + force_v_n + force_theta_n + force_rate_n;
    force_n = chassis_limit_abs(force_n,
                                current_limit_a /
                                BALANCE_FORCE_TO_EACH_CURRENT_A_PER_N);

#if BALANCE_LQR_OUTPUT_MODE == BALANCE_LQR_OUTPUT_MODE_SPEED
    speed_ref_rpm = chassis_limit_abs(force_n *
                                      BALANCE_LQR_FORCE_TO_SPEED_RPM_PER_N,
                                      BALANCE_LQR_SPEED_REF_LIMIT_RPM);
    left_out = chassis_speed_pid_update(speed_ref_rpm,
                                        left_rpm,
                                        LEFT_WHEEL_SPEED_PID_POLARITY,
                                        &left_speed_pid_integral,
                                        &left_speed_last_output,
                                        BALANCE_LQR_DT_SEC,
                                        current_limit_a,
                                        current_slew_a_per_sec);
    right_out = chassis_speed_pid_update(speed_ref_rpm,
                                         right_rpm,
                                         RIGHT_WHEEL_SPEED_PID_POLARITY,
                                         &right_speed_pid_integral,
                                         &right_speed_last_output,
                                         BALANCE_LQR_DT_SEC,
                                         current_limit_a,
                                         current_slew_a_per_sec);
    current_each_a = 0.5f * (left_out + right_out);
    balance_lqr_current_target_a = force_n * BALANCE_FORCE_TO_EACH_CURRENT_A_PER_N;
    balance_lqr_current_target_a = chassis_limit_abs(balance_lqr_current_target_a,
                                                     current_limit_a);
#else
    {
        float max_current_step;
        float zero_current_step;

        current_each_a = force_n * BALANCE_FORCE_TO_EACH_CURRENT_A_PER_N;
        current_each_a = chassis_limit_abs(current_each_a,
                                           current_limit_a);
        current_each_a = chassis_apply_min_effective_current(current_each_a,
                                                             theta_deg,
                                                             theta_rate_dps);
        current_each_a = chassis_limit_abs(current_each_a,
                                           current_limit_a);
        balance_lqr_current_target_a = current_each_a;

        left_out = current_each_a;
        right_out = current_each_a;

        max_current_step = current_slew_a_per_sec *
                           BALANCE_LQR_DT_SEC;
        zero_current_step = BALANCE_LQR_CURRENT_ZERO_SLEW_A_PER_SEC *
                            BALANCE_LQR_DT_SEC;
        left_out = chassis_step_current_with_zero_unload(balance_lqr_last_left_a,
                                                         left_out,
                                                         max_current_step,
                                                         zero_current_step);
        right_out = chassis_step_current_with_zero_unload(balance_lqr_last_right_a,
                                                          right_out,
                                                          max_current_step,
                                                          zero_current_step);

        balance_lqr_last_left_a = left_out;
        balance_lqr_last_right_a = right_out;
    }
#endif

    zero_bias_input.current_abs_a = chassis_zero_bias_current_abs(left_out, right_out);
    BalanceZeroBiasSearch_Update(&balance_zero_bias_search,
                                 &zero_bias_config,
                                 &zero_bias_input);

    balance_lqr_theta_deg = theta_deg;
    balance_lqr_theta_ref_deg = theta_ref_deg;
    balance_lqr_zero_bias_deg = balance_manual_bias_deg +
                                BalanceZeroBiasSearch_GetBias(&balance_zero_bias_search);
    balance_zero_bias_frozen = BalanceZeroBiasSearch_IsFrozen(&balance_zero_bias_search);
    balance_lqr_theta_rate_dps = theta_rate_dps;
    balance_lqr_position_m = measured_position_m;
    balance_lqr_velocity_mps = measured_velocity_mps;
    balance_lqr_force_n = force_n;
    balance_lqr_force_x_n = force_x_n;
    balance_lqr_force_v_n = force_v_n;
    balance_lqr_force_theta_n = force_theta_n;
    balance_lqr_force_rate_n = force_rate_n;
    balance_lqr_current_cmd_a = 0.5f * (left_out + right_out);
    balance_lqr_current_feedback_a = 0.5f * (left_wheel_motor.current +
                                             right_wheel_motor.current);
    balance_lqr_output_left_a = left_out;
    balance_lqr_output_right_a = right_out;

    wheel_speed_ref_left_rpm = speed_ref_rpm;
    wheel_speed_actual_left_rpm = left_rpm;
    wheel_speed_output_left_a = left_out;
    wheel_speed_ref_right_rpm = speed_ref_rpm;
    wheel_speed_actual_right_rpm = right_rpm;
    wheel_speed_output_right_a = right_out;
    wheel_speed_encoder_left_rpm = left_wheel_motor.speed;
    wheel_speed_encoder_right_rpm = right_wheel_motor.speed;
    wheel_speed_feedback_left_a = left_wheel_motor.current;
    wheel_speed_feedback_right_a = right_wheel_motor.current;
    wheel_speed_eso_z1_rpm = force_n;
    wheel_speed_eso_z2_rpm_s = avg_rpm;
    wheel_speed_pi_integral = position_state_m;
    wheel_speed_pi_integral_right = velocity_state_mps;

    *left_current = left_out;
    *right_current = right_out;
}

static void chassis_calc_motor_output(float *left_current, float *right_current)
{
#if BALANCE_LQR_ENABLE
    if (chassis_motor_state == CHASSIS_MOTOR_BALANCE_LQR) {
        chassis_calc_balance_lqr_output(left_current, right_current);
        return;
    }
    chassis_update_balance_view_zero_output();
    *left_current = 0.0f;
    *right_current = 0.0f;
    return;
#endif

    chassis_calc_speed_test_output(left_current, right_current);
}

void Gimbal_Task(void *argument)
{
    (void)argument;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(WHEEL_SAFE_TASK_PERIOD_MS);

    for (;;)
    {
        uint32_t now_tick = (uint32_t)xTaskGetTickCount();
        float left_current = 0.0f;
        float right_current = 0.0f;
        uint8_t left_feedback_ok = Motor_FeedbackReady(&left_wheel_motor,
                                                       now_tick,
                                                       pdMS_TO_TICKS(WHEEL_FEEDBACK_TIMEOUT_MS));
        uint8_t right_feedback_ok = Motor_FeedbackReady(&right_wheel_motor,
                                                        now_tick,
                                                        pdMS_TO_TICKS(WHEEL_FEEDBACK_TIMEOUT_MS));

        chassis_motor_feedback_ok = (left_feedback_ok && right_feedback_ok) ? 1U : 0U;
        chassis_motor_enabled_ok = (left_wheel_motor.enabled && right_wheel_motor.enabled) ? 1U : 0U;

        if (chassis_motor_feedback_ok == 0U || chassis_motor_enabled_ok == 0U) {
            chassis_fault_code = (chassis_motor_feedback_ok == 0U) ?
                                 CHASSIS_FAULT_FEEDBACK :
                                 CHASSIS_FAULT_ENABLE;
            chassis_speed_test_armed = 0U;
            emergency_stop = 1U;
        }

        chassis_update_key();
        chassis_update_state();
        if (chassis_motor_state == CHASSIS_MOTOR_BALANCE_LQR) {
            gimbal_state = GIMBAL_STATE_IMU_STAB;
        } else if (chassis_motor_state == CHASSIS_MOTOR_SPEED_TEST) {
            gimbal_state = GIMBAL_STATE_ENC_LOCK;
        } else {
            gimbal_state = GIMBAL_STATE_RELAX;
        }

        if (last_chassis_motor_state != chassis_motor_state) {
            if (last_chassis_motor_state == CHASSIS_MOTOR_BALANCE_LQR) {
                chassis_reset_balance_lqr();
            }

            if (chassis_motor_state == CHASSIS_MOTOR_SPEED_TEST) {
                chassis_reset_speed_ctrl();
            } else if (chassis_motor_state == CHASSIS_MOTOR_BALANCE_LQR) {
                chassis_reset_balance_lqr();
            }
        }
        last_chassis_motor_state = chassis_motor_state;

        if (wheel_speed_target_changed != 0U) {
            wheel_speed_target_changed = 0U;
            left_speed_pid_integral = 0.0f;
            right_speed_pid_integral = 0.0f;
        }

        chassis_calc_motor_output(&left_current, &right_current);

        Motor_SetCurrent(LEFT_WHEEL_MOTOR_ID, left_current);
        Motor_SetCurrent(RIGHT_WHEEL_MOTOR_ID, right_current);

        chassis_update_led();

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}
