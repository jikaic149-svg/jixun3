#ifndef GIMBAL_TASK_H
#define GIMBAL_TASK_H

#include "stdint.h"

typedef enum {
    GIMBAL_STATE_RELAX = 0,
    GIMBAL_STATE_ENC_LOCK,
    GIMBAL_STATE_CALIBRATING,
    GIMBAL_STATE_IMU_STAB
} GimbalState_e;

typedef enum {
    CHASSIS_MOTOR_ESTOP = 0,
    CHASSIS_MOTOR_WAIT_FEEDBACK,
    CHASSIS_MOTOR_WAIT_ENABLE,
    CHASSIS_MOTOR_READY,
    CHASSIS_MOTOR_SPEED_TEST,
    CHASSIS_MOTOR_BALANCE_LQR
} ChassisMotorState_e;

extern volatile GimbalState_e gimbal_state;
extern volatile ChassisMotorState_e chassis_motor_state;
extern volatile uint8_t chassis_motor_feedback_ok;
extern volatile uint8_t chassis_motor_enabled_ok;
extern volatile uint8_t chassis_speed_test_armed;
extern volatile uint8_t chassis_fault_code;
extern volatile uint8_t emergency_stop;

extern volatile float wheel_speed_ref_left_rpm;
extern volatile float wheel_speed_actual_left_rpm;
extern volatile float wheel_speed_output_left_a;
extern volatile float wheel_speed_ref_right_rpm;
extern volatile float wheel_speed_actual_right_rpm;
extern volatile float wheel_speed_output_right_a;
extern volatile float wheel_speed_encoder_left_rpm;
extern volatile float wheel_speed_encoder_right_rpm;
extern volatile float wheel_speed_feedback_left_a;
extern volatile float wheel_speed_feedback_right_a;
extern volatile float wheel_speed_eso_z1_rpm;
extern volatile float wheel_speed_eso_z2_rpm_s;
extern volatile float wheel_speed_pi_integral;
extern volatile float wheel_speed_pi_integral_right;
extern volatile float wheel_speed_target_rpm;
extern volatile float balance_lqr_theta_deg;
extern volatile float balance_lqr_theta_ref_deg;
extern volatile float balance_lqr_zero_bias_deg;
extern volatile float balance_lqr_theta_rate_dps;
extern volatile float balance_lqr_position_m;
extern volatile float balance_lqr_velocity_mps;
extern volatile float balance_lqr_force_n;
extern volatile float balance_lqr_force_x_n;
extern volatile float balance_lqr_force_v_n;
extern volatile float balance_lqr_force_theta_n;
extern volatile float balance_lqr_force_rate_n;
extern volatile float balance_lqr_current_target_a;
extern volatile float balance_lqr_current_cmd_a;
extern volatile float balance_lqr_current_feedback_a;
extern volatile float balance_lqr_output_left_a;
extern volatile float balance_lqr_output_right_a;
extern volatile uint8_t balance_zero_bias_frozen;

extern void Gimbal_Task(void *argument);
void WheelSpeed_SetTargetRpm(float rpm);
void Balance_SetManualBiasDeg(float bias_deg);

#endif /* GIMBAL_TASK_H */
