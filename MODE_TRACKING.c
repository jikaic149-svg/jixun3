/**
 * @file  MODE_TRACKING.c
 * @brief 极速自适应循迹 (二阶微分预判 + 非线性变速 + 丢线惯性保持)
 *        输出为 PWM 占空比 (0~1000), 无速度闭环
 */
#include "MODE_TRACKING.h"
#include "Application/broker/broker.h"
#include "config.h"
#include <math.h>

/* ======================== 内部状态 ======================== */
static float s_last_error = 0.0f;
static float s_error_integral = 0.0f;
static float s_last_turn_out = 0.0f;
static float s_filtered_error = 0.0f;
static float s_last_derivative = 0.0f;
static uint8_t s_was_lost = 1; /* 上一拍是否丢线 */

void MODE_Tracking_Init(void) {
  s_last_error = 0.0f;
  s_error_integral = 0.0f;
  s_last_turn_out = 0.0f;
  s_filtered_error = 0.0f;
  s_last_derivative = 0.0f;
  s_was_lost = 1;

  Broker_Publish_Float(TOPIC_TARGET_SPEED_LEFT, 0.0f);
  Broker_Publish_Float(TOPIC_TARGET_SPEED_RIGHT, 0.0f);
}

void MODE_Tracking_Exec(float max_pwm, float current_error) {
  float turn_out = 0.0f;

  if (current_error != 999.0f) {
    /* 0. 丢线恢复：清除旧状态，避免微分跳变 */
    if (s_was_lost) {
      s_filtered_error = current_error;
      s_last_error = current_error;
      s_last_derivative = 0.0f;
      s_error_integral = 0.0f;
      s_was_lost = 0;
    }

    /* 1. 误差低通滤波 */
    s_filtered_error = TRACK_ERROR_FILTER * s_filtered_error +
                       (1.0f - TRACK_ERROR_FILTER) * current_error;

    /* 2. 积分分离 + 抗饱和 */
    if (s_filtered_error > -3.0f && s_filtered_error < 3.0f) {
      s_error_integral += s_filtered_error;
      if (s_error_integral > 100.0f)
        s_error_integral = 100.0f;
      if (s_error_integral < -100.0f)
        s_error_integral = -100.0f;
    } else {
      s_error_integral *= 0.70f;
    }

    /* 3. PID + 二阶微分前馈 */
    float derivative = s_filtered_error - s_last_error;
    float accel = derivative - s_last_derivative;
    s_last_derivative = derivative;

    float d2_feedforward = 0.0f; /* 二阶前馈暂时关闭，减少噪声抖动 */

    turn_out = (PID_TRACK_KP * s_filtered_error) +
               (PID_TRACK_KI * s_error_integral) + (PID_TRACK_KD * derivative) +
               d2_feedforward;

    s_last_error = s_filtered_error;

    /* 4. 非线性自适应变速 */
    float abs_err = fabsf(s_filtered_error);
    float err_rate = fabsf(derivative);

    float speed_scale = 1.0f;

    if (abs_err > TRACK_CURVE_THRESHOLD) {
      float norm =
          (abs_err - TRACK_CURVE_THRESHOLD) / (7.5f - TRACK_CURVE_THRESHOLD);
      if (norm > 1.0f)
        norm = 1.0f;
      speed_scale = 1.0f - norm * norm * (1.0f - TRACK_MIN_SPEED_RATIO);
    }

    if (err_rate > 0.25f) {
      float rate_penalty = 1.0f - (err_rate - 0.25f) * 0.4f;
      if (rate_penalty < 0.7f)
        rate_penalty = 0.7f;
      speed_scale *= rate_penalty;
    }

    if (speed_scale < TRACK_MIN_SPEED_RATIO)
      speed_scale = TRACK_MIN_SPEED_RATIO;

    float actual_pwm = max_pwm * speed_scale;

    /* 5. 转向限幅 */
    float max_turn = actual_pwm * TRACK_MAX_TURN_RATIO;
    if (turn_out > max_turn)
      turn_out = max_turn;
    if (turn_out < -max_turn)
      turn_out = -max_turn;

    s_last_turn_out = turn_out;

    /* 6. 差速输出 */
    Broker_Publish_Float(TOPIC_TARGET_SPEED_LEFT, actual_pwm - turn_out);
    Broker_Publish_Float(TOPIC_TARGET_SPEED_RIGHT, actual_pwm + turn_out);
  } else {
    s_was_lost = 1;
    /* 丢线惯性保持 */
    float safe_pwm = max_pwm * TRACK_MIN_SPEED_RATIO;
    turn_out = s_last_turn_out * 1.10f;
    turn_out += PID_TRACK_KI * s_error_integral * 0.3f;

    float max_turn = safe_pwm * TRACK_MAX_TURN_RATIO;
    if (turn_out > max_turn)
      turn_out = max_turn;
    if (turn_out < -max_turn)
      turn_out = -max_turn;

    Broker_Publish_Float(TOPIC_TARGET_SPEED_LEFT, safe_pwm - turn_out);
    Broker_Publish_Float(TOPIC_TARGET_SPEED_RIGHT, safe_pwm + turn_out);
  }
}
