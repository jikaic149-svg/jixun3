#ifndef __MODE_TRACKING_H
#define __MODE_TRACKING_H

#include <stdint.h>

void MODE_Tracking_Init(void);

/**
 * @brief 循迹模式执行（红外传感器闭环 + 自适应变速）
 * @param max_pwm       循迹最大 PWM (0~1000，弯道自动降速)
 * @param current_error 循迹误差 (>0 线在左, <0 线在右, 999.0 丢线)
 */
void MODE_Tracking_Exec(float max_pwm, float current_error);

#endif /* __MODE_TRACKING_H */
