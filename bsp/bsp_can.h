#ifndef __BSP_CAN_H
#define __BSP_CAN_H

#include "main.h"
#include "config.h"

// 平衡车左右轮电机 ID。YAW/PITCH 名称仅为旧代码兼容别名。
#define YAW_MOTOR_ID   LEFT_WHEEL_MOTOR_ID
#define PITCH_MOTOR_ID RIGHT_WHEEL_MOTOR_ID

// 电机反馈数据结构体
typedef struct {
    uint8_t enabled;   // 使能状态: 1-使能, 0-失能
    float current;     // 实际电流 (A)
    float speed;       // 实际转速 (rpm)
    float angle;       // 编码器当前单圈角度 (rad), 范围 [0, 2PI]
    float last_angle;  // 上一次更新的单圈角度 (rad)
    float total_angle; // 经过 0/360 过零点处理后的连续多圈总角度 (rad)
    uint8_t init_flag; // 初始帧获取标志
} Motor_Measure_t;

extern Motor_Measure_t yaw_motor;
extern Motor_Measure_t pitch_motor;

void CAN_Filter_Init(void);
void Motor_Enable(uint8_t id);
void Motor_Disable(uint8_t id);
void Motor_SetCurrent(uint8_t id, float current);
void Motor_SetAngle(uint8_t id, float angle);

#endif /* __BSP_CAN_H */
