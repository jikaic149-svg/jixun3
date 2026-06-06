#include "bsp_can.h"

extern CAN_HandleTypeDef hcan1;

Motor_Measure_t yaw_motor;   // Yaw 电机数据, id = 0x00
Motor_Measure_t pitch_motor; // Pitch 电机数据, id = 0x01

/**
  * @brief  CAN1 滤波器配置与启动
  * @note   需要在此函数前确保 MX_CAN1_Init() 已成功执行
  */
void CAN_Filter_Init(void) {
    CAN_FilterTypeDef sFilterConfig;
    sFilterConfig.FilterBank = 0;
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
    sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
    sFilterConfig.FilterIdLow = 0x0000;
    sFilterConfig.FilterIdHigh = 0x0000;
    sFilterConfig.FilterMaskIdLow = 0x0000;
    sFilterConfig.FilterMaskIdHigh = 0x0000;
    sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
    sFilterConfig.FilterActivation = ENABLE;
    sFilterConfig.SlaveStartFilterBank = 14;
    
    if (HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK) {
        // 错误处理
    }
    if (HAL_CAN_Start(&hcan1) != HAL_OK) {
        // 错误处理
    }
    if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) {
        // 错误处理
    }
}

/**
  * @brief  底层 CAN 发送指令封装
  */
static void CAN_SendCommand(uint8_t id, uint8_t cmd, int16_t value) {
    uint8_t TxBuffer[3];
    TxBuffer[0] = cmd;
    // 小端模式：低位在前
    TxBuffer[1] = (uint8_t)(value & 0xFF);
    TxBuffer[2] = (uint8_t)((value >> 8) & 0xFF);
    
    uint32_t txMailbox;
    CAN_TxHeaderTypeDef TxHeader;
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.StdId = 0x400 + id; // 命令帧ID: 0x400 + motor_id
    TxHeader.ExtId = 0x400 + id;
    TxHeader.TransmitGlobalTime = DISABLE;
    TxHeader.DLC = 3;
    
    /* 无空闲邮箱则丢帧，避免 CAN 断线时 Gimbal_Task 死锁 */
    if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0)
        return;

    HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxBuffer, &txMailbox);
}

/**
  * @brief  使能电机
  */
void Motor_Enable(uint8_t id) {
    CAN_SendCommand(id, 0x01, 0x0000);
}

/**
  * @brief  失能电机
  */
void Motor_Disable(uint8_t id) {
    CAN_SendCommand(id, 0x02, 0x0000);
}

/**
  * @brief  设置电机电流
  * @param  current: 目标电流 [-10A, 10A]
  */
void Motor_SetCurrent(uint8_t id, float current) {
    // 限幅
    if (current > 10.0f) current = 10.0f;
    if (current < -10.0f) current = -10.0f;
    
    int16_t value = (int16_t)(current / 10.0f * 32767.0f);
    CAN_SendCommand(id, 0x03, value);
}

/**
  * @brief  设置电机角度 (位置模式)
  * @param  id: 电机ID
  * @param  angle: 目标角度 (弧度)
  */
void Motor_SetAngle(uint8_t id, float angle) {
    // 限制角度在 [0, 360]
    if (angle < 0.0f) angle = 0.0f;
    if (angle > 360.0f) angle = 360.0f;
    
    // angle(度) / 360 * UINT16_MAX
    uint16_t value = (uint16_t)(angle / 360.0f * 65535.0f);
    CAN_SendCommand(id, 0x05, (int16_t)value);
}

/**
  * @brief  CAN 接收回调函数（在此处回传并解析编码器数值）
  * @param  hcan: CAN 句柄
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    if (hcan->Instance == CAN1) {
        CAN_RxHeaderTypeDef rx_header;
        uint8_t rx_data[8];
        HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);
        
        // 接收反馈帧ID: 0x500 + motor_id
        if (rx_header.StdId >= 0x500 && rx_header.StdId <= 0x508) {
            Motor_Measure_t *motor = NULL;
            if (rx_header.StdId == 0x500) {
                motor = &yaw_motor;
            } else if (rx_header.StdId == 0x501) {
                motor = &pitch_motor;
            }
            
            if (motor != NULL) {
                // 完全参考 QGimbal 的方式，直接使用指针强转，抛弃复杂的累加处理
                motor->enabled = rx_data[0] & 0x01;
                motor->current = *(int16_t *)(rx_data + 2) * 10.0f / 32767.0f;
                motor->speed   = *(int16_t *)(rx_data + 4) * 1000.0f / 32767.0f;
                motor->angle   = *(uint16_t *)(rx_data + 6) * 360.0f / 65535.0f;  // 直接输出角度制 (0~360°)
                
                // 强制将 total_angle 设为直接等价，不需要自行累加了！
                motor->total_angle = motor->angle;
            }
        }
    }
}
