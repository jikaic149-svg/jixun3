#include "bsp_can.h"
#include "FreeRTOS.h"
#include "task.h"

extern CAN_HandleTypeDef hcan1;

volatile Motor_Measure_t left_wheel_motor;   // 左轮反馈, id = 0x00
volatile Motor_Measure_t right_wheel_motor;  // 右轮反馈, id = 0x01
volatile uint32_t can_rx_count = 0U;
volatile uint32_t can_wheel_rx_count = 0U;
volatile uint32_t can_last_std_id = 0U;
volatile uint32_t can_last_dlc = 0U;

static int16_t can_read_i16_le(const uint8_t *data)
{
    return (int16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8));
}

static uint16_t can_read_u16_le(const uint8_t *data)
{
    return (uint16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8));
}

static uint32_t motor_feedback_tick_from_isr(void)
{
    if (xTaskGetSchedulerState() == taskSCHEDULER_NOT_STARTED) {
        return HAL_GetTick();
    }
    return (uint32_t)xTaskGetTickCountFromISR();
}

static float motor_feedback_sign(uint8_t id)
{
    if (id == RIGHT_WHEEL_MOTOR_ID) {
        return RIGHT_WHEEL_FEEDBACK_SIGN;
    }
    return LEFT_WHEEL_FEEDBACK_SIGN;
}

static float motor_current_sign(uint8_t id)
{
    if (id == RIGHT_WHEEL_MOTOR_ID) {
        return RIGHT_WHEEL_CURRENT_SIGN;
    }
    return LEFT_WHEEL_CURRENT_SIGN;
}

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
    current *= motor_current_sign(id);

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

uint8_t Motor_FeedbackReady(const volatile Motor_Measure_t *motor,
                            uint32_t now_tick,
                            uint32_t timeout_ms)
{
    if (motor == NULL || motor->init_flag == 0U) {
        return 0U;
    }
    if ((uint32_t)(now_tick - motor->last_rx_tick) > timeout_ms) {
        return 0U;
    }
    return 1U;
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

        can_rx_count++;
        can_last_std_id = rx_header.StdId;
        can_last_dlc = rx_header.DLC;
        
        // 接收反馈帧ID: 0x500 + motor_id
        if (rx_header.StdId >= 0x500 && rx_header.StdId <= 0x508) {
            can_wheel_rx_count++;
            volatile Motor_Measure_t *motor = NULL;
            if (rx_header.StdId == 0x500) {
                motor = &left_wheel_motor;
            } else if (rx_header.StdId == 0x501) {
                motor = &right_wheel_motor;
            }
            
            if (motor != NULL && rx_header.DLC >= 8U) {
                float sign;
                float angle;
                float delta;
                float elapsed_sec;
                float speed_raw;
                float speed_alpha;
                uint32_t now_tick;
                uint32_t elapsed_tick;
                uint16_t encoder_raw;

                sign = motor_feedback_sign((uint8_t)(rx_header.StdId - 0x500U));
                now_tick = motor_feedback_tick_from_isr();
                motor->enabled = rx_data[0] & 0x01;
                motor->current = sign * can_read_i16_le(rx_data + 2) * 10.0f / 32767.0f;
                motor->speed_can_rpm = sign * can_read_i16_le(rx_data + 4) * 1000.0f / 32767.0f;

                encoder_raw = can_read_u16_le(rx_data + 6);
                motor->encoder_raw = encoder_raw;
                angle = encoder_raw * 360.0f / 65535.0f;
                motor->angle = angle;

                if (motor->init_flag == 0U) {
                    motor->total_angle = 0.0f;
                    motor->speed = 0.0f;
                    motor->last_angle = angle;
                    motor->speed_window_angle = 0.0f;
                    motor->speed_window_tick = now_tick;
                    motor->speed_ready = 0U;
                    motor->init_flag = 1U;
                } else {
                    delta = angle - motor->last_angle;
                    if (delta > 180.0f) {
                        delta -= 360.0f;
                    } else if (delta < -180.0f) {
                        delta += 360.0f;
                    }
                    motor->total_angle += sign * delta;
                    elapsed_tick = now_tick - motor->speed_window_tick;
                    if (elapsed_tick >= pdMS_TO_TICKS(WHEEL_ENCODER_SPEED_WINDOW_MS)) {
                        elapsed_sec = (float)elapsed_tick / (float)configTICK_RATE_HZ;
                        speed_raw = (motor->total_angle - motor->speed_window_angle) *
                                    (60.0f / 360.0f) / elapsed_sec;
                        speed_alpha = WHEEL_ENCODER_SPEED_LPF_ALPHA;
                        if (speed_alpha < 0.0f) {
                            speed_alpha = 0.0f;
                        } else if (speed_alpha > 1.0f) {
                            speed_alpha = 1.0f;
                        }

                        if (motor->speed_ready == 0U) {
                            motor->speed = speed_raw;
                            motor->speed_ready = 1U;
                        } else {
                            motor->speed += speed_alpha * (speed_raw - motor->speed);
                        }
                        motor->speed_window_angle = motor->total_angle;
                        motor->speed_window_tick = now_tick;
                    }
                    motor->last_angle = angle;
                }

                motor->rx_count++;
                motor->last_rx_tick = now_tick;
            }
        }
    }
}
