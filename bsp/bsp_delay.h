#ifndef BSP_DELAY_H
#define BSP_DELAY_H

/**
 * @file    bsp_delay.h
 * @brief   BSP层 — 精确延时接口
 */

#include <stdint.h>

/**
 * @brief  微秒级延时 (SysTick 精确计数)
 * @param  us 微秒数
 */
void BSP_Delay_Us(uint16_t us);

/**
 * @brief  毫秒级延时
 * @param  ms 毫秒数
 */
void BSP_Delay_Ms(uint16_t ms);

#endif /* BSP_DELAY_H */
