/**
 * @file    bsp_delay.c
 * @brief   BSP层 — SysTick 精确延时 (STM32F407 @ 168MHz)
 */

#include "bsp_delay.h"
#include "main.h"

/* STM32F407 主频 168MHz, SysTick 以 HCLK 驱动 */
#define SYSTICK_LOAD_PER_US  168U

void BSP_Delay_Us(uint16_t us)
{
    uint32_t ticks = 0;
    uint32_t told = 0;
    uint32_t tnow = 0;
    uint32_t tcnt = 0;
    uint32_t reload = 0;

    reload = SysTick->LOAD;
    ticks = (uint32_t)us * SYSTICK_LOAD_PER_US;
    told = SysTick->VAL;

    while (1)
    {
        tnow = SysTick->VAL;
        if (tnow != told)
        {
            if (tnow < told)
                tcnt += told - tnow;
            else
                tcnt += reload - tnow + told;
            told = tnow;
            if (tcnt >= ticks)
                break;
        }
    }
}

void BSP_Delay_Ms(uint16_t ms)
{
    while (ms--)
    {
        BSP_Delay_Us(1000);
    }
}
