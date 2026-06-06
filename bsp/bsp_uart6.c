/**
 * @file    bsp_uart6.c
 * @brief   BSP 层 — USART6 (视觉)
 * @note    严格参照官方 8.USART_receive_and_send (引脚也一致)
 *          PG14 = TX (AF8), PG9 = RX (AF8), 115200 8N1
 *          (DJI C 板 silkscreen "UART1" 接口物理走线就是 PG14/PG9)
 *          RX: RXNE 中断逐字节
 *          TX: DMA2_Stream6_Channel5 非阻塞
 */

#include "bsp_uart6.h"
#include "stm32f4xx_hal.h"
#include <string.h>

/* ======================== 硬件句柄 ======================== */
static UART_HandleTypeDef huart6;
static DMA_HandleTypeDef  hdma_usart6_tx;

#define UART6_TX_BUF_SIZE 64U
static uint8_t           tx_buf[UART6_TX_BUF_SIZE];
static volatile uint8_t  tx_busy = 0;

static BSP_UART6_RxByteCallback_t rx_cb   = NULL;
static BSP_UART6_IdleCallback_t   idle_cb = NULL;

/* ======================== 初始化 ======================== */

void BSP_UART6_Init(void)
{
    /* ----- 1. 时钟 ----- */
    __HAL_RCC_USART6_CLK_ENABLE();
    __HAL_RCC_GPIOG_CLK_ENABLE();
    __HAL_RCC_DMA2_CLK_ENABLE();

    /* ----- 2. GPIO: PG14 (TX) / PG9 (RX), AF8 (与官方一致) ----- */
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin       = GPIO_PIN_14 | GPIO_PIN_9;
    gpio.Mode      = GPIO_MODE_AF_PP;
    gpio.Pull      = GPIO_PULLUP;
    gpio.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = GPIO_AF8_USART6;
    HAL_GPIO_Init(GPIOG, &gpio);

    /* ----- 3. USART6 ----- */
    huart6.Instance          = USART6;
    huart6.Init.BaudRate     = 115200;
    huart6.Init.WordLength   = UART_WORDLENGTH_8B;
    huart6.Init.StopBits     = UART_STOPBITS_1;
    huart6.Init.Parity       = UART_PARITY_NONE;
    huart6.Init.Mode         = UART_MODE_TX_RX;
    huart6.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart6.Init.OverSampling = UART_OVERSAMPLING_16;
    HAL_UART_Init(&huart6);

    /* ----- 4. DMA TX: DMA2_Stream6_Channel5 ----- */
    hdma_usart6_tx.Instance                 = DMA2_Stream6;
    hdma_usart6_tx.Init.Channel             = DMA_CHANNEL_5;
    hdma_usart6_tx.Init.Direction           = DMA_MEMORY_TO_PERIPH;
    hdma_usart6_tx.Init.PeriphInc           = DMA_PINC_DISABLE;
    hdma_usart6_tx.Init.MemInc              = DMA_MINC_ENABLE;
    hdma_usart6_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart6_tx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
    hdma_usart6_tx.Init.Mode                = DMA_NORMAL;
    hdma_usart6_tx.Init.Priority            = DMA_PRIORITY_MEDIUM;
    hdma_usart6_tx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
    HAL_DMA_Init(&hdma_usart6_tx);
    __HAL_LINKDMA(&huart6, hdmatx, hdma_usart6_tx);

    /* ----- 5. NVIC ----- */
    HAL_NVIC_SetPriority(USART6_IRQn,       6, 0);
    HAL_NVIC_EnableIRQ(USART6_IRQn);
    HAL_NVIC_SetPriority(DMA2_Stream6_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(DMA2_Stream6_IRQn);

    /* ----- 6. 使能 RXNE + IDLE 中断 ----- */
    __HAL_UART_ENABLE_IT(&huart6, UART_IT_RXNE);
    __HAL_UART_ENABLE_IT(&huart6, UART_IT_IDLE);
}

/* ======================== 注册 ======================== */
void BSP_UART6_RegisterRxCallback(BSP_UART6_RxByteCallback_t cb) { rx_cb   = cb; }
void BSP_UART6_RegisterIdleCallback(BSP_UART6_IdleCallback_t cb) { idle_cb = cb; }

/* ======================== TX ======================== */
bool BSP_UART6_SendBytes(const uint8_t *data, uint16_t len)
{
    if (tx_busy || len == 0 || len > UART6_TX_BUF_SIZE) return false;
    memcpy(tx_buf, data, len);
    tx_busy = 1;
    if (HAL_UART_Transmit_DMA(&huart6, tx_buf, len) != HAL_OK) {
        tx_busy = 0;
        return false;
    }
    return true;
}

/* ======================== 中断 ======================== */

void BSP_UART6_IRQHandler(void)
{
    uint32_t sr  = huart6.Instance->SR;
    uint32_t cr1 = huart6.Instance->CR1;

    if ((sr & USART_SR_RXNE) && (cr1 & USART_CR1_RXNEIE))
    {
        uint8_t b = (uint8_t)(huart6.Instance->DR & 0xFF);
        if (rx_cb) rx_cb(b);
        return;
    }

    if ((sr & USART_SR_IDLE) && (cr1 & USART_CR1_IDLEIE))
    {
        (void)huart6.Instance->DR;
        if (idle_cb) idle_cb();
        return;
    }

    if ((sr & USART_SR_TC) && (cr1 & USART_CR1_TCIE))
    {
        __HAL_UART_CLEAR_FLAG(&huart6, UART_FLAG_TC);
        __HAL_UART_DISABLE_IT(&huart6, UART_IT_TC);
        huart6.gState = HAL_UART_STATE_READY;
        tx_busy = 0;
        return;
    }

    if (sr & USART_SR_ORE) (void)huart6.Instance->DR;
}

void BSP_UART6_DMA_TX_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&hdma_usart6_tx);
}

void BSP_UART6_NotifyTxDone(void)
{
    tx_busy = 0U;
}

void BSP_UART6_NotifyTxError(void)
{
    tx_busy = 0U;
}
