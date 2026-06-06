#ifndef VISION_H
#define VISION_H

/**
 * @file    vision.h
 * @brief   Device 层 — 视觉模块 (文本行协议)
 * @note    协议: ASCII 行, 格式 "dx,dy\n" (例 "12,-34\n"), 末尾 \r 可有可无
 *          BSP 层逐字节喂入, 本模块完成行装配 + 字段解析
 *
 *          遵循 ti.md Rule 2/3: 不依赖任何 MCU 头文件; 注入 BSP 函数指针
 */

#include <stdint.h>
#include <stdbool.h>

#define VISION_LINE_BUF_SIZE   64U

/* TX/RX 接口注入 (由 App 层在 Init 时给出) */
typedef bool (*Vision_SendBytes_t)(const uint8_t *data, uint16_t len);
typedef void (*Vision_RegisterRxByte_t)(void (*cb)(uint8_t byte));

/* 解析后的视觉数据 (公开但只读, 严禁外部直接写) */
typedef struct
{
    /* --- 注入的 BSP 接口 --- */
    Vision_SendBytes_t      send_bytes;

    /* --- 行装配缓冲 --- */
    uint8_t  line_buf[VISION_LINE_BUF_SIZE];
    uint16_t line_idx;

    /* --- 解析结果 --- */
    volatile int16_t  dx;
    volatile int16_t  dy;
    volatile uint32_t frame_count;

    /* --- 统计 (诊断用) --- */
    volatile uint32_t rx_byte_count;
    volatile uint32_t rx_line_count;
    volatile uint32_t parse_err_count;
} Vision_Instance_t;

/**
 * @brief 初始化视觉实例并把字节回调注册到 BSP
 */
void Vision_Init(Vision_Instance_t *inst,
                 Vision_SendBytes_t      send_fn,
                 Vision_RegisterRxByte_t register_fn);

/* --- 数据读取 (App 层调用) --- */
int16_t  Vision_GetDx(const Vision_Instance_t *inst);
int16_t  Vision_GetDy(const Vision_Instance_t *inst);
uint32_t Vision_GetFrameCount(const Vision_Instance_t *inst);
uint32_t Vision_GetRxByteCount(const Vision_Instance_t *inst);

/* --- 主动发送 (App 层调用) --- */
bool Vision_Send(Vision_Instance_t *inst, const uint8_t *data, uint16_t len);

#endif /* VISION_H */
