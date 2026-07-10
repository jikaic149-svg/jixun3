/**
 * @file track.c
 * @brief 8路红外模拟循迹模块驱动实现
 */
#include "track.h"
#include <string.h>
#include <stdlib.h>

static Track_Instance *track_bound_inst = NULL;

/* 2026-07-10 daylight VOFA calibration; black tape is high ADC. */
static const uint16_t TRACK_WHITE_REFERENCE[8] = {
    416, 802, 599, 726, 454, 508, 772, 377
};
static const uint16_t TRACK_BLACK_REFERENCE[8] = {
    1854, 2237, 2307, 2507, 2259, 2093, 2125, 1521
};
static const float TRACK_BLACK_DETECT_RATIO = 0.30f;
static const float TRACK_WEIGHTS[8] = {
    -9.0f, -5.0f, -2.0f, -1.0f, 1.0f, 2.0f, 5.0f, 9.0f
};

static float track_black_strength(uint8_t sensor, uint16_t adc)
{
    float white = (float)TRACK_WHITE_REFERENCE[sensor];
    float span = (float)(TRACK_BLACK_REFERENCE[sensor] -
                         TRACK_WHITE_REFERENCE[sensor]);
    float strength = ((float)adc - white) / span;

    if (strength < 0.0f) {
        return 0.0f;
    }
    if (strength > 1.0f) {
        return 1.0f;
    }
    return strength;
}

void Track_Init(Track_Instance *inst, Track_UartSend_t send_fn)
{
    inst->uart_send = send_fn;
    inst->rx_len = 0U;
    inst->data_ready = 0;
    inst->frame_count = 0U;
    inst->rx_byte_count = 0U;
    inst->parse_error_count = 0U;
    memset(inst->adc_value, 0, sizeof(inst->adc_value));
    memset(inst->rx_buf, 0, sizeof(inst->rx_buf));
    track_bound_inst = inst;

    /* 发送启动命令 "$0,1,0#" 让循迹模块开始输出数据 */
    /* 必须用 static，否则 DMA 还没传完数据函数就返回了，栈变量会被覆盖！ */
    if (inst->uart_send) {
        static const uint8_t cmd[] = "$0,1,0#";
        inst->uart_send((uint8_t *)cmd, sizeof(cmd) - 1);
    }
}

void Track_ParsePacket(Track_Instance *inst, const uint8_t *data, uint16_t len)
{
    /*
     * 协议格式: "$A,x1:val,x2:val,x3:val,x4:val,x5:val,x6:val,x7:val,x8:val#"
     * 用手动解析代替 sscanf 以节省代码空间和提升速度
     */
    if (inst == NULL || data == NULL || len < 5) return;

    /* 查找帧头 '$A,' */
    const uint8_t *p = data;
    const uint8_t *end = data + len;

    while (p < end - 4) {
        if (p[0] == '$' && p[1] == 'A' && p[2] == ',') {
            p += 3;  /* 跳过 "$A," */
            break;
        }
        p++;
    }
    if (p >= end - 4) {
        inst->parse_error_count++;
        return;
    }

    /* 逐个解析 "xN:val," */
    uint8_t ch = 0;
    while (p < end && ch < TRACK_SENSOR_COUNT) {
        /* 跳过 "xN:" 前缀 */
        if (*p == 'x') {
            p++;  /* 跳过 'x' */
            if (p < end) p++;  /* 跳过通道数字 */
            if (p < end && *p == ':') p++;  /* 跳过 ':' */
        }

        /* 解析数值 */
        uint16_t val = 0;
        while (p < end && *p >= '0' && *p <= '9') {
            val = val * 10 + (*p - '0');
            p++;
        }
        inst->adc_value[ch] = val;
        ch++;

        /* 跳过分隔符 ',' 或终止符 '#' */
        if (p < end && (*p == ',' || *p == '#')) p++;
    }

    if (ch > 0) {
        inst->data_ready = 1;
        inst->frame_count++;
    } else {
        inst->parse_error_count++;
    }
}

void Track_OnRxByte(uint8_t byte)
{
    Track_Instance *inst = track_bound_inst;

    if (inst == NULL) {
        return;
    }

    inst->rx_byte_count++;

    if (byte == '$') {
        inst->rx_len = 0U;
    }

    if (inst->rx_len < sizeof(inst->rx_buf)) {
        inst->rx_buf[inst->rx_len++] = byte;
    } else {
        inst->rx_len = 0U;
        inst->parse_error_count++;
        return;
    }

    if (byte == '#') {
        Track_ParsePacket(inst, inst->rx_buf, inst->rx_len);
        inst->rx_len = 0U;
    }
}

float Track_GetError(Track_Instance *inst)
{
    float weighted_sum = 0.0f;
    float total_black  = 0.0f;
    uint8_t black_count = 0U;

    if (inst == NULL) {
        return TRACK_LOST_LINE;
    }

    for (int i = 0; i < 8; i++) {
        uint16_t adc = inst->adc_value[i];

        float strength = track_black_strength((uint8_t)i, adc);

        if (strength > TRACK_BLACK_DETECT_RATIO) {
            weighted_sum += strength * TRACK_WEIGHTS[i];
            total_black  += strength;
            black_count++;
        }
    }

    if (black_count >= 7U) {
        return 0.0f;
    }

    if (black_count == 0U || total_black <= 0.0f) {
        return TRACK_LOST_LINE;
    }

    return (weighted_sum / total_black);
}

uint8_t Track_GetBlackCount(Track_Instance *inst)
{
    uint8_t black_count = 0U;

    if (inst == NULL) {
        return 0U;
    }

    for (int i = 0; i < 8; i++) {
        uint16_t adc = inst->adc_value[i];

        if (track_black_strength((uint8_t)i, adc) >
            TRACK_BLACK_DETECT_RATIO) {
            black_count++;
        }
    }

    return black_count;
}

uint32_t Track_GetFrameCount(const Track_Instance *inst)
{
    return (inst != NULL) ? inst->frame_count : 0U;
}

uint32_t Track_GetRxByteCount(const Track_Instance *inst)
{
    return (inst != NULL) ? inst->rx_byte_count : 0U;
}
