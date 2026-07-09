/**
 * @file track.c
 * @brief 8路红外模拟循迹模块驱动实现
 */
#include "track.h"
#include <string.h>
#include <stdlib.h>

static Track_Instance *track_bound_inst = NULL;

/* 2026-07-09 VOFA: floor is low ADC, black tape is high ADC. */
static const uint16_t TRACK_WHITE_BASE[8] = {
    929, 1210, 1176, 1222, 901, 737, 893, 613
};
static const float TRACK_DIFF_THRESHOLD = 400.0f;
static const float TRACK_BLACK_TOTAL_MIN = 700.0f;
static const float TRACK_CROSSBAR_RANGE_MAX = 900.0f;
static const float TRACK_WEIGHTS[8] = {
    -18.0f, -9.0f, -3.0f, -1.0f, 1.0f, 3.0f, 9.0f, 18.0f
};

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
    uint16_t min_adc = 0xFFFFU;
    uint16_t max_adc = 0U;
    uint8_t black_count = 0U;

    for (int i = 0; i < 8; i++) {
        float diff = 0.0f;
        uint16_t adc = inst->adc_value[i];

        if (adc < min_adc) {
            min_adc = adc;
        }
        if (adc > max_adc) {
            max_adc = adc;
        }

        if (adc > TRACK_WHITE_BASE[i]) {
            diff = (float)(adc - TRACK_WHITE_BASE[i]);
        }

        if (diff > TRACK_DIFF_THRESHOLD) {
            weighted_sum += diff * TRACK_WEIGHTS[i];
            total_black  += diff;
            black_count++;
        }
    }

    if ((black_count >= 7U) &&
        ((float)(max_adc - min_adc) < TRACK_CROSSBAR_RANGE_MAX)) {
        return 0.0f;
    }

    if (total_black < TRACK_BLACK_TOTAL_MIN) return TRACK_LOST_LINE;

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

        if ((adc > TRACK_WHITE_BASE[i]) &&
            (((float)(adc - TRACK_WHITE_BASE[i])) > TRACK_DIFF_THRESHOLD)) {
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
