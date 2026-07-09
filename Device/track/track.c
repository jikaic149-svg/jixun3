/**
 * @file track.c
 * @brief 8路红外模拟循迹模块驱动实现
 */
#include "track.h"
#include <string.h>
#include <stdlib.h>

static Track_Instance *track_bound_inst = NULL;

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
    /* 探头位置权重 (左正右负) — 边缘探头权重加大 */
    const float weights[8] = {5.0f, 3.0f, 2.0f, 1.0f, -1.0f, -2.0f, -3.0f, -5.0f};

    /* 纯白底基准值（2026-03-23 实测当前环境白底均值） */
    const uint16_t WHITE_BASE[8] = {2365, 1808, 1481, 1502, 1569, 1757, 2070, 2523};

    /* 触发阈值：白底漂移约 ±20, 黑线 diff 约 400~650, 设 150 增强灵敏度减少出线 */
    const float DIFF_THRESHOLD = 150.0f;

    float weighted_sum = 0.0f;
    float total_black  = 0.0f;

    for (int i = 0; i < 8; i++) {
        float diff = 0.0f;
        if (inst->adc_value[i] > WHITE_BASE[i]) {
            diff = (float)(inst->adc_value[i] - WHITE_BASE[i]);
        }

        if (diff > DIFF_THRESHOLD) {
            weighted_sum += diff * weights[i];
            total_black  += diff;
        }
    }

    if (total_black == 0.0f) return TRACK_LOST_LINE;

    return (weighted_sum / total_black);
}

uint32_t Track_GetFrameCount(const Track_Instance *inst)
{
    return (inst != NULL) ? inst->frame_count : 0U;
}

uint32_t Track_GetRxByteCount(const Track_Instance *inst)
{
    return (inst != NULL) ? inst->rx_byte_count : 0U;
}
