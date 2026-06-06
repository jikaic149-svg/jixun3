/**
 * @file    vision.c
 * @brief   Device 层 — 视觉模块实现
 * @note    单例: 因 BSP 字节回调是 C 风格函数指针, 这里维护一个文件作用域指针
 *          指向当前实例; Init 时绑定。如需多实例改造, 把回调改为带 inst* 的形式。
 */

#include "vision.h"
#include <string.h>

/* ======================== 实例绑定 ======================== */
/* BSP 的字节回调不带 user_data, 这里用文件级 g_inst 绑回实例 */
static Vision_Instance_t *g_inst = NULL;

/* ======================== 工具: 解析整数 ======================== */
/* 从 s[*idx] 开始解析带符号十进制整数, 解析完后 *idx 指向第一个非数字字符
 * 返回 true 表示至少读到一位数字 */
static bool parse_int(const uint8_t *s, uint16_t len, uint16_t *idx, int16_t *out)
{
    int32_t  val  = 0;
    int      sign = 1;
    bool     any  = false;

    while (*idx < len && s[*idx] == ' ') (*idx)++;
    if (*idx < len && s[*idx] == '-') { sign = -1; (*idx)++; }
    else if (*idx < len && s[*idx] == '+') {              (*idx)++; }
    while (*idx < len && s[*idx] >= '0' && s[*idx] <= '9') {
        val = val * 10 + (s[*idx] - '0');
        any = true;
        (*idx)++;
    }
    if (!any) return false;
    *out = (int16_t)(val * sign);
    return true;
}

/* ======================== 行解析 ======================== */
/* line[0..len-1] 是一行 (不含 \n), 解析 "dx,dy" */
static void parse_line(Vision_Instance_t *inst, const uint8_t *line, uint16_t len)
{
    /* 去掉行尾 \r */
    while (len > 0 && line[len - 1] == '\r') len--;
    if (len == 0) return;

    inst->rx_line_count++;

    uint16_t i = 0;
    int16_t  dx = 0, dy = 0;
    if (!parse_int(line, len, &i, &dx))  { inst->parse_err_count++; return; }
    while (i < len && (line[i] == ' ' || line[i] == ',')) i++;
    if (!parse_int(line, len, &i, &dy))  { inst->parse_err_count++; return; }

    inst->dx = dx;
    inst->dy = dy;
    inst->frame_count++;
}

/* ======================== 字节回调 (BSP 在 ISR 上下文调用) ======================== */

static void on_rx_byte(uint8_t b)
{
    Vision_Instance_t *inst = g_inst;
    if (inst == NULL) return;

    inst->rx_byte_count++;

    if (b == '\n')
    {
        parse_line(inst, inst->line_buf, inst->line_idx);
        inst->line_idx = 0;
    }
    else
    {
        if (inst->line_idx < VISION_LINE_BUF_SIZE - 1)
            inst->line_buf[inst->line_idx++] = b;
        else
            inst->line_idx = 0;  /* 溢出: 丢弃整行 */
    }
}

/* ======================== 公开接口 ======================== */

void Vision_Init(Vision_Instance_t *inst,
                 Vision_SendBytes_t      send_fn,
                 Vision_RegisterRxByte_t register_fn)
{
    memset(inst, 0, sizeof(*inst));
    inst->send_bytes = send_fn;
    g_inst = inst;
    if (register_fn) register_fn(on_rx_byte);
}

int16_t  Vision_GetDx(const Vision_Instance_t *inst)            { return inst->dx; }
int16_t  Vision_GetDy(const Vision_Instance_t *inst)            { return inst->dy; }
uint32_t Vision_GetFrameCount(const Vision_Instance_t *inst)    { return inst->frame_count; }
uint32_t Vision_GetRxByteCount(const Vision_Instance_t *inst)   { return inst->rx_byte_count; }

bool Vision_Send(Vision_Instance_t *inst, const uint8_t *data, uint16_t len)
{
    if (!inst->send_bytes) return false;
    return inst->send_bytes(data, len);
}
