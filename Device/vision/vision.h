#ifndef VISION_H
#define VISION_H

#include <stdint.h>

/* MaixCAM2 -> STM32, ASCII, 115200 8N1: "dx,dy,valid\n". */
#define VISION_MAX_ABS_ERROR_PX 2048

typedef enum
{
    VISION_PARSE_NOT_VISION = 0,
    VISION_PARSE_VALID,
    VISION_PARSE_INVALID
} Vision_ParseResult_t;

typedef struct
{
    volatile int16_t dx;
    volatile int16_t dy;
    volatile uint8_t valid;
    volatile uint32_t publish_version;
    volatile uint32_t frame_count;
    volatile uint32_t rx_line_count;
    volatile uint32_t parse_err_count;
} Vision_Instance_t;

void Vision_Init(Vision_Instance_t *inst);

/* Numeric comma-separated candidates are always consumed, even if invalid. */
Vision_ParseResult_t Vision_ParseLine(Vision_Instance_t *inst,
                                      const uint8_t *line,
                                      uint16_t len);

/* Coherent ISR-to-task snapshot. The return value is the frame counter. */
uint32_t Vision_GetSnapshot(const Vision_Instance_t *inst,
                            int16_t *dx,
                            int16_t *dy,
                            uint8_t *valid);

#endif /* VISION_H */
