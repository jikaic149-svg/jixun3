#include "chassis_serial_cmd.h"

#include "gimbal_task.h"

static uint8_t cmd_to_lower(uint8_t c)
{
    if (c >= 'A' && c <= 'Z') {
        return (uint8_t)(c + ('a' - 'A'));
    }
    return c;
}

static uint8_t cmd_is_delimiter(uint8_t c)
{
    return (c == ' ') || (c == '\t') || (c == ':') ||
           (c == '=') || (c == ',');
}

static uint8_t cmd_match_word(const uint8_t *line,
                              uint8_t len,
                              uint8_t start,
                              const char *word)
{
    uint8_t i = 0U;

    if ((line == 0) || (word == 0)) {
        return 0U;
    }

    while (word[i] != '\0') {
        if ((uint8_t)(start + i) >= len) {
            return 0U;
        }
        if (cmd_to_lower(line[start + i]) != (uint8_t)word[i]) {
            return 0U;
        }
        i++;
    }

    if (((uint8_t)(start + i) < len) &&
        (cmd_is_delimiter(line[start + i]) == 0U)) {
        return 0U;
    }

    return 1U;
}

static uint8_t cmd_parse_float(const uint8_t *line, uint8_t len, float *value_out)
{
    uint8_t i = 0U;
    uint8_t has_digit = 0U;
    uint8_t has_frac = 0U;
    float sign = 1.0f;
    float value = 0.0f;
    float frac_scale = 0.1f;

    if (line == 0 || value_out == 0 || len == 0U) {
        return 0U;
    }

    while (i < len) {
        uint8_t c = line[i];
        if (c == '+' || c == '-' || c == '.' || (c >= '0' && c <= '9')) {
            break;
        }
        i++;
    }

    if (i >= len) {
        return 0U;
    }

    if (line[i] == '-') {
        sign = -1.0f;
        i++;
    } else if (line[i] == '+') {
        i++;
    }

    while (i < len) {
        uint8_t c = line[i];
        if (c >= '0' && c <= '9') {
            has_digit = 1U;
            if (has_frac == 0U) {
                value = value * 10.0f + (float)(c - '0');
            } else {
                value += (float)(c - '0') * frac_scale;
                frac_scale *= 0.1f;
            }
        } else if (c == '.' && has_frac == 0U) {
            has_frac = 1U;
        } else {
            break;
        }
        i++;
    }

    if (has_digit == 0U) {
        return 0U;
    }

    *value_out = sign * value;
    return 1U;
}

static uint8_t cmd_parse_bias(const uint8_t *line, uint8_t len, float *bias_deg)
{
    uint8_t i = 0U;

    if (line == 0 || bias_deg == 0 || len == 0U) {
        return 0U;
    }

    while ((i < len) && line[i] == ' ') {
        i++;
    }

    if (i >= len) {
        return 0U;
    }

    if (line[i] == 'B' || line[i] == 'b') {
        i++;
        return cmd_parse_float(line + i, (uint8_t)(len - i), bias_deg);
    }

    if ((i + 4U <= len) &&
        (line[i] == 'b' || line[i] == 'B') &&
        (line[i + 1U] == 'i' || line[i + 1U] == 'I') &&
        (line[i + 2U] == 'a' || line[i + 2U] == 'A') &&
        (line[i + 3U] == 's' || line[i + 3U] == 'S')) {
        i += 4U;
        return cmd_parse_float(line + i, (uint8_t)(len - i), bias_deg);
    }

    return 0U;
}

static uint8_t cmd_mode_from_digit(uint8_t c, ChassisMode_e *mode)
{
    if ((mode == 0) || (c < '0') || (c > '4')) {
        return 0U;
    }

    *mode = (ChassisMode_e)(c - '0');
    return 1U;
}

static uint8_t cmd_mode_from_word(const uint8_t *line,
                                  uint8_t len,
                                  uint8_t start,
                                  ChassisMode_e *mode)
{
    if (mode == 0) {
        return 0U;
    }

    if ((cmd_match_word(line, len, start, "stand") != 0U) ||
        (cmd_match_word(line, len, start, "balance") != 0U)) {
        *mode = CHASSIS_MODE_STAND;
        return 1U;
    }
    if ((cmd_match_word(line, len, start, "forward") != 0U) ||
        (cmd_match_word(line, len, start, "go") != 0U)) {
        *mode = CHASSIS_MODE_FORWARD;
        return 1U;
    }
    if ((cmd_match_word(line, len, start, "backward") != 0U) ||
        (cmd_match_word(line, len, start, "back") != 0U) ||
        (cmd_match_word(line, len, start, "reverse") != 0U)) {
        *mode = CHASSIS_MODE_BACKWARD;
        return 1U;
    }
    if ((cmd_match_word(line, len, start, "stop") != 0U) ||
        (cmd_match_word(line, len, start, "park") != 0U)) {
        *mode = CHASSIS_MODE_STAND;
        return 1U;
    }
    if (cmd_match_word(line, len, start, "left") != 0U) {
        *mode = CHASSIS_MODE_TURN_LEFT;
        return 1U;
    }
    if (cmd_match_word(line, len, start, "right") != 0U) {
        *mode = CHASSIS_MODE_TURN_RIGHT;
        return 1U;
    }

    return 0U;
}

static uint8_t cmd_parse_mode(const uint8_t *line,
                              uint8_t len,
                              ChassisMode_e *mode)
{
    uint8_t i = 0U;

    if ((line == 0) || (mode == 0) || (len == 0U)) {
        return 0U;
    }

    while ((i < len) && (cmd_is_delimiter(line[i]) != 0U)) {
        i++;
    }
    if (i >= len) {
        return 0U;
    }

    if (cmd_mode_from_word(line, len, i, mode) != 0U) {
        return 1U;
    }

    if ((cmd_mode_from_digit(line[i], mode) != 0U) &&
        (((uint8_t)(i + 1U) >= len) ||
         (cmd_is_delimiter(line[i + 1U]) != 0U))) {
        return 1U;
    }

    if (cmd_to_lower(line[i]) != 'm') {
        return 0U;
    }

    i++;
    if ((i + 3U <= len) &&
        (cmd_to_lower(line[i]) == 'o') &&
        (cmd_to_lower(line[i + 1U]) == 'd') &&
        (cmd_to_lower(line[i + 2U]) == 'e')) {
        i += 3U;
    }

    while ((i < len) && (cmd_is_delimiter(line[i]) != 0U)) {
        i++;
    }
    if (i >= len) {
        return 0U;
    }

    if (cmd_mode_from_digit(line[i], mode) != 0U) {
        return 1U;
    }
    return cmd_mode_from_word(line, len, i, mode);
}

static uint8_t cmd_parse_pid(const uint8_t *line,
                             uint8_t len,
                             BalancePidRuntimeParam_e *param,
                             float *value)
{
    uint8_t i = 0U;
    uint8_t c0;
    uint8_t c1;

    if (line == 0 || param == 0 || value == 0 || len == 0U) {
        return 0U;
    }

    while ((i < len) && (cmd_is_delimiter(line[i]) != 0U)) {
        i++;
    }
    if (i >= len) {
        return 0U;
    }

    c0 = cmd_to_lower(line[i]);
    c1 = ((uint8_t)(i + 1U) < len) ? cmd_to_lower(line[i + 1U]) : 0U;

    if (c0 == 'k') {
        if (c1 == 'p') {
            *param = BALANCE_PID_RUNTIME_KP;
            i += 2U;
        } else if (c1 == 'i') {
            *param = BALANCE_PID_RUNTIME_KI;
            i += 2U;
        } else if (c1 == 'd') {
            *param = BALANCE_PID_RUNTIME_KD;
            i += 2U;
        } else {
            return 0U;
        }
    } else if (c0 == 'p') {
        *param = BALANCE_PID_RUNTIME_KP;
        i++;
    } else if (c0 == 'i') {
        *param = BALANCE_PID_RUNTIME_KI;
        i++;
    } else if (c0 == 'd') {
        *param = BALANCE_PID_RUNTIME_KD;
        i++;
    } else if ((c0 == 'o') || (c0 == 'l')) {
        *param = BALANCE_PID_RUNTIME_SPEED_LIMIT_RPM;
        i++;
        while ((i < len) &&
               ((cmd_to_lower(line[i]) >= 'a') &&
                (cmd_to_lower(line[i]) <= 'z'))) {
            i++;
        }
    } else if (c0 == 'c') {
        *param = BALANCE_PID_RUNTIME_CURRENT_LIMIT_A;
        i++;
        while ((i < len) &&
               ((cmd_to_lower(line[i]) >= 'a') &&
                (cmd_to_lower(line[i]) <= 'z'))) {
            i++;
        }
    } else if (c0 == 'r') {
        *param = BALANCE_PID_RUNTIME_CURRENT_SLEW_A_PER_SEC;
        i++;
        while ((i < len) &&
               ((cmd_to_lower(line[i]) >= 'a') &&
                (cmd_to_lower(line[i]) <= 'z'))) {
            i++;
        }
    } else if ((c0 == 's') &&
               ((uint8_t)(i + 4U) <= len) &&
               (cmd_to_lower(line[i + 1U]) == 'l') &&
               (cmd_to_lower(line[i + 2U]) == 'e') &&
               (cmd_to_lower(line[i + 3U]) == 'w')) {
        *param = BALANCE_PID_RUNTIME_CURRENT_SLEW_A_PER_SEC;
        i += 4U;
    } else {
        return 0U;
    }

    return cmd_parse_float(line + i, (uint8_t)(len - i), value);
}

uint8_t ChassisSerialCmd_HandleLine(const uint8_t *line, uint8_t len)
{
    float value;
    ChassisMode_e mode;
    BalancePidRuntimeParam_e pid_param;

    chassis_serial_cmd_rx_count++;

    if (cmd_parse_mode(line, len, &mode) != 0U) {
        Chassis_SetControlMode(mode);
        chassis_serial_cmd_ok_count++;
        chassis_serial_cmd_last_type = 1U;
        chassis_serial_cmd_last_value = (float)mode;
        return 1U;
    }
    if (cmd_parse_bias(line, len, &value) != 0U) {
        Balance_SetManualBiasDeg(value);
        chassis_serial_cmd_ok_count++;
        chassis_serial_cmd_last_type = 2U;
        chassis_serial_cmd_last_value = value;
        return 1U;
    }
    if (cmd_parse_pid(line, len, &pid_param, &value) != 0U) {
        Balance_SetPidRuntimeParam(pid_param, value);
        chassis_serial_cmd_ok_count++;
        chassis_serial_cmd_last_type = (uint8_t)(10U + (uint8_t)pid_param);
        chassis_serial_cmd_last_value = value;
        return 1U;
    }
    if (cmd_parse_float(line, len, &value) != 0U) {
        WheelSpeed_SetTargetRpm(value);
        chassis_serial_cmd_ok_count++;
        chassis_serial_cmd_last_type = 3U;
        chassis_serial_cmd_last_value = value;
        return 1U;
    }

    return 0U;
}
