#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "chassis_serial_cmd.h"
#include "gimbal_task.h"
#include "vision.h"

volatile uint32_t chassis_serial_cmd_rx_count = 0U;
volatile uint32_t chassis_serial_cmd_ok_count = 0U;
volatile uint8_t chassis_serial_cmd_last_type = 0U;
volatile float chassis_serial_cmd_last_value = 0.0f;

static int failures = 0;
static uint32_t mode_calls = 0U;
static uint32_t wheel_calls = 0U;
static ChassisMode_e last_mode = CHASSIS_MODE_STAND;
static float last_wheel_rpm = 0.0f;
static Vision_Instance_t vision;

#define CHECK(condition, message)                                      \
    do {                                                               \
        if (!(condition)) {                                            \
            printf("FAIL: %s (line %d)\n", (message), __LINE__);      \
            failures++;                                                \
        }                                                              \
    } while (0)

void Chassis_SetControlMode(ChassisMode_e mode)
{
    mode_calls++;
    last_mode = mode;
}

void WheelSpeed_SetTargetRpm(float rpm)
{
    wheel_calls++;
    last_wheel_rpm = rpm;
}

void Balance_SetManualBiasDeg(float bias_deg)
{
    (void)bias_deg;
}

void Balance_SetPidRuntimeParam(BalancePidRuntimeParam_e param, float value)
{
    (void)param;
    (void)value;
}

static Vision_ParseResult_t route_line(const char *text)
{
    Vision_ParseResult_t result =
        Vision_ParseLine(&vision,
                         (const uint8_t *)text,
                         (uint16_t)strlen(text));

    if (result == VISION_PARSE_NOT_VISION) {
        (void)ChassisSerialCmd_HandleLine((const uint8_t *)text,
                                          (uint8_t)strlen(text));
    }
    return result;
}

static void reset_calls(void)
{
    mode_calls = 0U;
    wheel_calls = 0U;
    last_mode = CHASSIS_MODE_STAND;
    last_wheel_rpm = 0.0f;
}

static void expect_mode(const char *text, ChassisMode_e expected)
{
    reset_calls();
    CHECK(route_line(text) == VISION_PARSE_NOT_VISION,
          "mode command must not be classified as vision");
    CHECK(mode_calls == 1U && last_mode == expected && wheel_calls == 0U,
          "mode command must select only the expected mode");
}

static void expect_rejected(const char *text)
{
    reset_calls();
    (void)route_line(text);
    CHECK(mode_calls == 0U && wheel_calls == 0U,
          "invalid explicit mode must not alter mode or wheel speed");
}

static void test_mode_compatibility(void)
{
    static const ChassisMode_e legacy_modes[] = {
        CHASSIS_MODE_STAND,
        CHASSIS_MODE_FORWARD,
        CHASSIS_MODE_BACKWARD,
        CHASSIS_MODE_TURN_LEFT,
        CHASSIS_MODE_TURN_RIGHT,
        CHASSIS_MODE_TRACKING
    };

    CHECK(CHASSIS_MODE_STAND == 0 && CHASSIS_MODE_TRACKING == 5 &&
              CHASSIS_MODE_SQUARE_TRACKING == 6,
          "mode numeric ABI must remain 0..5 with square appended as 6");

    for (uint8_t i = 0U; i < 6U; i++) {
        char digit[2] = {(char)('0' + i), '\0'};
        expect_mode(digit, legacy_modes[i]);
    }
    expect_mode("track", CHASSIS_MODE_TRACKING);
    expect_mode("mode 5", CHASSIS_MODE_TRACKING);
    expect_mode("square", CHASSIS_MODE_SQUARE_TRACKING);
    expect_mode("box", CHASSIS_MODE_SQUARE_TRACKING);
    expect_mode("mode 6", CHASSIS_MODE_SQUARE_TRACKING);
    expect_mode("mode6", CHASSIS_MODE_SQUARE_TRACKING);
    expect_mode("m6", CHASSIS_MODE_SQUARE_TRACKING);

    reset_calls();
    CHECK(route_line("6") == VISION_PARSE_NOT_VISION,
          "bare 6 must not be classified as vision");
    CHECK(mode_calls == 0U && wheel_calls == 1U && last_wheel_rpm == 6.0f,
          "bare 6 must retain the legacy wheel-speed meaning");
}

static void test_invalid_explicit_modes(void)
{
    static const char *const invalid[] = {
        "mode 7", "m7", "m 7", "mode6x", "track6", "square6",
        "square,garbage", "mode 6,0,2", "5x", "6x", "6junk"
    };

    for (uint8_t i = 0U;
         i < (uint8_t)(sizeof(invalid) / sizeof(invalid[0]));
         i++) {
        expect_rejected(invalid[i]);
    }
}

static void test_vision_frames_never_become_commands(void)
{
    static const char *const invalid[] = {
        "6,0,2", "6000,0,1", "6,", "5,0,2", "5000,0,1", "5,"
    };
    int16_t dx = 0;
    int16_t dy = 0;
    uint8_t valid = 0U;
    uint32_t frame_count;

    reset_calls();
    CHECK(route_line("6,0,1") == VISION_PARSE_VALID,
          "valid square-leading visual frame must remain vision data");
    frame_count = Vision_GetSnapshot(&vision, &dx, &dy, &valid);
    CHECK(frame_count == 1U && dx == 6 && dy == 0 && valid == 1U,
          "valid visual frame must update only the vision snapshot");
    CHECK(mode_calls == 0U && wheel_calls == 0U,
          "valid visual frame must not issue a control command");

    for (uint8_t i = 0U;
         i < (uint8_t)(sizeof(invalid) / sizeof(invalid[0]));
         i++) {
        reset_calls();
        CHECK(route_line(invalid[i]) == VISION_PARSE_INVALID,
              "bad numeric comma candidate must be consumed as invalid vision");
        CHECK(mode_calls == 0U && wheel_calls == 0U,
              "bad visual candidate must not issue a control command");
        CHECK(Vision_GetSnapshot(&vision, &dx, &dy, &valid) == frame_count &&
                  dx == 6 && dy == 0 && valid == 1U,
              "bad visual candidate must preserve the last good snapshot");
    }
}

int main(void)
{
    Vision_Init(&vision);
    test_mode_compatibility();
    test_invalid_explicit_modes();
    test_vision_frames_never_become_commands();

    if (failures != 0) {
        printf("%d square command assertion(s) failed\n", failures);
        return 1;
    }
    printf("square command tests passed\n");
    return 0;
}
