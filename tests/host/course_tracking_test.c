#include <stdint.h>
#include <stdio.h>

#include "config.h"
#include "mode_control.h"
#include "track.h"

static int failures = 0;
static uint32_t now_ms = 0U;

#define CHECK(condition, message)                                      \
    do {                                                               \
        if (!(condition)) {                                            \
            printf("FAIL: %s (line %d)\n", (message), __LINE__);      \
            failures++;                                                \
        }                                                              \
    } while (0)

static ModeCommand_t get_command(void)
{
    ModeCommand_t command;

    Mode_GetCommand(&command);
    return command;
}

static void tick_frame(float error,
                       uint8_t black_count,
                       float actual_velocity_mps,
                       uint8_t laser_safe)
{
    now_ms++;
    Mode_SetActualVelocity(actual_velocity_mps);
    Mode_SetLaserSafe(laser_safe);
    Mode_TrackingSetSample(error, black_count, 0U, now_ms);
    Mode_Update(now_ms);
}

static void run_frames(uint32_t duration_ms,
                       uint32_t frame_period_ms,
                       float error,
                       uint8_t black_count,
                       float actual_velocity_mps,
                       uint8_t laser_safe)
{
    for (uint32_t elapsed = 1U; elapsed <= duration_ms; elapsed++) {
        now_ms++;
        Mode_SetActualVelocity(actual_velocity_mps);
        Mode_SetLaserSafe(laser_safe);
        if ((elapsed == 1U) ||
            ((frame_period_ms != 0U) &&
             ((elapsed % frame_period_ms) == 0U))) {
            Mode_TrackingSetSample(error, black_count, 0U, now_ms);
        }
        Mode_Update(now_ms);
    }
}

static void enter_course_tracking(void)
{
    ModeCommand_t command;

    now_ms = 0U;
    Mode_Init();
    Mode_Set(CHASSIS_MODE_TRACKING);
    tick_frame(0.0f, 1U, 0.05f, 0U);
    command = get_command();
    CHECK(command.drive_enabled == 0U && command.laser_enabled == 0U,
          "first course frame must stay gated");
    run_frames(TRACKING_REFIND_CONFIRM_MS + 30U,
               20U, 0.0f, 1U, 0.05f, 0U);
    CHECK(Mode_TrackingGetPhase() ==
              MODE_TRACKING_PHASE_WAIT_START_MARKER,
          "fresh frames must arm start-marker search");
    command = get_command();
    CHECK(command.drive_enabled != 0U && command.laser_enabled == 0U,
          "start search must move with laser off");
}

static void test_reverse_markers_are_rejected(void)
{
    ModeCommand_t command;

    enter_course_tracking();
    run_frames(TRACKING_START_BLOCK_CONFIRM_MS + 100U,
               20U, 0.0f, TRACKING_BLOCK_BLACK_COUNT_MIN,
               -0.03f, 0U);
    CHECK(Mode_TrackingGetPhase() ==
              MODE_TRACKING_PHASE_WAIT_START_MARKER,
          "reverse start-marker contact must not advance the course");
    command = get_command();
    CHECK(command.drive_enabled != 0U && command.laser_enabled == 0U,
          "reverse false marker must keep requesting safe forward recovery");

    run_frames(TRACKING_START_BLOCK_CONFIRM_MS + 30U,
               20U, 0.0f, TRACKING_BLOCK_BLACK_COUNT_MIN,
               0.05f, 0U);
    CHECK(Mode_TrackingGetPhase() == MODE_TRACKING_PHASE_WAIT_START_EXIT,
          "the start marker may advance only during measured forward motion");
}

static void advance_to_target_runout(void)
{
    enter_course_tracking();
    run_frames(TRACKING_START_BLOCK_CONFIRM_MS + 30U,
               20U, 0.0f, TRACKING_BLOCK_BLACK_COUNT_MIN,
               0.05f, 0U);
    run_frames(TRACKING_START_BLOCK_EXIT_MS + 40U,
               20U, 0.0f, 1U, 0.05f, 0U);
    CHECK(Mode_TrackingGetPhase() == MODE_TRACKING_PHASE_FOLLOW_TO_TARGET,
          "leaving A must enter target follow");

    run_frames(TRACKING_TARGET_BLOCK_CONFIRM_MS + 40U,
               20U, 0.0f, TRACKING_TARGET_BLOCK_BLACK_COUNT_MIN,
               0.05f, 0U);
    CHECK(Mode_TrackingGetPhase() == MODE_TRACKING_PHASE_TARGET_RUNOUT,
          "forward target marker must enter target runout");
}

static void test_reverse_runout_is_aborted(void)
{
    ModeCommand_t command;

    advance_to_target_runout();
    tick_frame(0.0f, TRACKING_TARGET_BLOCK_BLACK_COUNT_MIN, -0.03f, 0U);
    CHECK(Mode_TrackingGetPhase() == MODE_TRACKING_PHASE_FOLLOW_TO_TARGET,
          "target runout must abort when measured motion reverses");
    command = get_command();
    CHECK(command.drive_enabled != 0U && command.laser_enabled == 0U,
          "aborted runout must recover forward without laser");
}

static void test_full_target_and_finish_sequence(void)
{
    ModeCommand_t command;

    advance_to_target_runout();
    run_frames(TRACKING_TARGET_BLOCK_RUNOUT_MS + 10U,
               20U, 0.0f, TRACKING_TARGET_BLOCK_BLACK_COUNT_MIN,
               0.05f, 0U);
    CHECK(Mode_TrackingGetPhase() ==
              MODE_TRACKING_PHASE_TARGET_WAIT_SETTLE,
          "target runout must finish only after measured forward time");
    command = get_command();
    CHECK(command.drive_enabled == 0U && command.laser_enabled == 0U,
          "target wait-settle must stop with laser off");

    tick_frame(0.0f, TRACKING_TARGET_BLOCK_BLACK_COUNT_MIN, 0.0f, 1U);
    CHECK(Mode_TrackingGetPhase() ==
              MODE_TRACKING_PHASE_TARGET_LASER_DWELL,
          "laser-safe stop must enter target dwell");
    command = get_command();
    CHECK(command.drive_enabled == 0U && command.laser_enabled != 0U,
          "target dwell may request laser only while stopped and safe");

    run_frames(1000U, 20U, 0.0f,
               TRACKING_TARGET_BLOCK_BLACK_COUNT_MIN, 0.0f, 1U);
    tick_frame(0.0f, TRACKING_TARGET_BLOCK_BLACK_COUNT_MIN, 0.0f, 0U);
    command = get_command();
    CHECK(Mode_TrackingGetPhase() ==
              MODE_TRACKING_PHASE_TARGET_WAIT_SETTLE &&
              command.laser_enabled == 0U,
          "one unsafe cycle must extinguish laser and reset dwell");

    tick_frame(0.0f, TRACKING_TARGET_BLOCK_BLACK_COUNT_MIN, 0.0f, 1U);
    run_frames(TRACKING_TARGET_BLOCK_LASER_MS - 100U,
               20U, 0.0f, TRACKING_TARGET_BLOCK_BLACK_COUNT_MIN,
               0.0f, 1U);
    CHECK(Mode_TrackingGetPhase() ==
              MODE_TRACKING_PHASE_TARGET_LASER_DWELL,
          "interrupted dwell must not reuse elapsed laser time");
    command = get_command();
    CHECK(command.laser_enabled != 0U,
          "laser may remain on during the restarted safe dwell");

    run_frames(150U, 20U, 0.0f,
               TRACKING_TARGET_BLOCK_BLACK_COUNT_MIN, 0.0f, 1U);
    CHECK(Mode_TrackingGetPhase() == MODE_TRACKING_PHASE_TARGET_DEPART,
          "full uninterrupted dwell must start target departure");
    command = get_command();
    CHECK(command.laser_enabled == 0U && command.drive_enabled != 0U,
          "departure must turn laser off before moving");

    run_frames(TRACKING_TARGET_BLOCK_DEPART_MS + 20U,
               20U, 0.0f, 1U, 0.05f, 0U);
    CHECK(Mode_TrackingGetPhase() == MODE_TRACKING_PHASE_WAIT_TARGET_EXIT,
          "measured forward departure must reach target-exit wait");
    run_frames(TRACKING_TARGET_BLOCK_EXIT_MS + 40U,
               20U, 0.0f, 1U, 0.05f, 0U);
    CHECK(Mode_TrackingGetPhase() == MODE_TRACKING_PHASE_FOLLOW_TO_FINISH,
          "leaving the target must arm finish follow");

    run_frames(TRACKING_FINISH_BLOCK_CONFIRM_MS + 30U,
               20U, 0.0f, TRACKING_BLOCK_BLACK_COUNT_MIN,
               0.05f, 0U);
    CHECK(Mode_TrackingGetPhase() == MODE_TRACKING_PHASE_FINISH_RUNOUT,
          "forward B marker must enter finish runout");
    run_frames(TRACKING_FINISH_BLOCK_RUNOUT_MS + 10U,
               20U, 0.0f, TRACKING_BLOCK_BLACK_COUNT_MIN,
               0.05f, 0U);
    command = get_command();
    CHECK(Mode_TrackingGetPhase() == MODE_TRACKING_PHASE_FINISHED,
          "finish runout must latch course completion");
    CHECK(command.drive_enabled == 0U && command.turn_rpm == 0.0f &&
              command.laser_enabled == 0U,
          "finished course must latch a laser-off hold");
}

static void test_stand_interrupts_immediately(void)
{
    ModeCommand_t command;

    enter_course_tracking();
    Mode_Set(CHASSIS_MODE_STAND);
    now_ms++;
    Mode_Update(now_ms);
    command = get_command();
    CHECK(Mode_Get() == CHASSIS_MODE_STAND &&
              command.drive_enabled == 0U && command.turn_rpm == 0.0f &&
              command.laser_enabled == 0U,
          "stand must interrupt moving tracking in one mode update");

    advance_to_target_runout();
    run_frames(TRACKING_TARGET_BLOCK_RUNOUT_MS + 10U,
               20U, 0.0f, TRACKING_TARGET_BLOCK_BLACK_COUNT_MIN,
               0.05f, 0U);
    tick_frame(0.0f, TRACKING_TARGET_BLOCK_BLACK_COUNT_MIN, 0.0f, 1U);
    CHECK(get_command().laser_enabled != 0U,
          "test setup must reach active laser dwell");
    Mode_Set(CHASSIS_MODE_STAND);
    now_ms++;
    Mode_Update(now_ms);
    command = get_command();
    CHECK(command.drive_enabled == 0U && command.turn_rpm == 0.0f &&
              command.laser_enabled == 0U,
          "stand must extinguish an active laser in one mode update");
}

int main(void)
{
    test_reverse_markers_are_rejected();
    test_reverse_runout_is_aborted();
    test_full_target_and_finish_sequence();
    test_stand_interrupts_immediately();

    if (failures != 0) {
        printf("%d course tracking assertion(s) failed\n", failures);
        return 1;
    }
    printf("course tracking tests passed\n");
    return 0;
}
