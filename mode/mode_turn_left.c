#include "mode_private.h"

#include "config.h"

void Mode_TurnLeftBuild(ModeCommand_t *command)
{
    Mode_MotionBuild(command,
                     0.0f,
                     BALANCE_MODE_STAND_BIAS_DEG,
                     BALANCE_MODE_TURN_SPEED_RPM);
}
