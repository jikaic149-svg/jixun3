#include "mode_private.h"

#include "config.h"

void Mode_StopBuild(ModeCommand_t *command)
{
    Mode_MotionBuild(command, 0.0f, BALANCE_MODE_STAND_BIAS_DEG, 0.0f);
}
