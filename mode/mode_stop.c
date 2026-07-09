#include "mode_private.h"

#include "config.h"

void Mode_StopBuild(ModeCommand_t *command)
{
    Mode_CommandClear(command);
    command->bias_deg = BALANCE_MODE_STAND_BIAS_DEG;
}
