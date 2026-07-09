#include "mode_private.h"

#include "config.h"

void Mode_ForwardBuild(ModeCommand_t *command)
{
    Mode_CommandClear(command);
    command->velocity_mps = BALANCE_MODE_DRIVE_VELOCITY_MPS;
    command->bias_deg = BALANCE_MODE_FORWARD_BIAS_DEG;
    command->drive_enabled = 1U;
}
