#include "mode_private.h"

#include "config.h"

void Mode_TurnLeftBuild(ModeCommand_t *command)
{
    Mode_CommandClear(command);
    command->turn_rpm = BALANCE_MODE_TURN_SPEED_RPM;
}
