#include "mode_private.h"

#include "config.h"

void Mode_BackwardBuild(ModeCommand_t *command)
{
    Mode_MotionBuild(command,
                     -BALANCE_MODE_DRIVE_VELOCITY_MPS,
                     BALANCE_MODE_BACKWARD_BIAS_DEG,
                     0.0f);
}
