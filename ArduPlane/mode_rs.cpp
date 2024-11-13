#include "mode.h"
#include "Plane.h"

void ModeRS::update()
{
    // set nav_roll and nav_pitch using sticks
    plane.nav_roll_cd = 0;
    plane.nav_pitch_cd = -2700;
    cmd_throttle = 0.0f;
}
