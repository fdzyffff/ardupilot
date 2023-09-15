#include "mode.h"
#include "Plane.h"

bool ModeAttack::_enter()
{
    return true;
}

void ModeAttack::update()
{
    // plane.nav_roll_cd = 0;//plane.ahrs.roll_sensor;
    plane.nav_pitch_cd = plane.ahrs.pitch_sensor;
}

