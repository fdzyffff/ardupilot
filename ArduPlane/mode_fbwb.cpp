#include "mode.h"
#include "Plane.h"

bool ModeFBWBFS::_enter()
{
#if HAL_SOARING_ENABLED
    // for ArduSoar soaring_controller
    plane.g2.soaring_controller.init_cruising();
#endif

    plane.set_target_altitude_current();

    return true;
}

void ModeFBWBFS::update()
{
    // Thanks to Yury MonZon for the altitude limit code!
    plane.nav_roll_cd = 0;
    plane.update_load_factor();

    plane.altitude_error_cm = plane.calc_altitude_error_cm();

    plane.calc_throttle();
    plane.calc_nav_pitch();
}

