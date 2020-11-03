#include "mode.h"
#include "Plane.h"

bool ModeFsAuto::_enter()
{
    plane.throttle_allows_nudging = false;
    plane.auto_throttle_mode = false;
    plane.auto_navigation_mode = false;

    gcs().send_text(MAV_SEVERITY_INFO, "Fs auto");
    plane.aparm.pitch_limit_min_cd.set(-2500);
    return true;
}

void ModeFsAuto::update()
{
    // set nav_roll and nav_pitch using sticks
    plane.nav_roll_cd  = 0;
    plane.update_load_factor();
    plane.nav_pitch_cd = -500;

    SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, 0);
}
