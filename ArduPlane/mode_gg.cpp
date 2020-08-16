#include "mode.h"
#include "Plane.h"

bool ModeGG::_enter()
{
    plane.throttle_allows_nudging = false;
    plane.auto_throttle_mode = false;
    plane.auto_navigation_mode = false;

	gcs().send_text(MAV_SEVERITY_INFO, "GG wp");
    plane.aparm.pitch_limit_min_cd.set(-8500);
    return true;
}

void ModeGG::update()
{
    // set nav_roll and nav_pitch using sticks
    plane.nav_roll_cd  = 0;
    plane.update_load_factor();
    plane.nav_pitch_cd = -8500;

    SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, 100);
}
