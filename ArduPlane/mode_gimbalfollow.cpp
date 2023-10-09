#include "mode.h"
#include "Plane.h"

bool ModeGimbalFollow::_enter()
{
    if (plane.ts_ctrl.valid()) {
        plane.set_target_altitude_current();
        return true;
    }
    gcs().send_text(MAV_SEVERITY_ALERT, "No Target Position");
    return false;
}

void ModeGimbalFollow::update()
{

    if (plane.ts_ctrl.valid()) {
        plane.prev_WP_loc = plane.current_loc;
        plane.next_WP_loc = plane.ts_ctrl.get_target_loc();
    } else {
        Location tmp_loc = plane.next_WP_loc;
        if (plane.set_mode(plane.mode_loiter, ModeReason::MISSION_END)) {
            plane.next_WP_loc = tmp_loc;
        } else {
            plane.set_mode(plane.mode_circle, ModeReason::MISSION_END);
        }
        gcs().send_text(MAV_SEVERITY_ALERT, "Exit GF mode");
    }

    plane.calc_nav_roll();
    plane.calc_nav_pitch();
    plane.calc_throttle();
}

void ModeGimbalFollow::navigate()
{
    // Zero indicates to use WP_LOITER_RAD
    plane.update_loiter(0);
}