#include "mode.h"
#include "Plane.h"

bool ModeMission::_enter()
{
#if HAL_SOARING_ENABLED
    // for ArduSoar soaring_controller
    plane.g2.soaring_controller.init_cruising();
#endif

    plane.set_target_altitude_current();

    return true;
}

void ModeMission::update()
{
    // Thanks to Yury MonZon for the altitude limit code!
    plane.nav_roll_cd = plane.channel_roll->norm_input() * plane.roll_limit_cd;

    if (!plane.umission.valid())
    {
        plane.set_mode(plane.mode_loiter, ModeReason::GCS_COMMAND);
    }
    if (plane.umission.get_control_type() == 0)
    {
        // roll control
        plane.nav_roll_cd = plane.umission.get_control_roll()*100.f;
    } else {
        if (AP::gps().status() >= AP_GPS::GPS_OK_FIX_3D && AP::gps().ground_speed() > 10.0f) {
            plane.nav_controller->update_heading_hold(plane.umission.get_control_course()*100.f);
        } else {
            plane.nav_controller->update_level_flight();
            if (!plane.set_mode(plane.mode_loiter, ModeReason::GCS_COMMAND)) {
                plane.set_mode(plane.mode_circle, ModeReason::GCS_COMMAND);
            }
        }
        plane.calc_nav_roll();
    }
    plane.target_altitude.amsl_cm = plane.home.alt + (int32_t)(plane.umission.get_control_altitude()*100.f);
    plane.update_load_factor();
    plane.calc_throttle();
    plane.calc_nav_pitch();
}

