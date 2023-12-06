#include "mode.h"
#include "Plane.h"

bool ModeWSLD::_enter()
{
    gcs().send_text(MAV_SEVERITY_INFO, "Water Slide Mode");

    return true;
}

void ModeWSLD::update()
{
    // set nav_roll and nav_pitch using sticks
    plane.nav_roll_cd  = plane.channel_roll->norm_input() * plane.roll_limit_cd;
    plane.update_load_factor();
    float pitch_input = plane.channel_pitch->norm_input();
    if (pitch_input > 0) {
        plane.nav_pitch_cd = pitch_input * plane.aparm.pitch_limit_max_cd;
    } else {
        plane.nav_pitch_cd = -(pitch_input * plane.pitch_limit_min_cd);
    }
    plane.nav_pitch_cd = constrain_int32(plane.nav_pitch_cd, plane.pitch_limit_min_cd, plane.aparm.pitch_limit_max_cd.get());
    if (plane.failsafe.rc_failsafe) {
        // FBWA failsafe glide
        plane.nav_roll_cd = 0;
        plane.nav_pitch_cd = 0;
        SRV_Channels::set_output_limit(SRV_Channel::k_throttle, SRV_Channel::Limit::MIN);
    }
    plane.calc_throttle();

    // static uint32_t _last_ms = millis();
    // if (millis() - _last_ms > 1000) {
    //     gcs().send_text(MAV_SEVERITY_INFO, "Thr %f", plane.WaterSlide_controller.get_throttle_demand());
    //     _last_ms = millis();
    // }

    // get scaled throttle input

}
