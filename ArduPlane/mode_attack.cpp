#include "mode.h"
#include "Plane.h"

bool ModeAttack::_enter()
{
    if (plane.uattack.is_active()) {
        gcs().send_text(MAV_SEVERITY_INFO, "Attack!");
        _cmd_throttle = MAX(SRV_Channels::get_output_scaled(SRV_Channel::k_throttle), plane.aparm.throttle_cruise);
        return true;
    }
    else {
        gcs().send_text(MAV_SEVERITY_INFO, "No target, Can NOT attack!");
    }
    return false;
}

void ModeAttack::update()
{
    // plane.nav_roll_cd = 0;//plane.ahrs.roll_sensor;
    plane.nav_pitch_cd = plane.ahrs.pitch_sensor;

    float throtle_rate = plane.g2.user_attack_throttle_rate*plane.G_Dt;
    float target_throttle = plane.g2.user_attack_throttle;
    _cmd_throttle = _cmd_throttle + constrain_float(target_throttle - _cmd_throttle, -throtle_rate, throtle_rate);

    if (!plane.uattack.is_active()) {
        if (plane.set_mode(plane.mode_auto, ModeReason::ATTACK_END)) {
            gcs().send_text(MAV_SEVERITY_INFO, "Back to AUTO");
        } else {
            plane.set_mode(plane.mode_fbwb, ModeReason::ATTACK_END);
            gcs().send_text(MAV_SEVERITY_INFO, "Back to FBWB");
        }
    }
}

float ModeAttack::get_cmd_throttle() {
    return _cmd_throttle;
}