#include "mode.h"
#include "Plane.h"

bool ModeExternal::_enter()
{
    plane.next_WP_loc = plane.current_loc;
    update_stage();
    return true;
}

void ModeExternal::update()
{
    update_stage();
    switch (stage) {
        case stage_class::HOVER:
            update_hover();
            break;
        case stage_class::ANGLE:
            update_angle();
            break;
        case stage_class::RATE:
            update_rate();
            break;
        case stage_class::FBWB:
            update_fbwb();
            break;
        case stage_class::WP:
            update_wp();
            break;
    }
}

void ModeExternal::update_stage()
{
    if (plane.uart.control_status.valid) {
        switch (plane.uart.control_status.type) {
            case 0x1A:
            {
                set_stage(stage_class::ANGLE);
                break;
            }
            case 0xFD:
            {
                set_stage(stage_class::RATE);
                break;
            }
            case 0x3C:
            {
                set_stage(stage_class::FBWB);
                break;
            }
            case 0x55:
            {
                set_stage(stage_class::WP);
                break;
            }
            default:
            {
                set_stage(stage_class::HOVER);
                break;
            }
        }
    } else {
        set_stage(stage_class::HOVER);
    }
}

void ModeExternal::update_hover()
{
    plane.calc_nav_roll();
    if (plane.stick_mixing_enabled() && plane.flight_option_enabled(FlightOptions::ENABLE_LOITER_ALT_CONTROL)) {
        plane.update_fbwb_speed_height();
    } else {
        plane.calc_nav_pitch();
        plane.calc_throttle();
    }
}

void ModeExternal::update_angle()
{
    plane.nav_roll_cd = plane.uart.control_status.cmd_roll * 100.f;
    plane.nav_pitch_cd = (plane.uart.control_status.cmd_pitch - plane.g.pitch_trim) * 100.f;
    if (plane.g2.user_external_auto_throttle.get() == 1) {
        SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, attack_throttle_with_comp());
    } else {
        SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, attack_throttle_raw());
    }
}

void ModeExternal::update_rate()
{
    // plane.nav_roll_cd = plane.uart.control_status.cmd_roll * 100.f;
    plane.nav_pitch_cd = plane.ahrs.pitch_sensor;
    if (plane.g2.user_external_auto_throttle.get() == 1) {
        SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, attack_throttle_with_comp());
    } else {
        SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, attack_throttle_raw());
    }
}

void ModeExternal::update_fbwb()
{
    plane.nav_roll_cd = plane.uart.control_status.cmd_roll * 100.f;
    plane.update_load_factor();
    plane.target_altitude.amsl_cm = plane.uart.control_status.cmd_alt * 100.f;
    plane.new_airspeed_cm = plane.uart.control_status.cmd_speed * 100.f;
    plane.calc_nav_pitch();
    plane.calc_throttle();
}

void ModeExternal::update_wp()
{
    plane.calc_nav_roll();
    plane.calc_nav_pitch();
    plane.calc_throttle();
}

float ModeExternal::attack_throttle_raw()
{
    float ret = plane.aparm.throttle_cruise;
    if (plane.g2.user_external_cmd_throttle.get() == 1) {
        ret = plane.uart.control_status.cmd_throttle;
    }
    return ret;
}

float ModeExternal::attack_throttle_with_comp()
{
    float quad_hover_throttle = 35.f;
    float plane_hover_throttle = attack_throttle_raw();
    float pitch_deg = 0.5* (plane.nav_pitch_cd * 0.01f + degrees(AP::ahrs().get_pitch()));
    pitch_deg = constrain_float(pitch_deg, -30.f, 30.f);
    float ret = (quad_hover_throttle * sinf(radians(pitch_deg)) + plane_hover_throttle * cosf(radians(pitch_deg)));
    return ret;
}

void ModeExternal::navigate()
{
    switch (stage) {
        case stage_class::HOVER:
            plane.update_loiter(0);
            break;
        case stage_class::ANGLE:
            break;
        case stage_class::RATE:
            break;
        case stage_class::FBWB:
            break;
        case stage_class::WP:
        {
            plane.next_WP_loc = plane.uart.control_status.cmd_loc;
            if (plane.current_loc.get_distance(plane.prev_WP_loc) > 200.0f) {
                plane.prev_WP_loc = plane.current_loc;
            }
            plane.nav_controller->update_waypoint(plane.prev_WP_loc, plane.next_WP_loc);
            break;
        }
        default:
            break;
    }
}

void ModeExternal::run()
{
    switch (stage) {
        case stage_class::HOVER:
        case stage_class::FBWB:
        case stage_class::WP:
        case stage_class::ANGLE:
        default:
            // Direct stick mixing functionality has been removed, so as not to remove all stick mixing from the user completely
            // the old direct option is now used to enable fbw mixing, this is easier than doing a param conversion.
            if ((plane.g.stick_mixing == StickMixing::FBW) || (plane.g.stick_mixing == StickMixing::DIRECT_REMOVED)) {
                plane.stabilize_stick_mixing_fbw();
            }
            plane.stabilize_roll();
            plane.stabilize_pitch();
            plane.stabilize_yaw();
            break;
        case stage_class::RATE:
            {
                plane.stabilize_external_rate();
            }
            break;
    }
}

bool ModeExternal::is_angle_mode()
{
    return (stage == stage_class::ANGLE);
}

bool ModeExternal::is_rate_mode()
{
    return (stage == stage_class::RATE);
}

void ModeExternal::set_stage(ModeExternal::stage_class stage_in)
{
    if (stage == stage_in) {return;}
    stage = stage_in;
    switch(stage) {
        case stage_class::HOVER:
            plane.next_WP_loc = plane.current_loc;
            gcs().send_text(MAV_SEVERITY_INFO, "In Hover");
            break;
        case stage_class::ANGLE:
            gcs().send_text(MAV_SEVERITY_INFO, "In ANGLE");
            break;
        case stage_class::RATE:
            gcs().send_text(MAV_SEVERITY_INFO, "In RATE");
            break;
        case stage_class::FBWB:
            gcs().send_text(MAV_SEVERITY_INFO, "In FBWB");
            break;
        case stage_class::WP:
            plane.prev_WP_loc = plane.current_loc;
            plane.next_WP_loc = plane.uart.control_status.cmd_loc;
            gcs().send_text(MAV_SEVERITY_INFO, "In WP");
            break;
        default:
            gcs().send_text(MAV_SEVERITY_INFO, "ERR UNKNOW in 50");
            break;
    }
}
