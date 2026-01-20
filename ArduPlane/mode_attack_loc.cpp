#include "mode.h"
#include "Plane.h"

bool ModeAttackLoc::_enter()
{
    if (plane.uattack.is_active_loc()) {
        target_loc = plane.uattack._Target_ptr_loc->target_loc;
        if (check_approach()) {
            set_stage(stage_class::HOVER);                
        } else {
            set_stage(stage_class::APPROACH);
            build_path();
            gcs().send_text(MAV_SEVERITY_INFO, "Attack LOC!");
        }
        _cmd_throttle = MAX(SRV_Channels::get_output_scaled(SRV_Channel::k_throttle), plane.aparm.throttle_cruise);
        return true;
    } else {
        gcs().send_text(MAV_SEVERITY_INFO, "No target, Can NOT attack!");
    }
    return false;
}

void ModeAttackLoc::run()
{
    switch (stage) {
        case stage_class::APPROACH:
        case stage_class::HOVER:

            // Direct stick mixing functionality has been removed, so as not to remove all stick mixing from the user completely
            // the old direct option is now used to enable fbw mixing, this is easier than doing a param conversion.
            if ((plane.g.stick_mixing == StickMixing::FBW) || (plane.g.stick_mixing == StickMixing::DIRECT_REMOVED)) {
                plane.stabilize_stick_mixing_fbw();
            }
            plane.stabilize_roll();
            plane.stabilize_pitch();
            plane.stabilize_yaw();
            break;
        case stage_class::ATTACK:
            plane.stabilize_attack();
            break;
        default:
            break;
    }
}

void ModeAttackLoc::build_path()
{
    // copy the current location into the OldWP slot
    // ---------------------------------------
    plane.prev_WP_loc = plane.current_loc;

    // Load the next_WP slot
    // ---------------------
    // gcs().send_text(MAV_SEVERITY_INFO, "BEFORE alt:%d", plane.next_WP_loc.alt);
    plane.next_WP_loc = target_loc;
    plane.next_WP_loc.alt = plane.current_loc.alt;
    // always over target for a distance
    float bearing_cd = plane.current_loc.get_bearing_to(target_loc);
    plane.next_WP_loc.offset_bearing(bearing_cd*0.01f, 200);
    // gcs().send_text(MAV_SEVERITY_INFO, "AFTER alt:%d", plane.next_WP_loc.alt);

    // used to control FBW and limit the rate of climb
    // -----------------------------------------------
    plane.set_target_altitude_current();

    plane.setup_glide_slope();
    plane.setup_turn_angle();

    // disable crosstrack, head directly to the point
    plane.auto_state.crosstrack = false;

    // reset loiter start time.
    plane.loiter.start_time_ms = 0;

    // start in non-VTOL mode
    plane.auto_state.vtol_loiter = false;
    
    plane.loiter_angle_reset();
}

void ModeAttackLoc::update()
{
    switch (stage) {
        case stage_class::APPROACH:
            update_approach();
            if (check_approach()) {
                set_stage(stage_class::HOVER);
            }
            break;
        case stage_class::ATTACK:
            if (plane.uattack.is_active_loc()) {
                set_stage(stage_class::HOVER);
            }
            update_attack();
            break;
        case stage_class::HOVER:
            update_hover();
            break;
        default:
            break;
    }
}

bool ModeAttackLoc::check_approach()
{
    return (plane.current_loc.get_distance(target_loc) < plane.uattack._Target_ptr_loc->nav_radius.get());
}

void ModeAttackLoc::update_approach()
{
    plane.calc_nav_roll();
    plane.calc_nav_pitch();
    plane.calc_throttle();
}

void ModeAttackLoc::update_hover()
{
    plane.calc_nav_roll();
    plane.calc_nav_pitch();
    plane.calc_throttle();
}

void ModeAttackLoc::update_attack()
{
    // plane.nav_roll_cd = 0;//plane.ahrs.roll_sensor;
    plane.nav_pitch_cd = plane.ahrs.pitch_sensor;

    float throtle_rate = plane.uattack.attack_throttle_rate*plane.G_Dt;
    float target_throttle = plane.uattack.attack_throttle;
    _cmd_throttle = _cmd_throttle + constrain_float(target_throttle - _cmd_throttle, -throtle_rate, throtle_rate);
    SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, plane.mode_attack_loc.get_cmd_throttle());
}

void ModeAttackLoc::navigate()
{
    switch (stage) {
        case stage_class::HOVER:
            plane.update_loiter(0);
            break;
        case stage_class::APPROACH:
            plane.next_WP_loc = target_loc;
            plane.next_WP_loc.alt = plane.current_loc.alt;
            plane.nav_controller->update_waypoint(plane.prev_WP_loc, plane.next_WP_loc);
            break;
        default:
            break;
    }
}

void ModeAttackLoc::set_stage(ModeAttackLoc::stage_class stage_in)
{
    stage = stage_in;
    switch(stage) {
        case stage_class::HOVER:
            plane.next_WP_loc = target_loc;
            plane.next_WP_loc.alt = plane.current_loc.alt;
            gcs().send_text(MAV_SEVERITY_INFO, "In Hover");
            break;
        case stage_class::ATTACK:
            gcs().send_text(MAV_SEVERITY_INFO, "In Attack");
            plane.uattack.attack_roll_pid.reset_I();
            plane.uattack.attack_roll_pid.reset_filter();
            plane.uattack.attack_roll_pid.set_integrator(0);
            break;
        default:
            break;
    }
}

float ModeAttackLoc::get_cmd_throttle() {
    return _cmd_throttle;
}
