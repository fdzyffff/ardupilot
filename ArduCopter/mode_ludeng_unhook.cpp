#include "Copter.h"


/*
 * Init and run calls for althold, flight mode
 */

// althold_init - initialise althold controller
bool ModeLudeng_unhook::init(bool ignore_checks)
{
    // initialise horizontal speed, acceleration
    pos_control->set_max_speed_accel_xy(wp_nav->get_default_speed_xy(), wp_nav->get_wp_acceleration());
    pos_control->set_correction_speed_accel_xy(wp_nav->get_default_speed_xy(), wp_nav->get_wp_acceleration());

    // initialize vertical speeds and acceleration
    pos_control->set_max_speed_accel_z(wp_nav->get_default_speed_down(), wp_nav->get_default_speed_up(), wp_nav->get_accel_z());
    pos_control->set_correction_speed_accel_z(wp_nav->get_default_speed_down(), wp_nav->get_default_speed_up(), wp_nav->get_accel_z());

    // initialise velocity controller
    pos_control->init_xy_controller();

    // initialise the vertical position controller
    if (!pos_control->is_active_z()) {
        pos_control->init_z_controller();
    }

    set_stage(Stage::UP);

    return true;
}

// althold_run - runs the althold controller
// should be called at 100hz or more
void ModeLudeng_unhook::run()
{
    update_stage();
    if (_stage == Stage::AWAY) {
        copter.mode_guided.run();
    } else {
        unhook_run();
    }
}

void ModeLudeng_unhook::unhook_run() 
{
    // if not armed set throttle to zero and exit immediately
    if (is_disarmed_or_landed()) {
        // do not spool down tradheli when on the ground with motor interlock enabled
        make_safe_ground_handling(copter.is_tradheli() && motors->get_interlock());
        return;
    }

    // get pilot's desired yaw rate
    // float target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->norm_input_dz());
    // get pilot desired climb rate
    // float target_climb_rate = get_pilot_desired_climb_rate(channel_throttle->get_control_in());
    // target_climb_rate = constrain_float(target_climb_rate, -get_pilot_speed_dn(), g.pilot_speed_up);

    _vel_target_cms.zero();
    _accel_target_cmss.zero();

    float target_yaw_rate = 0.0f;
    float target_climb_rate = 0.0f;

    switch (_stage) {
        case Stage::UP:
            _vel_target_cms.zero();
            target_yaw_rate = 0.f;
            target_climb_rate = 10.0f;
            // target_climb_rate = get_pilot_desired_climb_rate(channel_throttle->get_control_in());
            // target_climb_rate = constrain_float(target_climb_rate, -get_pilot_speed_dn(), g.pilot_speed_up);
            break;
        case Stage::UNLOCK:
            _vel_target_cms.zero();
            target_yaw_rate = 2500.f;
            target_climb_rate = 0.0f;
            // target_climb_rate = get_pilot_desired_climb_rate(channel_throttle->get_control_in());
            // target_climb_rate = constrain_float(target_climb_rate, -get_pilot_speed_dn(), g.pilot_speed_up);
            break;
        case Stage::DOWN:
            _vel_target_cms.zero();
            target_yaw_rate = 0.f;
            target_climb_rate = -25.0f;
            // target_climb_rate = get_pilot_desired_climb_rate(channel_throttle->get_control_in());
            // target_climb_rate = constrain_float(target_climb_rate, -get_pilot_speed_dn(), g.pilot_speed_up);
            break;
        case Stage::LAND:
            _vel_target_cms.zero();
            target_yaw_rate = 0.f;
            target_climb_rate = -25.0f;
            // target_climb_rate = get_pilot_desired_climb_rate(channel_throttle->get_control_in());
            // target_climb_rate = constrain_float(target_climb_rate, -get_pilot_speed_dn(), g.pilot_speed_up);
            break;
        default:
            _vel_target_cms.zero();
            target_climb_rate = 0.0f;
            target_yaw_rate = 0.0f;
            break;
    }

    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    pos_control->input_vel_accel_xy(_vel_target_cms.xy(), _accel_target_cmss.xy(), false);

    // Send the commanded climb rate to the position controller
    pos_control->set_pos_target_z_from_climb_rate_cm(target_climb_rate);

    // call velocity controller which includes z axis controller
    pos_control->update_xy_controller();
    pos_control->update_z_controller();

    // call attitude controller with auto yaw
    attitude_control->input_thrust_vector_rate_heading(pos_control->get_thrust_vector(), target_yaw_rate);
}

void ModeLudeng_unhook::update_stage()
{
    uint32_t dt = millis() - _stage_time;
    switch (_stage) {
        case Stage::UP:
            if ((dt > 8000) || motors->get_throttle() > 0.7f) {
                set_stage(Stage::UNLOCK);
            }
            break;
        case Stage::UNLOCK:
            if (dt > 3000) {
                set_stage(Stage::DOWN);
            }
            break;
        case Stage::DOWN:
            if (check_down()) {
                set_stage(Stage::AWAY);
            }
            if (dt > 4000) {
                set_stage(Stage::UP);
            }
            break;
        case Stage::AWAY:
            if (copter.mode_guided.wp_distance() < 100) {
                set_stage(Stage::LAND);
            }
            break;
        case Stage::LAND:
            break;
        default:
            set_stage(Stage::UP);
            break;
    }
}

bool ModeLudeng_unhook::check_down() 
{
    bool ret = false;
    static uint32_t time_ms = millis();
    uint32_t dt = millis() - _stage_time;
    bool rngfnd_ok = (!copter.rangefinder_alt_ok() || (copter.rangefinder_alt_ok() && copter.rangefinder_state.alt_cm_filt.get() < 80.f));
    bool vel_up_ok = copter.inertial_nav.get_velocity_z_up_cms() < -15.f;
    if (vel_up_ok && rngfnd_ok) {
        if ((millis() - time_ms > 2000 ) && (dt > 2000)) {
            ret = true;
        }
    } else {
        time_ms = millis();
    }
    return ret;
}

bool ModeLudeng_unhook::away_init()
{
    // bool loc_A_OK = (lat_A != 0 && lng_A !=0);
    int32_t lat_A = copter.g2.user_parameters.loc_A_lat.get()*1e7;
    int32_t lng_A = copter.g2.user_parameters.loc_A_lng.get()*1e7;
    int32_t alt_A = copter.g2.user_parameters.loc_A_alt.get();
    bool loc_A_OK = (lat_A != 0 && lng_A !=0);
    if (!loc_A_OK) {
        return false;
    }
    Location loc = Location(lat_A, lng_A, alt_A, Location::AltFrame::ABOVE_HOME);
    bool use_yaw = true;
    float yaw_cd = copter.g2.user_parameters.loc_A_yaw.get()*100.f;
    bool use_yaw_rate = false;
    float yaw_rate_cds = 0.0;
    if (copter.mode_guided.set_destination(loc, use_yaw, yaw_cd, use_yaw_rate, yaw_rate_cds)) {
        return true;
    }
    return false;
}

void ModeLudeng_unhook::set_stage(Stage stage_in) {
    _stage = stage_in;
    _stage_time = millis();
    switch (_stage) {
        case Stage::UP:
            gcs().send_text(MAV_SEVERITY_INFO, "Stage UP");
            copter.set_auto_armed(true);
            set_land_complete(false);
            break;
        case Stage::UNLOCK:
            gcs().send_text(MAV_SEVERITY_INFO, "Stage UNLOCK");
            break;
        case Stage::DOWN:
            gcs().send_text(MAV_SEVERITY_INFO, "Stage DOWN");
            break;
        case Stage::AWAY:
            if (away_init()) {
                gcs().send_text(MAV_SEVERITY_INFO, "Stage AWAY");
            } else {
                gcs().send_text(MAV_SEVERITY_INFO, "NO LOC!");
                set_stage(Stage::LAND);
            }
            break;
        case Stage::LAND:
            gcs().send_text(MAV_SEVERITY_INFO, "Stage LAND");
            break;
        default:
            gcs().send_text(MAV_SEVERITY_INFO, "Stage UNKNOWN");
            break;
    }
}
