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
        copter.mode_auto.run();
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
    float target_roll = 0.0f;
    float target_pitch = 0.0f;

    bool use_posctrl = true;
    static bool old_use_posctrl = true;

    switch (_stage) {
        case Stage::UP:
            _vel_target_cms.zero();
            target_yaw_rate = 0.f;
            target_climb_rate = 10.0f;
            use_posctrl = false;
            // target_climb_rate = get_pilot_desired_climb_rate(channel_throttle->get_control_in());
            // target_climb_rate = constrain_float(target_climb_rate, -get_pilot_speed_dn(), g.pilot_speed_up);
            break;
        case Stage::UNLOCK:
            _vel_target_cms.zero();
            target_yaw_rate = -2500.f;
            target_climb_rate = 10.0f;
            use_posctrl = false;
            // target_climb_rate = get_pilot_desired_climb_rate(channel_throttle->get_control_in());
            // target_climb_rate = constrain_float(target_climb_rate, -get_pilot_speed_dn(), g.pilot_speed_up);
            break;
        case Stage::DOWN:
            _vel_target_cms.zero();
            target_yaw_rate = 0.f;
            target_climb_rate = -25.0f;
            copter.user_update_assit(target_roll, target_pitch);
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

    if (!old_use_posctrl && use_posctrl) {
        pos_control->init_xy_controller();
        gcs().send_text(MAV_SEVERITY_INFO, "Init POSCTRL");
    }
    old_use_posctrl = use_posctrl;

    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    if (use_posctrl) {
        pos_control->input_vel_accel_xy(_vel_target_cms.xy(), _accel_target_cmss.xy(), false);

        pos_control->update_xy_controller();

        // call attitude controller with auto yaw
        attitude_control->input_thrust_vector_rate_heading(pos_control->get_thrust_vector(), target_yaw_rate);
    } else {
        // call attitude controller
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(0.0f, 0.0f, target_yaw_rate);
    }
    // Send the commanded climb rate to the position controller
    pos_control->set_pos_target_z_from_climb_rate_cm(target_climb_rate);
    // run the vertical position controller and set output throttle
    pos_control->update_z_controller();
}

void ModeLudeng_unhook::update_stage()
{
    uint32_t dt = millis() - _stage_time;
    switch (_stage) {
        case Stage::UP:
            if ((dt > 5000) || ((dt > 3000) && (motors->get_throttle() > MIN(motors->get_throttle_hover()*1.5f, motors->get_throttle_hover()+0.15f)))) {
                set_stage(Stage::UNLOCK);
            }
            break;
        case Stage::UNLOCK:
            if (dt > 5000) {
                set_stage(Stage::DOWN);
            }
            break;
        case Stage::DOWN:
            if (check_down()) {
                set_home_to_current_alt();
                set_stage(Stage::AWAY);
            }
            if (dt > 10000) {
                set_stage(Stage::UP);
            }
            break;
        case Stage::AWAY:
            if (copter.mode_auto.mission.state() == AP_Mission::mission_state::MISSION_COMPLETE) {
                if (copter.uk230.is_valid()) {
                    set_mode(Mode::Number::LDHOOK, ModeReason::AUTO_HOOK);
                } else {
                    set_stage(Stage::LAND);
                }
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
    bool vel_dn_ok = copter.inertial_nav.get_velocity_z_up_cms() < -15.f;
    if (vel_dn_ok && rngfnd_ok && (dt > 100)) {
        if ((millis() - time_ms > 1500 ) && (dt > 2000)) {
            ret = true;
        }
    } else {
        time_ms = millis();
    }
    return ret;
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
            if (copter.mode_auto.init(false) && copter.mode_auto.mission.set_current_cmd(copter.g2.user_parameters.hook_mission_idx.get())) {
                gcs().send_text(MAV_SEVERITY_INFO, "Stage AWAY");
            } else {
                gcs().send_text(MAV_SEVERITY_INFO, "No Mission");
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

// set_home_to_current_alt - set home to current vertically
void ModeLudeng_unhook::set_home_to_current_alt() {
    // get current location from EKF
    Location temp_loc;
    if (copter.ahrs.get_location(temp_loc)) {
        temp_loc.lat = copter.ahrs.get_home().lat;
        temp_loc.lng = copter.ahrs.get_home().lng;
        temp_loc.alt -= copter.g2.user_parameters.hook_mission_alt.get();
        if (!copter.set_home(temp_loc, false)) {
            return;
        }
        // we have successfully set AHRS home, set it for SmartRTL
#if MODE_SMARTRTL_ENABLED == ENABLED
        copter.g2.smart_rtl.set_home(true);
#endif
    }
}

