#include "Copter.h"


/*
 * Init and run calls for althold, flight mode
 */

// althold_init - initialise althold controller
bool ModeLudeng_hook::init(bool ignore_checks)
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

    set_stage(Stage::AUTO);

    return true;
}

// althold_run - runs the althold controller
// should be called at 100hz or more
void ModeLudeng_hook::run()
{
    update_stage();
    if (_stage == Stage::AUTO) {
        copter.mode_auto.run();
    } else {
        hook_run();
    }
}

void ModeLudeng_hook::hook_run() 
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

    Matrix3f tmp_body_m;
    Vector3f tmp_vel_input = Vector3f(copter.uk230.get_target_bf_vel_x()*100.f, copter.uk230.get_target_bf_vel_y()*100.f, 0.0f);
    tmp_body_m.from_euler(0.0f, 0.0f, copter.ahrs_view->yaw);
    _vel_target_cms = tmp_body_m*tmp_vel_input;
    _accel_target_cmss.zero();

    float target_yaw_rate = 0.0f;
    float target_climb_rate = 0.0f;

    bool use_posctrl = true;
    static bool old_use_posctrl = true;

    switch (_stage) {
        case Stage::STANDBY:
            _vel_target_cms.zero();
            target_climb_rate = 10.0f;
            // target_climb_rate = get_pilot_desired_climb_rate(channel_throttle->get_control_in());
            // target_climb_rate = constrain_float(target_climb_rate, -get_pilot_speed_dn(), g.pilot_speed_up);
            break;
        case Stage::AIM:
            target_climb_rate = 0.0f;
            target_yaw_rate = copter.uk230.get_target_yaw_rate()*100.0f;
            // target_climb_rate = get_pilot_desired_climb_rate(channel_throttle->get_control_in());
            // target_climb_rate = constrain_float(target_climb_rate, -get_pilot_speed_dn(), g.pilot_speed_up);
            break;
        case Stage::UP:
            target_climb_rate = 15.0f;
            target_yaw_rate = copter.uk230.get_target_yaw_rate()*100.0f;
            // target_climb_rate = get_pilot_desired_climb_rate(channel_throttle->get_control_in());
            // target_climb_rate = constrain_float(target_climb_rate, -get_pilot_speed_dn(), g.pilot_speed_up);
            break;
        case Stage::LOCK:
            _vel_target_cms.zero();
            target_climb_rate = 10.0f;
            target_yaw_rate = -2500.f;
            use_posctrl = false;
            // target_climb_rate = get_pilot_desired_climb_rate(channel_throttle->get_control_in());
            // target_climb_rate = constrain_float(target_climb_rate, -get_pilot_speed_dn(), g.pilot_speed_up);
            break;
        case Stage::DOWN:
            _vel_target_cms.zero();
            target_climb_rate = -20.0f;
            target_yaw_rate = 0.0f;
            break;
        case Stage::DONE:
            _vel_target_cms.zero();
            target_climb_rate = -20.0f;
            target_yaw_rate = 0.0f;
            break;
        default:
            _vel_target_cms.zero();
            target_climb_rate = 0.0f;
            target_yaw_rate = 0.0f;
            break;
    }

    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    if (!old_use_posctrl && use_posctrl) {
        pos_control->init_xy_controller();
        gcs().send_text(MAV_SEVERITY_INFO, "Init POSCTRL");
    }
    old_use_posctrl = use_posctrl;
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

void ModeLudeng_hook::update_stage()
{
    uint32_t dt = millis() - _stage_time;
    switch (_stage) {
        case Stage::AUTO:
            {
                if (copter.uk230.is_valid()) {
                    set_stage(Stage::STANDBY);
                }
            }
            break;
        case Stage::STANDBY:
            if (copter.uk230.is_valid()) {
                set_stage(Stage::AIM);
            }
            break;
        case Stage::AIM:
            if (!copter.uk230.is_valid()) {
                set_stage(Stage::STANDBY);
            }
            if (dt > 5000) {
                set_stage(Stage::UP);
            }
            break;
        case Stage::UP:
            if (!copter.uk230.is_valid()) {
                set_stage(Stage::STANDBY);
            }
            if (check_touch()) {
                set_stage(Stage::LOCK);
            }
            break;
        case Stage::LOCK:
            if (dt > 3000) {
                set_stage(Stage::DOWN);
            }
            break;
        case Stage::DOWN:
            if (check_done()) {
                set_stage(Stage::DONE);
            } else {
                if (dt > 6000) {
                    set_stage(Stage::STANDBY);
                }
            }
            break;
        case Stage::DONE:
            break;
        default:
            set_stage(Stage::STANDBY);
            break;
    }
}

bool ModeLudeng_hook::check_touch() 
{
    bool ret = false;
    static uint32_t time_ms = millis();
    uint32_t dt = millis() - _stage_time;
    bool rngfnd_ok = (!copter.rangefinder_alt_ok() || (copter.rangefinder_alt_ok() && copter.rangefinder_state.alt_cm_filt.get() > 120.f));
    bool vel_up_ok = copter.inertial_nav.get_velocity_z_up_cms() < 10.f;
    bool thr_ok = (motors->get_throttle() > MIN(motors->get_throttle_hover()*1.5f, motors->get_throttle_hover()+0.15f));
    if ((vel_up_ok||thr_ok) && rngfnd_ok && (dt > 100)) {
        if ((millis() - time_ms > 1500 ) && (dt > 2000)) {
            ret = true;
        }
    } else {
        time_ms = millis();
    }
    return ret;
}

bool ModeLudeng_hook::check_done() 
{
    bool ret = false;
    static uint32_t time_ms = millis();
    uint32_t dt = millis() - _stage_time;
    bool rngfnd_ok = (!copter.rangefinder_alt_ok() || (copter.rangefinder_alt_ok() && copter.rangefinder_state.alt_cm_filt.get() > 120.f));
    bool vel_down_ok = copter.inertial_nav.get_velocity_z_up_cms() > -10.f;
    if (vel_down_ok && rngfnd_ok && (dt > 100)) {
        if ((millis() - time_ms > 3000 ) && (dt > 3000)) {
            ret = true;
        }
    } else {
        time_ms = millis();
    }
    return ret;
}


bool ModeLudeng_hook::is_taking_off() const
{
    return ((_stage == Stage::AUTO) && copter.mode_auto.is_taking_off());
}

void ModeLudeng_hook::set_stage(Stage stage_in) {
    _stage = stage_in;
    _stage_time = millis();
    switch (_stage) {
        case Stage::AUTO:
            if (copter.mode_auto.init(false)) {
                copter.mode_auto.mission.reset();
                gcs().send_text(MAV_SEVERITY_INFO, "Stage AUTO");
            } else {
                gcs().send_text(MAV_SEVERITY_INFO, "NO AUTO!");
                set_stage(Stage::STANDBY);
            }
            break;
        case Stage::STANDBY:
            gcs().send_text(MAV_SEVERITY_INFO, "Stage STANDBY");
            break;
        case Stage::AIM:
            gcs().send_text(MAV_SEVERITY_INFO, "Stage AIM");
            break;
        case Stage::UP:
            gcs().send_text(MAV_SEVERITY_INFO, "Stage UP");
            break;
        case Stage::LOCK:
            gcs().send_text(MAV_SEVERITY_INFO, "Stage LOCK");
            break;
        case Stage::DOWN:
            gcs().send_text(MAV_SEVERITY_INFO, "Stage DOWN");
            break;
        case Stage::DONE:
            gcs().send_text(MAV_SEVERITY_INFO, "Stage DONE");
            break;
        default:
            gcs().send_text(MAV_SEVERITY_INFO, "Stage UNKNOWN");
            break;
    }
}
