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

    set_stage(Stage::STANDBY);

    return true;
}

// althold_run - runs the althold controller
// should be called at 100hz or more
void ModeLudeng_hook::run()
{
    update_stage();

    hook_run();
}

void ModeLudeng_hook::exit()
{
    copter.ua8.set_gimbal_front();
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


    _accel_target_cmss.zero();
    _vel_target_cms.zero();

    float target_roll = 0.0f;
    float target_pitch = 0.0f;
    float target_yaw_rate = 0.0f; //cd/s
    float target_climb_rate = 0.0f; //cm/s

    bool use_posctrl = true;
    static bool old_use_posctrl = true;

    switch (_stage) {
        case Stage::SEARCH1:
            _vel_target_cms.zero();
            _vel_target_cms.z = 0.0f;
            target_yaw_rate = 0.0f;
            break;
        case Stage::SEARCH2:
            _vel_target_cms.zero();
            _vel_target_cms.z = get_surface_vel();
            target_yaw_rate = 1500.0f;
            break;
        case Stage::APPROACH:
        {
            _vel_target_cms.zero();
            set_approach_vel();
            target_climb_rate = copter.ua8.get_front_vel_z()*100.0f;
            target_yaw_rate = copter.ua8.get_front_yaw_rate()*100.0f;
            break;
        }
        case Stage::STANDBY:
        {
            _vel_target_cms.zero();
            target_yaw_rate = 0.0f;
            target_climb_rate = 0.0f;//get_surface_vel();
            break;
        }
        case Stage::AIM:
            target_climb_rate = 0.0f;
            target_yaw_rate = copter.ua8.get_up_yaw_rate()*100.0f;
            set_hook_vel();
            break;
        case Stage::UP1:
            target_climb_rate = 15.0f;
            target_yaw_rate = copter.ua8.get_up_yaw_rate()*100.0f;
            set_hook_vel();
            break;
        case Stage::UP2:
            target_climb_rate = 0.0f;
            target_yaw_rate = copter.ua8.get_up_yaw_rate()*100.0f;
            set_hook_vel();
            break;
        case Stage::UP3:
            target_climb_rate = 15.0f;
            target_yaw_rate = copter.ua8.get_up_yaw_rate()*100.0f;
            set_hook_vel();
            break;
        case Stage::LOCK:
            _vel_target_cms.zero();
            target_climb_rate = 10.0f;
            target_yaw_rate = 2500.f;
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

    float rc_roll = 0.0f;
    float rc_pitch = 0.0f;
    float rc_yaw_rate = 0.0f;
    float rc_climb_rate = 0.0f;

    bool have_rc = false;
    static bool old_have_rc = false;
    if (!copter.failsafe.radio) {
        // convert pilot input to lean angles
        get_pilot_desired_lean_angles(rc_roll, rc_pitch, loiter_nav->get_angle_max_cd(), attitude_control->get_althold_lean_angle_max_cd());

        // get pilot's desired yaw rate
        rc_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->norm_input_dz());

        // get pilot desired climb rate
        rc_climb_rate = get_pilot_desired_climb_rate(channel_throttle->get_control_in());
        rc_climb_rate = constrain_float(rc_climb_rate, -get_pilot_speed_dn(), g.pilot_speed_up);

        have_rc = (!is_zero(rc_roll) || !is_zero(rc_pitch) || !is_zero(rc_yaw_rate) || !is_zero(rc_climb_rate));
    }

    if (have_rc) {
        use_posctrl = false;
        target_roll = rc_roll;
        target_pitch = rc_pitch;
        target_yaw_rate = rc_yaw_rate;
        target_climb_rate = rc_climb_rate;
        if (!old_have_rc) {
            gcs().send_text(MAV_SEVERITY_INFO, "Init RC, %f, %f, %f, %f",rc_roll, rc_pitch, rc_yaw_rate, rc_climb_rate);
        }
    }
    old_have_rc = have_rc;

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
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate);
    }
    // Send the commanded climb rate to the position controller
    pos_control->set_pos_target_z_from_climb_rate_cm(target_climb_rate);
    // run the vertical position controller and set output throttle
    pos_control->update_z_controller();
}

void ModeLudeng_hook::update_stage()
{
    float dt = (float)(millis() - _stage_time) * 0.001f;
    switch (_stage) {
        case Stage::STANDBY:
            copter.ua8.set_gimbal_up();
            if (copter.ua8.have_target_up()) {
                set_stage(Stage::AIM);
            }
            if (dt > 3.0f) {
                set_stage(Stage::SEARCH1);
            }
            break;
        case Stage::SEARCH1:
            {
                copter.ua8.set_gimbal_front();
                if (dt > 3.0f) {
                    set_stage(Stage::SEARCH2);
                }
            }
            break;
        case Stage::SEARCH2:
            {
                copter.ua8.set_gimbal_front();
                if (copter.ua8.have_target_front()) {
                    set_stage(Stage::APPROACH);
                }
                if (dt > 30.0f) {
                    set_stage(Stage::FAIL);
                }
            }
            break;
        case Stage::APPROACH:
            {
                copter.ua8.set_gimbal_front();
                if (copter.ua8.get_front_vel_xy().length() < 0.1f) {
                    set_stage(Stage::STANDBY);
                }
            }
            break;
        case Stage::AIM:
            copter.ua8.set_gimbal_up();
            if (!copter.ua8.have_target_up()) {
                set_stage(Stage::STANDBY);
            }
            if (dt > 5.0f) {
                set_stage(Stage::UP1);
            }
            break;
        case Stage::UP1:
            copter.ua8.set_gimbal_up();
            if (!copter.ua8.have_target_up()) {
                set_stage(Stage::STANDBY);
            }
            if (copter.ua8.get_up_dist_cm() < 60.f) {
                set_stage(Stage::UP2);
            }
            break;
        case Stage::UP2:
            if (!copter.ua8.have_target_up()) {
                set_stage(Stage::STANDBY);
            }
            if (check_vel_small()) {
                set_stage(Stage::UP3);
            }
            break;
        case Stage::UP3:
            copter.ua8.set_gimbal_up();
            if (!copter.ua8.have_target_up()) {
                set_stage(Stage::STANDBY);
            }
            if (check_touch()) {
                set_stage(Stage::LOCK);
            }
            break;
        case Stage::LOCK:
            copter.ua8.set_gimbal_up();
            if (dt > 2.5f) {
                set_stage(Stage::DOWN);
            }
            break;
        case Stage::DOWN:
            copter.ua8.set_gimbal_up();
            if (check_done()) {
                set_stage(Stage::DONE);
            } else {
                if (dt > 5.0f) {
                    set_stage(Stage::STANDBY);
                }
            }
            break;
        case Stage::DONE:
            break;
        case Stage::FAIL:
            break;
        default:
            set_stage(Stage::STANDBY);
            break;
    }
}

bool ModeLudeng_hook::finished()
{
    return (_stage == Stage::DONE || _stage == Stage::FAIL);
}

void ModeLudeng_hook::set_approach_vel()
{
    Matrix3f tmp_body_m;
    Vector3f tmp_vel_input = Vector3f(copter.ua8.get_front_vel_x() * 100.f, copter.ua8.get_front_vel_y() * 100.f, 0.0f);
    tmp_body_m.from_euler(0.0f, 0.0f, copter.ahrs_view->yaw);
    _vel_target_cms = tmp_body_m*tmp_vel_input;
}

void ModeLudeng_hook::set_hook_vel()
{
    Matrix3f tmp_body_m;
    Vector3f tmp_vel_input = Vector3f(copter.ua8.get_up_bf_vel_x() * 100.f, copter.ua8.get_up_bf_vel_y()*100.f, 0.0f);
    tmp_body_m.from_euler(0.0f, 0.0f, copter.ahrs_view->yaw);
    _vel_target_cms = tmp_body_m*tmp_vel_input;
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

bool ModeLudeng_hook::check_vel_small() 
{
    bool ret = false;
    static uint32_t time_ms = millis();
    uint32_t dt = millis() - _stage_time;
    bool vel_ok = (_vel_target_cms.xy().length() < 10.0f);
    if (vel_ok && (dt > 100)) {
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

float ModeLudeng_hook::get_surface_vel()
{
    bool rngfnd_ok = (!copter.rangefinder_alt_ok() || (copter.rangefinder_alt_ok() && copter.rangefinder_state.alt_cm_filt.get() > 20.f));
    if (!rngfnd_ok) {
        return 0.0f;
    }
    float current_rng_alt = copter.rangefinder_state.alt_cm_filt.get();
    float K_P = copter.pos_control->get_pos_z_p().kP();
    float accel_cmss = copter.pos_control->get_max_accel_z_cmss();
    float rate_min = -50.f;
    float alt_min = 120.f;
    if (is_zero(K_P)) {
        rate_min = MAX(rate_min, safe_sqrt(2.0f * (alt_min - current_rng_alt) * accel_cmss));
    } else {
        rate_min = MAX(rate_min, sqrt_controller((alt_min - current_rng_alt), K_P, accel_cmss, copter.G_Dt));
    }
    return rate_min;
}

void ModeLudeng_hook::set_stage(Stage stage_in) {
    _stage = stage_in;
    _stage_time = millis();
    switch (_stage) {
        case Stage::STANDBY:
            gcs().send_text(MAV_SEVERITY_INFO, "Hook STANDBY");
            break;
        case Stage::SEARCH1:
            gcs().send_text(MAV_SEVERITY_INFO, "Hook SEARCH1");
            break;
        case Stage::SEARCH2:
            gcs().send_text(MAV_SEVERITY_INFO, "Hook SEARCH2");
            break;
        case Stage::APPROACH:
            gcs().send_text(MAV_SEVERITY_INFO, "Hook APPROACH");
            break;
        case Stage::AIM:
            gcs().send_text(MAV_SEVERITY_INFO, "Hook AIM");
            break;
        case Stage::UP1:
            gcs().send_text(MAV_SEVERITY_INFO, "Hook UP1");
            break;
        case Stage::UP2:
            gcs().send_text(MAV_SEVERITY_INFO, "Hook UP2");
            break;
        case Stage::UP3:
            gcs().send_text(MAV_SEVERITY_INFO, "Hook UP3");
            break;
        case Stage::LOCK:
            gcs().send_text(MAV_SEVERITY_INFO, "Hook LOCK");
            break;
        case Stage::DOWN:
            gcs().send_text(MAV_SEVERITY_INFO, "Hook DOWN");
            break;
        case Stage::DONE:
            gcs().send_text(MAV_SEVERITY_INFO, "Hook DONE");
            break;
        case Stage::FAIL:
            gcs().send_text(MAV_SEVERITY_INFO, "Hook FAIL");
            break;
        default:
            gcs().send_text(MAV_SEVERITY_INFO, "Hook UNKNOWN");
            break;
    }
}
