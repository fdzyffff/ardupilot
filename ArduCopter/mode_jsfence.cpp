#include "Copter.h"


/*
 * Init and run calls for althold, flight mode
 */

// althold_init - initialise althold controller
bool ModeJSFence::init(bool ignore_checks)
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

    if (is_disarmed_or_landed()) {
        set_stage(Stage::TAKEOFF);
    } else {
        set_stage(Stage::FENCE);
    }

    return true;
}

// althold_run - runs the althold controller
// should be called at 100hz or more
void ModeJSFence::run()
{
    update_stage();
    if (_stage == Stage::TAKEOFF) {
        copter.set_auto_armed(true);
        auto_takeoff.run();
    } else {
        fence_run();
    }
}

void ModeJSFence::fence_run()
{
    // if not armed set throttle to zero and exit immediately
    if (is_disarmed_or_landed()) {
        // do not spool down tradheli when on the ground with motor interlock enabled
        make_safe_ground_handling(copter.is_tradheli() && motors->get_interlock());
        return;
    }

    float target_yaw_rate = 0.0f;
    if (!copter.failsafe.radio) {
        // get pilot's desired yaw rate
        target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->norm_input_dz());
    }
    // get pilot desired climb rate
    // float target_climb_rate = get_pilot_desired_climb_rate(channel_throttle->get_control_in());
    // target_climb_rate = constrain_float(target_climb_rate, -get_pilot_speed_dn(), g.pilot_speed_up);

    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    _vel_target_cms.x = copter.ufence.cmd_vel_enu.x;
    _vel_target_cms.y = copter.ufence.cmd_vel_enu.y;
    _vel_target_cms.z = 0.0f;

    _accel_target_cmss = Vector3f(0.0f, 0.0f, 0.0f);

    pos_control->input_vel_accel_xy(_vel_target_cms.xy(), _accel_target_cmss.xy(), false);

    pos_control->update_xy_controller();

    // call attitude controller with auto yaw
    attitude_control->input_thrust_vector_rate_heading(pos_control->get_thrust_vector(), target_yaw_rate);

    // Send the commanded climb rate to the position controller
    pos_control->set_pos_target_z_from_climb_rate_cm(_vel_target_cms.z);
    // run the vertical position controller and set output throttle
    pos_control->update_z_controller();
}

void ModeJSFence::set_stage(Stage stage_in) {
    _stage = stage_in;
    _stage_time = millis();
    switch (_stage) {
        case Stage::TAKEOFF:
            // initialise alt for WP_NAVALT_MIN and set completion alt
            auto_takeoff.start(150, false);
            gcs().send_text(MAV_SEVERITY_INFO, "Stage Takeoff");
            break;
        case Stage::FENCE:
            gcs().send_text(MAV_SEVERITY_INFO, "Stage FENCE");
            break;
        default:
            gcs().send_text(MAV_SEVERITY_INFO, "Stage UNKNOWN");
            break;
    }
}

void ModeJSFence::update_stage()
{
    // uint32_t dt = millis() - _stage_time;
    switch (_stage) {
        case Stage::TAKEOFF:
            {
                if (auto_takeoff.complete) {
                    set_stage(Stage::FENCE);
                }
            }
            break;
        case Stage::FENCE:
            break;
        default:
            break;
    }
}

bool ModeJSFence::is_taking_off() const
{
    return ((_stage == Stage::TAKEOFF) && !auto_takeoff.complete);
}
