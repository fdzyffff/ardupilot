#include "Copter.h"


/*
 * Init and run calls for althold, flight mode
 */

// althold_init - initialise althold controller
bool ModeAttack::init(bool ignore_checks)
{
    if (copter.Ucam.is_active()) {
        // initialise position and desired velocity
        if (!pos_control->is_active_z()) {
            pos_control->set_alt_target_to_current_alt();
            pos_control->set_desired_velocity_z(inertial_nav.get_velocity_z());
        }
        return true;
    }
    return false;
}

// althold_run - runs the althold controller
// should be called at 100hz or more
void ModeAttack::run()
{
    // initialize vertical speeds and acceleration
    pos_control->set_max_speed_z(-get_pilot_speed_dn(), g.pilot_speed_up);
    pos_control->set_max_accel_z(g.pilot_accel_z);

    // apply SIMPLE mode transform to pilot inputs
    update_simple_mode();

    // get target lean angles
    float target_roll, target_pitch;

    my_get_target_angles(target_roll, target_pitch);

    // get target yaw rate
    float target_yaw_rate = my_get_target_yaw_rate();

    // get target climb rate
    float target_climb_rate = my_get_target_climb_rate();

    target_climb_rate = constrain_float(target_climb_rate, -get_pilot_speed_dn(), g.pilot_speed_up);

    // Alt Hold State Machine Determination
    AltHoldModeState althold_state = get_alt_hold_state(target_climb_rate);

    // Alt Hold State Machine
    switch (althold_state) {
    case AltHold_Flying:
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

        // adjust climb rate using rangefinder
        target_climb_rate = copter.surface_tracking.adjust_climb_rate(target_climb_rate);

        // get avoidance adjusted climb rate
        target_climb_rate = get_avoidance_adjusted_climbrate(target_climb_rate);

        pos_control->set_alt_target_from_climb_rate_ff(target_climb_rate, G_Dt, false);
        break;
    default:
        copter.set_mode(Mode::Number::STABILIZE, ModeReason::UNKNOWN);
    }

    // call attitude controller
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate);

    // call z-axis position controller
    pos_control->update_z_controller();

}

void ModeAttack::my_get_target_angles(float &target_roll, float &target_pitch)
{
    float pitch_target = (180.0f/M_PI) * atanf(copter.Ucam.get_raw_info().y/(0.5f*copter.g2.user_parameters.cam_pixel_y)*tanf(0.5f*copter.g2.user_parameters.cam_angle_y*(M_PI/180.f)));
    float pitch_limit = copter.g2.user_parameters.cam_angle_y*0.35f;
    float delta_pitch = constrain_float(pitch_target - pitch_limit, -copter.g2.user_parameters.cam_angle_y*0.5f, 5.0f)*copter.g2.user_parameters.fly_pitch_factor;
    target_pitch = (degrees(copter.ahrs_view->pitch) + delta_pitch)*100.f;
    float angle_limit = constrain_float(copter.aparm.angle_max, 1000.0f, attitude_control->get_althold_lean_angle_max());
    angle_limit = MIN(angle_limit, copter.g2.user_parameters.fly_pitch_limit);
    target_pitch = constrain_float(target_pitch,-angle_limit,angle_limit);

    target_roll = 0.0f;
    //my_get_pilot_desired_lean_angles(target_roll, target_pitch, copter.aparm.angle_max, attitude_control->get_althold_lean_angle_max());
}

// get_pilot_desired_angle - transform pilot's roll or pitch input into a desired lean angle
// returns desired angle in centi-degrees
void ModeAttack::my_get_pilot_desired_lean_angles(float &roll_out, float &pitch_out, float angle_max, float angle_limit) const
{
    // limit max lean angle
    angle_limit = constrain_float(angle_limit, 1000.0f, angle_max);

    // scale roll and pitch inputs to ANGLE_MAX parameter range
    float scaler = angle_max/(float)ROLL_PITCH_YAW_INPUT_MAX;
    roll_out *= scaler;
    pitch_out *= scaler;

    // do circular limit
    float total_in = norm(pitch_out, roll_out);
    if (total_in > angle_limit) {
        float ratio = angle_limit / total_in;
        roll_out *= ratio;
        pitch_out *= ratio;
    }

    // do lateral tilt to euler roll conversion
    roll_out = (18000/M_PI) * atanf(cosf(pitch_out*(M_PI/18000))*tanf(roll_out*(M_PI/18000)));

    // roll_out and pitch_out are returned
}

float ModeAttack::my_get_target_yaw_rate() {
    float info_x = copter.Ucam.get_raw_info().x;
    float x_length = copter.g2.user_parameters.cam_pixel_x;
    float x_angle = copter.g2.user_parameters.cam_angle_x;
    float yaw_rate_tc = copter.g2.user_parameters.fly_yaw_tc;
    float yaw_rate_cds = 100.f * (x_angle * info_x / x_length / yaw_rate_tc);
    return yaw_rate_cds;
}

float ModeAttack::my_get_target_climb_rate() {
    float climb_rate_factor = (copter.Ucam.get_raw_info().y - 0.25f*copter.g2.user_parameters.cam_pixel_y)/(0.5f*copter.g2.user_parameters.cam_pixel_y);
    climb_rate_factor *= copter.g2.user_parameters.fly_climb_factor; // -1.5f ~ 0.5f
    float pitch_scalar = copter.g2.user_parameters.fly_pitch_scalar*constrain_float(fabsf(attitude_control->get_att_target_euler_cd().y/copter.aparm.angle_max), 0.0f, 1.0f);

    float final_climb_rate = climb_rate_factor * get_pilot_speed_dn() * pitch_scalar;
    if (final_climb_rate > 0.33f*g.pilot_speed_up) {final_climb_rate = 0.33f*g.pilot_speed_up;}
    return final_climb_rate;
}
