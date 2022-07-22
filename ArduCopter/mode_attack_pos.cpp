#include "Copter.h"


/*
 * Init and run calls for althold, flight mode
 */

// althold_init - initialise althold controller
bool ModeAttack_pos::init(bool ignore_checks)
{
    // initialise position and desired velocity
    if (!copter.Utarget.Ucapture.is_active()) {return false;}
    if (!pos_control->is_active_z()) {
        pos_control->set_alt_target_to_current_alt();
        pos_control->set_desired_velocity_z(inertial_nav.get_velocity_z());
    }
    _direction = copter.Utarget.Ucapture._q_angle_cd;

    return true;
}

// althold_run - runs the althold controller
// should be called at 100hz or more
void ModeAttack_pos::run()
{
    float takeoff_climb_rate = 0.0f;

    // initialize vertical speeds and acceleration
    pos_control->set_max_speed_z(-get_pilot_speed_dn(), g.pilot_speed_up);
    pos_control->set_max_accel_z(g.pilot_accel_z);

    // apply SIMPLE mode transform to pilot inputs
    update_simple_mode();

    // get pilot desired lean angles
    float target_roll, target_pitch;
    update_attack_pos_mode(target_roll, target_pitch);

    // get pilot's desired yaw rate
    float target_yaw_rate = copter.Utarget.get_target_yaw_rate();;

    // get pilot desired climb rate
    float target_climb_rate = update_target_climb_rate();
    target_climb_rate = constrain_float(target_climb_rate, -get_pilot_speed_dn(), g.pilot_speed_up);

    // Alt Hold State Machine Determination
    AltHoldModeState althold_state = get_alt_hold_state(target_climb_rate);

    // Alt Hold State Machine
    switch (althold_state) {

    case AltHold_MotorStopped:
        attitude_control->reset_rate_controller_I_terms();
        attitude_control->set_yaw_target_to_current_heading();
        pos_control->relax_alt_hold_controllers(0.0f);   // forces throttle output to go to zero
        break;

    case AltHold_Landed_Ground_Idle:
        attitude_control->set_yaw_target_to_current_heading();
        // FALLTHROUGH

    case AltHold_Landed_Pre_Takeoff:
        attitude_control->reset_rate_controller_I_terms_smoothly();
        pos_control->relax_alt_hold_controllers(0.0f);   // forces throttle output to go to zero
        break;

    case AltHold_Takeoff:
        // initiate take-off
        if (!takeoff.running()) {
            takeoff.start(constrain_float(g.pilot_takeoff_alt,0.0f,1000.0f));
        }

        // get take-off adjusted pilot and takeoff climb rates
        takeoff.get_climb_rates(target_climb_rate, takeoff_climb_rate);

        // get avoidance adjusted climb rate
        target_climb_rate = get_avoidance_adjusted_climbrate(target_climb_rate);

        // set position controller targets
        pos_control->set_alt_target_from_climb_rate_ff(target_climb_rate, G_Dt, false);
        pos_control->add_takeoff_climb_rate(takeoff_climb_rate, G_Dt);
        break;

    case AltHold_Flying:
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

        // if (is_zero(target_yaw_rate) && copter.Utarget.is_active()) {
        //     target_yaw_rate = copter.Utarget.get_target_yaw_rate();
        // }

        // adjust climb rate using rangefinder
        target_climb_rate = copter.surface_tracking.adjust_climb_rate(target_climb_rate);

        // get avoidance adjusted climb rate
        target_climb_rate = get_avoidance_adjusted_climbrate(target_climb_rate);

        pos_control->set_alt_target_from_climb_rate_ff(target_climb_rate, G_Dt, false);
        break;
    }

    // call attitude controller
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate);

    // call z-axis position controller
    pos_control->update_z_controller();

}

void ModeAttack_pos::update_attack_pos_mode(float &roll_out, float &pitch_out)
{

    if (!copter.Utarget.is_active()) {copter.Ugcs.do_lockon();}

    _direction += copter.Utarget.Ucapture._q_cds*G_Dt;

    float simple_cos_yaw = ahrs.cos_yaw();
    float simple_sin_yaw = ahrs.sin_yaw();

    float sin_yaw = sinf(ToRad(_direction*0.01f));
    float cos_yaw = cosf(ToRad(_direction*0.01f));

    float rollx, pitchx;
    // turn to north
    rollx = 0.0f*cos_yaw - copter.g2.user_parameters.attack_pitch_angle.get()*sin_yaw;
    pitchx = 0.0f*sin_yaw + copter.g2.user_parameters.attack_pitch_angle.get()*cos_yaw;

    // turn to current yaw
    roll_out = rollx*simple_cos_yaw + pitchx*simple_sin_yaw;
    pitch_out = -rollx*simple_sin_yaw + pitchx*simple_cos_yaw;
}

float ModeAttack_pos::update_target_climb_rate() {
    float target_alt = copter.Utarget.Ucapture.target_pos.z;
    float kP = pos_control->get_pos_z_p().kP();
    float accel_cmss = pos_control->get_max_accel_z();
    float rate_max = g.pilot_speed_up.get();
    float rate_min = -get_pilot_speed_dn();
    float target_rate = 0.0f;
    if (is_zero(kP)) {
        target_rate = safe_sqrt(2.0f * (target_alt - copter.Utarget.Ucapture.current_pos.z) * accel_cmss);
    } else {
        target_rate = AC_AttitudeControl::sqrt_controller((target_alt - copter.Utarget.Ucapture.current_pos.z), kP, accel_cmss, G_Dt);
    }
    return constrain_float(target_rate, rate_min, rate_max);
}
