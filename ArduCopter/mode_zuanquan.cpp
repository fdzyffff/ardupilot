#include "Copter.h"


/*
 * Init and run calls for althold, flight mode
 */

// zqcc_init - initialise althold controller
bool Copter::ModeZQCC::init(bool ignore_checks)
{
    // initialise position and desired velocity
    if (!pos_control->is_active_z()) {
        pos_control->set_alt_target_to_current_alt();
        pos_control->set_desired_velocity_z(inertial_nav.get_velocity_z());
    }
    infoZQCC.reset_lean();
    return infoZQCC.running();
}

// zqcc_run - runs the althold controller
// should be called at 100hz or more
void Copter::ModeZQCC::run()
{
    if (!infoZQCC.running()) {
        gcs().send_text(MAV_SEVERITY_CRITICAL,"ZQCC err 0");
        set_mode(LAND, MODE_REASON_UNKNOWN);
        return;
    }
    ZQCCModeState zqcc_state;
    float takeoff_climb_rate = 0.0f;

    // initialize vertical speeds and acceleration
    pos_control->set_speed_z(-get_pilot_speed_dn(), g.pilot_speed_up);
    pos_control->set_accel_z(g.pilot_accel_z);

    // apply SIMPLE mode transform to pilot inputs
    update_simple_mode();

    // get pilot desired lean angles
    float target_roll, target_pitch;
    get_pilot_desired_lean_angles(target_roll, target_pitch, copter.aparm.angle_max, attitude_control->get_althold_lean_angle_max());

    // get pilot's desired yaw rate
    float target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());

    // get pilot desired climb rate
    float target_climb_rate = get_pilot_desired_climb_rate(channel_throttle->get_control_in());
    target_climb_rate = constrain_float(target_climb_rate, -get_pilot_speed_dn(), g.pilot_speed_up);

    // Alt Hold State Machine Determination
    if (!motors->armed() || !motors->get_interlock()) {
        zqcc_state = ZQCC_MotorStopped;
    } else if (takeoff.running() || takeoff.triggered(target_climb_rate)) {
        zqcc_state = ZQCC_Takeoff;
    } else if (!ap.auto_armed || ap.land_complete) {
        zqcc_state = ZQCC_Landed;
    } else {
        zqcc_state = ZQCC_Flying;
    }

    // Alt Hold State Machine
    switch (zqcc_state) {

    case ZQCC_MotorStopped:

        motors->set_desired_spool_state(AP_Motors::DESIRED_SHUT_DOWN);
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate);
        attitude_control->reset_rate_controller_I_terms();
        attitude_control->set_yaw_target_to_current_heading();
#if FRAME_CONFIG == HELI_FRAME    
        // force descent rate and call position controller
        pos_control->set_alt_target_from_climb_rate(-abs(g.land_speed), G_Dt, false);
        heli_flags.init_targets_on_arming=true;
        if (ap.land_complete_maybe) {
            pos_control->relax_alt_hold_controllers(0.0f);
        }
#else
        pos_control->relax_alt_hold_controllers(0.0f);   // forces throttle output to go to zero
#endif
        pos_control->update_z_controller();
        break;

    case ZQCC_Takeoff:
        if ( !(is_zero(target_roll) && is_zero(target_roll) && is_zero(target_yaw_rate) ) ) {
            gcs().send_text(MAV_SEVERITY_CRITICAL,"ZQCC exit, err 01");
            set_mode(LAND, MODE_REASON_UNKNOWN);
            return;            
        }
        if (!infoZQCC.adjust_roll_pitch_yaw(target_roll, target_pitch, attitude_control->get_althold_lean_angle_max(), target_yaw_rate)) {
            gcs().send_text(MAV_SEVERITY_CRITICAL,"ZQCC exit, err 02");
            set_mode(LAND, MODE_REASON_UNKNOWN);
            return;              
        }
        target_roll = 0.0f;
        target_pitch = 0.0f;
#if FRAME_CONFIG == HELI_FRAME    
        if (heli_flags.init_targets_on_arming) {
            heli_flags.init_targets_on_arming=false;
        }
#endif
        // set motors to full range
        motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

        // initiate take-off
        if (!takeoff.running()) {
            gcs().send_text(MAV_SEVERITY_CRITICAL,"ZQCC tak start running");
            takeoff.start(constrain_float(g.pilot_takeoff_alt,0.0f,1000.0f));
            // indicate we are taking off
            set_land_complete(false);
            // clear i terms
            set_throttle_takeoff();
        }

        // get take-off adjusted pilot and takeoff climb rates
        takeoff.get_climb_rates(target_climb_rate, takeoff_climb_rate);

        // get avoidance adjusted climb rate
        target_climb_rate = get_avoidance_adjusted_climbrate(target_climb_rate);

        // call attitude controller
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate);

        // call position controller
        pos_control->set_alt_target_from_climb_rate_ff(target_climb_rate, G_Dt, false);
        pos_control->add_takeoff_climb_rate(takeoff_climb_rate, G_Dt);
        pos_control->update_z_controller();
        break;

    case ZQCC_Landed:
        // set motors to spin-when-armed if throttle below deadzone, otherwise full range (but motors will only spin at min throttle)
        if (target_climb_rate < 0.0f) {
            motors->set_desired_spool_state(AP_Motors::DESIRED_SPIN_WHEN_ARMED);
        } else {
            motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);
        }

#if FRAME_CONFIG == HELI_FRAME    
        if (heli_flags.init_targets_on_arming) {
            attitude_control->reset_rate_controller_I_terms();
            attitude_control->set_yaw_target_to_current_heading();
            if (motors->get_interlock()) {
                heli_flags.init_targets_on_arming=false;
            }
        }
#else
        attitude_control->reset_rate_controller_I_terms();
        attitude_control->set_yaw_target_to_current_heading();
#endif
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate);
        pos_control->relax_alt_hold_controllers(0.0f);   // forces throttle output to go to zero
        pos_control->update_z_controller();
        break;

    case ZQCC_Flying:
        if ( !(is_zero(target_roll) && is_zero(target_roll) && is_zero(target_yaw_rate) ) ) {
            gcs().send_text(MAV_SEVERITY_CRITICAL,"ZQCC exit, err 1");
            set_mode(LAND, MODE_REASON_UNKNOWN);
            return;            
        }
        if (!infoZQCC.adjust_roll_pitch_yaw(target_roll, target_pitch, attitude_control->get_althold_lean_angle_max(), target_yaw_rate)) {
            gcs().send_text(MAV_SEVERITY_CRITICAL,"ZQCC exit, err 2");
            set_mode(LAND, MODE_REASON_UNKNOWN);
            return;              
        }
        if (!infoZQCC.adjust_climb_rate(target_climb_rate)) {
            gcs().send_text(MAV_SEVERITY_CRITICAL,"ZQCC exit, err 3");
            set_mode(LAND, MODE_REASON_UNKNOWN);
            return;           
        }
        target_climb_rate = constrain_float(target_climb_rate, -get_pilot_speed_dn(), g.pilot_speed_up);
        infoZQCC.update_sonar_alt();
        motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

#if AC_AVOID_ENABLED == ENABLED
        // apply avoidance
        copter.avoid.adjust_roll_pitch(target_roll, target_pitch, copter.aparm.angle_max);
#endif

        // call attitude controller
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate);

        // adjust climb rate using rangefinder
        target_climb_rate = get_surface_tracking_climb_rate(target_climb_rate, pos_control->get_alt_target(), G_Dt);

        // get avoidance adjusted climb rate
        target_climb_rate = get_avoidance_adjusted_climbrate(target_climb_rate);

        // call position controller
        pos_control->set_alt_target_from_climb_rate_ff(target_climb_rate, G_Dt, false);
        pos_control->update_z_controller();
        infoZQCC.accumulate_lean(target_roll, target_pitch, G_Dt);
        break;
    }
}
