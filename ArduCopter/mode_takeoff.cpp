#include "Copter.h"

#if MODE_GUIDED_ENABLED && MODE_AUTO_ENABLED

// init - initialise guided controller
bool ModeTakeoff::init(bool ignore_checks)
{
    if (copter.mode_guided.init(ignore_checks)) {
        if (copter.motors->armed() && !copter.ap.land_complete) {
            set_state(Mission_State::Wait);
        } else {
            set_state(Mission_State::Init);
        }
        gcs().send_text(MAV_SEVERITY_INFO, "In Takeoff MODE");
        _last_print_ms = millis();
        return true;
    }
    return false;
}

// run - runs the guided controller
// should be called at 100hz or more
void ModeTakeoff::run()
{
    update_state();

    switch (mission_state) {
        case Mission_State::Init:
        {
            copter.mode_guided.run();
        }
        break;
        case Mission_State::Takeoff:
        {
            copter.mode_guided.run();
        }
        break;
        case Mission_State::Wait:
        {
            copter.mode_guided.run();
        }
        break;
        case Mission_State::Althold:
        {
            althold_run();
        }
        break;
    }
}

void ModeTakeoff::althold_init()
{
    // initialise the vertical position controller
    if (!pos_control->is_active_z()) {
        pos_control->init_z_controller();
    }

    // set vertical speed and acceleration limits
    pos_control->set_max_speed_accel_z(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);
    pos_control->set_correction_speed_accel_z(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);
}

void ModeTakeoff::althold_run()
{
    float target_alt_cm = 3000.f;
    // set vertical speed and acceleration limits
    pos_control->set_max_speed_accel_z(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);

    // apply SIMPLE mode transform to pilot inputs
    update_simple_mode();

    // get pilot desired lean angles
    float target_roll = 0.0f;
    float target_pitch = 0.0f;

    // get pilot's desired yaw rate
    float target_yaw_rate = 0.0f;

    // get pilot desired climb rate
    float target_climb_rate = copter.g.pilot_speed_up;

    // Alt Hold State Machine Determination
    AltHoldModeState althold_state = get_alt_hold_state(target_climb_rate);

    // Alt Hold State Machine
    switch (althold_state) {

    case AltHold_MotorStopped:
        attitude_control->reset_rate_controller_I_terms();
        attitude_control->reset_yaw_target_and_rate(false);
        pos_control->relax_z_controller(0.0f);   // forces throttle output to decay to zero
        break;

    case AltHold_Landed_Ground_Idle:
        attitude_control->reset_yaw_target_and_rate();
        FALLTHROUGH;

    case AltHold_Landed_Pre_Takeoff:
        attitude_control->reset_rate_controller_I_terms_smoothly();
        pos_control->relax_z_controller(0.0f);   // forces throttle output to decay to zero
        break;

    case AltHold_Takeoff:
        // initiate take-off
        if (!takeoff.running()) {
            takeoff.start(target_alt_cm);
        }

        // get avoidance adjusted climb rate
        target_climb_rate = get_avoidance_adjusted_climbrate(target_climb_rate);

        // set position controller targets adjusted for pilot input
        takeoff.do_pilot_takeoff(target_climb_rate);
        break;

    case AltHold_Flying:
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

#if AC_AVOID_ENABLED == ENABLED
        // apply avoidance
        copter.avoid.adjust_roll_pitch(target_roll, target_pitch, copter.aparm.angle_max);
#endif

        // get avoidance adjusted climb rate
        target_climb_rate = get_avoidance_adjusted_climbrate(target_climb_rate);

        // update the vertical offset based on the surface measurement
        copter.surface_tracking.update_surface_offset();

        // Send the commanded climb rate to the position controller
        pos_control->set_pos_target_z_cm(target_alt_cm);
        break;
    }

    // call attitude controller
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate);

    // run the vertical position controller and set output throttle
    pos_control->update_z_controller();
}

void ModeTakeoff::update_state()
{

    switch (mission_state) {
        case Mission_State::Init:
        {
            if (copter.motors->armed() && copter.ap.land_complete) {
                set_state(Mission_State::Takeoff);
            }
        }
        break;
        case Mission_State::Takeoff:
        {
            if (copter.mode_guided.submode() != ModeGuided::SubMode::TakeOff) {
                set_state(Mission_State::Wait);
            }
            if (!copter.mode_guided.is_taking_off()) {
                set_state(Mission_State::Wait);
            }
            if (!copter.position_ok()) {
                set_state(Mission_State::Althold);
            }
        }
        break;
        case Mission_State::Wait:
        {
            if (!copter.position_ok()) {
                set_state(Mission_State::Althold);
            }
        }
        break;
        case Mission_State::Althold:
        {
            if (copter.position_ok()) {
                set_state(Mission_State::Wait);
            }
        }
        break;
    }
}

void ModeTakeoff::set_state(Mission_State state_in)
{
    if (mission_state == state_in) {
        return;
    }

    state_ms = millis();

    switch (state_in) {
        case Mission_State::Init:
        {
            mission_state = state_in;
            gcs().send_text(MAV_SEVERITY_INFO, "[Mis] State: Init");
        }
        break;
        case Mission_State::Takeoff:
        {
            if (copter.mode_guided.do_user_takeoff_start(500.f)) {
                copter.set_auto_armed(true);
                mission_state = state_in;
                gcs().send_text(MAV_SEVERITY_INFO, "[Mis] State: Takeoff");
            } else {
                gcs().send_text(MAV_SEVERITY_INFO, "[Mis] State: Takeoff Fail");
            }
        }
        break;
        case Mission_State::Wait:
        {
            copter.mode_guided.velaccel_control_start();
            mission_state = state_in;
            gcs().send_text(MAV_SEVERITY_INFO, "[Mis] State: Wait");
        }
        break;
        case Mission_State::Althold:
        {
            mission_state = state_in;
            althold_init();
            gcs().send_text(MAV_SEVERITY_INFO, "[Mis] State: Althold");
        }
        default:
        break;
    }
}

bool ModeTakeoff::is_taking_off() const
{
    return mission_state == Mission_State::Takeoff;
}

uint32_t ModeTakeoff::wp_distance() const
{
    switch(mission_state) {
        case Mission_State::Init:
        case Mission_State::Takeoff:
        case Mission_State::Wait:
            return copter.mode_guided.wp_distance();
            break;
        default:
            return 0;
            break;
    }
    return 0;
}

int32_t ModeTakeoff::wp_bearing() const
{
    switch(mission_state) {
        case Mission_State::Init:
        case Mission_State::Takeoff:
        case Mission_State::Wait:
            return copter.mode_guided.wp_bearing();
            break;
        default:
            return 0;
            break;
    }
    return 0;
}

float ModeTakeoff::crosstrack_error() const
{
    switch(mission_state) {
        case Mission_State::Init:
        case Mission_State::Takeoff:
        case Mission_State::Wait:
            return copter.mode_guided.crosstrack_error();
            break;
        default:
            return 0;
            break;
    }
    return 0;
}
#endif