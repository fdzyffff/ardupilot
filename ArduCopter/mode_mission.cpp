#include "Copter.h"

#if MODE_GUIDED_ENABLED

// init - initialise guided controller
bool ModeMission::init(bool ignore_checks)
{
    if (copter.mode_guided.init(ignore_checks)) {
        if (copter.motors->armed() && !copter.ap.land_complete) {
            set_state(State::Wait);
        } else {
            set_state(State::Init);
        }
        return true;
    }
    return false;
}

// run - runs the guided controller
// should be called at 100hz or more
void ModeMission::run()
{
    update_state();

    switch (mission_state) {
        case State::Init:
        {
            copter.mode_guided.run();
        }
        break;
        case State::Takeoff:
        {
            copter.mode_guided.run();
        }
        break;
        case State::Wait:
        {
            copter.mode_guided.run();
        }
        break;
        case State::Cruise:
        {
            copter.mode_guided.run();
        }
        break;
        case State::Search:
        {
            copter.mode_guided.run();
        }
        break;
        case State::Track:
        {
            copter.mode_guided.run();
        }
        break;
        case State::Return:
        {
            copter.mode_rtl.run();
        }
        break;
    }
}

void ModeMission::update_state()
{

    switch (mission_state) {
        case State::Init:
        {
            if (copter.motors->armed() && copter.ap.land_complete) {
                set_state(State::Takeoff);
            }
        }
        break;
        case State::Takeoff:
        {
            if (copter.mode_guided.submode() != ModeGuided::SubMode::TakeOff) {
                set_state(State::Wait);
            }
            if (copter.mode_guided.takeoff_complete) {
                set_state(State::Wait);
            }
        }
        break;
        case State::Wait:
        {
            if (copter.mode_guided.submode() != ModeGuided::SubMode::VelAccel) {
                copter.mode_guided.pva_control_start();
            }
            if (copter.umission.target_pos_prob_valid()) {
                set_state(State::Cruise);
            }
        }
        break;
        case State::Cruise:
        {
            if (!copter.umission.target_pos_prob_valid()) {
                set_state(State::Wait);
            }
            if (copter.mode_guided.submode() != ModeGuided::SubMode::PosVelAccel) {
                copter.mode_guided.posvelaccel_control_start();
            }
            if (millis() - copter.mode_guided.my_update_time_ms > 1000) {
                copter.mode_guided.set_destination(copter.umission.get_target_pos_prob());
            }
            if (copter.umission.get_target_pos().get_distance(copter.current_loc) < 200.f) {
                set_state(State::Search);
            }
            copter.ugimbal.set_state(UGimbal::Gimbal_State::Ahead);
        }
        break;
        case State::Search:
        {
            if (!copter.umission.target_pos_prob_valid()) {
                set_state(State::Wait);
            }
            if (copter.mode_guided.submode() != ModeGuided::SubMode::PosVelAccel) {
                copter.mode_guided.posvelaccel_control_start();
            }
            if (millis() - copter.mode_guided.my_update_time_ms > 1000) {
                copter.mode_guided.set_destination(copter.umission.get_target_pos_prob());
            }
            if (copter.ugimbal.have_target()) {
                set_state(State::Track);
            }
            copter.ugimbal.set_state(UGimbal::Gimbal_State::Search);
        }
        break;
        case State::Track:
        {
            if (copter.ugimbal.have_target() && (millis() - copter.mode_guided.my_update_time_ms > 1000)) {
                copter.mode_guided.set_destination(copter.umission.get_target_pos_prob());
            }
            if (!copter.ugimbal.have_target()) {
                set_state(State::Search);
            }
        }
        break;
        case State::Return:
        {
            ;
        }
        break;
    }
}

void ModeMission::set_state(State state_in)
{
    if (mission_state == state_in) {
        return;
    }

    switch (mission_state) {
        case State::Init:
        {
            mission_state = state_in;
        }
        break;
        case State::Takeoff:
        {
            if (copter.mode_guided.do_user_takeoff_start(200.f)) {
                mission_state = state_in;
                gcs().send_text(MAV_SEVERITY_INFO, "[Mis] State: Takeoff");
            } else {
                gcs().send_text(MAV_SEVERITY_INFO, "[Mis] State: Takeoff Fail");
            }
        }
        break;
        case State::Wait:
        {
            copter.mode_guided.pva_control_start();
            mission_state = state_in;
            gcs().send_text(MAV_SEVERITY_INFO, "[Mis] State: Wait");
        }
        break;
        case State::Cruise:
        {
            copter.mode_guided.pva_control_start();
            mission_state = state_in;
            gcs().send_text(MAV_SEVERITY_INFO, "[Mis] State: Cruise");
        }
        break;
        case State::Search:
        {
            copter.mode_guided.pva_control_start();
            mission_state = state_in;
            gcs().send_text(MAV_SEVERITY_INFO, "[Mis] State: Search");
        }
        break;
        case State::Track:
        {
            copter.mode_guided.pva_control_start();
            mission_state = state_in;
            gcs().send_text(MAV_SEVERITY_INFO, "[Mis] State: Track");
        }
        break;
        case State::Return:
        {
            if (copter.mode_rtl.init(false)) {
                mission_state = state_in;
            } else {
                gcs().send_text(MAV_SEVERITY_INFO, "[Mis] State: Return");
            }
        }
        break;
    }
}

#endif