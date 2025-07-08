#include "Copter.h"

#if MODE_GUIDED_ENABLED

// init - initialise guided controller
bool ModeMission::init(bool ignore_checks)
{
    if (copter.mode_guided.init()) {
        if (copter.motors->armed && !copter.land_complete) {
            set_state(State::Wait);
        } else {
            set_state(State::Init);
        }
    }
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
            if (copter.motors->armed && copter.land_complete) {
                set_state(State::Takeoff);
            }
        }
        break;
        case State::Takeoff:
        {
            if (mode_guided.guided_mode != SubMode::TakeOff) {
                set_state(State::Wait);
            }
            if (mode_guided.takeoff_complete) {
                set_state(State::Wait);
            }
        }
        break;
        case State::Wait:
        {
            if (mode_guided.guided_mode != SubMode::VelAccel) {
                copter.mode_guided.pva_control_start();
            }
            if (copter.umission.target_pos_valid()) {
                set_state(State::Cruise);
            }
        }
        break;
        case State::Cruise:
        {
            if (!copter.umission.target_pos_valid()) {
                set_state(State::Wait);
            }
            if (mode_guided.guided_mode != SubMode::PosVelAccel) {
                copter.mode_guided.posvelaccel_control_start();
            }
            if (millis() - copter.mode_guided.update_time_ms > 1000) {
                copter.mode_guided.set_destination_posvel(copter.umission.get_target_pos());
            }
            if (copter.umission.get_target_pos().get_distance(copter.current_loc) < 200.f) {
                set_state(State::Search);
            }
            ugimbal.set_state(UGimbal_State::Ahead);
        }
        break;
        case State::Search:
        {
            if (!copter.umission.target_pos_valid()) {
                set_state(State::Wait);
            }
            if (mode_guided.guided_mode != SubMode::PosVelAccel) {
                copter.mode_guided.posvelaccel_control_start();
            }
            if (millis() - copter.mode_guided.update_time_ms > 1000) {
                copter.mode_guided.set_destination_posvel(copter.umission.get_target_pos());
            }
            if (copter.ugimbal.have_target) {
                set_state(State::Track);
            }
            copter.ugimbal.set_state(UGimbal_State::Search);
        }
        break;
        case State::Track:
        {
            if (copter.ugimbal.have_target && (millis() - copter.mode_guided.update_time_ms > 1000)) {
                copter.mode_guided.set_destination_posvel(copter.ugimbal.get_target_pos());
            }
            if (!copter.ugimbal.have_target) {
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

void ModeMission::set_state(State::state_in)
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
            if (mode_guided.do_user_takeoff_start(200.f)) {
                mission_state = state_in;
            } else {
                gcs().send_text(MAV_SEVERITY_INFO, "[Mis] State: Takeoff");
            }
        }
        break;
        case State::Wait:
        {
            if (copter.mode_guided.pva_control_start()) {
                mission_state = state_in;
            } else {
                gcs().send_text(MAV_SEVERITY_INFO, "[Mis] State: Wait");
            }
        }
        break;
        case State::Cruise:
        {
            if (copter.mode_guided.pva_control_start()) {
                mission_state = state_in;
            } else {
                gcs().send_text(MAV_SEVERITY_INFO, "[Mis] State: Cruise");
            }
        }
        break;
        case State::Search:
        {
            if (copter.mode_guided.pva_control_start()) {
                mission_state = state_in;
            } else {
                gcs().send_text(MAV_SEVERITY_INFO, "[Mis] State: Search");
            }
        }
        break;
        case State::Track:
        {
            if (copter.mode_guided.pva_control_start()) {
                mission_state = state_in;
            } else {
                gcs().send_text(MAV_SEVERITY_INFO, "[Mis] State: Track");
            }
        }
        break;
        case State::Return:
        {
            if (copter.mode_rtl.init()) {
                mission_state = state_in;
            } else {
                gcs().send_text(MAV_SEVERITY_INFO, "[Mis] State: Return");
            }
        }
        break;
    }
}
