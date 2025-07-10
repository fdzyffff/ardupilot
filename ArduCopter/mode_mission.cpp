#include "Copter.h"

#if MODE_GUIDED_ENABLED

// init - initialise guided controller
bool ModeMission::init(bool ignore_checks)
{
    if (copter.mode_guided.init(ignore_checks)) {
        if (copter.motors->armed() && !copter.ap.land_complete) {
            set_state(Mission_State::Wait);
        } else {
            set_state(Mission_State::Init);
        }
        gcs().send_text(MAV_SEVERITY_INFO, "In Mis MODE");
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
        case Mission_State::Cruise:
        {
            copter.mode_guided.run();
        }
        break;
        case Mission_State::Search:
        {
            copter.mode_guided.run();
        }
        break;
        case Mission_State::Track:
        {
            copter.mode_guided.run();
        }
        break;
        case Mission_State::Return:
        {
            copter.mode_rtl.run();
        }
        break;
    }
}

void ModeMission::update_state()
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
            if (copter.mode_guided.takeoff_complete) {
                set_state(Mission_State::Wait);
            }
        }
        break;
        case Mission_State::Wait:
        {
            if (copter.mode_guided.submode() != ModeGuided::SubMode::VelAccel) {
                copter.mode_guided.velaccel_control_start();
            }
            if (copter.umission.target_pos_prob_valid()) {
                set_state(Mission_State::Cruise);
            }
        }
        break;
        case Mission_State::Cruise:
        {
            if (!copter.umission.target_pos_prob_valid()) {
                set_state(Mission_State::Wait);
            }
            if (millis() - copter.mode_guided.my_update_time_ms > 1000) {
                copter.mode_guided.set_destination(copter.umission.get_target_pos_prob());
            }
            if (copter.umission.get_target_pos().get_distance(copter.current_loc) < 200.f) {
                set_state(Mission_State::Search);
            }
            copter.ugimbal.set_state(UGimbal::Gimbal_State::Ahead);
        }
        break;
        case Mission_State::Search:
        {
            if (!copter.umission.target_pos_prob_valid()) {
                set_state(Mission_State::Wait);
            }
            if (millis() - copter.mode_guided.my_update_time_ms > 1000) {
                copter.mode_guided.set_destination(copter.umission.get_target_pos_prob());
            }
            if (copter.ugimbal.have_target()) {
                set_state(Mission_State::Track);
            }
            copter.ugimbal.set_state(UGimbal::Gimbal_State::Search);
        }
        break;
        case Mission_State::Track:
        {
            if (copter.ugimbal.have_target() && (millis() - copter.mode_guided.my_update_time_ms > 1000)) {
                copter.mode_guided.set_destination(copter.umission.get_target_pos_prob());
            }
            if (!copter.ugimbal.have_target()) {
                set_state(Mission_State::Search);
            }
        }
        break;
        case Mission_State::Return:
        {
            ;
        }
        break;
    }
}

void ModeMission::set_state(Mission_State state_in)
{
    if (mission_state == state_in) {
        return;
    }

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
        case Mission_State::Cruise:
        {
            copter.mode_guided.velaccel_control_start();
            mission_state = state_in;
            gcs().send_text(MAV_SEVERITY_INFO, "[Mis] State: Cruise");
        }
        break;
        case Mission_State::Search:
        {
            copter.mode_guided.velaccel_control_start();
            mission_state = state_in;
            gcs().send_text(MAV_SEVERITY_INFO, "[Mis] State: Search");
        }
        break;
        case Mission_State::Track:
        {
            copter.mode_guided.velaccel_control_start();
            mission_state = state_in;
            gcs().send_text(MAV_SEVERITY_INFO, "[Mis] State: Track");
        }
        break;
        case Mission_State::Return:
        {
            if (copter.mode_rtl.init(false)) {
                mission_state = state_in;
                gcs().send_text(MAV_SEVERITY_INFO, "[Mis] State: Return");
            } else {
                gcs().send_text(MAV_SEVERITY_INFO, "[Mis] State: Return Fail");
            }
        }
        break;
        default:
        break;
    }
}

bool ModeMission::is_taking_off() const
{
    return mission_state == Mission_State::Takeoff;
}

uint32_t ModeMission::wp_distance() const
{
    switch(mission_state) {
        case Mission_State::Init:
        case Mission_State::Takeoff:
        case Mission_State::Wait:
        case Mission_State::Cruise:
            return copter.mode_guided.wp_distance();
            break;
        case Mission_State::Search:
        case Mission_State::Track:
            return copter.mode_guided.wp_distance();
            break;
        case Mission_State::Return:
            return copter.mode_rtl.wp_distance();
            break;
        default:
            return 0;
            break;
    }
    return 0;
}

int32_t ModeMission::wp_bearing() const
{
    switch(mission_state) {
        case Mission_State::Init:
        case Mission_State::Takeoff:
        case Mission_State::Wait:
        case Mission_State::Cruise:
        case Mission_State::Search:
        case Mission_State::Track:
            return copter.mode_guided.wp_bearing();
            break;
        case Mission_State::Return:
            return copter.mode_rtl.wp_bearing();
            break;
        default:
            return 0;
            break;
    }
    return 0;
}

float ModeMission::crosstrack_error() const
{
    switch(mission_state) {
        case Mission_State::Init:
        case Mission_State::Takeoff:
        case Mission_State::Wait:
        case Mission_State::Cruise:
        case Mission_State::Search:
        case Mission_State::Track:
            return copter.mode_guided.crosstrack_error();
            break;
        case Mission_State::Return:
        default:
            return 0;
            break;
    }
    return 0;
}
#endif