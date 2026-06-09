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
            update_cruise();
            copter.mode_guided.run();
        }
        break;
        case Mission_State::Search:
        {
            copter.mode_circle.run();
        }
        break;
        case Mission_State::Track:
        {
            update_track();
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
        }
        break;
        case Mission_State::Cruise:
        {
            if (!copter.umission.have_target_loc()) {
                set_state(Mission_State::Wait);
            }

            if (copter.umission.get_target_loc().get_distance(copter.current_loc) < 20.f) {
                set_state(Mission_State::Search);
            }
            // copter.ugimbal.set_state(UGimbal::Gimbal_State::Ahead);
        }
        break;
        case Mission_State::Search:
        {
            // if (!copter.umission.target_pos_prob_valid()) {
            //     set_state(Mission_State::Wait);
            // }
            if (copter.ugimbal.status.have_target) {
                set_state(Mission_State::Track);
            }
        }
        break;
        case Mission_State::Track:
        {
            if (!copter.ugimbal.status.have_target) {
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

void ModeMission::update_cruise()
{
    if (millis() - _last_loc_update_time_ms > 1000) {
        _last_loc_update_time_ms = millis();
        copter.mode_guided.set_destination(copter.umission.get_target_loc());
    }
}

void ModeMission::update_track()
{
    Vector3f vel;
    vel.x = 0.f;
    vel.y = 0.f;
    vel.z = 0.f;
    float yaw_cd = copter.ugimbal.status.cam_yaw * 100.f;
    bool use_yaw = true;
    bool use_yaw_rate = false;
    float yaw_rate_cds = 0.0f;
    bool yaw_relative = true;//use relative yaw
    bool log_request = false;
    copter.mode_guided.set_velocity(vel, use_yaw, yaw_cd, use_yaw_rate, yaw_rate_cds, yaw_relative, log_request);
}

void ModeMission::set_cruise_state()
{
    if (copter.flightmode->mode_number() != Mode::Number::MISSION) {return;}
    switch (mission_state) {
        case Mission_State::Init:
        case Mission_State::Takeoff:
        case Mission_State::Cruise:
        {
            ;
        }
        break;
        case Mission_State::Wait:
        case Mission_State::Search:
        case Mission_State::Track:
        {
            set_state(Mission_State::Cruise);
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
            if (copter.mode_circle.init(false)) {
                mission_state = state_in;
                copter.circle_nav->set_center(copter.umission.get_target_loc());
                gcs().send_text(MAV_SEVERITY_INFO, "[Mis] State: Search");
            } else {
                gcs().send_text(MAV_SEVERITY_INFO, "[Mis] State: Search");
            }
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
            return copter.mode_circle.wp_distance();
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
        case Mission_State::Track:
            return copter.mode_guided.wp_bearing();
            break;
        case Mission_State::Search:
            return copter.mode_circle.wp_bearing();
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
        case Mission_State::Track:
            return copter.mode_guided.crosstrack_error();
            break;
        case Mission_State::Search:
        case Mission_State::Return:
        default:
            return 0;
            break;
    }
    return 0;
}
#endif