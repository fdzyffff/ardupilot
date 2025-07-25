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
            if (copter.umission.valid()) {
                set_state(Mission_State::Cruise);
            }
        }
        break;
        case Mission_State::Cruise:
        {
            if (!copter.umission.valid()) {
                set_state(Mission_State::Wait);
            }
            // if (millis() - copter.mode_guided.my_update_time_ms > 10) {
            //     cal_follow_info();
            // }
            cal_follow_info();
        }
        break;
    }
}

void ModeMission::cal_follow_info()
{
    if (copter.umission.valid()) {
        float kp = 1.0f;
        Vector3f body_vel;
        body_vel.x = 300.f;
        body_vel.y = kp * constrain_float(copter.umission.get_control_corr_bfy(), -3.0f, 3.0f) * 100.f;
        body_vel.z = kp * constrain_float(copter.umission.get_control_corr_bfz(), -3.0f, 3.0f) * 100.f;

        Matrix3f tmp_body_earth_m;
        tmp_body_earth_m.from_euler(0.0f, 0.0f, AP::ahrs().get_yaw());
        Vector3f follow_velocity = tmp_body_earth_m * body_vel;
        Vector3f follow_acceleration;
        follow_acceleration.zero();
        bool  use_yaw = false;
        float yaw_cd = degrees(AP::ahrs().get_yaw())*100.f;
        bool  use_yaw_rate = true;
        float yaw_rate_cds = 0.0f;
        bool  log_request = true;
        bool  relative_yaw = false;
        if (millis() - copter.mode_guided.my_update_time_ms > 100) {
            copter.mode_guided.set_velaccel(follow_velocity, follow_acceleration, use_yaw, yaw_cd, use_yaw_rate, yaw_rate_cds, relative_yaw, log_request);
        }
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
            return copter.mode_guided.crosstrack_error();
            break;
        default:
            return 0;
            break;
    }
    return 0;
}
#endif