#include "Copter.h"

#if MODE_GUIDED_ENABLED && MODE_AUTO_ENABLED

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
        case Mission_State::Auto:
        {
            copter.mode_auto.run();
        }
        break;
        case Mission_State::Search:
        {
            copter.mode_guided.run();
        }
        break;
        case Mission_State::Track:
        {
            if ((millis() - copter.mode_guided.my_update_time_ms > 100))
            {
                Vector3f velocity;
                velocity.x = 0.0f;
                velocity.y = 0.0f;
                velocity.z = copter.uattack.get_target_vel_z() * 100.f;
                const Vector3f& acceleration = Vector3f(0.0f, 0.0f, 0.0f);
                bool use_yaw = true;
                float yaw_cd = copter.uattack.get_target_angle_yaw()*100.f;
                bool use_yaw_rate = false;
                float yaw_rate_cds = 0.0f;
                bool relative_yaw = false;
                bool log_request = false;
                copter.mode_guided.set_velaccel(velocity, acceleration, use_yaw, yaw_cd, use_yaw_rate, yaw_rate_cds, relative_yaw, log_request);
            }
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
            if (copter.uattack.is_active()) {
                set_state(Mission_State::Search);
            }
            if (millis() - state_ms > 5000) {
                set_state(Mission_State::Auto);
            }
        }
        break;
        case Mission_State::Auto:
        {
            // if (mission.state() == AP_Mission::mission_state::MISSION_COMPLETE) {
            //     set_state(Mission_State::Search);
            // }
            if (copter.uattack.is_active()) {
                set_state(Mission_State::Search);
            }
        }
        break;
        // stop till track automatically or manually depends on param setting
        case Mission_State::Search:
        {
            if (copter.mode_guided.submode() != ModeGuided::SubMode::VelAccel) {
                copter.mode_guided.velaccel_control_start();
            }
            bool auto_track = false;
            if (copter.uattack.is_active() && auto_track) {
                set_state(Mission_State::Track);
            }
            // if (millis() - state_ms > 5000) {
            //     set_state(Mission_State::Return);
            // }
        }
        break;
        case Mission_State::Track:
        {
            // if (copter.mode_guided.submode() != ModeGuided::SubMode::VelAccel) {
            //     copter.mode_guided.velaccel_control_start();
            // }
            if (!copter.uattack.is_active()) {
                set_state(Mission_State::Wait);
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

void ModeMission::do_final_track()
{
    if (copter.flightmode->mode_number() != Mode::Number::MISSION) {return;}
    switch (mission_state) {
        case Mission_State::Init:
        case Mission_State::Takeoff:
        {
            ;
        }
        break;
        case Mission_State::Wait:
        case Mission_State::Auto:
        case Mission_State::Search:
        {
            if (copter.uattack.is_active()) {
                set_state(Mission_State::Track);
            }
        }
        break;
        case Mission_State::Track:
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
        case Mission_State::Auto:
        {
            copter.mode_auto.init(false);
            mission_state = state_in;
            gcs().send_text(MAV_SEVERITY_INFO, "[Mis] State: Auto");
        }
        break;
        case Mission_State::Search:
        {
            if (copter.mode_guided.init(false)) {
                copter.mode_guided.velaccel_control_start();
                mission_state = state_in;
                gcs().send_text(MAV_SEVERITY_INFO, "[Mis] State: Search");
            } else {
                gcs().send_text(MAV_SEVERITY_INFO, "[Mis] State: Search");
            }
        }
        break;
        case Mission_State::Track:
        {
            if (copter.mode_guided.init(false)) {
                copter.uattack.reset();
                copter.mode_guided.velaccel_control_start();
                mission_state = state_in;
                gcs().send_text(MAV_SEVERITY_INFO, "[Mis] State: Track");
            } else {
                gcs().send_text(MAV_SEVERITY_INFO, "[Mis] State: Track");
            }
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
            return copter.mode_guided.wp_distance();
            break;
        case Mission_State::Auto:
            return copter.mode_auto.wp_distance();
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
            return copter.mode_guided.wp_bearing();
            break;
        case Mission_State::Auto:
            return copter.mode_auto.wp_bearing();
            break;
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
            return copter.mode_guided.crosstrack_error();
            break;
        case Mission_State::Auto:
            return copter.mode_auto.crosstrack_error();
            break;
        case Mission_State::Track:
        case Mission_State::Search:
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