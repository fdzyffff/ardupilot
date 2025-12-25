#include "Copter.h"

#if MODE_GUIDED_ENABLED

// init - initialise guided controller
bool ModeMission::init(bool ignore_checks)
{
    if (copter.mode_guided.init(ignore_checks)) {
        if (copter.motors->armed() && !copter.ap.land_complete) {
            set_state(MISSION_State::Wait);
        } else {
            set_state(MISSION_State::Init);
        }
        gcs().send_text(MAV_SEVERITY_INFO, "In MIS MODE");
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
        case MISSION_State::Init:
        {
            copter.mode_guided.run();
        }
        break;
        case MISSION_State::Takeoff:
        {
            copter.mode_guided.run();
        }
        break;
        case MISSION_State::Wait:
        {
            copter.mode_guided.run();
        }
        break;
        case MISSION_State::Fly:
        {
            wp_run();
        }
        break;
    }
}

void ModeMission::update_state()
{
    static uint32_t last_update_ms = millis();
    if (millis() - last_update_ms < 100) {
        return;
    }
    last_update_ms = millis();
    switch (mission_state) {
        case MISSION_State::Init:
        {
            if (copter.motors->armed() && copter.ap.land_complete) {
                set_state(MISSION_State::Takeoff);
            }
        }
        break;
        case MISSION_State::Takeoff:
        {
            if (copter.mode_guided.submode() != ModeGuided::SubMode::TakeOff) {
                set_state(MISSION_State::Wait);
            }
            if (copter.mode_guided.takeoff_complete) {
                set_state(MISSION_State::Wait);
            }
        }
        break;
        case MISSION_State::Wait:
        {
            ;
        }
        break;
        case MISSION_State::Fly:
        {
            if (wp_nav->reached_wp_destination()) {
                if (do_next()) {
                    ;
                } else {
                    set_state(MISSION_State::Wait);
                }
            }
        }
        break;
    }
}

void ModeMission::set_state(MISSION_State state_in)
{
    if (mission_state == state_in) {
        return;
    }

    switch (state_in) {
        case MISSION_State::Init:
        {
            mission_state = state_in;
            gcs().send_text(MAV_SEVERITY_INFO, "[MIS] State: Init");
        }
        break;
        case MISSION_State::Takeoff:
        {
            if (copter.mode_guided.do_user_takeoff_start(120.f)) {
                copter.set_auto_armed(true);
                mission_state = state_in;
                gcs().send_text(MAV_SEVERITY_INFO, "[MIS] State: Takeoff");
            } else {
                gcs().send_text(MAV_SEVERITY_INFO, "[MIS] State: Takeoff Fail");
            }
        }
        break;
        case MISSION_State::Wait:
        {
            copter.mode_guided.velaccel_control_start();
            mission_state = state_in;
            target_loc = copter.current_loc;
            gcs().send_text(MAV_SEVERITY_INFO, "[MIS] State: Wait");
        }
        break;
        case MISSION_State::Fly:
        {
            target_loc = copter.current_loc;
            do_next();
            mission_state = state_in;
            gcs().send_text(MAV_SEVERITY_INFO, "[MIS] State: Fly");
        }
        break;
        default:
        break;
    }
}

void ModeMission::set_loc(Location& dest_1, Location& dest_2) 
{
    loc1 = dest_1;
    loc2 = dest_2;
    if (mission_state == MISSION_State::Wait) {
        set_state(MISSION_State::Fly);
    } else if (mission_state == MISSION_State::Fly) {
        do_next();
    }
}

bool ModeMission::do_next() {
    wp_control_start();
    bool ret = false;
    if (target_loc.lat == loc1.lat && target_loc.lng == loc1.lng) {
        if (wp_nav->set_wp_destination_loc(loc2) && wp_nav->set_wp_destination_next_loc(loc2)) {
            target_loc = loc2;
            ret = true;
        } else {
            ret = false;
        }
    } else if (target_loc.lat == loc2.lat && target_loc.lng == loc2.lng) {
        gcs().send_text(MAV_SEVERITY_INFO, "[MIS] Finish, wait");
        ret = false;
    } else {
        if (wp_nav->set_wp_destination_loc(loc1) && wp_nav->set_wp_destination_next_loc(loc1)) {
            target_loc = loc1;
            ret = true;
        } else {
            ret = false;
        }
    }
#if HAL_LOGGING_ENABLED
    if (ret) {
        // log target
        copter.Log_Write_Guided_Position_Target(ModeGuided::SubMode::WP, Vector3f(target_loc.lat, target_loc.lng, target_loc.alt), (target_loc.get_alt_frame() == Location::AltFrame::ABOVE_TERRAIN), Vector3f(), Vector3f());
    }
#endif

    if (ret) {
        gcs().send_text(MAV_SEVERITY_INFO, "WP : %d, %d", (int)target_loc.lat, (int)target_loc.lng);
    }
    return ret;
}

// initialise guided mode's waypoint navigation controller
void ModeMission::wp_control_start()
{
    // init wpnav and set origin if transitioning from takeoff
    if (!wp_nav->is_active()) {
        // initialise waypoint and spline controller
        wp_nav->wp_and_spline_init();

        // initialise wpnav to stopping point
        Vector3f stopping_point;
        wp_nav->get_wp_stopping_point(stopping_point);
        if (!wp_nav->set_wp_destination(stopping_point, false)) {
            // this should never happen because terrain data is not used
            INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
        }

        // initialise yaw
        auto_yaw.set_mode_to_default(false);
    }
}

void ModeMission::wp_run()
{
    // if not armed set throttle to zero and exit immediately
    if (is_disarmed_or_landed()) {
        make_safe_ground_handling();
        return;
    }

    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // run waypoint controller
    copter.failsafe_terrain_set_status(wp_nav->update_wpnav());

    // WP_Nav has set the vertical position control targets
    // run the vertical position controller and set output throttle
    pos_control->update_z_controller();

    // call attitude controller with auto yaw
    attitude_control->input_thrust_vector_heading(pos_control->get_thrust_vector(), auto_yaw.get_heading());
}

bool ModeMission::is_taking_off() const
{
    return mission_state == MISSION_State::Takeoff;
}

uint32_t ModeMission::wp_distance() const
{
    switch(mission_state) {
        case MISSION_State::Init:
        case MISSION_State::Takeoff:
        case MISSION_State::Wait:
            return copter.mode_guided.wp_distance();
            break;
        case MISSION_State::Fly:
            return wp_nav->get_wp_distance_to_destination();
        default:
            return 0;
            break;
    }
    return 0;
}

int32_t ModeMission::wp_bearing() const
{
    switch(mission_state) {
        case MISSION_State::Init:
        case MISSION_State::Takeoff:
        case MISSION_State::Wait:
            return copter.mode_guided.wp_bearing();
            break;
        case MISSION_State::Fly:
            return wp_nav->get_wp_bearing_to_destination();
        default:
            return 0;
            break;
    }
    return 0;
}

float ModeMission::crosstrack_error() const
{
    switch(mission_state) {
        case MISSION_State::Init:
        case MISSION_State::Takeoff:
        case MISSION_State::Wait:
            return copter.mode_guided.crosstrack_error();
            break;
        case MISSION_State::Fly:
            return wp_nav->crosstrack_error();
        default:
            return 0;
            break;
    }
    return 0;
}
#endif