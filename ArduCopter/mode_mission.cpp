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
        case MISSION_State::LAND:
        {
            copter.mode_land.run();
        }
        break;
        case MISSION_State::RETURN:
        {
            copter.mode_rtl.run();
        }
        break;
        default:
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
            // if (copter.motors->armed() && copter.ap.land_complete) {
            //     set_state(MISSION_State::Takeoff);
            // }
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
        case MISSION_State::LAND:
        {
            ;
        }
        break;
        case MISSION_State::RETURN:
        {
            ;
        }
        break;
        default:
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
            if (copter.mode_guided.init(false)) {
                mission_state = state_in;
                gcs().send_text(MAV_SEVERITY_INFO, "[MIS] State: Init");
            }
        }
        break;
        case MISSION_State::Takeoff:
        {
            if (copter.mode_guided.init(false) && copter.mode_guided.do_user_takeoff_start(120.f)) {
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
            if (copter.mode_guided.init(false)) {
                copter.mode_guided.velaccel_control_start();
                mission_state = state_in;
                target_loc = copter.current_loc;
                gcs().send_text(MAV_SEVERITY_INFO, "[MIS] State: Wait");
            } else {
                gcs().send_text(MAV_SEVERITY_INFO, "[MIS] State: Wait Fail");
            }
        }
        break;
        case MISSION_State::Fly:
        {
            if (copter.mode_guided.init(false)) {
                target_loc = copter.current_loc;
                do_next();
                mission_state = state_in;
                gcs().send_text(MAV_SEVERITY_INFO, "[MIS] State: Fly");
            } else {
                gcs().send_text(MAV_SEVERITY_INFO, "[MIS] State: Fly Fail");
            }
        }
        break;
        case MISSION_State::LAND:
        {
            if (copter.mode_land.init(false)) {
                mission_state = state_in;
                gcs().send_text(MAV_SEVERITY_INFO, "[MIS] State: Land");
            } else {
                gcs().send_text(MAV_SEVERITY_INFO, "[MIS] State: Land Fail");
            }
        }
        break;
        case MISSION_State::RETURN:
        {
            if (copter.mode_rtl.init(false)) {
                mission_state = state_in;
                gcs().send_text(MAV_SEVERITY_INFO, "[MIS] State: Return");
            } else {
                gcs().send_text(MAV_SEVERITY_INFO, "[MIS] State: Return Fail");
            }
        }
        break;
        default:
        break;
    }
}

void ModeMission::set_loc(Location& dest_in, uint16_t spd_xy_in, uint16_t spd_z_in, uint8_t id_in) 
{
    loc_list[id_in] = dest_in;
    spd_xy_list[id_in] = spd_xy_in;
    spd_z_list[id_in] = spd_z_in;
}

void ModeMission::set_wp_number(uint8_t wp_number_in)
{
    wp_number = wp_number_in;
    wp_idx = 0;
    if (mission_state == MISSION_State::Wait) {
        set_state(MISSION_State::Fly);
    }
    else if (mission_state == MISSION_State::Fly) {
        do_next();
        gcs().send_text(MAV_SEVERITY_INFO, "[MIS] State: Fly new");
    }
}

bool ModeMission::do_next() {
    wp_control_start();
    bool ret = false;
    if (wp_idx < wp_number) {
        if (wp_nav->set_wp_destination_loc(loc_list[wp_idx]) && wp_nav->set_wp_destination_next_loc(loc_list[wp_idx])) {
            target_loc = loc_list[wp_idx];
            ret = true;
        }
    } else {
        ret = false;
    }
#if HAL_LOGGING_ENABLED
    if (ret) {
        // log target
        copter.Log_Write_Guided_Position_Target(ModeGuided::SubMode::WP, Vector3f(target_loc.lat, target_loc.lng, target_loc.alt), (target_loc.get_alt_frame() == Location::AltFrame::ABOVE_TERRAIN), Vector3f(), Vector3f());
    }
#endif

    if (ret) {
        gcs().send_text(MAV_SEVERITY_INFO, "WP : %d, %d, %d", (int)target_loc.lat, (int)target_loc.lng, (int)target_loc.alt);
        gcs().send_text(MAV_SEVERITY_INFO, "WP : [%d/%d]", (int)wp_idx, (int)wp_number);
        wp_idx++;
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

void ModeMission::handle_msg(const mavlink_message_t &msg)
{
    if (msg.msgid == MAVLINK_MSG_ID_GUIDED_WAYPOINTS) {
        // decode packet
        // gcs().send_text(MAV_SEVERITY_WARNING, "Target mavpkg");
        // decode packet
        mavlink_guided_waypoints_t packet;
        mavlink_msg_guided_waypoints_decode(&msg, &packet);
        for (uint8_t i_wp = 0; i_wp < packet.num; i_wp++) {
            Location temp_loc;
            temp_loc.lat = packet.lat[i_wp];
            temp_loc.lng = packet.lng[i_wp];
            temp_loc.alt = packet.alt[i_wp];
            set_loc(temp_loc, packet.vel_xy[i_wp], packet.vel_z[i_wp], i_wp);
        }
        set_wp_number(packet.num);
        gcs().send_text(MAV_SEVERITY_INFO, "[%d] waypoints received", packet.num);
    }
}

#endif