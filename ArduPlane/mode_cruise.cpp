#include "mode.h"
#include "Plane.h"

bool ModeCruise::_enter()
{
    locked_heading = false;
    reach_heading = false;
    lock_timer_ms = 0;
    locked_heading_cd = plane.gps.ground_course_cd();
    _control_state = CruiseState::MANUAL;

#if HAL_SOARING_ENABLED
    // for ArduSoar soaring_controller
    plane.g2.soaring_controller.init_cruising();
#endif

    plane.set_target_altitude_current();

    return true;
}

void ModeCruise::update()
{
    /*
      in CRUISE mode we use the navigation code to control
      roll when heading is locked. Heading becomes unlocked on
      any aileron or rudder input
    */
    if (plane.channel_roll->get_control_in() != 0 || plane.channel_rudder->get_control_in() != 0) {
        locked_heading = false;
        lock_timer_ms = 0;
        _control_state = CruiseState::MANUAL;
    }

    if (!locked_heading && (_control_state == CruiseState::MANUAL)) {
        plane.nav_roll_cd = plane.channel_roll->norm_input() * plane.roll_limit_cd;
        plane.update_load_factor();
    } else {
        plane.calc_nav_roll();
    }
    plane.update_fbwb_speed_height();
}

/*
  handle CRUISE mode, locking heading to GPS course when we have
  sufficient ground speed, and no aileron or rudder input
 */
void ModeCruise::navigate()
{
    if (_control_state == CruiseState::MANUAL) {
        if (!locked_heading &&
            plane.channel_roll->get_control_in() == 0 &&
            plane.rudder_input() == 0 &&
            plane.gps.status() >= AP_GPS::GPS_OK_FIX_2D &&
            plane.gps.ground_speed() >= 3 &&
            lock_timer_ms == 0) {
            // user wants to lock the heading - start the timer
            lock_timer_ms = millis();
        }
        if (lock_timer_ms != 0 &&
            (millis() - lock_timer_ms) > 500) {
            // lock the heading after 0.5 seconds of zero heading input
            // from user
            locked_heading = true;
            lock_timer_ms = 0;
            locked_heading_cd = plane.gps.ground_course_cd();
            plane.prev_WP_loc = plane.current_loc;
        }
        if (locked_heading) {
            plane.next_WP_loc = plane.prev_WP_loc;
            // always look 1km ahead
            plane.next_WP_loc.offset_bearing(locked_heading_cd*0.01f, plane.prev_WP_loc.get_distance(plane.current_loc) + 1000);
            plane.nav_controller->update_waypoint(plane.prev_WP_loc, plane.next_WP_loc);
        }
    }
    if (_control_state == CruiseState::OFFBOARD) {
        if (!reach_heading && (labs(wrap_180_cd(locked_heading_cd - plane.gps.ground_course_cd())) > 1000)) {
            plane.prev_WP_loc = plane.current_loc;
        } else {
            reach_heading = true;
        }

        if (cmd_state_1 != ControlState_1::P0) {
            update_1();
        }
        if (cmd_state_2 != ControlState_2::P0) {
            update_2();
        }

        plane.next_WP_loc = plane.prev_WP_loc;
        // always look 1km ahead
        plane.next_WP_loc.offset_bearing(locked_heading_cd*0.01f, plane.prev_WP_loc.get_distance(plane.current_loc) + 1000);
        plane.nav_controller->update_waypoint(plane.prev_WP_loc, plane.next_WP_loc);
    }

}

bool ModeCruise::get_target_heading_cd(int32_t &target_heading)
{
    target_heading = locked_heading_cd;
    return locked_heading;
}

bool ModeCruise::change_target_heading_cd(int32_t delta_cd) {
    locked_heading_cd = wrap_360_cd(locked_heading_cd + delta_cd);
    plane.prev_WP_loc = plane.current_loc;
    locked_heading = true;
    lock_timer_ms = 0;
    _control_state = CruiseState::OFFBOARD;
    return true;
}

bool ModeCruise::change_target_altitude_cm(int32_t delta_cm) {
    plane.change_target_altitude(delta_cm);
    return true;
}

void ModeCruise::set_state_1(ControlState_1 state_in) {
    cmd_state_1 = state_in;
    _last_ms_state_1 = millis();
    _control_state = CruiseState::OFFBOARD;
}

void ModeCruise::set_state_2(ControlState_2 state_in) {
    cmd_state_2 = state_in;
    _last_ms_state_2 = millis();
    _control_state = CruiseState::OFFBOARD;
}

void  ModeCruise::do_cmd_1() {
    set_state_1(ControlState_1::P1);
    reach_heading = false;
    locked_heading_cd = wrap_360_cd(locked_heading_cd + 4000);
}

void  ModeCruise::do_cmd_2() {
    set_state_2(ControlState_2::P1);
    reach_heading = false;
    locked_heading_cd = wrap_360_cd(locked_heading_cd + 6000);
}

void ModeCruise::update_1() {
    uint32_t delta_t = millis() - _last_ms_state_1;
    switch (cmd_state_1) {
        case ControlState_1::P1: // 右转40
            if (delta_t > 6000 || reach_heading) {
                set_state_1(ControlState_1::P2);
            }
            break;
        case ControlState_1::P2: // 飞5s
            if (delta_t > 5000) {
                set_state_1(ControlState_1::P3);
                reach_heading = false;
                locked_heading_cd = wrap_360_cd(locked_heading_cd - 6000);
            }
            break;
        case ControlState_1::P3: // 左转60
            if (delta_t > 8000 || reach_heading) {
                set_state_1(ControlState_1::P4);
            }
            break;
        case ControlState_1::P4: // 飞6s
            if (delta_t > 6000) {
                set_state_1(ControlState_1::P5);
                reach_heading = false;
                locked_heading_cd = wrap_360_cd(locked_heading_cd + 2000);
            }
            break;
        case ControlState_1::P5: // 右转20
            if (delta_t > 4000 || reach_heading) {
                set_state_1(ControlState_1::P0);
            }
            break;
        case ControlState_1::P0:
        default:
            break;
    }
}

void ModeCruise::update_2() {
    uint32_t delta_t = millis() - _last_ms_state_2;
    switch (cmd_state_2) {
        case ControlState_2::P1: // 右转60
            if (delta_t > 8000 || reach_heading) {
                set_state_2(ControlState_2::P2);
            }
            break;
        case ControlState_2::P2: // 飞3s
            if (delta_t > 3000) {
                set_state_2(ControlState_2::P3);
                reach_heading = false;
                locked_heading_cd = wrap_360_cd(locked_heading_cd - 12000);
            }
            break;
        case ControlState_2::P3: // 左转120
            if (delta_t > 14000 || reach_heading) {
                set_state_2(ControlState_2::P4);
            }
            break;
        case ControlState_2::P4: // 飞6s
            if (delta_t > 6000) {
                set_state_2(ControlState_2::P5);
                reach_heading = false;
                locked_heading_cd = wrap_360_cd(locked_heading_cd + 12000);
            }
            break;
        case ControlState_2::P5: // 右转120
            if (delta_t > 14000 || reach_heading) {
                set_state_2(ControlState_2::P6);
            }
            break;
        case ControlState_2::P6: // 飞3s
            if (delta_t > 3000) {
                set_state_2(ControlState_2::P7);
                reach_heading = false;
                locked_heading_cd = wrap_360_cd(locked_heading_cd - 6000);
            }
            break;
        case ControlState_2::P7: // 左转60
            if (delta_t > 8000 || reach_heading) {
                set_state_2(ControlState_2::P0);
            }
            break;
        case ControlState_2::P0:
        default:
            break;
    }
}
