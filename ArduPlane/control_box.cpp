#include "Plane.h"

bool Plane::box_init() {
    if (g2.box_option == 0) {
        box_info_init();
        box_build_loiter_standby();
        do_loiter_unlimited(box_info_cmd_loiter_standby);
        gcs().send_text(MAV_SEVERITY_WARNING, "BOX stand by");
        return true;
    } else if (box_info.box_location_recieved) {
        box_info_init_option1();
        return true;
    }
    return false;
}

void Plane::box_info_init() {
	box_info.state = loiter_standby;
	box_info.box_location_recieved = false;
	box_info.path_build_allow = true;
}

void Plane::box_info_init_option1() {
    box_info.box_heading = box_info.box_heading_in;
    box_build_loiter_standby();
    box_info_cmd_loiter_standby.content.location.alt = 5000;
    box_info_cmd_loiter_standby.content.location.flags.relative_alt = true;
    box_info_cmd_loiter_standby.content.location.flags.terrain_alt = false;
    box_info_cmd_loiter_standby.content.location.flags.origin_alt = false;

    box_build_loiter_to_alt();
    box_build_way_point_0();
    box_build_way_point_1();
    box_build_way_point_2();
    box_build_way_point_4();

    box_info.state = way_point_2;
    auto_state.next_wp_crosstrack = true;
    do_nav_wp(box_info_cmd_way_point_2);

    prev_WP_loc = box_info_cmd_way_point_2.content.location;
    prev_WP_loc.alt = box_info_cmd_way_point_2.content.location.alt;
    location_update(prev_WP_loc,
                        box_get_heading(box_info.box_heading - 180.f),
                        1000.f);
    gcs().send_text(MAV_SEVERITY_WARNING, "BOX to way point 2 direct");
}

void Plane::box_cruise_init() {
    throttle_allows_nudging = false;
    auto_throttle_mode = true;
    auto_navigation_mode = false;
                
    set_target_altitude_location(box_info.box_location);
}

bool Plane::box_in_cruise() {
    return (control_mode == BOX && box_info.state == box);
}

bool Plane::box_start_check() {
    if (box_info.path_build_allow && box_info.box_location_recieved) {
        box_info.box_heading = box_info.box_heading_in;
        box_build_loiter_to_alt();
        box_build_way_point_0();
        box_build_way_point_1();
        box_build_way_point_2();
        box_build_way_point_4();
        return true;
    }
    return false;
}

void Plane::box_build_loiter_standby() {
    box_info_cmd_loiter_standby.content.location = current_loc;
    box_info_cmd_loiter_standby.content.location.flags.loiter_ccw = true;


    box_info_cmd_loiter_to_alt.content.location = current_loc;
    box_info_cmd_loiter_to_alt.content.location.flags.loiter_ccw = true;
}

void Plane::box_build_loiter_to_alt() {
    box_info_cmd_loiter_to_alt.content.location.alt = box_info.box_location.alt + 2000;
    box_info_cmd_loiter_to_alt.content.location.flags.relative_alt = true;
    box_info_cmd_loiter_to_alt.content.location.flags.terrain_alt = false;
    box_info_cmd_loiter_to_alt.content.location.flags.origin_alt = false;
    box_info_cmd_loiter_to_alt.p1 = 0;
}

void Plane::box_build_way_point_0() {
    box_info_cmd_way_point_0.content.location = box_info.box_location;

    box_info_cmd_way_point_0.content.location.alt = box_info.box_location.alt + 2000;
    box_info_cmd_way_point_0.content.location.flags.relative_alt = true;
    box_info_cmd_way_point_0.content.location.flags.terrain_alt = false;
    box_info_cmd_way_point_0.content.location.flags.origin_alt = false;

    location_update(box_info_cmd_way_point_0.content.location,
                        box_get_heading(box_info.box_heading - 180.f),
                        g2.box_offset_back );
    location_update(box_info_cmd_way_point_0.content.location,
                        box_get_heading(box_info.box_heading - 90.f),
                        g2.box_offset_left );
}

void Plane::box_build_way_point_1() {
    box_info_cmd_way_point_1.content.location = box_info.box_location;

    box_info_cmd_way_point_1.content.location.alt = box_info.box_location.alt + 2000;
    box_info_cmd_way_point_1.content.location.flags.relative_alt = true;
    box_info_cmd_way_point_1.content.location.flags.terrain_alt = false;
    box_info_cmd_way_point_1.content.location.flags.origin_alt = false;

    location_update(box_info_cmd_way_point_1.content.location,
                        box_get_heading(box_info.box_heading - 180.f),
                        g2.box_offset_back);
    location_update(box_info_cmd_way_point_1.content.location,
                        box_get_heading(box_info.box_heading - 90.f),
                        g2.box_offset_left_wp1 );
}

void Plane::box_build_way_point_2() {
    box_info_cmd_way_point_2.content.location = box_info.box_location;

    box_info_cmd_way_point_2.content.location.alt = box_info.box_location.alt;
    box_info_cmd_way_point_2.content.location.flags.relative_alt = true;
    box_info_cmd_way_point_2.content.location.flags.terrain_alt = false;
    box_info_cmd_way_point_2.content.location.flags.origin_alt = false;

    location_update(box_info_cmd_way_point_2.content.location,
                        box_get_heading(box_info.box_heading - 180.f),
                        50 );
}

void Plane::box_build_way_point_4() {
    box_info_cmd_way_point_4.content.location = current_loc;

    box_info_cmd_way_point_4.content.location.alt = box_info.box_location.alt + 2000;
    box_info_cmd_way_point_4.content.location.flags.relative_alt = true;
    box_info_cmd_way_point_4.content.location.flags.terrain_alt = false;
    box_info_cmd_way_point_4.content.location.flags.origin_alt = false;

    location_update(box_info_cmd_way_point_4.content.location,
                        box_get_heading(box_info.box_heading),
                        100 );
}

void Plane::box_reset_cmd() {
    box_info.box_location_recieved = false;
    box_info.path_build_allow = false;
}

bool Plane::box_allow_receive() {
    if (g2.box_option == 1) {
        return true;
    }

    if (control_mode != BOX) {
        return false;
    }

    switch (box_info.state) {
        case way_point_4: {
            return false ;
            break;
        }
        default : {
            return true;
            break;
        }
    }
    return false;
}

bool Plane::verify_box_box() {
    if ( (get_distance(prev_WP_loc, current_loc) - get_distance(prev_WP_loc, box_info.box_location) ) > 30) {
        return true;
    }
    return false;
}

bool Plane::verify_loiter_to_alt_box(const AP_Mission::Mission_Command& cmd) 
{
    bool result = false;
    update_loiter(cmd.p1);

    // condition_value == 0 means alt has never been reached
    if (condition_value == 0) {
        // primary goal, loiter to alt
        if (labs(loiter.sum_cd) > 1 && (loiter.reached_target_alt || loiter.unable_to_acheive_target_alt)) {
            // primary goal completed, initialize secondary heading goal
            if (loiter.unable_to_acheive_target_alt) {
                gcs().send_text(MAV_SEVERITY_INFO,"Loiter to alt was stuck at %d", current_loc.alt/100);
            }

            condition_value = 1;
        }
    } else {
        // secondary goal, loiter to heading
        result = verify_loiter_heading_box(box_info_cmd_way_point_0);
    }

    if (result) {
        gcs().send_text(MAV_SEVERITY_INFO,"Loiter to alt complete");
    }
    return result;
}

bool Plane::verify_loiter_heading_box(const AP_Mission::Mission_Command& next_nav_cmd) {
    if (quadplane.in_vtol_auto()) {
        // skip heading verify if in VTOL auto
        return true;
    }

    if (get_distance(next_WP_loc, next_nav_cmd.content.location) < abs(aparm.loiter_radius)) {
        /* Whenever next waypoint is within the loiter radius,
           maintaining loiter would prevent us from ever pointing toward the next waypoint.
           Hence break out of loiter immediately
         */
        return true;
    }

    // Bearing in degrees
    int32_t bearing_cd = get_bearing_cd(current_loc,next_nav_cmd.content.location);

    // get current heading.
    int32_t heading_cd = gps.ground_course_cd();

    int32_t heading_err_cd = wrap_180_cd(bearing_cd - heading_cd);

    /*
      Check to see if the the plane is heading toward the land
      waypoint. We use 20 degrees (+/-10 deg) of margin so that
      we can handle 200 degrees/second of yaw.

      After every full circle, extend acceptance criteria to ensure
      aircraft will not loop forever in case high winds are forcing
      it beyond 200 deg/sec when passing the desired exit course
    */

    // Use integer division to get discrete steps
    int32_t expanded_acceptance = 1000 * (loiter.sum_cd / 36000);

    if (labs(heading_err_cd) <= 1000 + expanded_acceptance) {
        // Want to head in a straight line from _here_ to the next waypoint instead of center of loiter wp

        // 0 to xtrack from center of waypoint, 1 to xtrack from tangent exit location
        if (next_WP_loc.flags.loiter_xtrack) {
            next_WP_loc = current_loc;
        }
        return true;
    }
    return false;
}

void Plane::update_box_navigation() {
    switch (box_info.state) {
        case loiter_standby: {
            verify_loiter_unlim();
            if (g2.box_option == 0 && box_start_check()) {
                box_info.state = loiter_to_alt;
                do_loiter_to_alt(box_info_cmd_loiter_to_alt);
                gcs().send_text(MAV_SEVERITY_WARNING, "BOX to loiter to alt");
            }
            break;
        }
        case loiter_to_alt: {
            if (verify_loiter_to_alt_box(box_info_cmd_loiter_to_alt)) {
                box_info.state = way_point_0;
                auto_state.next_wp_crosstrack = false;
                do_nav_wp(box_info_cmd_way_point_0);
                gcs().send_text(MAV_SEVERITY_WARNING, "BOX to way point 0");
            }
            break;
        }
        case way_point_0: {
            if (verify_nav_wp(box_info_cmd_way_point_0)) {
                box_info.state = way_point_1;
                auto_state.next_wp_crosstrack = false;
                do_nav_wp(box_info_cmd_way_point_1);
                gcs().send_text(MAV_SEVERITY_WARNING, "BOX to way point 1");
            }
            break;
        }
        
        case way_point_1: {
            if (verify_nav_wp(box_info_cmd_way_point_1)) {
                box_info.state = way_point_2;
                auto_state.next_wp_crosstrack = true;
                do_nav_wp(box_info_cmd_way_point_2);
                location_update(prev_WP_loc,
                        box_get_heading(box_info.box_heading + 90.f),
                        g2.box_offset_left_wp1 );
/*                location_update(prev_WP_loc,
                        box_get_heading(box_info.box_heading - 180.f),
                        200 );*/
                gcs().send_text(MAV_SEVERITY_WARNING, "BOX to way point 2");
            }
            break;
        }
        case way_point_2: {
            if (verify_nav_wp(box_info_cmd_way_point_2) || get_distance(current_loc, box_info.box_location) < 40) {
                box_info.state = box;
                auto_state.next_wp_crosstrack = true;
                box_cruise_init();
                prev_WP_loc = box_info_cmd_way_point_1.content.location;
                prev_WP_loc.alt = box_info.box_location.alt;
                location_update(prev_WP_loc,
                        box_get_heading(box_info.box_heading + 90.f),
                        g2.box_offset_left_wp1 );
                gcs().send_text(MAV_SEVERITY_WARNING, "BOX in boxing");
            }
            break;
        }
        case box: {
            update_cruise_box();
            if (verify_box_box()) {
                box_info.state = way_point_4;
                box_build_way_point_4();
                do_nav_wp(box_info_cmd_way_point_4);
                gcs().send_text(MAV_SEVERITY_WARNING, "BOX fail");
            }
            break;
        }
        case way_point_4: {
            if (verify_nav_wp(box_info_cmd_way_point_4)) {
                box_reset_cmd();
                box_info.state = loiter_standby;
                do_loiter_unlimited(box_info_cmd_loiter_standby);
                gcs().send_text(MAV_SEVERITY_WARNING, "BOX to stand by");
            }
            break;
        }
    }
}


void Plane::handle_box_mode(void) {
    switch (box_info.state) {
        case loiter_standby:
        case loiter_to_alt:
        case way_point_0:
        case way_point_1:
        case way_point_2:
        case way_point_4: {
            calc_nav_roll();
            calc_nav_pitch();
            calc_throttle();
            break;
        }
        case box: {
            calc_nav_roll();
            update_fbwb_speed_height();
            break;
        }
    }
}

void Plane::update_cruise_box() {
        next_WP_loc = box_info.box_location;
        // always look 100 ahead
        static int32_t box_curise_cd = 0;
        //if (get_distance(prev_WP_loc, current_loc) < get_distance(prev_WP_loc, box_info.box_location) ) {
        box_curise_cd = get_bearing_cd(prev_WP_loc, box_info.box_location);
        //}
        location_update(next_WP_loc,
                        box_curise_cd*0.01f, 100);
        
        nav_controller->update_waypoint(prev_WP_loc, next_WP_loc);
}

float Plane::box_get_heading(float in_bearing) {
	int16_t n_bearing = 0;
	if (in_bearing > 0.f) {
		n_bearing = int32_t(in_bearing) / 360;
	} else {
		n_bearing = int32_t(in_bearing) / 360;
		n_bearing = n_bearing - 1;
	}
    in_bearing = in_bearing - float(n_bearing) * 360.0f;
    in_bearing = constrain_float(in_bearing, 0.0f, 360.0f);
    return in_bearing;
}
