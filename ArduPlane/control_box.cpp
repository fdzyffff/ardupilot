#include "Plane.h"

bool Plane::box_init() {
	box_info_init();
    box_build_loiter_standby();
    do_loiter_unlimited(box_info.cmd_loiter_standby);
    gcs().send_text(MAV_SEVERITY_WARNING, "BOX stand by");
	return true;
}

void Plane::box_info_init() {
	box_info.state = loiter_standby;
	box_info.box_location_recieved = false;
	box_info.path_build_allow = true;
}

void Plane::box_cruise_init() {
    throttle_allows_nudging = false;
    auto_throttle_mode = true;
    auto_navigation_mode = false;
                
    set_target_altitude_location(box_info.cmd_way_point_2.content.location);
}

bool Plane::box_start_check() {
    if (box_info.path_build_allow && box_info.box_location_recieved) {
        box_build_loiter_to_alt();
        box_build_way_point_1();
        box_build_way_point_2();
        box_build_way_point_4();
        return true;
    }
    return false;
}

void Plane::box_build_loiter_standby() {
    box_info.cmd_loiter_standby.content.location = current_loc;
    box_info.cmd_loiter_standby.content.location.flags.loiter_ccw = true;
}

void Plane::box_build_loiter_to_alt() {
    box_info.cmd_loiter_to_alt.content.location = box_info.box_location;

    box_info.cmd_loiter_to_alt.content.location.alt = box_info.box_location.alt + 2000;
    box_info.cmd_loiter_to_alt.content.location.flags.relative_alt = true;
    box_info.cmd_loiter_to_alt.content.location.flags.terrain_alt = true;
    box_info.cmd_loiter_to_alt.content.location.flags.origin_alt = true;
    box_info.cmd_loiter_to_alt.p1 = 0;

    location_update(box_info.cmd_loiter_to_alt.content.location,
                        box_get_heading(box_info.box_heading - 180.f),
                        200 );
    location_update(box_info.cmd_loiter_to_alt.content.location,
                        box_get_heading(box_info.box_heading - 90.f),
                        100 );
}

void Plane::box_build_way_point_1() {
    box_info.cmd_way_point_1.content.location = box_info.box_location;

    box_info.cmd_way_point_1.content.location.flags.relative_alt = true;
    box_info.cmd_way_point_1.content.location.flags.terrain_alt = true;
    box_info.cmd_way_point_1.content.location.flags.origin_alt = true;
    location_update(box_info.cmd_way_point_1.content.location,
                        box_get_heading(box_info.box_heading - 180.f),
                        200 );
    box_info.cmd_way_point_1.content.location.alt = box_info.box_location.alt + 1000;
}

void Plane::box_build_way_point_2() {
    box_info.cmd_way_point_2.content.location = box_info.box_location;

    box_info.cmd_way_point_2.content.location.flags.relative_alt = true;
    box_info.cmd_way_point_2.content.location.flags.terrain_alt = true;
    box_info.cmd_way_point_2.content.location.flags.origin_alt = true;
    location_update(box_info.cmd_way_point_2.content.location,
                        box_get_heading(box_info.box_heading - 180.f),
                        100 );
    box_info.cmd_way_point_2.content.location.alt = box_info.box_location.alt;
}

void Plane::box_build_way_point_4() {
    box_info.cmd_way_point_4.content.location = current_loc;

    box_info.cmd_way_point_4.content.location.flags.relative_alt = true;
    box_info.cmd_way_point_4.content.location.flags.terrain_alt = true;
    box_info.cmd_way_point_4.content.location.flags.origin_alt = true;
    location_update(box_info.cmd_way_point_4.content.location,
                        box_get_heading(box_info.box_heading),
                        100 );
    box_info.cmd_way_point_2.content.location.alt = box_info.box_location.alt + 2000;
}

void Plane::box_reset_cmd() {
    box_info.box_location_recieved = false;
    box_info.path_build_allow = false;
}

bool Plane::box_allow_receive() {
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
            result = verify_loiter_heading(true);
        }
    } else {
        // secondary goal, loiter to heading
        result = verify_loiter_heading(false);
    }

    if (result) {
        gcs().send_text(MAV_SEVERITY_INFO,"Loiter to alt complete");
    }
    return result;
}

void Plane::update_box_navigation() {
    switch (box_info.state) {
        case loiter_standby: {
            verify_loiter_unlim();
            if (box_start_check()) {
                box_info.state = loiter_to_alt;
                do_loiter_to_alt(box_info.cmd_loiter_to_alt);
                gcs().send_text(MAV_SEVERITY_WARNING, "BOX loiter to alt");
            }
            break;
        }
        case loiter_to_alt: {
            if (verify_loiter_to_alt_box(box_info.cmd_loiter_to_alt)) {
                box_info.state = way_point_1;
                do_nav_wp(box_info.cmd_way_point_1);
                gcs().send_text(MAV_SEVERITY_WARNING, "BOX way point 1");
            }
            break;
        }
        case way_point_1: {
            if (verify_nav_wp(box_info.cmd_way_point_1)) {
                box_info.state = way_point_2;
                do_nav_wp(box_info.cmd_way_point_2);
                gcs().send_text(MAV_SEVERITY_WARNING, "BOX way point 2");
            }
            break;
        }
        case way_point_2: {
            if (verify_nav_wp(box_info.cmd_way_point_2)) {
                box_info.state = box;
                box_cruise_init();
                gcs().send_text(MAV_SEVERITY_WARNING, "BOX boxing");
            }
            break;
        }
        case box: {
            update_cruise_box();
            if (verify_box_box()) {
                box_info.state = way_point_4;
                box_build_way_point_4();
                do_nav_wp(box_info.cmd_way_point_4);
                gcs().send_text(MAV_SEVERITY_WARNING, "BOX fail");
            }
            break;
        }
        case way_point_4: {
            if (verify_nav_wp(box_info.cmd_way_point_4)) {
                box_reset_cmd();
                box_info.state = loiter_standby;
                do_loiter_unlimited(box_info.cmd_loiter_standby);
                gcs().send_text(MAV_SEVERITY_WARNING, "BOX stand by");
            }
            break;
        }
    }
}


void Plane::handle_box_mode(void) {
    switch (box_info.state) {
        case loiter_standby:
        case loiter_to_alt:
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
        prev_WP_loc = box_info.cmd_way_point_2.content.location;
        next_WP_loc = box_info.box_location;
        // always look 100 ahead
        static int32_t box_curise_cd = 0;
        if (get_distance(prev_WP_loc, current_loc) < get_distance(prev_WP_loc, box_info.box_location) ) {
            box_curise_cd = get_bearing_cd(current_loc, box_info.box_location);
        }
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
