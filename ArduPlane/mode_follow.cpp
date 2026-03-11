#include "mode.h"
#include "Plane.h"

bool ModeFollow::_enter()
{
    if (!plane.g2.follow.enabled()) {
        gcs().send_text(MAV_SEVERITY_INFO, "Set FOLL_ENABLE and reboot");
        gcs().send_text(MAV_SEVERITY_INFO, "Set FOLL_ENABLE and reboot");
        gcs().send_text(MAV_SEVERITY_INFO, "Set FOLL_ENABLE and reboot");
        return false;
    }

    if (plane.g2.follow.have_target()) {
        gcs().send_text(MAV_SEVERITY_INFO, "No Follow Target");
        return false;
    }

    plane.guided_throttle_passthru = false;
    /*
      when entering guided mode we set the target as the current
      location. This matches the behaviour of the copter code
    */
    Location loc{plane.current_loc};

#if HAL_QUADPLANE_ENABLED
    if (plane.quadplane.guided_mode_enabled()) {
        /*
          if using Q_GUIDED_MODE then project forward by the stopping distance
        */
        loc.offset_bearing(degrees(ahrs.groundspeed_vector().angle()),
                           plane.quadplane.stopping_distance());
    }
#endif

    plane.set_guided_WP(loc);
    _target_speed = plane.aparm.airspeed_cruise.get()*100.f;;
    return true;
}

void ModeFollow::update()
{
    plane.calc_nav_roll();
    plane.calc_nav_pitch();
    plane.calc_throttle();
}

void ModeFollow::navigate()
{
    // Zero indicates to use WP_LOITER_RAD
    plane.update_loiter(0);

    update_follow();
}

void ModeFollow::update_follow()
{
    bool valid_target = false;
    if (plane.g2.follow.have_target()) {
        if (plane.g2.follow.get_target_location_and_velocity(_raw_target_loc, _target_vel)) {
            if (plane.g2.follow.get_target_heading_deg(_target_bearing)) {
                valid_target = true;
                _raw_target_loc.change_alt_frame(Location::AltFrame::ABSOLUTE);
            }
        }
    }
    if (!valid_target) {
        gcs().send_text(MAV_SEVERITY_INFO, "Follow MSG Lost!");
    }

    Location target_loc = _raw_target_loc;
    float ufollow_dir = _target_bearing;
    float target_dist = target_loc.get_distance(plane.current_loc);
    float length_cut = 500.0f; //meter
    float vel_length = 0.f;
    if (target_dist > length_cut) {
        plane.prev_WP_loc = plane.current_loc;
        plane.next_WP_loc = target_loc;
        plane.auto_state.crosstrack = false;
        vel_length = length_cut * 0.7f;
    } else {
        plane.prev_WP_loc = target_loc;
        plane.next_WP_loc = target_loc;
        plane.next_WP_loc.offset_bearing(ufollow_dir, 500.f);
        plane.auto_state.crosstrack = true;
        vel_length = 500.f;
    }
    float delta_dist = (plane.next_WP_loc.get_distance(plane.current_loc) - vel_length);
    float spd_kp = plane.g2.follow_speed_ratio.get();
    _target_speed = plane.aparm.airspeed_cruise.get()*100.f + constrain_float(delta_dist*spd_kp*100.f, -plane.g2.follow_speed_range*100.f, plane.g2.follow_speed_range*100.f);
}
