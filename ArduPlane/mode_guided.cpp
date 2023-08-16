#include "mode.h"
#include "Plane.h"

bool ModeGuided::_enter()
{
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
        loc.offset_bearing(degrees(plane.ahrs.groundspeed_vector().angle()),
                           plane.quadplane.stopping_distance());
    }
#endif

    plane.set_guided_WP(loc);
    return true;
}

void ModeGuided::update()
{
#if HAL_QUADPLANE_ENABLED
    if (plane.auto_state.vtol_loiter && plane.quadplane.available()) {
        plane.quadplane.guided_update();
        return;
    }
#endif
    plane.calc_nav_roll();
    plane.calc_nav_pitch();
    plane.calc_throttle();
}

void ModeGuided::navigate()
{
    // Zero indicates to use WP_LOITER_RAD
    plane.update_loiter(0);

    update_follow();
}

bool ModeGuided::handle_guided_request(Location target_loc)
{
    // add home alt if needed
    if (target_loc.relative_alt) {
        target_loc.alt += plane.home.alt;
        target_loc.relative_alt = 0;
    }

    plane.set_guided_WP(target_loc);

    return true;
}

void ModeGuided::update_follow()
{
    if (!plane.ufollow.is_active()) {
        plane.prev_WP_loc = plane.current_loc;
        return;
    }
    Location target_loc = plane.current_loc;
    plane.ufollow.get_target_pos(target_loc);
    float ufollow_dir = plane.ufollow.get_target_bearing();
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
    float target_spd = plane.ufollow.get_target_vel()*100.f + constrain_float(delta_dist*spd_kp*100.f, -plane.g2.follow_speed_range*100.f, plane.g2.follow_speed_range*100.f);
    plane.aparm.airspeed_cruise_cm.set(target_spd);
}
