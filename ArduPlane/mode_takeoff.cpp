#include "mode.h"
#include "Plane.h"

/*
  mode takeoff parameters
 */
const AP_Param::GroupInfo ModeTakeoff::var_info[] = {
    // @Param: ALT
    // @DisplayName: Takeoff mode altitude
    // @Description: This is the target altitude for TAKEOFF mode
    // @Range: 0 200
    // @Increment: 1
    // @Units: m
    // @User: User
    AP_GROUPINFO("ALT", 1, ModeTakeoff, target_alt, 70),

    // @Param: LVL_ALT
    // @DisplayName: Takeoff mode altitude level altitude
    // @Description: This is the altitude below which wings are held level for TAKEOFF mode
    // @Range: 0 50
    // @Increment: 1
    // @Units: m
    // @User: User
    AP_GROUPINFO("LVL_ALT", 2, ModeTakeoff, level_alt, 50),

    // @Param: LVL_PITCH
    // @DisplayName: Takeoff mode altitude initial pitch
    // @Description: This is the target pitch for the initial climb to TKOFF_LVL_ALT
    // @Range: 0 30
    // @Increment: 1
    // @Units: deg
    // @User: User
    AP_GROUPINFO("LVL_PITCH", 3, ModeTakeoff, level_pitch, 35),

    // @Param: DIST
    // @DisplayName: Takeoff mode distance
    // @Description: This is the distance from the takeoff location where the plane will loiter. The loiter point will be in the direction of takeoff (the direction the plane is facing when the motor starts)
    // @Range: 0 500
    // @Increment: 1
    // @Units: m
    // @User: User
    AP_GROUPINFO("DIST", 4, ModeTakeoff, target_dist, 200),
    
    AP_GROUPEND
};

ModeTakeoff::ModeTakeoff()
{
    AP_Param::setup_object_defaults(this, var_info);
}

bool ModeTakeoff::_enter()
{
    // the altitude to circle at is taken from the current altitude
    plane.throttle_allows_nudging = true;
    plane.auto_throttle_mode = true;
    plane.auto_navigation_mode = true;

    takeoff_started = false;

    return true;
}

void ModeTakeoff::update()
{
    if (!takeoff_started  && (plane.takeoff_state.accel_event_counter == 0 )) {
        // see if we will skip takeoff as already flying
        if (plane.is_flying() && plane.ahrs.groundspeed() > 15) {
            gcs().send_text(MAV_SEVERITY_INFO, "Takeoff skipped - circling");
            plane.prev_WP_loc = plane.current_loc;
            plane.next_WP_loc = plane.current_loc;
            takeoff_started = true;
            plane.set_flight_stage(AP_Vehicle::FixedWing::FLIGHT_NORMAL);
            if (plane.HB1_status_noGPS_check()) {
                plane.HB1_status_set_HB_Mission_Action(Plane::HB1_Mission_FsNoGPS);
            } else {
                plane.HB1_status_set_HB_Mission_Action(Plane::HB1_Mission_WP);
                plane.mission.set_current_cmd(1);
            }
        }
    }

    if (!takeoff_started) {
        // setup target location 1.5 times loiter radius from the
        // takeoff point, at a height of TKOFF_ALT
        const float dist = target_dist;
        const float alt = target_alt;
        const float direction = degrees(plane.ahrs.yaw);

        start_loc = plane.current_loc;
        plane.prev_WP_loc = plane.current_loc;
        plane.next_WP_loc = plane.current_loc;
        plane.next_WP_loc.alt += alt*100.0;
        plane.next_WP_loc.offset_bearing(direction, dist);

        plane.crash_state.is_crashed = false;

        plane.auto_state.takeoff_pitch_cd = level_pitch * 100;

        plane.set_flight_stage(AP_Vehicle::FixedWing::FLIGHT_TAKEOFF);

        if (!plane.throttle_suppressed) {
            gcs().send_text(MAV_SEVERITY_INFO, "Takeoff to %.0fm at %.1fm to %.1f deg",
                            alt, dist, direction);
            takeoff_started = true;
        }
    }

    // we finish the initial level takeoff if we climb past
    // TKOFF_LVL_ALT or we pass the target location. The check for
    // target location prevents us flying forever if we can't climb
    if (plane.flight_stage == AP_Vehicle::FixedWing::FLIGHT_TAKEOFF &&
        (plane.current_loc.alt - start_loc.alt >= level_alt*100)) {
        // reached level alt, re-calculate bearing to cope with systems with no compass
        // or with poor initial compass
        plane.set_flight_stage(AP_Vehicle::FixedWing::FLIGHT_NORMAL);
        plane.complete_auto_takeoff();
        if (plane.HB1_status_noGPS_check()) {
            plane.HB1_status_set_HB_Mission_Action(Plane::HB1_Mission_FsNoGPS);
        } else {
            plane.HB1_status_set_HB_Mission_Action(Plane::HB1_Mission_WP);
            plane.mission.set_current_cmd(1);
        }
    }

    if (plane.flight_stage == AP_Vehicle::FixedWing::FLIGHT_TAKEOFF) {
        SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, 100);
        plane.takeoff_calc_roll();
        //plane.takeoff_calc_pitch();
        plane.nav_pitch_cd = level_pitch * 100;
    } else {
        plane.nav_roll_cd  = plane.roll_limit_cd / 3;
        plane.update_load_factor();
        plane.calc_nav_pitch();
        plane.calc_throttle();
    }
}

