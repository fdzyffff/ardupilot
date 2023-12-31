#include "Copter.h"

#if MODE_GUIDED_ENABLED == ENABLED

/*
 * Init and run calls for guided flight mode
 */

// init - initialise guided controller
bool ModeMYFOLLOW::init(bool ignore_checks)
{
    if (!g2.follow.enabled()) {
        gcs().send_text(MAV_SEVERITY_WARNING, "Set FOLL_ENABLE = 1");
        return false;
    }

    // initialise waypoint and spline controller
    wp_nav->wp_and_spline_init();

    if (copter.mode_guided.init(ignore_checks)) {
        myfollow_mode = SubMode::STANDBY;
        // copter.ufollow.set_role(0);
        gcs().send_text(MAV_SEVERITY_WARNING, "In Group Mode");
        return true;
    }
    return false;
}

// run - runs the guided controller
// should be called at 100hz or more
void ModeMYFOLLOW::run()
{
    // call the correct auto controller
    switch (myfollow_mode) {

    case SubMode::TAKEOFF:
        // run takeoff controller
        takeoff_run();
        if (takeoff_complete) {
            if (copter.ufollow.get_role() == 0) {
                standby_start();
            } else {
                loiter_start();
            }
        }
        break;

    case SubMode::STANDBY:
        // run position controller
        if (!is_disarmed_or_landed() && g2.follow.have_target()) {
            follow_start();
        } else {
            copter.mode_guided.run();
        }
        break;

    case SubMode::LOITER:
        // run position controller
        copter.mode_loiter.run();
        break;

    case SubMode::FOLLOW:
        copter.mode_follow.run();
        break;
    }
}

void ModeMYFOLLOW::exit()
{
    ;
}

bool ModeMYFOLLOW::allows_arming(AP_Arming::Method method) const
{
    // always allow arming from the ground station
    // if (method == AP_Arming::Method::MAVLINK) {
    //     return true;
    // }
    return true;
};

// initialises position controller to implement take-off
// takeoff_alt_cm is interpreted as alt-above-home (in cm) or alt-above-terrain if a rangefinder is available
bool ModeMYFOLLOW::do_user_takeoff_start(float takeoff_alt_cm)
{
    // calculate target altitude and frame (either alt-above-ekf-origin or alt-above-terrain)
    int32_t alt_target_cm;
    bool alt_target_terrain = false;
    if (wp_nav->rangefinder_used_and_healthy() &&
        wp_nav->get_terrain_source() == AC_WPNav::TerrainSource::TERRAIN_FROM_RANGEFINDER &&
        takeoff_alt_cm < copter.rangefinder.max_distance_cm_orient(ROTATION_PITCH_270)) {
        // can't takeoff downwards
        if (takeoff_alt_cm <= copter.rangefinder_state.alt_cm) {
            return false;
        }
        // provide target altitude as alt-above-terrain
        alt_target_cm = takeoff_alt_cm;
        alt_target_terrain = true;
    } else {
        // interpret altitude as alt-above-home
        Location target_loc = copter.current_loc;
        target_loc.set_alt_cm(takeoff_alt_cm, Location::AltFrame::ABOVE_HOME);

        // provide target altitude as alt-above-ekf-origin
        if (!target_loc.get_alt_cm(Location::AltFrame::ABOVE_ORIGIN, alt_target_cm)) {
            // this should never happen but we reject the command just in case
            return false;
        }
    }

    myfollow_mode = SubMode::TAKEOFF;

    // initialise yaw
    auto_yaw.set_mode(AutoYaw::Mode::HOLD);

    // clear i term when we're taking off
    pos_control->init_z_controller();

    // initialise alt for WP_NAVALT_MIN and set completion alt
    auto_takeoff_start(alt_target_cm, alt_target_terrain);

    // record takeoff has not completed
    takeoff_complete = false;

    gcs().send_text(MAV_SEVERITY_INFO, "[MODE Takeoff]");
    return true;
}

void ModeMYFOLLOW::standby_start()
{
    if (copter.mode_guided.init(false)) {
        myfollow_mode = SubMode::STANDBY;
        gcs().send_text(MAV_SEVERITY_INFO, "[MODE Standby]");
    } else {
        gcs().send_text(MAV_SEVERITY_INFO, "[MODE Standby Failed]");
    }
    g2.follow.clear_offsets_if_required();
}

void ModeMYFOLLOW::loiter_start()
{
    if (copter.mode_loiter.init(false)) {
        myfollow_mode = SubMode::LOITER;
        gcs().send_text(MAV_SEVERITY_INFO, "[MODE Loiter]");
        copter.ufollow.set_role(1);
    } else {
        gcs().send_text(MAV_SEVERITY_INFO, "[MODE Loiter Failed]");
    }
    g2.follow.clear_offsets_if_required();
}

void ModeMYFOLLOW::follow_start()
{
    if (copter.mode_follow.init(false)) {
        myfollow_mode = SubMode::FOLLOW;
        gcs().send_text(MAV_SEVERITY_INFO, "[MODE Follow]");
    } else {
        gcs().send_text(MAV_SEVERITY_INFO, "[MODE Follow Failed]");
    }
}

// takeoff_run - takeoff in guided mode
//      called by guided_run at 100hz or more
void ModeMYFOLLOW::takeoff_run()
{
    if (!copter.ap.auto_armed && (copter.ufollow.get_role() == 0)) {
        copter.set_auto_armed(true);
    }
    auto_takeoff_run();
    if (auto_takeoff_complete && !takeoff_complete) {
        takeoff_complete = true;
#if AP_LANDINGGEAR_ENABLED
        // optionally retract landing gear
        copter.landinggear.retract_after_takeoff();
#endif
    }
}

bool ModeMYFOLLOW::is_taking_off() const
{
    return myfollow_mode == SubMode::TAKEOFF && !takeoff_complete;
}

uint32_t ModeMYFOLLOW::wp_distance() const
{
    switch(myfollow_mode) {
    case SubMode::FOLLOW:
        return copter.mode_follow.wp_distance();
        break;
    default:
        return 0;
    }
    // compiler guarantees we don't get here
    return 0;
}

int32_t ModeMYFOLLOW::wp_bearing() const
{
    switch(myfollow_mode) {
    case SubMode::FOLLOW:
        return copter.mode_follow.wp_bearing();
        break;
    default:
        // these do not have bearings
        return 0;
    }
    // compiler guarantees we don't get here
    return 0;
}

float ModeMYFOLLOW::crosstrack_error() const
{
    switch (myfollow_mode) {
    case SubMode::FOLLOW:
        return copter.mode_follow.crosstrack_error();
        break;
    case SubMode::TAKEOFF:
        return pos_control->crosstrack_error();
        break;
    default:
        // no track to have a crosstrack to
        return 0;
    }
    // compiler guarantees we don't get here
    return 0;
}

/*
  get target position for mavlink reporting
 */
bool ModeMYFOLLOW::get_wp(Location &loc) const
{
    switch (myfollow_mode) {
    case SubMode::FOLLOW:
        return copter.mode_follow.get_wp(loc);
        break;
    default:
        // no track to have a crosstrack to
        loc = copter.current_loc;
        return true;
        break;
    }
    // compiler guarantees we don't get here
    return false;
}

#endif
