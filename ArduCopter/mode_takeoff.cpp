#include "Copter.h"

#if MODE_GUIDED_ENABLED == ENABLED

/*
 * Init and run calls for loiter flight mode
 */

// loiter_init - initialise loiter controller
bool ModeTakeoff::init(bool ignore_checks)
{
    _stage = 1;
    _takeoff_time = millis();
    if (copter.mode_guided.init(ignore_checks)) {
        copter.gcs().send_text(MAV_SEVERITY_WARNING, "mode takeoff");
        return true;
    }
    copter.gcs().send_text(MAV_SEVERITY_WARNING, "mode failed");
    return false;

}

// loiter_run - runs the loiter controller
// should be called at 100hz or more
void ModeTakeoff::run()
{
    // if (_stage == 1 ) {
    //     if ( (copter.rangefinder_alt_ok() && copter.rangefinder_state.alt_cm > 50.0f)
    //        ||(!copter.rangefinder_alt_ok() && copter.inertial_nav.get_altitude() > MAX(100.f, copter.g2.user_parameters.gcs_target_alt.get())) ) {
    //         const Vector3f& tmp_target = wp_nav->get_wp_destination();
    //         Vector3f target = Vector3f(tmp_target.x, tmp_target.y, copter.g2.user_parameters.gcs_target_alt.get());
    //         if (copter.mode_guided.set_destination(target)) {
    //             _stage = 2;
    //             copter.gcs().send_text(MAV_SEVERITY_WARNING, "takeoff stage 2");
    //         }
    //     }
    // }
    copter.mode_guided.run();
}

uint32_t ModeTakeoff::wp_distance() const
{
    return wp_nav->get_wp_distance_to_destination();
}

int32_t ModeTakeoff::wp_bearing() const
{
    return wp_nav->get_wp_bearing_to_destination();
}

#endif
