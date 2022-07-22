#include "Copter.h"

#if MODE_GUIDED_ENABLED == ENABLED

/*
 * Init and run calls for loiter flight mode
 */

// loiter_init - initialise loiter controller
bool ModeFly::init(bool ignore_checks)
{
    if (copter.mode_guided.init(false)) {
        _last_ms = millis();
        return true;
    }
    return false;

}

// loiter_run - runs the loiter controller
// should be called at 100hz or more
void ModeFly::run()
{
    if (copter.Ugcs.is_leader()) {
        if (copter.Ugcs.dest_pos_update()) {
            copter.mode_guided.set_destination(copter.Ugcs.get_dest_loc_vec());
            copter.Ugcs.dest_pos_update(false);
        }
    } else {
        if (millis() - _last_ms > 200) {
            copter.mode_guided.set_destination_posvel(copter.Ugcs.get_follow_loc_vec(), copter.Ugcs.get_follow_vel_vec(), true, copter.Ugcs.get_follow_yaw_cd(), false, 0.0f, false);
            _last_ms = millis();
        }
    }
    copter.mode_guided.run();
}

uint32_t ModeFly::wp_distance() const
{
    return wp_nav->get_wp_distance_to_destination();
}

int32_t ModeFly::wp_bearing() const
{
    return wp_nav->get_wp_bearing_to_destination();
}

#endif
