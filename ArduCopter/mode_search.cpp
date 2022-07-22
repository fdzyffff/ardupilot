#include "Copter.h"

#if MODE_GUIDED_ENABLED == ENABLED

/*
 * Init and run calls for loiter flight mode
 */

// loiter_init - initialise loiter controller
bool ModeSearch::init(bool ignore_checks)
{
    if (copter.mode_guided.init(false)) {
        if (copter.mode_guided.set_destination(copter.Ugcs.get_search_dest())) {
            copter.gcs().send_text(MAV_SEVERITY_WARNING, "mode search");
            return true;
        }
    }
    return false;

}

// loiter_run - runs the loiter controller
// should be called at 100hz or more
void ModeSearch::run()
{
    if (copter.Utarget.is_active()) {
        copter.Ugcs.do_lockon();
    }
    copter.mode_guided.run();
}

uint32_t ModeSearch::wp_distance() const
{
    return wp_nav->get_wp_distance_to_destination();
}

int32_t ModeSearch::wp_bearing() const
{
    return wp_nav->get_wp_bearing_to_destination();
}

#endif
