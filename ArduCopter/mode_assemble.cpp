#include "Copter.h"

#if MODE_GUIDED_ENABLED == ENABLED

/*
 * Init and run calls for loiter flight mode
 */

// loiter_init - initialise loiter controller
bool ModeAssemble::init(bool ignore_checks)
{
    if (copter.mode_guided.init(false)) {
        if (copter.mode_guided.set_destination(copter.Ugcs.get_assemble_dest(), false, 0.0f, false, 0.0f, false, true)) {
            copter.gcs().send_text(MAV_SEVERITY_WARNING, "mode assemble T");
            return true;
        } else {
            if (copter.mode_guided.set_destination(copter.Ugcs.get_assemble_dest(), false, 0.0f, false, 0.0f, false, false)) {
                copter.gcs().send_text(MAV_SEVERITY_WARNING, "mode assemble");
                return true;
            }
        }
    }
    return false;

}

// loiter_run - runs the loiter controller
// should be called at 100hz or more
void ModeAssemble::run()
{
    copter.mode_guided.run();

    if (reached_position()) {
        copter.set_mode(Mode::Number::LOCKON, ModeReason::GCS_COMMAND);
    }

}

bool ModeAssemble::reached_position() {
    static uint32_t _last_ms = millis();
    if (wp_distance() > 100.f) {
        _last_ms = millis();
    } else {
        if (millis() - _last_ms > 3000) {
            return true;
        }
    }
    return false;
}

uint32_t ModeAssemble::wp_distance() const
{
    return wp_nav->get_wp_distance_to_destination();
}

int32_t ModeAssemble::wp_bearing() const
{
    return wp_nav->get_wp_bearing_to_destination();
}

#endif
