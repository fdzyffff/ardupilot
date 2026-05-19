#include "mode.h"
#include "Plane.h"

bool ModeQTakeoff::_enter()
{
#if HAL_QUADPLANE_ENABLED
    if (plane.quadplane.is_flying()) {
        gcs().send_text(MAV_SEVERITY_INFO, "Already flying - no takeoff");
        return false;
    }
    // set vertical speed and acceleration limits
    _cmd.id = MAV_CMD_NAV_TAKEOFF;
    _cmd.content.location = plane.current_loc;
    uint32_t target_alt = MAX(plane.quadplane.takeoff_q_alt, 1.0f) * 100.f;
    if (_cmd.content.location.change_alt_frame(Location::AltFrame::ABOVE_HOME)) {
        _cmd.content.location.set_alt_cm(target_alt, Location::AltFrame::ABOVE_HOME);
    } else {
        return false;
    }

    // reset takeoff if we aren't armed
    plane.quadplane.do_vtol_takeoff(_cmd);
    gcs().send_text(MAV_SEVERITY_INFO, "Vtol Q Takeoff");
    return true;
#else
    return false;
#endif
}

void ModeQTakeoff::_exit()
{
    ;
}

void ModeQTakeoff::update()
{
    // plane.quadplane.throttle_wait = false;
    // plane.quadplane.set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
    plane.quadplane.takeoff_controller();
    return;
}

void ModeQTakeoff::navigate()
{
    // reset takeoff if we aren't armed
    if (!plane.arming.is_armed_and_safety_off()) {
        plane.quadplane.do_vtol_takeoff(_cmd);
    }
}
