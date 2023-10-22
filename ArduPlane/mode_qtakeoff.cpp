#include "mode.h"
#include "Plane.h"

bool ModeQTakeoff::_enter()
{
#if HAL_QUADPLANE_ENABLED

    // set vertical speed and acceleration limits
    quadplane.do_user_takeoff(40.0f);
    gcs().send_text(MAV_SEVERITY_INFO, "tk");
    plane.quadplane.poscontrol.set_state(QuadPlane::QPOS_POSITION2);
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
    plane.quadplane.throttle_wait = false;
    plane.quadplane.set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
    plane.quadplane.takeoff_controller();
    return;
}

void ModeQTakeoff::navigate()
{
    ;// verify
}
