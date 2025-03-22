#include "Copter.h"

UFence::UFence()
{
    ;
}
    
void UFence::init()
{
    _triggered = false;
    _last_loc = copter.current_loc;
}

void UFence::update()
{
    if (copter.g2.user_parameters.fence_mode.get() == 0) {return;}
    if (!copter.position_ok() || !copter.motors->armed()) {
        _last_loc = copter.current_loc;
        _triggered = false;
    }
    float dist = _last_loc.get_distance(copter.current_loc);
    if (dist > 1.0f) {
        _last_loc = copter.current_loc;
        if (infence()) {
            _triggered = true;
        }
    }
}

bool UFence::infence()
{
    bool ret = false;
#if AP_FENCE_ENABLED
    const uint8_t orig_breaches = copter.fence.get_breaches();

    // check for new breaches; new_breaches is bitmask of fence types breached
    const uint8_t new_breaches = copter.fence.check();

    // we still don't do anything when disarmed, but we do check for fence breaches.
    // fence pre-arm check actually checks if any fence has been breached 
    // that's not ever going to be true if we don't call check on AP_Fence while disarmed.
    if (!copter.motors->armed()) {
        return false;
    }

    // if there is a new breach take action
    if (new_breaches && !orig_breaches) {
        if (!copter.ap.land_complete) {
            GCS_SEND_TEXT(MAV_SEVERITY_NOTICE, "Fence Breached");
            ret = true;
        }
    }
#endif
    return ret;
}

bool UFence::triggered() 
{
    return _triggered;
}

void UFence::set_triggered(bool b)
{
    _triggered = b;
}
