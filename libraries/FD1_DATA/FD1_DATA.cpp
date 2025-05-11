#include <AP_Math/AP_Math.h>
#include "FD1_DATA.h"

extern const AP_HAL::HAL& hal;

/*
 * init - perform required initialisation
 */


FD1_DATA *FD1_DATA::_singleton;

// constructor
FD1_DATA::FD1_DATA()
{
    if (_singleton != nullptr) {
        AP_HAL::panic("FD1_DATA must be singleton");
    }
    _singleton = this;
    _new_data = false;
}


// yaw in degrees if available
bool FD1_DATA::get_yaw_deg(float &yaw_deg, float &accuracy_deg) {
    if (!_new_data) {
        return false;
    }
    yaw_deg = _yaw_deg;
    accuracy_deg = _accuracy_deg;
    _new_data = false;
    return true;
}


void FD1_DATA::set_yaw_deg(float yaw_deg, float accuracy_deg) {
    _yaw_deg = yaw_deg;
    _accuracy_deg = accuracy_deg;
    _new_data = true;
}

namespace AP {

FD1_DATA &fd1_data()
{
    return *FD1_DATA::get_singleton();
}

};
