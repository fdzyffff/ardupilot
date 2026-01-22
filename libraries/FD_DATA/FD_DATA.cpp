#include <AP_Math/AP_Math.h>
#include "FD_DATA.h"

extern const AP_HAL::HAL& hal;

FD_DATA *FD_DATA::_singleton;

// table of user settable parameters
const AP_Param::GroupInfo FD_DATA::var_info[] = {

    AP_GROUPINFO("_GCS_LOCK",    0, FD_DATA, use_gcs_lock, 0),
    AP_GROUPINFO("_UAV_TYPE",    1, FD_DATA, uav_type, 0),

    AP_GROUPEND
};

// constructor
FD_DATA::FD_DATA()
{
    if (_singleton != nullptr) {
        AP_HAL::panic("FD_DATA must be singleton");
    }

    AP_Param::setup_object_defaults(this, var_info);

    _singleton = this;
}

void FD_DATA::update()
{
    update_flying_s();
    update_allow_arm();
}

namespace AP {

FD_DATA &fd_data()
{
    return *FD_DATA::get_singleton();
}

};
