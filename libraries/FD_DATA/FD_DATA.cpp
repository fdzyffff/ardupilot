#include "FD_DATA.h"

FD_DATA *FD_DATA::_singleton;

const AP_Param::GroupInfo FD_DATA::var_info[] = {
    AP_GROUPINFO("_GCS_LOCK", 0, FD_DATA, use_gcs_lock, 0),
    AP_GROUPINFO("_UAV_TYPE", 1, FD_DATA, uav_type, 0),
    AP_GROUPINFO("_MOT_FNUM", 2, FD_DATA, mot_fail_number, 0),
    AP_GROUPEND
};

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
}

namespace AP {
FD_DATA &fd_data()
{
    return *FD_DATA::get_singleton();
}
}
