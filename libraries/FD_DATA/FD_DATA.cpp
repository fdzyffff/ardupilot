#include "FD_DATA.h"

FD_DATA *FD_DATA::_singleton;

// HUOJIAN: 参数组 HUOJ（当前无参数，骨架保留便于后续扩展）
const AP_Param::GroupInfo FD_DATA::var_info[] = {
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
    // HUOJIAN: 1Hz 周期处理扩展点，当前无内容
}

namespace AP {
FD_DATA &fd_data()
{
    return *FD_DATA::get_singleton();
}
}
