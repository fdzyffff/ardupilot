#include <AP_Math/AP_Math.h>
#include "FD_DATA.h"

extern const AP_HAL::HAL& hal;

// storage object
StorageAccess FD_DATA::_storage(StorageManager::StorageFDData);

assert_storage_size<FD_DATA_T, 8> _assert_storage_size_FD_DATA_T;
/*
 * init - perform required initialisation
 */

FD_DATA *FD_DATA::_singleton;

// Convenience macros //////////////////////////////////////////////////////////
//
// const AP_Param::GroupInfo FD_DATA::var_info[] = {

//     AP_GROUPINFO("TEST",   0, FD_Target_K230, test_mode,        0),

//     AP_GROUPEND
// };

// constructor
FD_DATA::FD_DATA()
{
    if (_singleton != nullptr) {
        AP_HAL::panic("FD_DATA must be singleton");
    }
    _singleton = this;
}

void FD_DATA::update()
{
    update_flying_s();
}

void FD_DATA::handle_message(const mavlink_message_t &msg)
{
    handle_message_sn(msg);
    handle_message_rt(msg);
    handle_message_command_long_sn(msg);
    handle_message_command_long_rt(msg);
}
namespace AP {

FD_DATA &fd_data()
{
    return *FD_DATA::get_singleton();
}

};
