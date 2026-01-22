#include <AP_Math/AP_Math.h>
#include "FD_DATA.h"

extern const AP_HAL::HAL& hal;

// storage object
StorageAccess FD_DATA::_storage(StorageManager::StorageFDData);

assert_storage_size<FD_DATA_T, 44> _assert_storage_size_FD_DATA_T;
/*
 * init - perform required initialisation
 */

bool FD_DATA::get_serial_number(char *serial_number)
{
    if (local_data.serial_number[0] != 0) {
        memcpy(serial_number, local_data.serial_number, sizeof(local_data.serial_number));
        return true;
    }

    if (_storage.read_block(&local_data, 0, sizeof(FD_DATA_T))) {
        memcpy(serial_number, local_data.serial_number, sizeof(local_data.serial_number));
        return true;
    }
    return false;
}

bool FD_DATA::read_serial_number()
{
    if (_storage.read_block(&local_data, 0, sizeof(FD_DATA_T))) {
        return true;
    }
    return false;
}

bool FD_DATA::set_serial_number(char *serial_number)
{
    if (_storage.read_block(&local_data, 0, sizeof(FD_DATA_T))) {
        memcpy(local_data.serial_number, serial_number, sizeof(local_data.serial_number));
        return _storage.write_block(0, &local_data, sizeof(FD_DATA_T));
    }
    return false;
}

bool FD_DATA::get_uas_number(char *uas_number)
{
    if (local_data.uas_number[0] != 0) {
        memcpy(uas_number, local_data.uas_number, sizeof(local_data.uas_number));
        return true;
    }

    if (_storage.read_block(&local_data, 0, sizeof(FD_DATA_T))) {
        memcpy(uas_number, local_data.uas_number, sizeof(local_data.uas_number));
        return true;
    }
    return false;
}

bool FD_DATA::read_uas_number()
{
    if (_storage.read_block(&local_data, 0, sizeof(FD_DATA_T))) {
        return true;
    }
    return false;
}

bool FD_DATA::set_uas_number(char *uas_number)
{
    if (_storage.read_block(&local_data, 0, sizeof(FD_DATA_T))) {
        memcpy(local_data.uas_number, uas_number, sizeof(local_data.uas_number));
        return _storage.write_block(0, &local_data, sizeof(FD_DATA_T));
    }
    return false;
}
bool FD_DATA::get_runtime_flying(uint32_t& runtime_flying)
{
    if (_storage.read_block(&local_data, 0, sizeof(FD_DATA_T))) {
        runtime_flying = local_data.runtime_flying;
        return true;
    }
    return false;
}

bool FD_DATA::reset_runtime_flying()
{
    if (_storage.read_block(&local_data, 0, sizeof(FD_DATA_T))) {
        local_data.runtime_flying = 0;
        return _storage.write_block(0, &local_data, sizeof(FD_DATA_T));
    }
    return false;
}

bool FD_DATA::set_flying_s(uint32_t dt_s)
{
    if (AP_HAL::millis() - _last_update_flying_ms > 10000) {
        _last_update_flying_ms = AP_HAL::millis();
        if (_storage.read_block(&local_data, 0, sizeof(FD_DATA_T))) {
            local_data.runtime_flying = local_data.runtime_flying + dt_s;
            _storage.write_block(0, &local_data, sizeof(FD_DATA_T));
        }
        return true;
    }
    return false;
}
