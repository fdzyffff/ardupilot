#include "FD_DATA.h"

StorageAccess FD_DATA::_storage(StorageManager::StorageFDData);
assert_storage_size<FD_DATA_T, 44> _assert_storage_size_FD_DATA_T;

bool FD_DATA::load_record()
{
    if (record_loaded) {
        return true;
    }
    if (_storage.size() < sizeof(local_data) ||
        !_storage.read_block(&local_data, 0, sizeof(local_data))) {
        return false;
    }
    if (uint8_t(local_data.serial_number[0]) == 0xFF) {
        memset(local_data.serial_number, 0, sizeof(local_data.serial_number));
    }
    if (uint8_t(local_data.uas_number[0]) == 0xFF) {
        memset(local_data.uas_number, 0, sizeof(local_data.uas_number));
    }
    if (local_data.runtime_flying == UINT32_MAX) {
        local_data.runtime_flying = 0;
    }
    record_loaded = true;
    return true;
}

bool FD_DATA::commit_record(const FD_DATA_T &candidate)
{
    if (_storage.size() < sizeof(candidate) ||
        !_storage.write_block(0, &candidate, sizeof(candidate))) {
        return false;
    }
    local_data = candidate;
    record_loaded = true;
    return true;
}

bool FD_DATA::get_serial_number(char serial_number[IDENTITY_TEXT_SIZE])
{
    if (!load_record()) {
        serial_number[0] = '\0';
        return false;
    }
    memcpy(serial_number, local_data.serial_number, IDENTITY_FIELD_SIZE);
    serial_number[IDENTITY_FIELD_SIZE] = '\0';
    return true;
}

bool FD_DATA::set_serial_number(const char *serial_number, uint8_t length)
{
    if (serial_number == nullptr || !load_record()) {
        return false;
    }
    FD_DATA_T candidate = local_data;
    memset(candidate.serial_number, 0, sizeof(candidate.serial_number));
    const uint8_t copy_length = MIN(length, IDENTITY_FIELD_SIZE);
    memcpy(candidate.serial_number, serial_number, copy_length);
    return commit_record(candidate);
}

bool FD_DATA::get_uas_number(char uas_number[IDENTITY_TEXT_SIZE])
{
    if (!load_record()) {
        uas_number[0] = '\0';
        return false;
    }
    memcpy(uas_number, local_data.uas_number, IDENTITY_FIELD_SIZE);
    uas_number[IDENTITY_FIELD_SIZE] = '\0';
    return true;
}

bool FD_DATA::set_uas_number(const char *uas_number, uint8_t length)
{
    if (uas_number == nullptr || !load_record()) {
        return false;
    }
    FD_DATA_T candidate = local_data;
    memset(candidate.uas_number, 0, sizeof(candidate.uas_number));
    const uint8_t copy_length = MIN(length, IDENTITY_FIELD_SIZE);
    memcpy(candidate.uas_number, uas_number, copy_length);
    return commit_record(candidate);
}

bool FD_DATA::get_runtime_flying(uint32_t &runtime_flying)
{
    if (!load_record()) {
        return false;
    }
    runtime_flying = local_data.runtime_flying;
    return true;
}

bool FD_DATA::reset_runtime_flying()
{
    if (!load_record()) {
        return false;
    }
    FD_DATA_T candidate = local_data;
    candidate.runtime_flying = 0;
    return commit_record(candidate);
}

bool FD_DATA::set_flying_s(uint32_t dt_s)
{
    const uint32_t now = AP_HAL::millis();
    if (now - _last_update_flying_ms < 10000U || !load_record()) {
        return false;
    }
    FD_DATA_T candidate = local_data;
    if (UINT32_MAX - candidate.runtime_flying < dt_s) {
        candidate.runtime_flying = UINT32_MAX;
    } else {
        candidate.runtime_flying += dt_s;
    }
    if (!commit_record(candidate)) {
        return false;
    }
    _last_update_flying_ms = now;
    return true;
}
