#include "FD_Target_Cam.h"

#include <AP_SerialManager/AP_SerialManager.h>

FD_Target_Cam::FD_Target_Cam() :
    _port(nullptr)
{
}

bool FD_Target_Cam::init_port()
{
    _port = AP::serialmanager().find_serial(AP_SerialManager::SerialProtocol_DYT, 0);
    return _port != nullptr;
}

void FD_Target_Cam::update_timeout(uint32_t now_ms)
{
    if ((target_timeout.get() > 0) &&
        (now_ms - _last_ms > (uint32_t)target_timeout.get())) {
        _valid = false;
    }
}

void FD_Target_Cam::write_bytes(const uint8_t *data, uint16_t len)
{
    if (_port == nullptr) {
        return;
    }
    _port->write(data, len);
}
