#include "FD_YOLO.h"

extern const AP_HAL::HAL& hal;

bool FD_YOLO::init()
{
    const AP_SerialManager &serial_manager = AP::serialmanager();
    _port = serial_manager.find_serial(_protocol, 0);
    if (_port != nullptr) {
        _initialized = true;
        _msg.set_enable();
    }
    return _initialized;
}

void FD_YOLO::read()
{
    if (!_initialized) {
        return;
    }
    while (_port->available() > 0) {
        uint8_t temp = _port->read();
        if (_msg.enable()) {
            _msg.parse(temp);
        }
    }
}
