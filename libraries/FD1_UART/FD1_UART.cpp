#include "FD1_UART.h"

extern const AP_HAL::HAL& hal;

/*
 * init - perform required initialisation
 */
bool FD1_UART::init()
{
    if (_initialized && _port != nullptr) {return true;}
    const AP_SerialManager &serial_manager = AP::serialmanager();

    // check for protocol configured for a serial port - only the first serial port with one of these protocols will then run (cannot have FrSky on multiple serial ports)
    if ((_port = serial_manager.find_serial(_protocol, 0))) {
        _initialized = true;
    }
    return _initialized;
}
