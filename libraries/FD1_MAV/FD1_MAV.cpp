#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include "FD1_MAV.h"

extern const AP_HAL::HAL& hal;

/*
 * init - perform required initialisation
 */
bool FD1_MAV::init()
{
    if (_initialized) {return _initialized;}
    const AP_SerialManager &serial_manager = AP::serialmanager();

    // check for protocol configured for a serial port - only the first serial port with one of these protocols will then run (cannot have FrSky on multiple serial ports)
    if ((_port = serial_manager.find_serial(_protocol, 0))) {
        _initialized = true;
    }
    return _initialized;
}


AP_HAL::UARTDriver* FD1_MAV::get_port(void)
{
    if(!initialized()) {
        return NULL;
    }
    return _port;
}
