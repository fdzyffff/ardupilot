#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include "FD_UART.h"

extern const AP_HAL::HAL& hal;

/*
 * init - perform required initialisation
 */
bool FD_UART::init()
{
    const AP_SerialManager &serial_manager = AP::serialmanager();

    // check for protocol configured for a serial port - only the first serial port with one of these protocols will then run (cannot have FrSky on multiple serial ports)
    if ((_port = serial_manager.find_serial(_protocol, 0))) {
        _initialized = true;
    }
    return _initialized;
}

void FD_UART::read(void)
{    
    if(!initialized()) {
        return ;
    }
    while (_port->available() > 0) {
        uint8_t temp = _port->read();
        //if (_msg_apm2payload.enable())   {_msg_apm2payload.parse(temp);}
        if (_msg_payload2apm.enable())   {_msg_payload2apm.parse(temp);}
    }
}

void FD_UART::write(void)
{
    if(!initialized()) {
        return ;
    }
    // if (_msg_apm2payload._msg_1.need_send)
    // {
    //     _msg_apm2payload.swap_message();
    //     _msg_apm2payload.cal_sumcheck();
    //     _port->write(_msg_apm2payload._msg_1.content.data, _msg_apm2payload._msg_1.length);
    //     _msg_apm2payload._msg_1.updated = false;
    //     _msg_apm2payload._msg_1.need_send = false;
    // }
}
