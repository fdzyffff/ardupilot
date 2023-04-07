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
        for (uint8_t i_instance = 0; i_instance < MAX_ENGINE_PORT_NUM; i_instance++) {
            if (_msg_engine[i_instance].enable())   {_msg_engine[i_instance].parse(temp);}
        }
    }
}

FD_engine& FD_UART::get_msg_engine()
{ 
    return _msg_engine[0]; 
}

FD_engine& FD_UART::get_msg_engine(uint8_t instance) 
{
    if (instance < MAX_ENGINE_PORT_NUM) {
        return _msg_engine[instance]; 
    }
    return _msg_engine[0]; 
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
