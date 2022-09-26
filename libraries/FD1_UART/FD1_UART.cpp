#include "FD1_UART.h"

extern const AP_HAL::HAL& hal;

/*
 * init - perform required initialisation
 */
bool FD1_UART::init()
{
    const AP_SerialManager &serial_manager = AP::serialmanager();

    // check for protocol configured for a serial port - only the first serial port with one of these protocols will then run (cannot have FrSky on multiple serial ports)
    if ((_port = serial_manager.find_serial(_protocol, 0))) {
        _initialized = true;
    }
    return _initialized;
}

uint32_t FD1_UART::port_avaliable(void) {
    if(!initialized()) {
        return false;
    }
    return _port->available();
}
void FD1_UART::read(void)
{    
    // if(!initialized()) {
    //     return ;
    // }
    // while (_port->available() > 0) {
        uint8_t temp = _port->read();
        if (_msg_ep4_in.enable())   {_msg_ep4_in.parse(temp);}
        if (_msg_ts_in.enable())    {_msg_ts_in.parse(temp);}
    // }
}

void FD1_UART::write(void)
{
    if(!initialized()) {
        return ;
    }
    int16_t i = 0;
    if (_msg_ep4_in._msg_1.need_send)
    {
        _msg_ep4_in.swap_message();
        for(i = 0;i < _msg_ep4_in._msg_1.length ; i ++) {
            _port->write(_msg_ep4_in._msg_1.content.data[i]);
        }
        //_msg_ep4_in._msg_1.updated = false;
        _msg_ep4_in._msg_1.need_send = false;
        _msg_ep4_in.swap_message();
    }
    if (_msg_ep4_out._msg_1.need_send)
    {
        _msg_ep4_out.swap_message();
        for(i = 0;i < _msg_ep4_out._msg_1.length ; i ++) {
            _port->write(_msg_ep4_out._msg_1.content.data[i]);
        }
        //_msg_ep4_out._msg_1.updated = false;
        _msg_ep4_out._msg_1.need_send = false;
        _msg_ep4_out.swap_message();
    }
    if (_msg_ts_in._msg_1.need_send)
    {
        _msg_ts_in.swap_message();
        for(i = 0;i <= _msg_ts_in._msg_1.length+2 ; i ++) {
            _port->write(_msg_ts_in._msg_1.content.data[i]);
        }
        //_msg_ts_in._msg_1.updated = false;
        _msg_ts_in._msg_1.need_send = false;
        _msg_ts_in.swap_message();
    }
    if (_msg_ts_out._msg_1.need_send)
    {
        _msg_ts_out.swap_message();
        for(i = 0;i <= _msg_ts_out._msg_1.length+2 ; i ++) {
            _port->write(_msg_ts_out._msg_1.content.data[i]);
        }
        //_msg_ts_out._msg_1.updated = false;
        _msg_ts_out._msg_1.need_send = false;
        _msg_ts_out.swap_message();
    }
    if (_msg_ts_route._msg_1.need_send)
    {
        _msg_ts_route.swap_message();
        for(i = 0;i <= _msg_ts_route._msg_1.length+2 ; i ++) {
            _port->write(_msg_ts_route._msg_1.content.data[i]);
        }
        //_msg_ts_route._msg_1.updated = false;
        _msg_ts_route._msg_1.need_send = false;
        _msg_ts_route.swap_message();
    }
}
