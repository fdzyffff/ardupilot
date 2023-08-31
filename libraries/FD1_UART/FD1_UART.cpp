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
    if(!initialized()) {
        return ;
    }
    while (_port->available() > 0) {
        uint8_t temp = _port->read();
        read(temp);
    }
}

void FD1_UART::read(uint8_t temp)
{    
    if(!initialized()) {
        return ;
    }

    if (_msg_DYTTELEM.enable())   {_msg_DYTTELEM.parse(temp);}
    if (_msg_DYTTARGET.enable())   {_msg_DYTTARGET.parse(temp);}
    if (_msg_APM2DYTCONTROL.enable())   {_msg_APM2DYTCONTROL.parse(temp);}
    if (_msg_APM2DYTTELEM.enable())   {_msg_APM2DYTTELEM.parse(temp);}
}

void FD1_UART::write(void)
{
    if(!initialized()) {
        return ;
    }
    int16_t i = 0;
    if (_msg_DYTTELEM._msg_1.need_send)
    {
        _msg_DYTTELEM.swap_message();
        for(i = 0;i < _msg_DYTTELEM._msg_1.length ; i ++) {
            _port->write(_msg_DYTTELEM._msg_1.content.data[i]);
        }
        //_msg_DYTTELEM._msg_1.updated = false;
        _msg_DYTTELEM._msg_1.need_send = false;
        _msg_DYTTELEM.swap_message();
    }
    if (_msg_DYTTARGET._msg_1.need_send)
    {
        _msg_DYTTARGET.swap_message();
        for(i = 0;i < _msg_DYTTARGET._msg_1.length ; i ++) {
            _port->write(_msg_DYTTARGET._msg_1.content.data[i]);
        }
        //_msg_DYTTARGET._msg_1.updated = false;
        _msg_DYTTARGET._msg_1.need_send = false;
        _msg_DYTTARGET.swap_message();
    }
    if (_msg_APM2DYTCONTROL._msg_1.need_send)
    {
        _msg_APM2DYTCONTROL.swap_message();
        for(i = 0;i < _msg_APM2DYTCONTROL._msg_1.length ; i ++) {
            _port->write(_msg_APM2DYTCONTROL._msg_1.content.data[i]);
        }
        //_msg_APM2DYTCONTROL._msg_1.updated = false;
        _msg_APM2DYTCONTROL._msg_1.need_send = false;
        _msg_APM2DYTCONTROL.swap_message();
    }
    if (_msg_APM2DYTTELEM._msg_1.need_send)
    {
        _msg_APM2DYTTELEM.swap_message();
        for(i = 0;i < _msg_APM2DYTTELEM._msg_1.length ; i ++) {
            _port->write(_msg_APM2DYTTELEM._msg_1.content.data[i]);
        }
        //_msg_APM2DYTTELEM._msg_1.updated = false;
        _msg_APM2DYTTELEM._msg_1.need_send = false;
        _msg_APM2DYTTELEM.swap_message();
    }
}
