#include "FD_HaoFu.h"

extern const AP_HAL::HAL& hal;

/*
 * init - perform required initialisation
 */
bool FD_HaoFu::init()
{
    const AP_SerialManager &serial_manager = AP::serialmanager();

    // check for protocol configured for a serial port - only the first serial port with one of these protocols will then run (cannot have FrSky on multiple serial ports)
    if ((_port = serial_manager.find_serial(_protocol, 0))) {
        _initialized = true;
    }
    return _initialized;
}

uint32_t FD_HaoFu::port_avaliable(void) {
    if(!initialized()) {
        return false;
    }
    return _port->available();
}

void FD_HaoFu::read(void)
{    
    if(!initialized()) {
        return ;
    }
    while (_port->available() > 0) {
        uint8_t temp = _port->read();
        read(temp);
    }
}

void FD_HaoFu::read(uint8_t temp)
{    
    if(!initialized()) {
        return ;
    }

    if (_msg_HaoFu.enable())   {_msg_HaoFu.parse(temp);}
}

void FD_HaoFu::write(uint8_t temp)
{
    if(!initialized()) {
        return ;
    }
    _port->write(temp);

}

void FD_HaoFu::write(void)
{
    if(!initialized()) {
        return ;
    }
    int16_t i = 0;
    if (_msg_HaoFu._msg_1.need_send)
    {
        _msg_HaoFu.swap_message();
        for(i = 0;i < _msg_HaoFu._msg_1.length ; i ++) {
            _port->write(_msg_HaoFu._msg_1.content.data[i]);
        }
        //_msg_HaoFu._msg_1.updated = false;
        _msg_HaoFu._msg_1.need_send = false;
        _msg_HaoFu.swap_message();
    }
}
