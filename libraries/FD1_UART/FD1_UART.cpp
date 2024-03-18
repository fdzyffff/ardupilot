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
        if (_msg_ue4_ahrs.enable())   {_msg_ue4_ahrs.parse(temp);}
        if (_msg_ID1.enable())   {_msg_ID1.parse(temp);}
        if (_msg_ID2.enable())   {_msg_ID2.parse(temp);}
        if (_msg_ID6.enable())   {_msg_ID6.parse(temp);}
    }
}

void FD1_UART::write(void)
{
    if(!initialized()) {
        return ;
    }
    int16_t i = 0;
    if (_msg_ue4_ahrs._msg_1.need_send) {
        _msg_ue4_ahrs.swap_message();
        for(i = 0;i < _msg_ue4_ahrs._msg_1.length ; i ++) {
            _port->write(_msg_ue4_ahrs._msg_1.content.data[i]);
        }
        _msg_ue4_ahrs._msg_1.need_send = false;
        _msg_ue4_ahrs.swap_message();
    }
    if (_msg_ID1._msg_1.need_send) {
        _msg_ID1.swap_message();
        for(i = 0;i < _msg_ID1._msg_1.length ; i ++) {
            _port->write(_msg_ID1._msg_1.content.data[i]);
        }
        _msg_ID1._msg_1.need_send = false;
        _msg_ID1.swap_message();
    }
    if (_msg_ID2._msg_1.need_send) {
        _msg_ID2.swap_message();
        for(i =0;i < _msg_ID2._msg_1.length ; i ++) {
            _port->write(_msg_ID2._msg_1.content.data[i]);
        }
        _msg_ID2._msg_1.need_send = false;
        _msg_ID2.swap_message();
    }
    if (_msg_ID5._msg_1.need_send) {
        _msg_ID5.swap_message();
        for(i = 0;i < _msg_ID5._msg_1.length ; i ++) {
            _port->write(_msg_ID5._msg_1.content.data[i]);
        }
        _msg_ID5._msg_1.need_send = false;
        _msg_ID5.swap_message();
    }
    if (_msg_ID6._msg_1.need_send) {
        _msg_ID6.swap_message();
        for(i = 0;i < _msg_ID6._msg_1.length ; i ++) {
            _port->write(_msg_ID6._msg_1.content.data[i]);
        }
        _msg_ID6._msg_1.need_send = false;
        _msg_ID6.swap_message();
    }
}
