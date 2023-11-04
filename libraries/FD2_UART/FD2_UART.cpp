#include "FD2_UART.h"

extern const AP_HAL::HAL& hal;

/*
 * init - perform required initialisation
 */
bool FD2_UART::init()
{
    const AP_SerialManager &serial_manager = AP::serialmanager();

    // check for protocol configured for a serial port - only the first serial port with one of these protocols will then run (cannot have FrSky on multiple serial ports)
    if ((_port = serial_manager.find_serial(_protocol, 0))) {
        _initialized = true;
    }
    return _initialized;
}

uint32_t FD2_UART::port_avaliable(void) {
    if(!initialized()) {
        return false;
    }
    return _port->available();
}
void FD2_UART::read(void)
{    
    if(!initialized()) {
        return ;
    }
    while (_port->available() > 0) {
        uint8_t temp = _port->read();
        if (_msg_ue4_ahrs.enable())   {_msg_ue4_ahrs.parse(temp);}
        if (_msg2apm_ue4_gimbal.enable())   {_msg2apm_ue4_gimbal.parse(temp);}
    }
}

void FD2_UART::write(void)
{
    if(!initialized()) {
        return ;
    }
    int16_t i = 0;
    if (_msg_ue4_ahrs._msg_1.need_send)
    {
        _msg_ue4_ahrs.swap_message();
        for(i = 0;i < _msg_ue4_ahrs._msg_1.length ; i ++) {
            _port->write(_msg_ue4_ahrs._msg_1.content.data[i]);
        }
        //_msg_ue4_ahrs._msg_1.updated = false;
        _msg_ue4_ahrs._msg_1.need_send = false;
        _msg_ue4_ahrs.swap_message();
    }
}
