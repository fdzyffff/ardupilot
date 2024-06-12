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

uint32_t FD_UART::port_avaliable(void) 
{
    if(!initialized()) {
        return false;
    }
    return _port->available();
}

uint8_t FD_UART::read(void)
{    
    if(!initialized()) {
        return 0;
    }
    if (_port->available() > 0) {
        return _port->read();
    }
    return 0;
}

void FD_UART::read_and_parse()
{
    if(!initialized()) {
        return;
    }
    while (port_avaliable()) 
    {
        parse(read());
    }

}

void FD_UART::parse(uint8_t byte_in) 
{
    if(!initialized()) {
        return ;
    }
    // if (_msg_ue4_ahrs.enable())   {_msg_ue4_ahrs.parse(byte_in);}
    // if (_msg2apm_ue4_gimbal.enable())   {_msg2apm_ue4_gimbal.parse(byte_in);}
}

void FD_UART::write(void)
{
    if(!initialized()) {
        return ;
    }
    // int16_t i = 0;
    // if (_msg_ue4_ahrs._msg_1.need_send)
    // {
    //     _msg_ue4_ahrs.swap_message();
    //     for(i = 0;i < _msg_ue4_ahrs._msg_1.length ; i ++) {
    //         _port->write(_msg_ue4_ahrs._msg_1.content.data[i]);
    //     }
    //     //_msg_ue4_ahrs._msg_1.updated = false;
    //     _msg_ue4_ahrs._msg_1.need_send = false;
    //     _msg_ue4_ahrs.swap_message();
    // }
    return;
}
