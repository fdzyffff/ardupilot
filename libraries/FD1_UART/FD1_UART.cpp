#include "FD1_UART.h"

extern const AP_HAL::HAL& hal;

/*
 * init - perform required initialisation
 */
bool FD1_UART::init()
{
    const AP_SerialManager &serial_manager = AP::serialmanager();

    // check for protocol configured for a serial port - only the first serial port with one of these protocols will then run (cannot have FrSky on multiple serial pornacelle)
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

uint8_t FD1_UART::port_read(void)
{    
    uint8_t temp = _port->read();
    return temp;
}

void FD1_UART::read(uint8_t temp)
{    
    if (_msg_gcs2nacelle.enable())    {_msg_gcs2nacelle.parse(temp);}
    if (_msg_nacelle2gcs.enable())    {_msg_nacelle2gcs.parse(temp);}
}

void FD1_UART::write(void)
{
    if(!initialized()) {
        return ;
    }
    int16_t i = 0;

    if (_msg_gcs2nacelle._msg_1.need_send)
    {
        _msg_gcs2nacelle.swap_message();
        for(i = 0;i < _msg_gcs2nacelle._msg_1.length ; i ++) {
            _port->write(_msg_gcs2nacelle._msg_1.content.data[i]);
        }
        //_msg_gcs2nacelle._msg_1.updated = false;
        _msg_gcs2nacelle._msg_1.need_send = false;
        _msg_gcs2nacelle.swap_message();
    }
    if (_msg_nacelle2gcs._msg_1.need_send)
    {
        _msg_nacelle2gcs.swap_message();
        for(i = 0;i < _msg_nacelle2gcs._msg_1.length ; i ++) {
            _port->write(_msg_nacelle2gcs._msg_1.content.data[i]);
        }
        //_msg_nacelle2gcs._msg_1.updated = false;
        _msg_nacelle2gcs._msg_1.need_send = false;
        _msg_nacelle2gcs.swap_message();
    }
}
