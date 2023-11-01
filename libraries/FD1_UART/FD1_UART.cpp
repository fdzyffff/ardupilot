#include "FD1_UART.h"
#include <GCS_MAVLink/GCS.h>

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
        // gcs().send_text(MAV_SEVERITY_INFO, "X :%x", temp);
        if (_msg_init.enable())   {_msg_init.parse(temp);}
        if (_msg_control.enable())   {_msg_control.parse(temp);}
        if (_msg_mission.enable())   {_msg_mission.parse(temp);}
        if (_msg_guide.enable())   {_msg_guide.parse(temp);}
        if (_msg_info.enable())   {_msg_info.parse(temp);}
    // }
}

void FD1_UART::write(void)
{
    if(!initialized()) {
        return ;
    }
    
    // gcs().send_text(MAV_SEVERITY_INFO, "_msg_info._msg_1.need_send %d", _msg_info._msg_1.need_send);
    int16_t i = 0;
    if (_msg_init._msg_1.need_send)
    {
        _msg_init.swap_message();
        for(i = 0;i < _msg_init._msg_1.length ; i ++) {
            _port->write(_msg_init._msg_1.content.data[i]);
        }
        //_msg_init._msg_1.updated = false;
        _msg_init._msg_1.need_send = false;
        _msg_init.swap_message();
    }
    if (_msg_control._msg_1.need_send)
    {
        _msg_control.swap_message();
        for(i = 0;i < _msg_control._msg_1.length ; i ++) {
            _port->write(_msg_control._msg_1.content.data[i]);
        }
        //_msg_control._msg_1.updated = false;
        _msg_control._msg_1.need_send = false;
        _msg_control.swap_message();
    }
    if (_msg_mission._msg_1.need_send)
    {
        _msg_mission.swap_message();
        for(i = 0;i < _msg_mission._msg_1.length ; i ++) {
            _port->write(_msg_mission._msg_1.content.data[i]);
        }
        //_msg_mission._msg_1.updated = false;
        _msg_mission._msg_1.need_send = false;
        _msg_mission.swap_message();
    }
    if (_msg_guide._msg_1.need_send)
    {
        _msg_guide.swap_message();
        for(i = 0;i < _msg_guide._msg_1.length ; i ++) {
            _port->write(_msg_guide._msg_1.content.data[i]);
        }
        //_msg_guide._msg_1.updated = false;
        _msg_guide._msg_1.need_send = false;
        _msg_guide.swap_message();
    }
    if (_msg_info._msg_1.need_send)
    {
        _msg_info.swap_message();
        // for(i = 0;i < _msg_info._msg_1.length ; i ++) {
            _port->write(_msg_info._msg_1.content.data, _msg_info._msg_1.length);
        // }
        //_msg_info._msg_1.updated = false;
        _msg_info._msg_1.need_send = false;
        _msg_info.swap_message();
    }
}
