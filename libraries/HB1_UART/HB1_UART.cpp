#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include "HB1_UART.h"

extern const AP_HAL::HAL& hal;

/*
 * init - perform required initialisation
 */
bool HB1_UART::init()
{
    const AP_SerialManager &serial_manager = AP::serialmanager();

    // check for protocol configured for a serial port - only the first serial port with one of these protocols will then run (cannot have FrSky on multiple serial ports)
    if ((_port = serial_manager.find_serial(_protocol, 0))) {
        _initialized = true;
    }
    return _initialized;
}

void HB1_UART::read(void)
{    
    if(!initialized()) {
        return ;
    }
    while (_port->available() > 0) {
        uint8_t temp = _port->read();
        //if (_msg_mission2apm_v1.enable())   {_msg_mission2apm_v1.parse(temp);}
        if (_msg_mission2apm.enable())   {_msg_mission2apm.parse(temp);}
        if (_msg_mission2cam.enable())      {_msg_mission2cam.parse(temp);}
        if (_msg_cam2mission.enable())      {_msg_cam2mission.parse(temp);}
        if (_msg_power2apm.enable())        {_msg_power2apm.parse(temp);}
        //if (_msg_apm2mission.enable())      {_msg_apm2mission.parse(temp);}
        //if (_msg_apm2cam.enable())          {_msg_apm2cam.parse(temp);}
        //if (_msg_apm2power.enable())        {_msg_apm2power.parse(temp);}
    }
}

void HB1_UART::write(void)
{
    if(!initialized()) {
        return ;
    }
    int16_t i = 0;
    if (_msg_mission2apm._msg_1.need_send)
    {
        _msg_mission2apm.swap_message();
        for(i = 0;i < _msg_mission2apm._msg_1.length ; i ++) {
            _port->write(_msg_mission2apm._msg_1.content.data[i]);
        }
        _msg_mission2apm._msg_1.updated = false;
        _msg_mission2apm._msg_1.need_send = false;
    }

    if (_msg_mission2cam._msg_1.need_send)
    {
        _msg_mission2cam.swap_message();
        for(i = 0;i < _msg_mission2cam._msg_1.length ; i ++) {
            _port->write(_msg_mission2cam._msg_1.content.data[i]);
        }
        _msg_mission2cam._msg_1.updated = false;
        _msg_mission2cam._msg_1.need_send = false;
    }

    if (_msg_cam2mission._msg_1.need_send)
    {
        _msg_cam2mission.swap_message();
        for(i = 0;i < _msg_cam2mission._msg_1.length ; i ++) {
            _port->write(_msg_cam2mission._msg_1.content.data[i]);
        }
        _msg_cam2mission._msg_1.updated = false;
        _msg_cam2mission._msg_1.need_send = false;
    }

    if (_msg_power2apm._msg_1.need_send)
    {
        _msg_power2apm.swap_message();
        for(i = 0;i < _msg_power2apm._msg_1.length ; i ++) {
            _port->write(_msg_power2apm._msg_1.content.data[i]);
        }
        _msg_power2apm._msg_1.updated = false;
        _msg_power2apm._msg_1.need_send = false;
    }

    if (_msg_apm2mission._msg_1.need_send)
    {
        _msg_apm2mission.swap_message();
        for(i = 0;i < _msg_apm2mission._msg_1.length ; i ++) {
            _port->write(_msg_apm2mission._msg_1.content.data[i]);
        }
        _msg_apm2mission._msg_1.updated = false;
        _msg_apm2mission._msg_1.need_send = false;
    }

    if (_msg_apm2cam._msg_1.need_send)
    {
        _msg_apm2cam.swap_message();
        for(i = 0;i < _msg_apm2cam._msg_1.length ; i ++) {
            _port->write(_msg_apm2cam._msg_1.content.data[i]);
        }
        _msg_apm2cam._msg_1.updated = false;
        _msg_apm2cam._msg_1.need_send = false;
    }

    if (_msg_apm2power._msg_1.need_send)
    {
        _msg_apm2power.swap_message();
        for(i = 0;i < _msg_apm2power._msg_1.length ; i ++) {
            _port->write(_msg_apm2power._msg_1.content.data[i]);
        }
        _msg_apm2power._msg_1.updated = false;
        _msg_apm2power._msg_1.need_send = false;
    }
}
