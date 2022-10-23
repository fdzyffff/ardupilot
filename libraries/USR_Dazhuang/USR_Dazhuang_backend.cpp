#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include "USR_Dazhuang_backend.h"

extern const AP_HAL::HAL& hal;

/*
 * init - perform required initialisation
 */
bool USR_Dazhuang_backend::init()
{
    const AP_SerialManager &serial_manager = AP::serialmanager();

    // check for protocol configured for a serial port - only the first serial port with one of these protocols will then run (cannot have FrSky on multiple serial ports)
    if ((_port = serial_manager.find_serial(_protocol, 0))) {
        _initialized = true;
    }
    return _initialized;
}

void USR_Dazhuang_backend::update(void) {
    ;
}

void USR_Dazhuang_backend::read(void)
{    
    if(!initialized()) {
        return ;
    }
    while (_port->available() > 0) {
        uint8_t temp = _port->read();
        if (_msg_in.enable())   {_msg_in.parse(temp);}
    }
}

void USR_Dazhuang_backend::start(void) {
    _start_time_ms = AP_HAL::millis();
    _msg_out._msg_1.content.msg.thr = 100; // 0~1000
    _msg_out._msg_1.content.msg.cmd = 0x07;
}

void USR_Dazhuang_backend::close(void) {
    _msg_out._msg_1.content.msg.thr = 0; // 0~1000
    _msg_out._msg_1.content.msg.cmd = 0x00;
}

void USR_Dazhuang_backend::setup(float value) {
    if (AP_HAL::millis() - _start_time_ms < 1500) {
        return;
    } else {
        value = constrain_float(value, 0.1f, 1.0f);
        _msg_out._msg_1.content.msg.thr = (uint16_t)(value*1000.f); // 0~1000
        _msg_out._msg_1.content.msg.cmd = 0x00;
    }
}

void USR_Dazhuang_backend::write(void)
{
    if(!initialized()) {
        return ;
    }
    int16_t i = 0;
    if (_msg_out._msg_1.need_send)
    {
        _msg_out.swap_message();
        _msg_out._msg_1.content.msg.count++; //accumulate the count number
        for(i = 0;i < _msg_out._msg_1.length ; i ++) {
            _port->write(_msg_out._msg_1.content.data[i]);
        }
        _msg_out._msg_1.updated = false;
        _msg_out._msg_1.need_send = false;
    }
}
