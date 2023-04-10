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

    _initialized = false;
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
    gcs().send_text(MAV_SEVERITY_INFO, "Motor[%d] Start", _protocol);
}

void USR_Dazhuang_backend::close(void) {
    _msg_out._msg_1.content.msg.thr = 0; // 0~1000
    _msg_out._msg_1.content.msg.cmd = 0x00;
}

void USR_Dazhuang_backend::setup(float value) {
    if (AP_HAL::millis() - _start_time_ms < 2500) {
        return;
    } else {
        value = constrain_float(value, 0.1f, 1.0f);
        _msg_out._msg_1.content.msg.thr = (uint16_t)(value*1000.f); // 0~1000
        _msg_out._msg_1.content.msg.cmd = 0x00;
    }
}

void USR_Dazhuang_backend::make_frame() {
    _msg_out._msg_1.content.msg.header.head_1 = FD1_msg_out::PREAMBLE1;
    _msg_out._msg_1.content.msg.header.head_2 = FD1_msg_out::PREAMBLE2;
    _msg_out._msg_1.content.msg.sum_check = 0;

    for(uint8_t i = 2; i < _msg_out._msg_1.length-1; i ++) {
        _msg_out._msg_1.content.msg.sum_check += _msg_out._msg_1.content.data[i];
    }

    _msg_out._msg_1.need_send = true;
}

void USR_Dazhuang_backend::write(void)
{
    if(!initialized()) {
        return ;
    }
    if (_msg_out._msg_1.need_send)
    {
        _msg_out.swap_message();
        _msg_out._msg_1.content.msg.count++; //accumulate the count number
        _port->write(_msg_out._msg_1.content.data, sizeof(_msg_out._msg_1.content.data));
        _msg_out._msg_1.updated = false;
        _msg_out._msg_1.need_send = false;
    }
}
