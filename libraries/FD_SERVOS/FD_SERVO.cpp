#include "FD_SERVOS.h"

void FD_SERVO::update()
{
    if (AP_HAL::millis() - last_update_ms < 200) {
        return;
    }
    last_update_ms = AP_HAL::millis();
    // gcs().send_text(MAV_SEVERITY_INFO, "%d update", _id);

    if (is_zero(value)) {
        if (stop_count < 3) {
            stop_count++;
            do_stop();
        } else if (AP_HAL::millis() - last_reset_ms > 10000) {
            do_reset();
            last_reset_ms = AP_HAL::millis();
        }
    } else {
        do_speed();
        stop_count = 0;
    }
}

void FD_SERVO::set_vel(float servo_vel_in)
{
    _servo_vel = servo_vel_in;
}

void FD_SERVO::set_value(float value_in)
{
    value = value_in;
}

void FD_SERVO::do_stop()
{
    _msg_SERVO_24._msg_1.content.msg.id = _id;
    _msg_SERVO_24._msg_1.content.msg.method = 0x10;
    _msg_SERVO_24._msg_1.content.msg.Power = 0;
    _msg_SERVO_24.sum_check();
    if (_fronted != nullptr) {
        if (_fronted->get_port() != nullptr && _fronted->initialized()) {
            _fronted->get_port()->write(_msg_SERVO_24._msg_1.content.data, sizeof(_msg_SERVO_24._msg_1.content.data));
        }
    }
}

void FD_SERVO::do_reset()
{
    _msg_SERVO_17._msg_1.content.msg.id = _id;
    _msg_SERVO_17.sum_check();
    if (_fronted != nullptr) {
        if (_fronted->get_port() != nullptr && _fronted->initialized()) {
            _fronted->get_port()->write(_msg_SERVO_17._msg_1.content.data, sizeof(_msg_SERVO_17._msg_1.content.data));
        }
    }
}

void FD_SERVO::do_speed()
{
    _msg_SERVO_15._msg_1.content.msg.id = _id;
    _msg_SERVO_15._msg_1.content.msg.angle = 0x0000FFFF;
    _msg_SERVO_15._msg_1.content.msg.targetVelocity = (uint16_t)(value * _servo_vel * 10.f);
    _msg_SERVO_15._msg_1.content.msg.accInterval = 300;
    _msg_SERVO_15._msg_1.content.msg.decInterval = 300;
    _msg_SERVO_15._msg_1.content.msg.power = 0;
    _msg_SERVO_15.sum_check();
    if (_fronted != nullptr) {
        if (_fronted->get_port() != nullptr && _fronted->initialized()) {
            _fronted->get_port()->write(_msg_SERVO_15._msg_1.content.data, sizeof(_msg_SERVO_15._msg_1.content.data));
        }
    }
}