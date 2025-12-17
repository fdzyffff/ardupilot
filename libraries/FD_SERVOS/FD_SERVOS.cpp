#include "FD_SERVOS.h"

extern const AP_HAL::HAL& hal;

// Convenience macros //////////////////////////////////////////////////////////
//
const AP_Param::GroupInfo FD_SERVOS::var_info[] = {

    AP_GROUPINFO("_VEL",  1, FD_SERVOS, servo_vel,         30.f),

    AP_GROUPEND
};

FD_SERVOS *FD_SERVOS::_singleton;
/*
 * init - perform required initialisation
 */
FD_SERVOS::FD_SERVOS()
{
    _port = NULL;
    _initialized = false;
    if (_singleton != nullptr) {
        AP_HAL::panic("FD_SERVOS must be singleton");
    }
    _singleton = this;
    AP_Param::setup_object_defaults(this, var_info);
}

bool FD_SERVOS::init()
{
    const AP_SerialManager &serial_manager = AP::serialmanager();

    // check for protocol configured for a serial port - only the first serial port with one of these protocols will then run (cannot have FrSky on multiple serial ports)
    if ((_port = serial_manager.find_serial(AP_SerialManager::SerialProtocol_FDSERVO, 0))) {
        _initialized = true;
    }

    for (uint8_t i_servo = 0; i_servo < FD_SERVO_MAX_NUM; i_servo++) {
        servo_instance[i_servo] = new FD_SERVO(this, i_servo);
    }

    _enable = true;

    return _initialized;
}

void FD_SERVOS::update()
{
    read_uart();
    update_control();

    for (uint8_t i_servo = 0; i_servo < FD_SERVO_MAX_NUM; i_servo++) {
        if (servo_instance[i_servo] != nullptr) {
            servo_instance[i_servo]->update();
        }
    }
}

void FD_SERVOS::read_uart(void)
{    
    if(!initialized()) {
        return ;
    }
    while (_port->available() > 0) {
        uint8_t temp = _port->read();
        _msg_SERVO_receive.parse(temp);
    }
}

void FD_SERVOS::update_control()
{
    if (_enable) {
        float speed_norm_in = -RC_Channels::rc_channel(1)->norm_input();
        float turn_norm_in = RC_Channels::rc_channel(3)->norm_input();
        set_speed(speed_norm_in, turn_norm_in);
    } else {
        set_speed(0.0f, 0.0f);
    }
}

void FD_SERVOS::set_speed(float speed_norm_in, float turn_norm_in)
{
    for (uint8_t i_servo = 0; i_servo < FD_SERVO_MAX_NUM; i_servo++) {
        float speed_servo_norm_in = 0.0f;
        if (i_servo < (FD_SERVO_MAX_NUM/2)) {
            speed_servo_norm_in = speed_norm_in+turn_norm_in;
        } else {
            speed_servo_norm_in = speed_norm_in-turn_norm_in;
        }
        speed_servo_norm_in = constrain_float(speed_servo_norm_in, 0.0f, 1.0f);
        if (servo_instance[i_servo] != nullptr) {
            servo_instance[i_servo]->set_vel(servo_vel);
            servo_instance[i_servo]->set_value(speed_servo_norm_in);
        }
    }
}

void FD_SERVOS::set_enable(bool enable_in)
{
    _enable = enable_in;
}


namespace AP {

FD_SERVOS &fd_servos()
{
    return *FD_SERVOS::get_singleton();
}

};
