#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include "USR_Dazhuang.h"

extern const AP_HAL::HAL& hal;

USR_Dazhuang *USR_Dazhuang::_singleton;
/*
 * init - perform required initialisation
 */
USR_Dazhuang::USR_Dazhuang() {
    _initialized = false;
    _count = 0;
    if (_singleton != nullptr) {
        AP_HAL::panic("USR_Dazhuang must be singleton");
    }
    _singleton = this;
}

bool USR_Dazhuang::add_new(enum AP_SerialManager::SerialProtocol protocol, SRV_Channel::Aux_servo_function_t function_in)
{
    if (_count >= USR_DAZHUANG_MAX) {
        gcs().send_text(MAV_SEVERITY_INFO, "Motor[%d] number exceed",protocol);
        return false;
    }

    for (uint8_t i_count=0; i_count<_count; i_count++) {
        if (USR_Dazhuang_instance[i_count]->get_protocol() == protocol) {
            gcs().send_text(MAV_SEVERITY_INFO, "Motor %d duplictated",protocol);
            return false;
        }
    }

    USR_Dazhuang_backend* new_instance = new USR_Dazhuang_backend(protocol, function_in);
    if (new_instance != nullptr) {
        if (new_instance->init()) {
            USR_Dazhuang_instance[_count] = new_instance;
            _count+=1;
            if (!_initialized) {_initialized = true;}
            gcs().send_text(MAV_SEVERITY_INFO, "Motor[%d] Added",protocol);
            return true;
        } else {
            gcs().send_text(MAV_SEVERITY_INFO, "Motor[%d] Failed",protocol);
        }
    }
    return false;
}

void USR_Dazhuang::read(void)
{    
    if(!initialized()) {
        return ;
    }
    for (uint8_t i_count=0; i_count<_count; i_count++) {
        USR_Dazhuang_instance[i_count]->read();
    }
}

void USR_Dazhuang::start()
{
    if(!initialized()) {
        return ;
    }

    for (uint8_t i_count=0; i_count<_count; i_count++) {
        USR_Dazhuang_instance[i_count]->start();
    }
}

void USR_Dazhuang::close()
{
    if(!initialized()) {
        return ;
    }

    for (uint8_t i_count=0; i_count<_count; i_count++) {
        USR_Dazhuang_instance[i_count]->close();
    }
}

void USR_Dazhuang::setup(SRV_Channel::Aux_servo_function_t function_in, float value)
{
    if(!initialized()) {
        return ;
    }

    for (uint8_t i_count=0; i_count<_count; i_count++) {
        if (USR_Dazhuang_instance[i_count]->get_function() == function_in) {
            USR_Dazhuang_instance[i_count]->setup(value);
            return;
        }
    }
}

void USR_Dazhuang::make_frame(void)
{
    if(!initialized()) {
        return ;
    }
    for (uint8_t i_count=0; i_count<_count; i_count++) {
        USR_Dazhuang_instance[i_count]->make_frame();
    }

}

void USR_Dazhuang::write(void)
{
    if(!initialized()) {
        return ;
    }
    for (uint8_t i_count=0; i_count<_count; i_count++) {
        USR_Dazhuang_instance[i_count]->write();
    }

}

void USR_Dazhuang::print()
{
    gcs().send_text(MAV_SEVERITY_INFO, "[%d]%d",_initialized, _count);
}

namespace AP {

USR_Dazhuang &user_dazhuang()
{
    return *USR_Dazhuang::get_singleton();
}

};
