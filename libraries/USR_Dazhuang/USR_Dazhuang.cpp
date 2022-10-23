#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include "USR_Dazhuang.h"

extern const AP_HAL::HAL& hal;

USR_Dazhuang *USR_Dazhuang::_singleton;
/*
 * init - perform required initialisation
 */
bool USR_Dazhuang::add_new(enum AP_SerialManager::SerialProtocol protocol)
{
    for (uint8_t i_count=0; i_count<_count; i_count++) {
        if (USR_Dazhuang_instance[i_count]->get_protocol() == protocol) {
            gcs().send_text(MAV_SEVERITY_INFO, "Motor %d duplictated",protocol);
            return false;
        }
    }

    if (_count >= USR_DAZHUANG_MAX) {
        gcs().send_text(MAV_SEVERITY_INFO, "Motor[%d] number exceed",protocol);
        return false;
    }

    USR_Dazhuang_backend* new_instance = new USR_Dazhuang_backend(protocol);
    if (new_instance != nullptr) {
        USR_Dazhuang_instance[_count] = new_instance;
        _count+=1;
        if (!_initialized) {_initialized = true;}
        return true;
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

void USR_Dazhuang::setup(int16_t instance, float value)
{
    if(!initialized()) {
        return ;
    }
    if (instance >= _count) {
        return ;
    }
    USR_Dazhuang_instance[instance]->setup(value);
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

namespace AP {

USR_Dazhuang &user_dazhuang()
{
    return *USR_Dazhuang::get_singleton();
}

};
