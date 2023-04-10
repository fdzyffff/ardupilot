#pragma once

#include <AP_HAL/AP_HAL.h>
#include "USR_Dazhuang_backend.h"

#define USR_DAZHUANG_MAX 6

class USR_Dazhuang {
public:

    USR_Dazhuang();

    /* Do not allow copies */
    USR_Dazhuang(const USR_Dazhuang &other) = delete;
    USR_Dazhuang &operator=(const USR_Dazhuang&) = delete;

    static USR_Dazhuang *get_singleton() {
        return _singleton;
    }

    // init - perform required initialisation
    bool add_new(enum AP_SerialManager::SerialProtocol protocol, SRV_Channel::Aux_servo_function_t function_in);
    bool initialized() {return _initialized;}
    void read(void);
    void start();
    void close();
    void setup(SRV_Channel::Aux_servo_function_t function_in, float value);
    void make_frame();
    void write(void);
    void active(bool v_in) {_active = v_in;}
    bool active() {return _active;}
    void print();

private:

    static USR_Dazhuang *_singleton;
    bool _initialized;
    bool _active;
    int16_t _count;
    USR_Dazhuang_backend* USR_Dazhuang_instance[USR_DAZHUANG_MAX];

};

namespace AP {
    USR_Dazhuang &user_dazhuang();
};
