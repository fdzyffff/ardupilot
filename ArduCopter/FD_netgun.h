#pragma once

#include <SRV_Channel/SRV_Channel.h>
#define NETGUN_NUM 2

class UserBarrel {
public:
    enum class BarrelState{
        NORMAL = 0,
        FIRED = 1
    };
    void Init(SRV_Channel::Aux_servo_function_t in_srv_function);
    void Update();
    void update_state();
    void release();
    void trigger();
    uint16_t get_output();
    void set_state(UserBarrel::BarrelState in_state);
    bool is_state(UserBarrel::BarrelState in_state);
    SRV_Channel::Aux_servo_function_t get_srv_function() {return _srv_function;}


private:
    BarrelState _state;
    uint16_t _output;
    uint32_t _last_state_ms;
    SRV_Channel::Aux_servo_function_t _srv_function;
};    

class UserNetgun {
public:
    void Init();
    void Update();
    void Trigger();

    UserBarrel _barrels[NETGUN_NUM];
    uint8_t _barrel_idx;
};
