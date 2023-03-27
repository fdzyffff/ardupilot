#pragma once

#include <SRV_Channel/SRV_Channel.h>
#define NETGUN_NUM 2

class UserBarrel {
public:
    enum class BarrelState{
        NORMAL = 0,
        FIRED = 1,
        CUT = 2
    };
    enum cut_channel{
        FIRE_CHANNEL = 0,
        CUT_CHANNEL = 1
    };
    void Init(SRV_Channel::Aux_servo_function_t in_srv_function_fire, SRV_Channel::Aux_servo_function_t in_srv_function_cut);
    void Update();
    void update_state();
    void cut();
    void fire();
    uint16_t get_output(uint16_t channel);
    void set_state(UserBarrel::BarrelState in_state);
    bool is_state(UserBarrel::BarrelState in_state);

private:
    BarrelState _state;
    uint16_t _output[2];
    uint32_t _last_state_ms;
    SRV_Channel::Aux_servo_function_t _srv_function[2];
};    

class UserNetgun {
public:
    void Init();
    void Update();
    void Fire(uint16_t channel);
    void Cut(uint16_t channel);
    void Stabilize();
    void set_target_angle(float angle_in);
    void set_target(float v_in);
    void set_trim(float v_in);
    void set_stabilize(bool v_in);
    void check_param();
    void handle_info(float p1, float p2, float p3, float p4, float p5, float p6, float p7);

    UserBarrel _barrels[NETGUN_NUM];
    bool _do_stab;
    float _angle_current;
    float _angle_target;
    float _angle_stab; //cd
    float _angle_trim; //cd
    float _angle_max;
    float _angle_min;
    float _slew_rate; // cd/s
};
