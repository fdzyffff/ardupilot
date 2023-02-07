#pragma once

#include <FD_UART/FD_UART.h>
#include <SRV_Channel/SRV_Channel.h>
#define ENGINE_NUM 7

class UserEngine {
public:
    enum class EngineState{
        Boost_1 = 0,
        Boost_2 = 1,
        Boost_3 = 2,
        Boost_4 = 3,
        Normal = 10,
        Brake = 20
    };

    void Init(AP_SerialManager::SerialProtocol in_protocol, SRV_Channel::Aux_servo_function_t in_srv_function);
    void Update();
    bool connected() {return _connected;}    //初始化和连接判定
    void update_uart() ;
    void update_state();
    void boost();
    void brake();
    void set_state(UserEngine::EngineState in_state);
    bool is_state(UserEngine::EngineState in_state);
    bool can_override();
    uint16_t get_output();
    SRV_Channel::Aux_servo_function_t get_srv_function() {return _srv_function;}


private:
    FD_UART* _uart;
    EngineState _state;
    uint32_t _last_state_ms;
    bool _connected;
    uint16_t _output;
    SRV_Channel::Aux_servo_function_t _srv_function;
};

class UserEngines {
public:
    enum class UserEnginesState{
        Stop = 0,
        Start = 1,
        None = 99
    };

    void Init();
    void Update();
    void set_state(UserEngines::UserEnginesState in_state);
    bool is_state(UserEngines::UserEnginesState in_state);
    void update_state();
    void update_output();

    UserEngine _engine[ENGINE_NUM];
    UserEnginesState _state;
    uint16_t _output[ENGINE_NUM];
    uint32_t _last_state_ms;
};
