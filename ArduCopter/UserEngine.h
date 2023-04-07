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
        Normal = 4,
        Running = 10,
        Brake_1 = 20,
        Brake_2 = 21
    };

    void Init(SRV_Channel::Aux_servo_function_t in_srv_function, uint8_t id_in);
    void Set_msg(FD_engine* engine_in);
    FD_engine* Get_msg();
    void Update();
    void set_connected(bool v_in);
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

    struct _msg_state_t {
        uint32_t last_ms;
    } _msg_state;
private:
    FD_engine* _msg;
    EngineState _state;
    uint32_t _last_state_ms;
    bool _connected;
    uint16_t _output;
    SRV_Channel::Aux_servo_function_t _srv_function;
    uint8_t _id;
};

class UserEngines {
public:
    enum class UserEnginesState{
        Stop = 0,
        Start = 1,
        Running = 2,
        None = 99
    };

    void Init();
    void Set_port(AP_SerialManager::SerialProtocol in_protocol);
    void Update();
    void set_state(UserEngines::UserEnginesState in_state);
    bool is_state(UserEngines::UserEnginesState in_state);
    void update_uart();
    void update_engine();
    void update_state();
    void update_output();
    void engine_msg_pack();

    UserEngine _engine[ENGINE_NUM];
    UserEnginesState _state;
    uint16_t _output[ENGINE_NUM];
    uint32_t _last_state_ms;

    uint8_t engine_msg[35];
    bool need_send;
private:
    FD_UART* _uart;
};
