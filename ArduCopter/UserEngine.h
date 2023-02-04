#pragma once

#include <FD_UART/FD_UART.h>
#define ENGINE_NUM 7

class UserEngine {
public:
    void Init();                             //串口初始化
    bool Connected();                        //初始化和连接判定
    bool IsState(EngineState in_state);      //是否处于相应状态
    void UpdateState();

    enum class EngineState{
        Lock = 0;
        Standby = 1;
        Running = 2;
    };

    FD_UART* _uart;
    EngineState _state;
}

class UserEngines {
public:
    void Init();            //串口初始化
    bool Check_Connected(); //初始化和连接判定
    bool IsState();         //是否处于相应状态
    void Update();

    enum class UserEnginesState{
        Stop = 0;
        Start = 1;
        None = 99;
    };

    UserEngine _engine[ENGINE_NUM];
    UserEnginesState _state;
    uint32_t last_state_ms;
}
