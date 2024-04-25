#pragma once

#include <HB1_UART/HB1_UART.h>
#include <AP_SerialManager/AP_SerialManager.h>      // Serial manager library

class UPower {

public:

    // constructor, destructor
    UPower();

    // initialise
    void init();
    void update();
    void pwm_update();
    void state_update();
    void uart_update();
    void throttle_update();
    bool running();
    int8_t engine_type();
    void set_Action(Power_Action_t action, bool Force_set = false);
    void msg_apm2power_set();
    void msg_apm2power_set_rocket_on();

    enum class Power_Action_t {
        None                   = 0,
        RocketON               = 1,
        EnginePullUP           = 2,
        EngineON               = 3,
        EngineOFF              = 4,
        ParachuteON            = 5,
        GROUND_RocketON        = 6,
        GROUND_EngineSTART_PRE = 7,
        GROUND_EngineSTART     = 8,
        GROUND_EngineON        = 9,
        GROUND_EngineOFF       = 10,
        GROUND_EngineFULL      = 11,
        GROUND_EngineMID       = 12,
    };

    HB1_UART uart_power{AP_SerialManager::SerialProtocol_HB1_POWER};
    Power_Action_t state;
    uint32_t state_timer;
    LowPassFilterFloat engine_rpm;
    uint8_t engine_status;
    float engine_temp;
    float engine_fuel;
    uint32_t status_ms;
    int8_t send_counter;
    int16_t engine_startcount;
};
