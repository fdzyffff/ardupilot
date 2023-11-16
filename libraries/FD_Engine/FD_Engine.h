#pragma once

#include <AP_Common/AP_Common.h>
#include <FD1_UART/FD1_UART.h>
#include <SRV_Channel/SRV_Channel.h>
#include <AP_SerialManager/AP_SerialManager.h>      // Serial manager library
#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS.h>

class Engine_backend;

class FD_Engine {
    friend class Engine_backend;
    friend class FD_Engine_EP4;
    friend class FD_Engine_LUTAN;
public:
    FD_Engine();

    enum class EngineState{
        Stop = 0,
        Prepare = 1,
        Start = 2,
        Running = 3,
    };

    struct status_t {
        uint16_t rpm;
        uint16_t fuel_pressure;
        uint16_t cylinder_temp1;
        uint16_t cylinder_temp2;
        uint16_t venting_temp1;
        uint16_t venting_temp2;
    } status;

    void init();
    void update();
    void start();
    void stop();
    void test_uart(uint8_t msg_id, uint8_t option);
    bool valid();

    // void send_mav();

    // parameter var table
    static const struct AP_Param::GroupInfo var_info[];

private:
    Engine_backend* driver;

    int8_t type;

    AP_Int8 engine_type;
    AP_Float throttle_min;
};