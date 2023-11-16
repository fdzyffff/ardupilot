#pragma once

#include <FD1_UART/FD1_UART.h>
#include <SRV_Channel/SRV_Channel.h>
#include <AP_SerialManager/AP_SerialManager.h>      // Serial manager library
#include "FD_Engine.h"

class Engine_backend {
public:

    Engine_backend(FD_Engine &_engine);

    virtual void init() = 0;
    virtual void update() = 0;
    virtual void start() = 0;
    virtual void stop() = 0;
    virtual void test_uart(uint8_t msg_id, uint8_t option) = 0;
    virtual bool valid() = 0;

protected:
    FD_Engine &engine;                        ///< access to frontend (for parameters)

};
