#pragma once

#include <FD1_UART/FD1_UART.h>

class UEngine {

public:

    // constructor, destructor
    UEngine();

    void init();
    void update();
    void read_uart();
    void write_uart();
    AP_HAL::UARTDriver* get_port(void) {return _port;}

    void do_engine_start();
    void do_engine_stop();
    void do_engine_standby();
    void do_engine_work();

private:

    AP_HAL::UARTDriver* _port;

    // message structure
    FD1_msg_mt400ecu uart_msg_mt400ecu; //MT 400 ECU
};
