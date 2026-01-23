#pragma once

#include <FD_UART/FD_UART.h>

class Uart {

public:

    // constructor, destructor
    Uart();

    void init();
    void update();
    void read_uart();
    void write_uart();
    AP_HAL::UARTDriver* get_port(void) {return _port;}

    void pack_status();
    void handle_LS_control_receive();

private:

    AP_HAL::UARTDriver* _port;

    // message structure
    FD1_msg_LS_control uart_msg_LS_control;
    FD1_msg_LS_status uart_msg_LS_status;
};
