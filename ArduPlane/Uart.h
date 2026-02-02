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

private:

    AP_HAL::UARTDriver* _port;

    // message structure
};
