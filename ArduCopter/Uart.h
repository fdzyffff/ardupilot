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
    void pack_uom_msg();
    AP_HAL::UARTDriver* get_port(void) {return _port;}

private:

    AP_HAL::UARTDriver* _port;

    uint32_t _last_uom_ms;
    // message structure
    FD1_msg_UOM _msg_UOM;
};
