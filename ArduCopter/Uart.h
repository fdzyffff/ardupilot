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
    void pack_rid_msg();
    void pack_rid_msg_test();
    void pack_rid_text();
    AP_HAL::UARTDriver* get_port(void) {return _port;}

private:

    AP_HAL::UARTDriver* _port;

    uint32_t _last_rid_ms;
    uint32_t _rid_text_start_ms;
    // message structure
    FD1_msg_RID _msg_RID;
};
