#pragma once

#include <FD1_UART/FD1_UART.h>

class UWeight {

public:

    // constructor, destructor
    UWeight();

    void init();
    void update();
    void read_uart();
    void write_uart();
    void set_rpm(uint16_t rpm_in);
    void send_request();

    AP_HAL::UARTDriver* get_port(void) {return _port;}


private:

    AP_HAL::UARTDriver* _port;

    mavlink_hxts_hy_weight_t hxts_hy_weight_packet;
    // message structure
    FD1_msg_weight uart_msg_weight;
};
