#pragma once

#include <FD1_UART/FD1_UART.h>

class Uart {

public:

    // constructor, destructor
    Uart();

    void init();
    void update();
    void read_uart();
    void write_uart();
    AP_HAL::UARTDriver* get_port(void) {return _port;}

    void send_0728_p1();
    void send_0728_p2();
    FD1_msg_0728_p1& get_msg_0728_p1() { return uart_msg_0728_p1; }
    FD1_msg_0728_p2& get_msg_0728_p2() { return uart_msg_0728_p2; }
    FD1_msg_0728_p3& get_msg_0728_p3() { return uart_msg_0728_p3; }
private:

    AP_HAL::UARTDriver* _port;

    // message structure
    FD1_msg_0728_p1 uart_msg_0728_p1; //规划器20250728   飞控→规划器 协议1
    FD1_msg_0728_p2 uart_msg_0728_p2; //规划器20250728   飞控→规划器 协议2
    FD1_msg_0728_p3 uart_msg_0728_p3; //规划器20250728   规划器→飞控 协议3
};
