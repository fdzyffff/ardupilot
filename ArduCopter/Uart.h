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
    void send_reply(uint8_t cmd_type);
    void send_0x11();
    void send_0x22();
    AP_HAL::UARTDriver* get_port(void) {return _port;}

    FD1_msg_reply& get_msg_reply() { return uart_msg_reply; }
    FD1_msg_0x11& get_msg_0x11() { return uart_msg_0x11; }
    FD1_msg_0x22& get_msg_0x22() { return uart_msg_0x22; }
    FD1_msg_0x31& get_msg_0x31() { return uart_msg_0x31; }
    FD1_msg_0x33& get_msg_0x33() { return uart_msg_0x33; }
    FD1_msg_0x36& get_msg_0x36() { return uart_msg_0x36; }
    FD1_msg_0x37& get_msg_0x37() { return uart_msg_0x37; }

private:

    AP_HAL::UARTDriver* _port;

    // message structure
    FD1_msg_reply uart_msg_reply; //通用应答格式  飞控→任务
    FD1_msg_0x11 uart_msg_0x11; //飞控状态信息    飞控→任务
    FD1_msg_0x22 uart_msg_0x22; //开始飞行        飞控→任务
    FD1_msg_0x31 uart_msg_0x31; //航点飞行指令    任务→飞控
    FD1_msg_0x33 uart_msg_0x33; //回收/自毁指令   任务→飞控
    FD1_msg_0x36 uart_msg_0x36; //进入攻击指令    任务→飞控
    FD1_msg_0x37 uart_msg_0x37; //退出攻击指令    任务→飞控
};
