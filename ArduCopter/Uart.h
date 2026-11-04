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
    void write_uart_rid();
    void write_uart_rtkrid();
    void pack_rid_msg();
    void pack_rid_msg_test();
    void pack_rid_text();
    uint8_t get_ua_status() const;
    AP_HAL::UARTDriver* get_port(void) {return _port;}

private:

    // 协议号 50：老 RID 模块，203 字节二进制国标帧
    AP_HAL::UARTDriver* _port;
    uint32_t _last_rid_ms;
    uint32_t _rid_text_start_ms;

    // 协议号 51：RID + RTK 一体化板卡，ASCII 命令（*NAME_W:value#）
    AP_HAL::UARTDriver* _port_rtkrid;
    uint32_t _last_rtkrid_identity_ms;
    bool _rtkrid_cfg_reported;

    // message structure
    FD1_msg_RID _msg_RID;
    FD1_msg_RTKRID _msg_RTKRID;
};
