#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <GCS_MAVLink/GCS.h>
#include <FD1_UART/FD1_UART.h>
#include <HXKY_Uart/HXKY_Uart.h>

// Uart: ArduPlane 业务层串口接口
// 使用 AP::hxky_uart() 访问底层库
// 处理业务逻辑（GUIDED切换、航点设置等）

class Uart {

public:

    Uart();

    void init();
    void update();

    // 发送接口（业务层调用）
    void send_0x11();
    void send_0x22();

    // 兼容旧接口
    AP_HAL::UARTDriver* get_port(void) { return AP::hxky_uart().get_port(); }

    FD1_msg_reply& get_msg_reply() { return _msg_reply; }
    FD1_msg_0x11& get_msg_0x11() { return _msg_0x11; }
    FD1_msg_0x22& get_msg_0x22() { return _msg_0x22; }
    FD1_msg_0x31& get_msg_0x31() { return _msg_0x31; }
    FD1_msg_0x33& get_msg_0x33() { return _msg_0x33; }
    FD1_msg_0x36& get_msg_0x36() { return _msg_0x36; }
    FD1_msg_0x37& get_msg_0x37() { return _msg_0x37; }

    // 兼容旧接口：缩放因子访问
    float get_sf_lng(void) const;
    float get_sf_lat(void) const;

private:

    // 兼容旧接口的消息结构（保留引用）
    FD1_msg_reply _msg_reply;
    FD1_msg_0x11 _msg_0x11;
    FD1_msg_0x22 _msg_0x22;
    FD1_msg_0x31 _msg_0x31;
    FD1_msg_0x33 _msg_0x33;
    FD1_msg_0x36 _msg_0x36;
    FD1_msg_0x37 _msg_0x37;
};
