#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_Logger/AP_Logger.h>
#include <FD1_UART/FD1_UART.h>

// HXKY_Weight: 称重传感器库
// 支持串口直连(FD1协议)和MAVLink中继两种模式

class HXKY_Weight {

public:

    HXKY_Weight();

    static HXKY_Weight *get_singleton() {
        return _singleton;
    }

    void init();
    void update();

    void read_uart();
    void write_uart();
    void check_alive();
    void send_request();
    void send_mavlink_msg(mavlink_channel_t chan);
    void handle_message(const mavlink_message_t &msg);
    void update_log();

    void do_print();
    bool is_alive(void) const { return _alive; }
    float get_front(void) const { return _packet.Front; }
    float get_left(void) const { return _packet.LEFT; }
    float get_right(void) const { return _packet.RIGHT; }
    const mavlink_hxts_hy_weight_t& get_packet(void) const { return _packet; }

    AP_HAL::UARTDriver* get_port(void) { return _port; }

    // var_info for holding Parameter information
    static const struct AP_Param::GroupInfo var_info[];

private:

    static HXKY_Weight *_singleton;

    AP_Int16 _print;        // 调试打印标志
    AP_Int16 _default_param; // 保留参数位

    uint32_t _last_update_ms;
    uint32_t _last_log_ms;
    bool _alive;

    AP_HAL::UARTDriver* _port;

    mavlink_hxts_hy_weight_t _packet;
    FD1_msg_weight _uart_msg;
};

namespace AP {
    HXKY_Weight &hxky_weight();
};
