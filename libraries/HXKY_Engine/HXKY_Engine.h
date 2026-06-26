#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_Logger/AP_Logger.h>
#include <FD1_UART/FD1_UART.h>

#define HXKY_ENGINE_MAX_NUM 5

// HXKY_Engine: 引擎数据库
// 支持串口直连(FD1协议)和MAVLink中继两种模式
// UEngines 管理多个引擎实例

class HXKY_Engine;

class HXKY_Engines {

public:

    HXKY_Engines();

    static HXKY_Engines *get_singleton() {
        return _singleton;
    }

    void init();
    void update();
    void send_mavlink_msg(mavlink_channel_t chan);
    void handle_message(const mavlink_message_t &msg);

    HXKY_Engine* engines[HXKY_ENGINE_MAX_NUM];

    // var_info for holding Parameter information
    static const struct AP_Param::GroupInfo var_info[];

private:

    static HXKY_Engines *_singleton;

    AP_Int16 _debug;         // 调试标志
    AP_Int16 _default_param; // 保留参数位

    friend class HXKY_Engine;
};

class HXKY_Engine {

public:

    HXKY_Engine(HXKY_Engines *frontend_in, uint8_t id_in);

    void init();
    void update();
    void read_uart();
    void write_uart();
    void check_alive();
    void send_request();
    void send_mavlink_msg(mavlink_channel_t chan);
    void handle_message(const mavlink_message_t &msg);
    void update_log();

    bool is_alive(void) const { return _alive; }
    const mavlink_hxts_hy_engine_t& get_packet(void) const { return _packet; }

    AP_HAL::UARTDriver* get_port(void) { return _port; }

private:

    HXKY_Engines *_frontend;
    uint8_t _id;
    uint32_t _last_update_ms;
    uint32_t _last_request_ms;
    uint32_t _last_log_ms;
    bool _alive;

    AP_HAL::UARTDriver* _port;

    mavlink_hxts_hy_engine_t _packet;
    FD1_msg_engine_request _uart_request;
    FD1_msg_engine_response _uart_response;
};

namespace AP {
    HXKY_Engines &hxky_engines();
};
