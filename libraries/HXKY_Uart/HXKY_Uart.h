#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <GCS_MAVLink/GCS.h>
#include <FD1_UART/FD1_UART.h>

// HXKY_Uart: 底层串口协议库，剥离业务逻辑
// 提供 FD1 协议解析和消息访问接口
// 业务逻辑（GUIDED切换、航点设置等）留在 ArduPlane 层

class HXKY_Uart {

public:

    HXKY_Uart();

    static HXKY_Uart *get_singleton() {
        return _singleton;
    }

    void init();
    void update();

    // 串口访问
    AP_HAL::UARTDriver* get_port(void) { return _port; }

    // 消息访问接口（业务层查询）
    bool has_new_0x31(void) const { return _new_0x31; }
    bool has_new_0x33(void) const { return _new_0x33; }
    bool has_new_0x36(void) const { return _new_0x36; }
    bool has_new_0x37(void) const { return _new_0x37; }

    void clear_0x31(void) { _new_0x31 = false; }
    void clear_0x33(void) { _new_0x33 = false; }
    void clear_0x36(void) { _new_0x36 = false; }
    void clear_0x37(void) { _new_0x37 = false; }

    const FD1_msg_0x31& get_msg_0x31(void) const { return _msg_0x31; }
    const FD1_msg_0x33& get_msg_0x33(void) const { return _msg_0x33; }
    const FD1_msg_0x36& get_msg_0x36(void) const { return _msg_0x36; }
    const FD1_msg_0x37& get_msg_0x37(void) const { return _msg_0x37; }

    // 发送接口
    void send_reply(uint8_t cmd_type);
    void send_0x11(const struct HXKY_Uart_0x11_data& data);
    void send_0x22(uint8_t status);

    // 缩放因子访问（send_0x11 需要）
    float get_sf_lng(void) const;
    float get_sf_lat(void) const;

    // var_info for holding Parameter information
    static const struct AP_Param::GroupInfo var_info[];

private:

    void read_uart();
    void write_uart();

    AP_Int16 _default_param;  // 保留参数位，方便以后扩展

    static HXKY_Uart *_singleton;

    AP_HAL::UARTDriver* _port;

    // 解析状态
    FD1_msg_0x31 _msg_0x31;
    FD1_msg_0x33 _msg_0x33;
    FD1_msg_0x36 _msg_0x36;
    FD1_msg_0x37 _msg_0x37;

    bool _new_0x31;
    bool _new_0x33;
    bool _new_0x36;
    bool _new_0x37;

    // 发送消息结构
    FD1_msg_reply _msg_reply;
    FD1_msg_0x11 _msg_0x11;
    FD1_msg_0x22 _msg_0x22;
};

namespace AP {
    HXKY_Uart &hxky_uart();
};

// 0x11 消息的数据结构（业务层填充）
struct HXKY_Uart_0x11_data {
    float gps_lng;
    float gps_lat;
    float relative_alt;
    float absolute_alt;
    float baro_alt;
    float pitch_angle;
    float roll_angle;
    float yaw_angle;
    float airspeed;
    float vel_n;
    float vel_e;
    float vel_d;
    uint16_t rest_time;
    uint8_t status;
    uint8_t gps_count;
    uint8_t pos_source;
    uint8_t flight_mode;
    uint32_t time2000;
    float vel_lat;
    float vel_lng;
    float vel_alt;
    float pitch_rate;
    float roll_rate;
    float yaw_rate;
};
