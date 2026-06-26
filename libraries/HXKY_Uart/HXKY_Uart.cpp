/*
   HXKY_Uart: 底层串口协议库
   提供 FD1 协议解析和消息访问接口
   业务逻辑留在 ArduPlane 层
*/

#include "HXKY_Uart.h"
#include <AP_HAL/AP_HAL.h>

HXKY_Uart *HXKY_Uart::_singleton;

const AP_Param::GroupInfo HXKY_Uart::var_info[] = {
    // @Param: DEFLT
    // @DisplayName: Default parameter placeholder
    // @Description: Reserved parameter for future expansion. No function currently.
    // @User: Advanced
    AP_GROUPINFO("DEFLT", 1, HXKY_Uart, _default_param, 0),
    AP_GROUPEND
};

HXKY_Uart::HXKY_Uart()
    : _port(nullptr),
      _new_0x31(false),
      _new_0x33(false),
      _new_0x36(false),
      _new_0x37(false)
{
    if (_singleton != nullptr) {
        AP_HAL::panic("HXKY_Uart must be singleton");
    }
    AP_Param::setup_object_defaults(this, var_info);
    _singleton = this;
}

void HXKY_Uart::init()
{
    const AP_SerialManager &serial_manager = AP::serialmanager();
    _port = serial_manager.find_serial(AP_SerialManager::SerialProtocol_UART, 0);
    if (_port != nullptr) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "HXKY_Uart init");
    }
}

void HXKY_Uart::update()
{
    read_uart();
    write_uart();
}

void HXKY_Uart::read_uart()
{
    if (_port == nullptr) {
        return;
    }
    while (_port->available() > 0) {
        uint8_t temp = _port->read();
        _msg_0x31.parse(temp);
        _msg_0x33.parse(temp);
        _msg_0x36.parse(temp);
        _msg_0x37.parse(temp);

        if (_msg_0x31._msg_1.updated) {
            _new_0x31 = true;
            _msg_0x31._msg_1.updated = false;
        }
        if (_msg_0x33._msg_1.updated) {
            _new_0x33 = true;
            _msg_0x33._msg_1.updated = false;
        }
        if (_msg_0x36._msg_1.updated) {
            _new_0x36 = true;
            _msg_0x36._msg_1.updated = false;
        }
        if (_msg_0x37._msg_1.updated) {
            _new_0x37 = true;
            _msg_0x37._msg_1.updated = false;
        }
    }
}

void HXKY_Uart::write_uart()
{
    // 业务层通过 send_reply / send_0x11 / send_0x22 主动调用
    // 这里不做自动发送
}

void HXKY_Uart::send_reply(uint8_t cmd_type)
{
    if (_port == nullptr) {
        return;
    }
    _msg_reply._msg_1.content.msg.header.head_1 = _msg_reply.PREAMBLE1;
    _msg_reply._msg_1.content.msg.header.head_2 = _msg_reply.PREAMBLE2;
    _msg_reply._msg_1.content.msg.length = _msg_reply._msg_1.length;
    _msg_reply._msg_1.content.msg.cmd_type = cmd_type;
    _msg_reply.make_sum();
    _port->write(_msg_reply._msg_1.content.data, sizeof(_msg_reply._msg_1.content.data));
}

void HXKY_Uart::send_0x11(const struct HXKY_Uart_0x11_data& data)
{
    if (_port == nullptr) {
        return;
    }
    _msg_0x11._msg_1.content.msg.header.head_1 = _msg_0x11.PREAMBLE1;
    _msg_0x11._msg_1.content.msg.header.head_2 = _msg_0x11.PREAMBLE2;
    _msg_0x11._msg_1.content.msg.length = _msg_0x11._msg_1.length;
    _msg_0x11._msg_1.content.msg.cmd_type = 0x11;

    _msg_0x11._msg_1.content.msg.gps_lng = data.gps_lng;
    _msg_0x11._msg_1.content.msg.gps_lat = data.gps_lat;
    _msg_0x11._msg_1.content.msg.relative_alt = data.relative_alt;
    _msg_0x11._msg_1.content.msg.absolute_alt = data.absolute_alt;
    _msg_0x11._msg_1.content.msg.baro_alt = data.baro_alt;
    _msg_0x11._msg_1.content.msg.pitch_angle = data.pitch_angle;
    _msg_0x11._msg_1.content.msg.roll_angle = data.roll_angle;
    _msg_0x11._msg_1.content.msg.yaw_angle = data.yaw_angle;
    _msg_0x11._msg_1.content.msg.airspeed = data.airspeed;
    _msg_0x11._msg_1.content.msg.vel_n = data.vel_n;
    _msg_0x11._msg_1.content.msg.vel_e = data.vel_e;
    _msg_0x11._msg_1.content.msg.vel_d = data.vel_d;
    _msg_0x11._msg_1.content.msg.rest_time = data.rest_time;
    _msg_0x11._msg_1.content.msg.status = data.status;
    _msg_0x11._msg_1.content.msg.gps_count = data.gps_count;
    _msg_0x11._msg_1.content.msg.pos_source = data.pos_source;
    _msg_0x11._msg_1.content.msg.flight_mode = data.flight_mode;
    _msg_0x11._msg_1.content.msg.time2000 = data.time2000;
    _msg_0x11._msg_1.content.msg.vel_lat = data.vel_lat;
    _msg_0x11._msg_1.content.msg.vel_lng = data.vel_lng;
    _msg_0x11._msg_1.content.msg.vel_alt = data.vel_alt;
    _msg_0x11._msg_1.content.msg.pitch_rate = data.pitch_rate;
    _msg_0x11._msg_1.content.msg.roll_rate = data.roll_rate;
    _msg_0x11._msg_1.content.msg.yaw_rate = data.yaw_rate;

    _msg_0x11.make_sum();
    _port->write(_msg_0x11._msg_1.content.data, sizeof(_msg_0x11._msg_1.content.data));
}

void HXKY_Uart::send_0x22(uint8_t status)
{
    if (_port == nullptr) {
        return;
    }
    _msg_0x22._msg_1.content.msg.header.head_1 = FD1_msg_reply::PREAMBLE1;
    _msg_0x22._msg_1.content.msg.header.head_2 = FD1_msg_reply::PREAMBLE2;
    _msg_0x22._msg_1.content.msg.length = _msg_0x22._msg_1.length;
    _msg_0x22._msg_1.content.msg.cmd_type = 0x22;
    _msg_0x22._msg_1.content.msg.status = status;

    _msg_0x22.make_sum();
    _port->write(_msg_0x22._msg_1.content.data, sizeof(_msg_0x22._msg_1.content.data));
}

float HXKY_Uart::get_sf_lng(void) const
{
    return _msg_0x11.SF_LNG;
}

float HXKY_Uart::get_sf_lat(void) const
{
    return _msg_0x11.SF_LAT;
}
