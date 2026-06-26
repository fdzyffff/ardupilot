/*
   HXKY_Weight: 称重传感器库
   支持串口直连(FD1协议)和MAVLink中继两种模式
*/

#include "HXKY_Weight.h"

HXKY_Weight *HXKY_Weight::_singleton;

const AP_Param::GroupInfo HXKY_Weight::var_info[] = {
    // @Param: PRINT
    // @DisplayName: Debug print flags
    // @Description: Bitmask for debug output. Bit0=1Hz print on new data.
    // @User: Advanced
    AP_GROUPINFO("PRINT", 1, HXKY_Weight, _print, 0),

    // @Param: DEFLT
    // @DisplayName: Default parameter placeholder
    // @Description: Reserved parameter for future expansion. No function currently.
    // @User: Advanced
    AP_GROUPINFO("DEFLT", 2, HXKY_Weight, _default_param, 0),
    AP_GROUPEND
};

HXKY_Weight::HXKY_Weight()
    : _port(nullptr),
      _last_update_ms(0),
      _last_log_ms(0),
      _alive(false)
{
    if (_singleton != nullptr) {
        AP_HAL::panic("HXKY_Weight must be singleton");
    }
    AP_Param::setup_object_defaults(this, var_info);
    _singleton = this;
}

void HXKY_Weight::init()
{
    const AP_SerialManager &serial_manager = AP::serialmanager();
    _port = serial_manager.find_serial(AP_SerialManager::SerialProtocol_WEIGHT, 0);
    if (_port != nullptr) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "HXKY_Weight init");
    }
}

void HXKY_Weight::update()
{
    read_uart();
    write_uart();
    check_alive();
    update_log();
}

void HXKY_Weight::read_uart()
{
    if (_port == nullptr) {
        return;
    }
    while (_port->available() > 0) {
        uint8_t temp = _port->read();
        _uart_msg.parse(temp);

        if (_uart_msg._msg_1.updated) {
            _uart_msg._msg_1.updated = false;
            _packet.Front = _uart_msg._msg_1.content.msg.value1;
            _packet.LEFT = _uart_msg._msg_1.content.msg.value2;
            _packet.RIGHT = _uart_msg._msg_1.content.msg.value3;
            _last_update_ms = AP_HAL::millis();
        }
    }
}

void HXKY_Weight::write_uart()
{
    // 当前无写操作
}

void HXKY_Weight::check_alive()
{
    if (_last_update_ms < 10000) {
        return;
    }
    const uint32_t now = AP_HAL::millis();
    if (now - _last_update_ms > 5000) {
        if (_alive) {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "HXKY_Weight lost");
        }
        _alive = false;
    } else {
        if (!_alive) {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "HXKY_Weight connect");
        }
        _alive = true;
    }
}

void HXKY_Weight::send_mavlink_msg(mavlink_channel_t chan)
{
    if (!_alive) {
        return;
    }
    mavlink_msg_hxts_hy_weight_send_struct(chan, &_packet);
}

void HXKY_Weight::handle_message(const mavlink_message_t &msg)
{
    // 串口直连模式下不处理MAVLink中继
    if (_port != nullptr) {
        return;
    }
    if (msg.msgid == MAVLINK_MSG_ID_HXTS_HY_WEIGHT) {
        mavlink_msg_hxts_hy_weight_decode(&msg, &_packet);
        _uart_msg._msg_1.content.msg.value1 = _packet.Front;
        _uart_msg._msg_1.content.msg.value2 = _packet.LEFT;
        _uart_msg._msg_1.content.msg.value3 = _packet.RIGHT;
        _last_update_ms = AP_HAL::millis();
    }
}

void HXKY_Weight::update_log()
{
    if (!_alive) {
        return;
    }
    const uint32_t now_ms = AP_HAL::millis();
    if (now_ms - _last_log_ms < 500) {
        return;
    }
    _last_log_ms = now_ms;

    AP::logger().WriteStreaming("HXWGT",
                                "TimeUS,Front,Left,Right",
                                "s---",
                                "F---",
                                "Qfff",
                                AP_HAL::micros64(),
                                (float)_packet.Front,
                                (float)_packet.LEFT,
                                (float)_packet.RIGHT);
}

void HXKY_Weight::do_print()
{
    if (_print.get() != 1) {
        return;
    }

    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "weight: %.1f %.1f %.1f",
                  (float)_packet.Front,
                  (float)_packet.LEFT,
                  (float)_packet.RIGHT);
}
