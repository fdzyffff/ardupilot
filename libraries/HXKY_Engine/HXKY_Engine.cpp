/*
   HXKY_Engine: 引擎数据库
   支持串口直连(FD1协议)和MAVLink中继两种模式
*/

#include "HXKY_Engine.h"

HXKY_Engines *HXKY_Engines::_singleton;

const AP_Param::GroupInfo HXKY_Engines::var_info[] = {
    // @Param: DBUG
    // @DisplayName: Debug flags
    // @Description: Bitmask for debug output. Bit0=1Hz print on new data per engine.
    // @User: Advanced
    AP_GROUPINFO("DBUG", 1, HXKY_Engines, _debug, 0),

    // @Param: DEFLT
    // @DisplayName: Default parameter placeholder
    // @Description: Reserved parameter for future expansion. No function currently.
    // @User: Advanced
    AP_GROUPINFO("DEFLT", 2, HXKY_Engines, _default_param, 0),
    AP_GROUPEND
};

HXKY_Engines::HXKY_Engines()
{
    if (_singleton != nullptr) {
        AP_HAL::panic("HXKY_Engines must be singleton");
    }
    AP_Param::setup_object_defaults(this, var_info);
    _singleton = this;
    for (uint8_t i = 0; i < HXKY_ENGINE_MAX_NUM; i++) {
        engines[i] = nullptr;
    }
}

void HXKY_Engines::init()
{
    for (uint8_t i = 0; i < HXKY_ENGINE_MAX_NUM; i++) {
        engines[i] = new HXKY_Engine(this, i);
        if (engines[i] != nullptr) {
            engines[i]->init();
        }
    }
}

void HXKY_Engines::update()
{
    for (uint8_t i = 0; i < HXKY_ENGINE_MAX_NUM; i++) {
        if (engines[i] != nullptr) {
            engines[i]->update();
        }
    }
}

void HXKY_Engines::send_mavlink_msg(mavlink_channel_t chan)
{
    for (uint8_t i = 0; i < HXKY_ENGINE_MAX_NUM; i++) {
        if (engines[i] != nullptr) {
            engines[i]->send_mavlink_msg(chan);
        }
    }
}

void HXKY_Engines::handle_message(const mavlink_message_t &msg)
{
    for (uint8_t i = 0; i < HXKY_ENGINE_MAX_NUM; i++) {
        if (engines[i] != nullptr) {
            engines[i]->handle_message(msg);
        }
    }
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

HXKY_Engine::HXKY_Engine(HXKY_Engines *frontend_in, uint8_t id_in)
    : _frontend(frontend_in),
      _id(id_in),
      _last_update_ms(0),
      _last_request_ms(0),
      _last_log_ms(0),
      _alive(false),
      _port(nullptr)
{
    _packet.Instance = _id;
}

void HXKY_Engine::init()
{
    const AP_SerialManager &serial_manager = AP::serialmanager();
    _port = serial_manager.find_serial(AP_SerialManager::SerialProtocol_ENGINE, _id);
    if (_port != nullptr) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "HXKY_Engine %d init", _id);
    }
}

void HXKY_Engine::update()
{
    read_uart();
    write_uart();
    check_alive();
    update_log();
}

void HXKY_Engine::read_uart()
{
    if (_port == nullptr) {
        return;
    }
    while (_port->available() > 0) {
        uint8_t temp = _port->read();
        _uart_response.parse(temp);

        if (_uart_response._msg_1.updated) {
            _uart_response._msg_1.updated = false;
            _packet.Instance = _id;
            _packet.Flag = _uart_response._msg_1.content.msg.flag;
            _packet.Seconds = _uart_response._msg_1.content.msg.seconds;
            _packet.RPM = _uart_response._msg_1.content.msg.rpm;
            _packet.Coolant = _uart_response._msg_1.content.msg.coolant;
            _packet.BattVolt = _uart_response._msg_1.content.msg.batteryvoltage;

            _last_update_ms = AP_HAL::millis();

            if (_frontend != nullptr && _frontend->_debug.get() == 1) {
                static uint32_t last_print_ms = 0;
                if (AP_HAL::millis() - last_print_ms > 1000) {
                    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "[%d]-Seconds %d", _id, _uart_response._msg_1.content.msg.seconds);
                    last_print_ms = AP_HAL::millis();
                }
            }
        }
    }
}

void HXKY_Engine::write_uart()
{
    send_request();
}

void HXKY_Engine::check_alive()
{
    if (_last_update_ms < 10000) {
        return;
    }
    const uint32_t now = AP_HAL::millis();
    if (now - _last_update_ms > 5000) {
        if (_alive) {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "HXKY_Engine %d lost", _id);
        }
        _alive = false;
    } else {
        if (!_alive) {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "HXKY_Engine %d connect", _id);
        }
        _alive = true;
    }
}

void HXKY_Engine::send_request()
{
    const uint32_t now = AP_HAL::millis();
    if (now - _last_request_ms < 200) {
        return;
    }
    _last_request_ms = now;

    if (_port == nullptr) {
        return;
    }
    _uart_request.make_sum();
    _port->write(_uart_request._msg_1.content.data, sizeof(_uart_request._msg_1.content.data));
}

void HXKY_Engine::send_mavlink_msg(mavlink_channel_t chan)
{
    if (!_alive) {
        return;
    }
    mavlink_msg_hxts_hy_engine_send_struct(chan, &_packet);
}

void HXKY_Engine::handle_message(const mavlink_message_t &msg)
{
    if (_port != nullptr) {
        return;
    }
    if (msg.msgid == MAVLINK_MSG_ID_HXTS_HY_ENGINE) {
        mavlink_hxts_hy_engine_t packet;
        mavlink_msg_hxts_hy_engine_decode(&msg, &packet);
        if (packet.Instance == _id) {
            _packet = packet;
            _uart_response._msg_1.content.msg.flag = packet.Flag;
            _uart_response._msg_1.content.msg.seconds = packet.Seconds;
            _uart_response._msg_1.content.msg.rpm = packet.RPM;
            _uart_response._msg_1.content.msg.coolant = packet.Coolant;
            _uart_response._msg_1.content.msg.batteryvoltage = packet.BattVolt;
            _last_update_ms = AP_HAL::millis();
        }
    }
}

void HXKY_Engine::update_log()
{
    if (!_alive) {
        return;
    }
    const uint32_t now_ms = AP_HAL::millis();
    if (now_ms - _last_log_ms < 500) {
        return;
    }
    _last_log_ms = now_ms;

    AP::logger().WriteStreaming("HXEN1",
                                "TimeUS,I,Flag,T,PW1,PW2,RPM,ADV,Baro,map,mat,clnt,tps,batv",
                                "s#------------",
                                "F-------------",
                                "QBBHHHHhhhhhhh",
                                AP_HAL::micros64(),
                                (uint8_t)_id,
                                (uint16_t)_uart_response._msg_1.content.msg.flag,
                                (uint16_t)_uart_response._msg_1.content.msg.seconds,
                                (uint16_t)_uart_response._msg_1.content.msg.pulsewidth1,
                                (uint16_t)_uart_response._msg_1.content.msg.pulsewidth2,
                                (uint16_t)_uart_response._msg_1.content.msg.rpm,
                                (int16_t)_uart_response._msg_1.content.msg.advance,
                                (int16_t)_uart_response._msg_1.content.msg.barometer,
                                (int16_t)_uart_response._msg_1.content.msg.map,
                                (int16_t)_uart_response._msg_1.content.msg.mat,
                                (int16_t)_uart_response._msg_1.content.msg.coolant,
                                (int16_t)_uart_response._msg_1.content.msg.tps,
                                (int16_t)_uart_response._msg_1.content.msg.batteryvoltage);

    AP::logger().WriteStreaming("HXEN2",
                                "TimeUS,I,afr1,afr2,barc,gamh,ve1,cad,tpsd,mapd,egov1,egov2",
                                "s#----------",
                                "F-----------",
                                "QBhhhhhhhhhh",
                                AP_HAL::micros64(),
                                (uint8_t)_id,
                                (int16_t)_uart_response._msg_1.content.msg.afr1,
                                (int16_t)_uart_response._msg_1.content.msg.afr2,
                                (int16_t)_uart_response._msg_1.content.msg.barocorrection,
                                (int16_t)_uart_response._msg_1.content.msg.gammaenrich,
                                (int16_t)_uart_response._msg_1.content.msg.ve1,
                                (int16_t)_uart_response._msg_1.content.msg.cold_adv_deg,
                                (int16_t)_uart_response._msg_1.content.msg.tpsdot,
                                (int16_t)_uart_response._msg_1.content.msg.mapdot,
                                (int16_t)_uart_response._msg_1.content.msg.egov1,
                                (int16_t)_uart_response._msg_1.content.msg.egov2);
}
