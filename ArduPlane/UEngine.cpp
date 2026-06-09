/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.
   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/


#include "Plane.h"

UEngines::UEngines()
{
    ;
}

// initialise
void UEngines::init()
{
    for (uint8_t i_engine = 0; i_engine < UENGINE_MAX_NUM; i_engine++) {
        engines[i_engine] = new UEngine(this, i_engine);
        if (engines[i_engine] != nullptr) {
            engines[i_engine]->init();
        }
    }
}

void UEngines::update()
{
    for (uint8_t i_engine = 0; i_engine < UENGINE_MAX_NUM; i_engine++) {
        if (engines[i_engine] != nullptr) {
            engines[i_engine]->update();
        }
    }
}

// void UEngines::set_rpm(uint8_t id_in, uint16_t rpm_in) 
// {
//     if (id_in < UENGINE_MAX_NUM) {
//         if (engines[id_in] != nullptr) {
//             engines[id_in]->set_rpm(rpm_in);
//         }
//     }
// }

void UEngines::send_mavlink_msg(mavlink_channel_t chan)
{
    for (uint8_t i_engine = 0; i_engine < UENGINE_MAX_NUM; i_engine++) {
        if (engines[i_engine] != nullptr) {
            engines[i_engine]->send_mavlink_msg(chan);
        }
    }
}

void UEngines::handle_message(const mavlink_message_t &msg)
{
    for (uint8_t i_engine = 0; i_engine < UENGINE_MAX_NUM; i_engine++) {
        if (engines[i_engine] != nullptr) {
            engines[i_engine]->handle_message(msg);
        }
    }
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

UEngine::UEngine(UEngines *fronted_in, uint8_t id_in)
{
    _fronted = fronted_in;
    _id = id_in;
    hxts_hy_engine_packet.Instance = _id;
}

// initialise
void UEngine::init()
{
    _port = nullptr;
    const AP_SerialManager &serial_manager = AP::serialmanager();

    // check for protocol configured for a serial port - only the first serial port with one of these protocols will then run (cannot have FrSky on multiple serial ports)
    _port = serial_manager.find_serial(AP_SerialManager::SerialProtocol_ENGINE, _id);
    if (_port != nullptr) {
        gcs().send_text(MAV_SEVERITY_WARNING, "UEngine %d init", _id);
        return;
    }
}

void UEngine::handle_message(const mavlink_message_t &msg)
{
    // only work without uart protocol
    if (get_port() != nullptr) {return;}
    if (msg.msgid == MAVLINK_MSG_ID_HXTS_HY_ENGINE) {
        // decode packet
        // gcs().send_text(MAV_SEVERITY_WARNING, "Target mavpkg");
        // decode packet
        mavlink_hxts_hy_engine_t packet;
        mavlink_msg_hxts_hy_engine_decode(&msg, &packet);
        if (packet.Instance == _id) {
            mavlink_msg_hxts_hy_engine_decode(&msg, &hxts_hy_engine_packet);
            uart_engine_response._msg_1.content.msg.flag = hxts_hy_engine_packet.Flag;
            uart_engine_response._msg_1.content.msg.seconds = hxts_hy_engine_packet.seconds;
            uart_engine_response._msg_1.content.msg.pulsewidth1 = hxts_hy_engine_packet.pulsewidth1;
            uart_engine_response._msg_1.content.msg.pulsewidth2 = hxts_hy_engine_packet.pulsewidth2;
            uart_engine_response._msg_1.content.msg.rpm = hxts_hy_engine_packet.rpm;
            uart_engine_response._msg_1.content.msg.advance = hxts_hy_engine_packet.advance;
            uart_engine_response._msg_1.content.msg.barometer = hxts_hy_engine_packet.barometer;
            uart_engine_response._msg_1.content.msg.map = hxts_hy_engine_packet.map;
            uart_engine_response._msg_1.content.msg.mat = hxts_hy_engine_packet.mat;
            uart_engine_response._msg_1.content.msg.coolant = hxts_hy_engine_packet.coolant;
            uart_engine_response._msg_1.content.msg.tps = hxts_hy_engine_packet.tps;
            uart_engine_response._msg_1.content.msg.batteryvoltage = hxts_hy_engine_packet.batteryvoltage;
            uart_engine_response._msg_1.content.msg.afr1 = hxts_hy_engine_packet.afr1;
            uart_engine_response._msg_1.content.msg.afr2 = hxts_hy_engine_packet.afr2;
            uart_engine_response._msg_1.content.msg.barocorrection = hxts_hy_engine_packet.barocorrection;
            uart_engine_response._msg_1.content.msg.gammaenrich = hxts_hy_engine_packet.gammaenrich;
            uart_engine_response._msg_1.content.msg.ve1 = hxts_hy_engine_packet.ve1;
            uart_engine_response._msg_1.content.msg.cold_adv_deg = hxts_hy_engine_packet.cold_adv_deg;
            uart_engine_response._msg_1.content.msg.tpsdot = hxts_hy_engine_packet.tpsdot;
            uart_engine_response._msg_1.content.msg.mapdot = hxts_hy_engine_packet.mapdot;
            uart_engine_response._msg_1.content.msg.egov1 = hxts_hy_engine_packet.egov1;
            uart_engine_response._msg_1.content.msg.egov2 = hxts_hy_engine_packet.egov2;
            _last_update_ms = millis();
        }
    }
}

void UEngine::update()
{
    read_uart();
    write_uart();
    check_alive();
}

void UEngine::read_uart()
{
    if (get_port() == nullptr) {return;}
    while (get_port()->available()>0) {
        uint8_t temp = get_port()->read();
        uart_engine_response.parse(temp);

        if (uart_engine_response._msg_1.updated) {
            uart_engine_response._msg_1.updated = false;
            hxts_hy_engine_packet.Instance = _id;
            hxts_hy_engine_packet.Flag = uart_engine_response._msg_1.content.msg.flag;
            hxts_hy_engine_packet.seconds = uart_engine_response._msg_1.content.msg.seconds;
            hxts_hy_engine_packet.pulsewidth1 = uart_engine_response._msg_1.content.msg.pulsewidth1;
            hxts_hy_engine_packet.pulsewidth2 = uart_engine_response._msg_1.content.msg.pulsewidth2;
            hxts_hy_engine_packet.rpm = uart_engine_response._msg_1.content.msg.rpm;
            hxts_hy_engine_packet.advance = uart_engine_response._msg_1.content.msg.advance;
            hxts_hy_engine_packet.barometer = uart_engine_response._msg_1.content.msg.barometer;
            hxts_hy_engine_packet.map = uart_engine_response._msg_1.content.msg.map;
            hxts_hy_engine_packet.mat = uart_engine_response._msg_1.content.msg.mat;
            hxts_hy_engine_packet.coolant = uart_engine_response._msg_1.content.msg.coolant;
            hxts_hy_engine_packet.tps = uart_engine_response._msg_1.content.msg.tps;
            hxts_hy_engine_packet.batteryvoltage = uart_engine_response._msg_1.content.msg.batteryvoltage;
            hxts_hy_engine_packet.afr1 = uart_engine_response._msg_1.content.msg.afr1;
            hxts_hy_engine_packet.afr2 = uart_engine_response._msg_1.content.msg.afr2;
            hxts_hy_engine_packet.barocorrection = uart_engine_response._msg_1.content.msg.barocorrection;
            hxts_hy_engine_packet.gammaenrich = uart_engine_response._msg_1.content.msg.gammaenrich;
            hxts_hy_engine_packet.ve1 = uart_engine_response._msg_1.content.msg.ve1;
            hxts_hy_engine_packet.cold_adv_deg = uart_engine_response._msg_1.content.msg.cold_adv_deg;
            hxts_hy_engine_packet.tpsdot = uart_engine_response._msg_1.content.msg.tpsdot;
            hxts_hy_engine_packet.mapdot = uart_engine_response._msg_1.content.msg.mapdot;
            hxts_hy_engine_packet.egov1 = uart_engine_response._msg_1.content.msg.egov1;
            hxts_hy_engine_packet.egov2 = uart_engine_response._msg_1.content.msg.egov2;

            _last_update_ms = millis();

            if (plane.g2.user_debug_engine.get() == 1) {
                if (millis() - _last_print_ms > 1000) {
                    gcs().send_text(MAV_SEVERITY_INFO, "[%d]-Seconds %d",_id, uart_engine_response._msg_1.content.msg.seconds);
                    _last_print_ms = millis();
                }
            }
            // gcs().send_text(MAV_SEVERITY_INFO, "Coolant %d",uart_engine_response._msg_1.content.msg.coolant);

            // gcs().send_text(MAV_SEVERITY_INFO, "Baro %d",uart_engine_response._msg_1.content.msg.barometer);
        }
    }
}

void UEngine::write_uart()
{
    send_request();
}

void UEngine::check_alive()
{
    if (_last_update_ms < 10000) {return;}
    if (millis() - _last_update_ms > 5000) {
        if (_alive) {
            gcs().send_text(MAV_SEVERITY_INFO, "Engine %d lost", _id);
        }
        _alive = false;
    } else {
        if (!_alive) {
            gcs().send_text(MAV_SEVERITY_INFO, "Engine %d connect", _id);
        }
        _alive = true;
    }
}

void UEngine::send_request()
{
    // check send condition
    uint32_t now = millis();
    if (now - _last_request_ms < 200) {
        return;
    }
    _last_request_ms = now;

    if (get_port() == nullptr) {return;}
    uart_engine_request.make_sum();

    get_port()->write(uart_engine_request._msg_1.content.data, sizeof(uart_engine_request._msg_1.content.data));
}

void UEngine::send_mavlink_msg(mavlink_channel_t chan)
{
    if (!_alive) {return;}
    mavlink_msg_hxts_hy_engine_send_struct(chan, &hxts_hy_engine_packet);
}

void UEngine::update_log()
{
    if (!_alive) {return;}
    uint32_t now_ms = millis();
    if (now_ms - _last_log_ms < 500) {return;}

    _last_log_ms = now_ms;

    AP::logger().WriteStreaming("UWGT",
                                "TimeUS,I,Flag,T,PW1,PW2,RPM,ADV,Baro,map,mat,clnt,tps,batv",
                                "s#------------",
                                "F-------------",
                                "QBBHHHHhhhhhhh",
                                AP_HAL::micros64(),
                                (uint8_t)hxts_hy_engine_packet.Instance,
                                (uint16_t)hxts_hy_engine_packet.Flag,
                                (uint16_t)hxts_hy_engine_packet.seconds,
                                (uint16_t)hxts_hy_engine_packet.pulsewidth1,
                                (uint16_t)hxts_hy_engine_packet.pulsewidth2,
                                (uint16_t)hxts_hy_engine_packet.rpm,
                                (int16_t)hxts_hy_engine_packet.advance,
                                (int16_t)hxts_hy_engine_packet.barometer,
                                (int16_t)hxts_hy_engine_packet.map,
                                (int16_t)hxts_hy_engine_packet.mat,
                                (int16_t)hxts_hy_engine_packet.coolant,
                                (int16_t)hxts_hy_engine_packet.tps,
                                (int16_t)hxts_hy_engine_packet.batteryvoltage);

    AP::logger().WriteStreaming("UWG2",
                                "TimeUS,I,afr1,afr2,barc,gamh,ve1,cad,tps,mapd,egov1,egov2",
                                "s#----------",
                                "F-----------",
                                "QBhhhhhhhhhh",
                                AP_HAL::micros64(),
                                (uint8_t)hxts_hy_engine_packet.Instance,
                                (int16_t)hxts_hy_engine_packet.afr1,
                                (int16_t)hxts_hy_engine_packet.afr2,
                                (int16_t)hxts_hy_engine_packet.barocorrection,
                                (int16_t)hxts_hy_engine_packet.gammaenrich,
                                (int16_t)hxts_hy_engine_packet.ve1,
                                (int16_t)hxts_hy_engine_packet.cold_adv_deg,
                                (int16_t)hxts_hy_engine_packet.tpsdot,
                                (int16_t)hxts_hy_engine_packet.mapdot,
                                (int16_t)hxts_hy_engine_packet.egov1,
                                (int16_t)hxts_hy_engine_packet.egov2);
}
