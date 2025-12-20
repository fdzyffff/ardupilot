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

void UEngines::set_rpm(uint8_t id_in, uint16_t rpm_in) 
{
    if (id_in < UENGINE_MAX_NUM) {
        if (engines[id_in] != nullptr) {
            engines[id_in]->set_rpm(rpm_in);
        }
    }
}

void UEngines::send_mavlink_msg(mavlink_channel_t chan)
{
    for (uint8_t i_engine = 0; i_engine < UENGINE_MAX_NUM; i_engine++) {
        if (engines[i_engine] != nullptr) {
            engines[i_engine]->send_mavlink_msg(chan);
        }
    }
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

UEngine::UEngine(UEngines *fronted_in, uint8_t id_in)
{
    _fronted = fronted_in;
    _id = id_in;
}

// initialise
void UEngine::init()
{
    _port = nullptr;
    const AP_SerialManager &serial_manager = AP::serialmanager();

    // check for protocol configured for a serial port - only the first serial port with one of these protocols will then run (cannot have FrSky on multiple serial ports)
    _port = serial_manager.find_serial(AP_SerialManager::SerialProtocol_UART, 0);
    if (_port != nullptr) {
        gcs().send_text(MAV_SEVERITY_WARNING, "UEngine init");
        return;
    }
}

void UEngine::update()
{
    read_uart();
    write_uart();
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
            hxts_hy_engine_packet.Seconds = uart_engine_response._msg_1.content.msg.seconds;
            hxts_hy_engine_packet.RPM = uart_engine_response._msg_1.content.msg.rpm;
            hxts_hy_engine_packet.Coolant = uart_engine_response._msg_1.content.msg.coolant;
            hxts_hy_engine_packet.BattVolt = uart_engine_response._msg_1.content.msg.battVolt;
        }
    }
}

void UEngine::write_uart()
{
    send_request();
}

void UEngine::send_request()
{
    // check send condition
    static uint32_t last_ms = millis();
    uint32_t now = millis();
    if (now - last_ms < 200) {
        return;
    }
    last_ms = now;

    if (get_port() == nullptr) {return;}
    uart_engine_send.make_sum();

    get_port()->write(uart_engine_send._msg_1.content.data, sizeof(uart_engine_send._msg_1.content.data));
}

void UEngine::send_mavlink_msg(mavlink_channel_t chan)
{
    mavlink_msg_hxts_hy_engine_send_struct(chan, &hxts_hy_engine_packet);
}
