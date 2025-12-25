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

UWeight::UWeight()
{
    ;
}

// initialise
void UWeight::init()
{
    _port = nullptr;
    const AP_SerialManager &serial_manager = AP::serialmanager();

    // check for protocol configured for a serial port - only the first serial port with one of these protocols will then run (cannot have FrSky on multiple serial ports)
    _port = serial_manager.find_serial(AP_SerialManager::SerialProtocol_UART, 0);
    if (_port != nullptr) {
        gcs().send_text(MAV_SEVERITY_WARNING, "UWeight init");
        return;
    }
}

void UWeight::update()
{
    read_uart();
    write_uart();
}

void UWeight::read_uart()
{
    if (get_port() == nullptr) {return;}
    while (get_port()->available()>0) {
        uint8_t temp = get_port()->read();
        uart_msg_weight.parse(temp);

        if (uart_msg_weight._msg_1.updated) {
            uart_msg_weight._msg_1.updated = false;
            hxts_hy_weight_packet.Front = uart_msg_weight._msg_1.content.msg.value1;
            hxts_hy_weight_packet.LEFT = uart_msg_weight._msg_1.content.msg.value2;
            hxts_hy_weight_packet.RIGHT = uart_msg_weight._msg_1.content.msg.value3;
        }
    }
}

void UWeight::write_uart()
{
    ;
}

void UWeight::send_mavlink_msg(mavlink_channel_t chan)
{
    mavlink_msg_hxts_hy_weight_send_struct(chan, &hxts_hy_weight_packet);
}
