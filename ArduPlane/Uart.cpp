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

const AP_Param::GroupInfo Uart::var_info[] = {

    AP_GROUPINFO("DEBUG",     0  , Uart, print,   0),

    AP_GROUPEND
};

Uart::Uart()
{
    AP_Param::setup_object_defaults(this, var_info);
    ;
}

// initialise
void Uart::init()
{
    _port = nullptr;
    // const AP_SerialManager &serial_manager = AP::serialmanager();

    // check for protocol configured for a serial port - only the first serial port with one of these protocols will then run (cannot have FrSky on multiple serial ports)
    _port = plane.serial_manager.find_serial(AP_SerialManager::SerialProtocol_MISSION, 0);
    if (_port != nullptr) {
        gcs().send_text(MAV_SEVERITY_WARNING, "Uart init");
        return;
    }
}

void Uart::update()
{
    read_uart();
    write_uart();
    update_status();
}

void Uart::read_uart()
{
    if (get_port() == nullptr) {return;}
    while (get_port()->available()>0) {
        uint8_t temp = get_port()->read();
        uart_msg_LS_control.parse(temp);
        if (uart_msg_LS_control._msg_1.updated) {
            uart_msg_LS_control._msg_1.updated = false;
            handle_LS_control();
        }
    }
}

void Uart::write_uart()
{
    pack_status();
}

void Uart::do_print()
{
    // put your 1Hz code here
    if ((print.get() & (1<<0)) && control_status.valid) { // 1
        gcs().send_text(MAV_SEVERITY_WARNING, "LS [%d, %d]", control_status.type, control_status.cmd);
    }
    if (print.get() & (1<<1)) { // 2
        gcs().send_text(MAV_SEVERITY_WARNING, "0x1A (%0.2f, %0.2f, %0.2f) cmd:%d",control_status.cmd_speed, control_status.cmd_pitch, control_status.cmd_roll, control_status.cmd);
    }
    if (print.get() & (1<<2)) { // 4
        gcs().send_text(MAV_SEVERITY_WARNING, "0x3C (%0.2f, %0.2f, %0.2f) cmd:%d",control_status.cmd_speed, control_status.cmd_alt, control_status.cmd_roll, control_status.cmd);
    }
    if (print.get() & (1<<3)) { // 8
        gcs().send_text(MAV_SEVERITY_WARNING, "0x55 (%0.2f, %0.2f, %0.2f) cmd:%d",((float)control_status.cmd_loc.lng * 1e-7), ((float)control_status.cmd_loc.lat * 1e-7), ((float)control_status.cmd_loc.alt * 1e-2), control_status.cmd);
    }
}
