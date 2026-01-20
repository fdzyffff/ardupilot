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

Uart::Uart()
{
    ;
}

// initialise
void Uart::init()
{
    _port = nullptr;
    // const AP_SerialManager &serial_manager = AP::serialmanager();

    // // check for protocol configured for a serial port - only the first serial port with one of these protocols will then run (cannot have FrSky on multiple serial ports)
    // _port = serial_manager.find_serial(AP_SerialManager::SerialProtocol_NET, 0);
    // if (_port != nullptr) {
    //     gcs().send_text(MAV_SEVERITY_WARNING, "Uart init");
    //     return;
    // }
}

void Uart::update()
{
    read_uart();
    write_uart();
}

void Uart::read_uart()
{
    if (get_port() == nullptr) {return;}
    ;
}

void Uart::write_uart()
{
    ;
}
