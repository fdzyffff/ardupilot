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


#include "Copter.h"

UEngine::UEngine()
{
    ;
}

// initialise
void UEngine::init()
{
    _port = nullptr;
    const AP_SerialManager &serial_manager = AP::serialmanager();

    // check for protocol configured for a serial port - only the first serial port with one of these protocols will then run (cannot have FrSky on multiple serial ports)
    _port = serial_manager.find_serial(AP_SerialManager::SerialProtocol_UART, 0);
    if (_port != nullptr) {
        gcs().send_text(MAV_SEVERITY_WARNING, "Uart init");
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
    // while (get_port()->available()>0) {
    //     uint8_t temp = get_port()->read();
    //     uart_msg_0728_p3.parse(temp);
    //     }
    // }
}

void UEngine::write_uart()
{
    if (get_port() == nullptr) {return;}

    if (uart_msg_mt400ecu._msg_1.need_send) {
        get_port()->write(uart_msg_mt400ecu._msg_1.content.data, sizeof(uart_msg_mt400ecu._msg_1.content.data));
        uart_msg_mt400ecu._msg_1.need_send = false;
    }
}

void UEngine::do_engine_start()
{
    uart_msg_mt400ecu._msg_1.content.msg.pump_h = 0;
    uart_msg_mt400ecu._msg_1.content.msg.pump_l = 0;
    uart_msg_mt400ecu._msg_1.content.msg.test_mode = 0;
    uart_msg_mt400ecu._msg_1.content.msg.spark = 0;
    uart_msg_mt400ecu._msg_1.content.msg.motor = 0;
    uart_msg_mt400ecu._msg_1.content.msg.rpm_h = 0;
    uart_msg_mt400ecu._msg_1.content.msg.rpm_l = 0;
    uart_msg_mt400ecu._msg_1.content.msg.thr_h = 0;
    uart_msg_mt400ecu._msg_1.content.msg.thr_l = 0;
    uart_msg_mt400ecu._msg_1.content.msg.spark_switch = 0;
    uart_msg_mt400ecu._msg_1.content.msg.do_pid = 1;
    uart_msg_mt400ecu._msg_1.content.msg.alt_h = 0;
    uart_msg_mt400ecu._msg_1.content.msg.alt_l = 0;

    uart_msg_mt400ecu.make_sum();
    uart_msg_mt400ecu._msg_1.need_send = true;
    gcs().send_text(MAV_SEVERITY_INFO, "Engine: start");
}

void UEngine::do_engine_stop()
{
    uart_msg_mt400ecu._msg_1.content.msg.pump_h = 0;
    uart_msg_mt400ecu._msg_1.content.msg.pump_l = 0;
    uart_msg_mt400ecu._msg_1.content.msg.test_mode = 0;
    uart_msg_mt400ecu._msg_1.content.msg.spark = 0;
    uart_msg_mt400ecu._msg_1.content.msg.motor = 0;
    uart_msg_mt400ecu._msg_1.content.msg.rpm_h = 0;
    uart_msg_mt400ecu._msg_1.content.msg.rpm_l = 0;
    uart_msg_mt400ecu._msg_1.content.msg.thr_h = 0;
    uart_msg_mt400ecu._msg_1.content.msg.thr_l = 0;
    uart_msg_mt400ecu._msg_1.content.msg.spark_switch = 1;
    uart_msg_mt400ecu._msg_1.content.msg.do_pid = 0;
    uart_msg_mt400ecu._msg_1.content.msg.alt_h = 0;
    uart_msg_mt400ecu._msg_1.content.msg.alt_l = 0;

    uart_msg_mt400ecu.make_sum();
    uart_msg_mt400ecu._msg_1.need_send = true;
    gcs().send_text(MAV_SEVERITY_INFO, "Engine: stop");
}

void UEngine::do_engine_standby()
{
    uint16_t target_rpm = constrain_int32(copter.g2.user_parameters._target_rpm1.get(), 3000, 90000);
    uart_msg_mt400ecu._msg_1.content.msg.pump_h = 0;
    uart_msg_mt400ecu._msg_1.content.msg.pump_l = 0;
    uart_msg_mt400ecu._msg_1.content.msg.test_mode = 0;
    uart_msg_mt400ecu._msg_1.content.msg.spark = 0;
    uart_msg_mt400ecu._msg_1.content.msg.motor = 0;
    uart_msg_mt400ecu._msg_1.content.msg.rpm_h = (uint8_t)(target_rpm >> 8);
    uart_msg_mt400ecu._msg_1.content.msg.rpm_l = (uint8_t)(target_rpm & 0xFF);
    uart_msg_mt400ecu._msg_1.content.msg.thr_h = 0;
    uart_msg_mt400ecu._msg_1.content.msg.thr_l = 0;
    uart_msg_mt400ecu._msg_1.content.msg.spark_switch = 0;
    uart_msg_mt400ecu._msg_1.content.msg.do_pid = 0;
    uart_msg_mt400ecu._msg_1.content.msg.alt_h = 0;
    uart_msg_mt400ecu._msg_1.content.msg.alt_l = 0;

    uart_msg_mt400ecu.make_sum();
    uart_msg_mt400ecu._msg_1.need_send = true;
    gcs().send_text(MAV_SEVERITY_INFO, "Engine: standby");
}

void UEngine::do_engine_work()
{
    uint16_t target_rpm = constrain_int32(copter.g2.user_parameters._target_rpm2.get(), 3000, 90000);
    uart_msg_mt400ecu._msg_1.content.msg.pump_h = 0;
    uart_msg_mt400ecu._msg_1.content.msg.pump_l = 0;
    uart_msg_mt400ecu._msg_1.content.msg.test_mode = 0;
    uart_msg_mt400ecu._msg_1.content.msg.spark = 0;
    uart_msg_mt400ecu._msg_1.content.msg.motor = 0;
    uart_msg_mt400ecu._msg_1.content.msg.rpm_h = (uint8_t)(target_rpm >> 8);
    uart_msg_mt400ecu._msg_1.content.msg.rpm_l = (uint8_t)(target_rpm & 0xFF);
    uart_msg_mt400ecu._msg_1.content.msg.thr_h = 0;
    uart_msg_mt400ecu._msg_1.content.msg.thr_l = 0;
    uart_msg_mt400ecu._msg_1.content.msg.spark_switch = 0;
    uart_msg_mt400ecu._msg_1.content.msg.do_pid = 0;
    uart_msg_mt400ecu._msg_1.content.msg.alt_h = 0;
    uart_msg_mt400ecu._msg_1.content.msg.alt_l = 0;

    uart_msg_mt400ecu.make_sum();
    uart_msg_mt400ecu._msg_1.need_send = true;
    gcs().send_text(MAV_SEVERITY_INFO, "Engine: work %d", target_rpm);
}


