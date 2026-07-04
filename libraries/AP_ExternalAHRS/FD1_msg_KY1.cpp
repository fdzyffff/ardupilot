#include "FD1_msg_KY1.h"
#include <GCS_MAVLink/GCS.h>

FD1_msg_KY1::FD1_msg_KY1(void)
{
    _enable = false;
    _msg_1.need_send = false;
    _msg_1.updated = false;
}

void FD1_msg_KY1::parse(uint8_t temp)
{
    uint32_t now_ms = AP_HAL::millis();
    if (now_ms - _last_byte_ms > 5) {
        _msg.msg_state = FD1UART_msg_parser::FD1UART_PREAMBLE1;
    }
    _last_byte_ms = now_ms;
    switch (_msg.msg_state)
    {
        default:
        case FD1UART_msg_parser::FD1UART_PREAMBLE1:
            _msg.read = 0;
            _msg.sum_check = 0;
            _msg.data[0] = temp;
            if (temp == PREAMBLE1) {
                _msg.read = 1;
                _msg.msg_state = FD1UART_msg_parser::FD1UART_DATA;
            }
            break;
        case FD1UART_msg_parser::FD1UART_DATA:
            if (_msg.read >= sizeof(_msg.data)-1) {
                _msg.msg_state = FD1UART_msg_parser::FD1UART_PREAMBLE1;
                break;
            }

            _msg.data[_msg.read] = temp;
            _msg.read++;

            if (_msg.read % 2 == 0 && _msg.read > 1) {
                _msg.sum_check += (uint16_t)_msg.data[_msg.read-2] | ((uint16_t)_msg.data[_msg.read-1] << 8);
            }

            if (_msg.read >= (_msg.length - 1))
            {
                _msg.msg_state = FD1UART_msg_parser::FD1UART_SUM;
            }
            break;
        case FD1UART_msg_parser::FD1UART_SUM:
            _msg.data[_msg.read] = temp;

            uint16_t temp_sum = (uint16_t)_msg.data[_msg.read-1] | ((uint16_t)_msg.data[_msg.read] << 8);

            // gcs().send_text(MAV_SEVERITY_INFO, "State: %d, Byte: %#x - %#x",_msg.msg_state, temp_sum, _msg.sum_check);
            if (temp_sum == _msg.sum_check)
            {
                process_message();
            }
            // gcs().send_text(MAV_SEVERITY_INFO, "temp: %d, _msg.sum_check: %d",temp, _msg.sum_check);
            _msg.msg_state = FD1UART_msg_parser::FD1UART_PREAMBLE1;
            break;
    }
}

void FD1_msg_KY1::process_message(void)
{
    int16_t i = 0;

    for (i = 0; i < _msg_1.length; i ++) {
        _msg_1.content.data[i] = _msg.data[i];
    }
    swap_message();
    _msg_1.updated = true;
    _msg_1.need_send = false;
    _msg_1.print = true;
}

void FD1_msg_KY1::swap_message(void)
{
    // KY1 sends big-endian (高字节在前), STM32 is little-endian
    _msg_1.content.msg.gyro_x = swap_message_int16_t(_msg_1.content.msg.gyro_x);
    _msg_1.content.msg.gyro_y = swap_message_int16_t(_msg_1.content.msg.gyro_y);
    _msg_1.content.msg.gyro_z = swap_message_int16_t(_msg_1.content.msg.gyro_z);
    _msg_1.content.msg.acc_x = swap_message_int16_t(_msg_1.content.msg.acc_x);
    _msg_1.content.msg.acc_y = swap_message_int16_t(_msg_1.content.msg.acc_y);
    _msg_1.content.msg.acc_z = swap_message_int16_t(_msg_1.content.msg.acc_z);
    _msg_1.content.msg.angle_roll = swap_message_int16_t(_msg_1.content.msg.angle_roll);
    _msg_1.content.msg.angle_pitch = swap_message_int16_t(_msg_1.content.msg.angle_pitch);
    _msg_1.content.msg.angle_yaw = swap_message_int16_t(_msg_1.content.msg.angle_yaw);
}
