#include "FD1_msg_control.h"
#include <GCS_MAVLink/GCS.h>

FD1_msg_control::FD1_msg_control(void)
{
    _enable = false;
    _msg_1.need_send = false;
    _msg_1.updated = false;
}

void FD1_msg_control::parse(uint8_t temp)
{
    // gcs().send_text(MAV_SEVERITY_INFO, "State: %d, Byte: %d",_msg.msg_state, temp);
    switch (_msg.msg_state)
    {
        default:
        case FD1UART_msg_parser::FD1UART_PREAMBLE1:
            _msg.read = 0;
            _msg.data[_msg.read] = temp;
            _msg.sum1 = 0;
            _msg.sum2 = 0;
            _msg.sum2 += temp;
            if (temp == PREAMBLE1) {
                _msg.msg_state = FD1UART_msg_parser::FD1UART_PREAMBLE2;
            }
            break;
        case FD1UART_msg_parser::FD1UART_PREAMBLE2:
            if (temp == PREAMBLE2)
            {
                _msg.read = 1;
                _msg.data[_msg.read] = temp;
                _msg.length = _msg_1.length;
                _msg.sum2 += temp;
                _msg.msg_state = FD1UART_msg_parser::FD1UART_PREAMBLE3;
            }
            else
            {
                _msg.msg_state = FD1UART_msg_parser::FD1UART_PREAMBLE1;
            }
            break;
        case FD1UART_msg_parser::FD1UART_PREAMBLE3:
            if (temp == PREAMBLE3)
            {
                _msg.read = 2;
                _msg.data[_msg.read] = temp;
                _msg.length = _msg_1.length;
                _msg.sum2 += temp;
                _msg.msg_state = FD1UART_msg_parser::FD1UART_DATA;
                _msg.read++;
            }
            else
            {
                _msg.msg_state = FD1UART_msg_parser::FD1UART_PREAMBLE1;
            }
            break;
        case FD1UART_msg_parser::FD1UART_DATA:
            if (_msg.read >= sizeof(_msg.data)-3) {
                _msg.msg_state = FD1UART_msg_parser::FD1UART_PREAMBLE1;
                break;
            }
            _msg.data[_msg.read] = temp;
            _msg.read++;
            if (_msg.read > 7) {
                _msg.sum1 += temp;
            } else {
                _msg.sum1 = 0;
            }
            _msg.sum2 += temp;

            if (_msg.read >= (_msg.length - 3))
            {
                _msg.msg_state = FD1UART_msg_parser::FD1UART_SUM1;
            }
            break;
        case FD1UART_msg_parser::FD1UART_SUM1:
            _msg.data[_msg.read] = temp;
            _msg.read++;
            _msg.sum2 += temp;
            // gcs().send_text(MAV_SEVERITY_INFO, "State: %d, Byte: %x - %x", _msg.msg_state, temp, _msg.sum1);

            if (temp == (_msg.sum1))
            {
                _msg.msg_state = FD1UART_msg_parser::FD1UART_SUM21;
            } else {
                _msg.msg_state = FD1UART_msg_parser::FD1UART_PREAMBLE1;
            }

            break;
        case FD1UART_msg_parser::FD1UART_SUM21:
            _msg.data[_msg.read] = temp;
            _msg.read++;
            // gcs().send_text(MAV_SEVERITY_INFO, "State: %d, Byte: %x - %x", _msg.msg_state, temp, (_msg.sum2>>8));

            if (temp == (uint8_t)(_msg.sum2>>8))
            {
                _msg.msg_state = FD1UART_msg_parser::FD1UART_SUM22;
            } else {
                _msg.msg_state = FD1UART_msg_parser::FD1UART_PREAMBLE1;
            }

            break;
        case FD1UART_msg_parser::FD1UART_SUM22:
            _msg.data[_msg.read] = temp;

            // gcs().send_text(MAV_SEVERITY_INFO, "State: %d, Byte: %x - %x", _msg.msg_state, temp, (_msg.sum2&0xFF));
            if (temp == (_msg.sum2&0xFF))
            {
                process_message();
            }
            // gcs().send_text(MAV_SEVERITY_INFO, "temp: %d, _msg.sum_check: %d",temp, _msg.sum_check);
            _msg.msg_state = FD1UART_msg_parser::FD1UART_PREAMBLE1;
            break;
    }
}

void FD1_msg_control::process_message(void)
{
    int32_t i = 0;

    _msg_1.length = _msg.length;
    for (i = 0; i <= _msg_1.length-1; i ++) {
        _msg_1.content.data[i] = _msg.data[i];
    }
    swap_message();
    _msg_1.updated = true;
    _msg_1.need_send = false;
    _msg_1.print = true;
}

void FD1_msg_control::make_sum()
{
    _msg_1.content.msg.header.head_1 = PREAMBLE1;
    _msg_1.content.msg.header.head_2 = PREAMBLE2;
    _msg_1.content.msg.header.head_3 = PREAMBLE3;
    _msg_1.content.msg.sum1 = 0;
    _msg_1.content.msg.sum2 = 0;
    for (int32_t i = 7; i < _msg_1.length - 3; i++) {
        _msg_1.content.msg.sum1 = (_msg_1.content.msg.sum1 + _msg_1.content.data[i]);
    }
    for (int32_t i = 0; i < _msg_1.length - 2; i++) {
        _msg_1.content.msg.sum2 = (_msg_1.content.msg.sum2 + _msg_1.content.data[i]);
    }
}

void FD1_msg_control::swap_message(void)
{
    _msg_1.content.msg.length = swap_message_uint16_t(_msg_1.content.msg.length);
    _msg_1.content.msg.target_alt_m = swap_message_float(_msg_1.content.msg.target_alt_m);
    _msg_1.content.msg.target_airspeed = swap_message_float(_msg_1.content.msg.target_airspeed);
    _msg_1.content.msg.target_roll_deg = swap_message_float(_msg_1.content.msg.target_roll_deg);
    _msg_1.content.msg.target_course = swap_message_float(_msg_1.content.msg.target_course);
    _msg_1.content.msg.sum2 = swap_message_uint16_t(_msg_1.content.msg.sum2);
}
