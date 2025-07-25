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
            _msg.sum = 0;
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
                _msg.sum = 0;
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
                _msg.sum = 0;
                _msg.msg_state = FD1UART_msg_parser::FD1UART_DATA;
                _msg.read++;
            }
            else
            {
                _msg.msg_state = FD1UART_msg_parser::FD1UART_PREAMBLE1;
            }
            break;
        case FD1UART_msg_parser::FD1UART_DATA:
            if (_msg.read >= sizeof(_msg.data)-1) {
                _msg.msg_state = FD1UART_msg_parser::FD1UART_PREAMBLE1;
                break;
            }
            _msg.data[_msg.read] = temp;
            _msg.read++;
            _msg.sum += temp;
            if (_msg.read >= (_msg.length - 1))
            {
                _msg.msg_state = FD1UART_msg_parser::FD1UART_SUM;
            }
            break;
        case FD1UART_msg_parser::FD1UART_SUM:
            _msg.data[_msg.read] = temp;

            // gcs().send_text(MAV_SEVERITY_INFO, "State: %d, Byte: %x - %x", _msg.msg_state, temp, _msg.sum);

            if (temp == (_msg.sum))
            {
                process_message();
            } else {
                _msg.msg_state = FD1UART_msg_parser::FD1UART_PREAMBLE1;
            }

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
    _msg_1.content.msg.sum = 0;
    for (int32_t i = 3; i < _msg_1.length - 1; i++) {
        _msg_1.content.msg.sum = (_msg_1.content.msg.sum + _msg_1.content.data[i]);
    }
}

void FD1_msg_control::swap_message(void)
{
    ;
}
