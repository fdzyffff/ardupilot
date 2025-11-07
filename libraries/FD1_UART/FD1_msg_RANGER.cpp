#include "FD1_msg_RANGER.h"
#include <GCS_MAVLink/GCS.h>

FD1_msg_RANGER::FD1_msg_RANGER(void)
{
    _enable = false;
    _msg_1.need_send = false;
    _msg_1.updated = false;
}

void FD1_msg_RANGER::parse(uint8_t temp)
{
    switch (_msg.msg_state)
    {
        default:
        case FD1UART_msg_parser::FD1UART_PREAMBLE1:
            _msg.read = 0;
            _msg.sum_check = 0;
            _msg.data[0] = temp;
            if (temp == PREAMBLE1) {
                _msg.msg_state = FD1UART_msg_parser::FD1UART_PREAMBLE2;
            }
            break;
        case FD1UART_msg_parser::FD1UART_PREAMBLE2:
            if (temp == PREAMBLE2)
            {
                _msg.length = _msg_1.length;
                _msg.read = 2;
                _msg.sum_check = 0;
                _msg.msg_state = FD1UART_msg_parser::FD1UART_ID;
                _msg.data[1] = temp;
            }
            else
            {
                _msg.msg_state = FD1UART_msg_parser::FD1UART_PREAMBLE1;
            }
            break;
        case FD1UART_msg_parser::FD1UART_ID:
            _msg.length = _msg_1.length;
            _msg.data[_msg.read] = temp;
            _msg.read++;
            _msg.sum_check = 0;
            _msg.msg_state = FD1UART_msg_parser::FD1UART_DATA;

            break;
        case FD1UART_msg_parser::FD1UART_DATA:
            if (_msg.read >= sizeof(_msg.data)-1) {
                _msg.msg_state = FD1UART_msg_parser::FD1UART_PREAMBLE1;
                break;
            }
            _msg.data[_msg.read] = temp;
            _msg.read++;
            _msg.sum_check = _msg.sum_check + temp;

            if (_msg.read >= (_msg.length - 1))
            {
                _msg.msg_state = FD1UART_msg_parser::FD1UART_SUM;
            }
            break;
        case FD1UART_msg_parser::FD1UART_SUM:
            _msg.data[_msg.read] = temp;
            _msg.read++;

            // gcs().send_text(MAV_SEVERITY_INFO, "State: %d, Byte: %x - %x",_msg.msg_state, temp, _msg.sum_check);
            if (temp == _msg.sum_check)
            {
                process_message();
            }
            // gcs().send_text(MAV_SEVERITY_INFO, "temp: %d, _msg.sum_check: %d",temp, _msg.sum_check);
            _msg.msg_state = FD1UART_msg_parser::FD1UART_PREAMBLE1;
            break;
    }
}

void FD1_msg_RANGER::process_message(void)
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

void FD1_msg_RANGER::swap_message(void)
{
    _msg_1.content.msg.high1 = swap_message_int16_t(_msg_1.content.msg.high1);
    _msg_1.content.msg.speed1 = swap_message_int16_t(_msg_1.content.msg.speed1);
    _msg_1.content.msg.high2 = swap_message_int16_t(_msg_1.content.msg.high2);
    _msg_1.content.msg.speed2 = swap_message_int16_t(_msg_1.content.msg.speed2);
    _msg_1.content.msg.high3 = swap_message_int16_t(_msg_1.content.msg.high3);
    _msg_1.content.msg.speed3 = swap_message_int16_t(_msg_1.content.msg.speed3);
    _msg_1.content.msg.high4 = swap_message_int16_t(_msg_1.content.msg.high4);
    _msg_1.content.msg.speed4 = swap_message_int16_t(_msg_1.content.msg.speed4);
    _msg_1.content.msg.high5 = swap_message_int16_t(_msg_1.content.msg.high5);
    _msg_1.content.msg.speed5 = swap_message_int16_t(_msg_1.content.msg.speed5);
}
