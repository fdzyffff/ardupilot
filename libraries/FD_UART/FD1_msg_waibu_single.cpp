#include "FD1_msg_waibu_single.h"
// #include <GCS_MAVLink/GCS.h>

FD1_msg_waibu_single::FD1_msg_waibu_single(void)
{
    _enable = false;
    _msg_1.need_send = false;
    _msg_1.updated = false;
}

void FD1_msg_waibu_single::parse(uint8_t temp)
{
    switch (_msg.msg_state)
    {
        default:
        case FD1UART_msg_parser::FD1UART_PREAMBLE1:
            _msg.read = 0;
            _msg.data[_msg.read] = temp;// 0
            if (temp == PREAMBLE1) {
                _msg.read++;
                _msg.msg_state = FD1UART_msg_parser::FD1UART_PREAMBLE2;
            }
            break;
        case FD1UART_msg_parser::FD1UART_PREAMBLE2:
            if (temp == PREAMBLE2)
            {
                _msg.data[_msg.read] = temp;// 1
                _msg.read++;
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
                _msg.data[_msg.read] = temp;// 1
                _msg.read++;
                _msg.length = 5;
                _msg.msg_state = FD1UART_msg_parser::FD1UART_DATA;
            }
            else
            {
                _msg.msg_state = FD1UART_msg_parser::FD1UART_PREAMBLE1;
            }
            break;
        case FD1UART_msg_parser::FD1UART_DATA:
            if (_msg.read >= sizeof(_msg.data)) {
                _msg.msg_state = FD1UART_msg_parser::FD1UART_PREAMBLE1;
                break;
            }

            if (_msg.read == 3) {
                _msg.length = 0xFF00 | temp;
            }

            if (_msg.read == 4) {
                _msg.length = ((uint16_t)temp<<8) + (0xFF & _msg.length);
            }

            _msg.data[_msg.read] = temp;
            _msg.read++;

            if (_msg.read >= (_msg.length))
            {
                process_message();
                _msg.msg_state = FD1UART_msg_parser::FD1UART_PREAMBLE1;
            }
            break;
    }
}

void FD1_msg_waibu_single::process_message(void)
{
    int16_t i = 0;
    for (i = 0; i <= _msg.length-1; i ++) {
        _msg_1.content.data[i] = _msg.data[i];
    }
    swap_message();
    _msg_1.updated = true;
    _msg_1.need_send = false;
    _msg_1.print = true;
}

void FD1_msg_waibu_single::make_sum()
{
    _msg_1.content.msg.header.head = FD1_msg_waibu_single::PREAMBLE1;
    _msg_1.content.msg.header.source = FD1_msg_waibu_single::PREAMBLE2;
    _msg_1.content.msg.header.destination = FD1_msg_waibu_single::PREAMBLE2;
    _msg_1.content.msg.count++;
}

void FD1_msg_waibu_single::swap_message(void)
{
    ;
}
