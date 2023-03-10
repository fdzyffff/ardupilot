#include "FD1_msg_nacelle.h"
// #include <GCS_MAVLink/GCS.h>

FD1_msg_nacelle::FD1_msg_nacelle(void)
{
    _enable = false;
    _msg_1.need_send = false;
    _msg_1.updated = false;
}

void FD1_msg_nacelle::parse(uint8_t temp)
{
    switch (_msg.msg_state)
    {
        default:
        case FD1UART_msg_parser::FD1UART_PREAMBLE1:
            _msg.read_idx = 0;
            _msg.data[0] = temp;
            if (temp == PREAMBLE1) {
                _msg.msg_state = FD1UART_msg_parser::FD1UART_PREAMBLE2;
            }
            break;
        case FD1UART_msg_parser::FD1UART_PREAMBLE2:
            if (temp == PREAMBLE2)
            {
                _msg.read_idx = 2;
                _msg.msg_state = FD1UART_msg_parser::FD1UART_LENGTH;
                _msg.data[1] = temp;
            }
            else
            {
                reset(temp);
            }
            break;
        case FD1UART_msg_parser::FD1UART_LENGTH:
            _msg.length = temp;
            _msg.data[_msg.read_idx] = temp;
            _msg.read_idx++;
            _msg.msg_state = FD1UART_msg_parser::FD1UART_DATA;
            break;
        case FD1UART_msg_parser::FD1UART_DATA:
            if (_msg.read_idx >= sizeof(_msg.data)-1) {
                reset(temp);
                break;
            }
            _msg.data[_msg.read_idx] = temp;
            _msg.read_idx++;

            if (_msg.read_idx >= (_msg.length - 2))
            {
                _msg.msg_state = FD1UART_msg_parser::FD1UART_POSTAMBLE1;
            }
            break;
        case FD1UART_msg_parser::FD1UART_POSTAMBLE1:
            if (temp == POSTAMBLE1) {
                _msg.data[_msg.read_idx] = temp;
                _msg.read_idx++;
                _msg.msg_state = FD1UART_msg_parser::FD1UART_POSTAMBLE2;
            }
            else
            {
                reset(temp);
            }
            break;
        case FD1UART_msg_parser::FD1UART_POSTAMBLE2:
            if (temp == POSTAMBLE2) {
                _msg.data[_msg.read_idx] = temp;
                _msg.msg_state = FD1UART_msg_parser::FD1UART_PREAMBLE1;
                process_message();
            }
            else
            {
                reset(temp);
            }
            break;
    }
}

void FD1_msg_nacelle::reset(uint8_t temp)
{
    _msg.msg_state = FD1UART_msg_parser::FD1UART_PREAMBLE1;
    parse(temp);
}

void FD1_msg_nacelle::process_message(void)
{
    _msg.count++;
    _msg_1.length = _msg.length;
    memcpy(_msg_1.content.data, _msg.data, sizeof(_msg.data));

    swap_message();
    _msg_1.updated = true;
    _msg_1.need_send = false;
    _msg_1.print = true;
}

void FD1_msg_nacelle::swap_message(void)
{
    ;
}
