#include "FD_engine.h"

FD_engine::FD_engine(void)
{
    _enable = false;
    _msg_1.need_send = false;
    _msg_1.updated = false;
}

void FD_engine::parse(uint8_t temp)
{
    switch (_msg.msg_state)
    {
        default:
        case FD_UART_msg_parser::FD_UART_PREAMBLE1:
            _msg.read_idx = 0;
            _msg.data[_msg.read_idx] = temp;
            if (temp == PREAMBLE1) {
                _msg.read_idx = 1;
                _msg.msg_state = FD_UART_msg_parser::FD_UART_PREAMBLE2;
            }
            break;
        case FD_UART_msg_parser::FD_UART_PREAMBLE2:
            _msg.read_idx++;
            _msg.data[_msg.read_idx] = temp;
            if (temp == PREAMBLE2) {
                _msg.read_idx = 1;
                _msg.msg_state = FD_UART_msg_parser::FD_UART_PREAMBLE_ID;
            } else {
                _msg.msg_state = FD_UART_msg_parser::FD_UART_PREAMBLE1;
            }
            break;
        case FD_UART_msg_parser::FD_UART_PREAMBLE_ID:
            _msg.read_idx++;
            _msg.data[_msg.read_idx] = temp;
            if (temp == PREAMBLE_ID) {
                _msg.length = _msg_1.length;
                _msg.msg_state = FD_UART_msg_parser::FD_UART_PREAMBLE_ID_2;
            } else {
                _msg.msg_state = FD_UART_msg_parser::FD_UART_PREAMBLE1;
            }
            break;
        case FD_UART_msg_parser::FD_UART_PREAMBLE_ID_2:
            _msg.read_idx++;
            _msg.data[_msg.read_idx] = temp;
            if (temp == PREAMBLE_ID_SUB) {
                _msg.msg_state = FD_UART_msg_parser::FD_UART_DATA;
            } else {
                _msg.msg_state = FD_UART_msg_parser::FD_UART_PREAMBLE1;
            }
            break;
        case FD_UART_msg_parser::FD_UART_DATA:
            _msg.read_idx++;
            if (_msg.read_idx > sizeof(_msg.data)-1) {
                _msg.msg_state = FD_UART_msg_parser::FD_UART_PREAMBLE1;
                break;
            }
            _msg.data[_msg.read_idx] = temp;

            if (_msg.read_idx >= (_msg.length - 1))
            {
                _msg.msg_state = FD_UART_msg_parser::FD_UART_PREAMBLE1;
                process_message();
            }
            break;
    }
}

void FD_engine::process_message(void)
{
    memcpy(_msg_1.content.data, _msg.data, sizeof(_msg.data));
    swap_message();
    _msg_1.updated = true;
    _msg_1.need_send = false;
    _msg_1.print = true;
    last_ms = AP_HAL::millis();
}
