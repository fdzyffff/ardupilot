#include "HB1_payload2apm.h"

HB1_payload2apm::HB1_payload2apm(void)
{
    _enable = false;
    _msg_1.need_send = false;
    _msg_1.updated = false;
}

void HB1_payload2apm::parse(uint8_t temp)
{
    switch (_msg.msg_state)
    {
        default:
        case HB1_UART_msg_parser::HB1_UART_PREAMBLE1:
            _msg.read_idx = 0;
            _msg.sum_check = temp;
            _msg.data[0] = temp;
            if (temp == PREAMBLE1) {
                _msg.msg_state = HB1_UART_msg_parser::HB1_UART_PREAMBLE2;
            }
            break;
        case HB1_UART_msg_parser::HB1_UART_PREAMBLE2:
            if (temp == PREAMBLE2)
            {
                _msg.read_idx = 2;
                _msg.length = _msg_1.length;
                _msg.sum_check += temp;
                _msg.msg_state = HB1_UART_msg_parser::HB1_UART_DATA;
                _msg.data[1] = temp;
            }
            else
            {
                _msg.msg_state = HB1_UART_msg_parser::HB1_UART_PREAMBLE1;
            }
            break;
        case HB1_UART_msg_parser::HB1_UART_DATA:
            if (_msg.read_idx >= sizeof(_msg.data)-1) {
                _msg.msg_state = HB1_UART_msg_parser::HB1_UART_PREAMBLE1;
                break;
            }
            _msg.data[_msg.read_idx] = temp;
            _msg.read_idx++;
            _msg.sum_check += temp;

            if (_msg.read_idx >= (_msg.length - 1))
            {
                _msg.msg_state = HB1_UART_msg_parser::HB1_UART_SUM;
            }
            break;
        case HB1_UART_msg_parser::HB1_UART_SUM:
            _msg.data[_msg.read_idx] = temp;
            _msg.msg_state = HB1_UART_msg_parser::HB1_UART_PREAMBLE1;

            if (_msg.sum_check == temp)
            {
                process_message();
            }
            break;
    }
}

void HB1_payload2apm::process_message(void)
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
