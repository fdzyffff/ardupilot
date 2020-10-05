#include "HB1_power2apm.h"

HB1_power2apm::HB1_power2apm(void)
{
    _enable = false;
    _msg_1.need_send = false;
    _msg_1.updated = false;
}

void HB1_power2apm::parse(uint8_t temp)
{
    switch (_msg.msg_state)
    {
        default:
        case HB1UART_msg_parser::HB1UART_PREAMBLE1:
            _msg.read = 0;
            _msg.sum_check = 0;
            _msg.sum_check += temp;
            if (temp == PREAMBLE1) {
                _msg.msg_state = HB1UART_msg_parser::HB1UART_PREAMBLE2;
            }
            break;
        case HB1UART_msg_parser::HB1UART_PREAMBLE2:
            if (temp == PREAMBLE2)
            {
                _msg.header.head_1 = PREAMBLE1;
                _msg.header.head_2 = PREAMBLE2;
                _msg.length = _msg_1.length;
                _msg.read = 2;
                _msg.sum_check += temp;
                _msg.msg_state = HB1UART_msg_parser::HB1UART_DATA;
            }
            else
            {
                _msg.msg_state = HB1UART_msg_parser::HB1UART_PREAMBLE1;
            }
            break;
        case HB1UART_msg_parser::HB1UART_DATA:
            if (_msg.read >= sizeof(_msg.data)) {
                _msg.msg_state = HB1UART_msg_parser::HB1UART_PREAMBLE1;
                break;
            }
            _msg.data[_msg.read-2] = temp;

            _msg.sum_check += temp;

            _msg.read++;
            if (_msg.read >= (_msg.length - 1))
            {
                _msg.msg_state = HB1UART_msg_parser::HB1UART_SUM;
            }
            break;
        case HB1UART_msg_parser::HB1UART_SUM:
            _msg.data[_msg.read-2] = temp;
            _msg.msg_state = HB1UART_msg_parser::HB1UART_PREAMBLE1;

            if (_msg.sum_check == temp)
            {
                process_message();
            }
            break;
    }
}

void HB1_power2apm::process_message(void)
{
    int16_t i = 0;

    _msg_1.content.data[0] = _msg.header.head_1;
    _msg_1.content.data[1] = _msg.header.head_2;
    for (i = 0; i < _msg_1.length - 2; i ++) {
        _msg_1.content.data[i+2] = _msg.data[i];
    }
    swap_message();
    _msg_1.updated = true;
    _msg_1.need_send = false;
    _msg_1.print = true;
}

void HB1_power2apm::swap_message(void)
{
    ;
    swap_message_sub(_msg_1.content.data[4], _msg_1.content.data[5]);
    swap_message_sub(_msg_1.content.data[6], _msg_1.content.data[7]);
}
