#include "HB1_ins2apm.h"

#include <GCS_MAVLink/GCS.h>

HB1_ins2apm::HB1_ins2apm(void)
{
    _enable = false;
    _msg_1.need_send = false;
    _msg_1.updated = false;
}

void HB1_ins2apm::parse(uint8_t temp)
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
                _msg.sum_check += temp;
                _msg.msg_state = HB1_UART_msg_parser::HB1_UART_LENGTH;
                _msg.data[1] = temp;
            }
            else
            {
                _msg.msg_state = HB1_UART_msg_parser::HB1_UART_PREAMBLE1;
            }
            break;
        case HB1_UART_msg_parser::HB1_UART_LENGTH:
            _msg.length = temp;
            _msg.data[_msg.read_idx] = temp;
            _msg.sum_check += temp;
            _msg.read_idx++;
            _msg.msg_state = HB1_UART_msg_parser::HB1_UART_ID;
            break;
        case HB1_UART_msg_parser::HB1_UART_ID:
            _msg.data[_msg.read_idx] = temp;
            _msg.sum_check += temp;
            _msg.read_idx++;
            _msg.msg_state = HB1_UART_msg_parser::HB1_UART_DATA;
            break;
        case HB1_UART_msg_parser::HB1_UART_DATA:
            if (_msg.read_idx >= sizeof(_msg.data)-1) {
                _msg.msg_state = HB1_UART_msg_parser::HB1_UART_PREAMBLE1;
                break;
            }
            _msg.data[_msg.read_idx] = temp;
            _msg.read_idx++;
            _msg.sum_check += temp;

            if (_msg.read_idx >= (_msg.length - 2))
            {
                _msg.msg_state = HB1_UART_msg_parser::HB1_UART_SUM1;
            }
            break;
        case HB1_UART_msg_parser::HB1_UART_SUM1:
            _msg.data[_msg.read_idx] = temp;
            _msg.read_idx++;
            _msg.msg_state = HB1_UART_msg_parser::HB1_UART_SUM2;
            break;
        case HB1_UART_msg_parser::HB1_UART_SUM2:
            _msg.data[_msg.read_idx] = temp;
            _msg.msg_state = HB1_UART_msg_parser::HB1_UART_PREAMBLE1;

            process_message();
            break;
    }
}

void HB1_ins2apm::process_message(void)
{
    uint16_t sum_check_in = ((uint16_t)_msg.data[_msg.read_idx]<<8)+_msg.data[_msg.read_idx-1];
    if (sum_check_in != _msg.sum_check) {return;}

    memcpy(_msg_1.content.data, _msg.data, sizeof(_msg.data));

    swap_message();
    _msg_1.updated = true;
    _msg_1.need_send = false;
    _msg_1.print = true;
}
