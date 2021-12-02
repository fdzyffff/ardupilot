#include "FD1_msg_cam_in.h"

FD1_msg_cam_in::FD1_msg_cam_in(void)
{
    _enable = false;
    _msg_1.need_send = false;
    _msg_1.updated = false;
}

void FD1_msg_cam_in::parse(uint8_t temp)
{
    switch (_msg.msg_state)
    {
        default:
        case FD1UART_msg_parser::FD1UART_PREAMBLE1:
            _msg.read = 0;
            _msg.sum_check = 0;
            _msg.sum_check += temp;
            _msg.data[0] = temp;
            if (temp == PREAMBLE1) {
                _msg.read += 1;
                _msg.msg_state = FD1UART_msg_parser::FD1UART_DATA;
            }
            break;
        case FD1UART_msg_parser::FD1UART_DATA:
            if (_msg.read >= sizeof(_msg.data)) {
                _msg.msg_state = FD1UART_msg_parser::FD1UART_PREAMBLE1;
                break;
            }

            if (_msg.read >= (_msg.length - 1))
            {
                _msg.msg_state = FD1UART_msg_parser::FD1UART_SUM;
            }

            _msg.data[_msg.read] = temp;
            _msg.read++;
            _msg.sum_check += temp;
            break;
        case FD1UART_msg_parser::FD1UART_SUM:
            _msg.data[_msg.read] = temp;
            _msg.msg_state = FD1UART_msg_parser::FD1UART_PREAMBLE1;

            if (_msg.sum_check == temp)
            {
                process_message();
            }
            break;
    }
}

void FD1_msg_cam_in::process_message(void)
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

void FD1_msg_cam_in::swap_message(void)
{
    swap_message_sub(_msg_1.content.data[2], _msg_1.content.data[3],_msg_1.content.data[4],_msg_1.content.data[5]);
    swap_message_sub(_msg_1.content.data[6], _msg_1.content.data[7],_msg_1.content.data[8],_msg_1.content.data[9]);
}
