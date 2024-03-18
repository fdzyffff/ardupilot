#include "FD1_msg_ID5.h"

FD1_msg_ID5::FD1_msg_ID5(void)
{
    _enable = false;
    _msg_1.need_send = false;
    _msg_1.updated = false;
}

void FD1_msg_ID5::parse(uint8_t temp)
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
                _msg.msg_state = FD1UART_msg_parser::FD1UART_PREAMBLE2;
            }
            break;
        case FD1UART_msg_parser::FD1UART_PREAMBLE2:
            if (temp == PREAMBLE2)
            {
                _msg.sum_check += temp;
                _msg.msg_state = FD1UART_msg_parser::FD1UART_PRETYPE;
                _msg.data[1] = temp;
            }
            else
            {
                _msg.msg_state = FD1UART_msg_parser::FD1UART_PREAMBLE1;
            }
            break;
        case FD1UART_msg_parser::FD1UART_PRETYPE:
            if (temp == PRETYPE)
            {
                _msg.length = _msg_1.length;
                _msg.read = 3;
                _msg.sum_check += temp;
                _msg.msg_state = FD1UART_msg_parser::FD1UART_DATA;
                _msg.data[2] = temp;
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
            _msg.sum_check += temp;

            if (_msg.read >= (_msg.length - 1))
            {
                _msg.msg_state = FD1UART_msg_parser::FD1UART_SUM;
            }
            break;
        case FD1UART_msg_parser::FD1UART_SUM:
            _msg.data[_msg.read] = temp;
            _msg.read++;

            if (_msg.sum_check == temp)
            {
                process_message();
            }
            _msg.msg_state = FD1UART_msg_parser::FD1UART_PREAMBLE1;
            break;
    }
}

void FD1_msg_ID5::process_message(void)
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

void FD1_msg_ID5::swap_message(void)
{
    // swap_message_sub(_msg_1.content.data[4-1] , _msg_1.content.data[5-1] , _msg_1.content.data[6-1] , _msg_1.content.data[7-1]);
    // swap_message_sub(_msg_1.content.data[8-1] , _msg_1.content.data[9-1] , _msg_1.content.data[10-1], _msg_1.content.data[11-1]);
    // swap_message_sub(_msg_1.content.data[12-1], _msg_1.content.data[13-1], _msg_1.content.data[14-1], _msg_1.content.data[15-1]);
    // swap_message_sub(_msg_1.content.data[16-1], _msg_1.content.data[17-1], _msg_1.content.data[18-1], _msg_1.content.data[19-1]);
    // swap_message_sub(_msg_1.content.data[20-1], _msg_1.content.data[21-1], _msg_1.content.data[22-1], _msg_1.content.data[23-1]);
    // swap_message_sub(_msg_1.content.data[24-1], _msg_1.content.data[25-1], _msg_1.content.data[26-1], _msg_1.content.data[27-1]);
}

void FD1_msg_ID5::cal_sumcheck(void)
{
    _msg_1.content.msg.header.head_1 = PREAMBLE1;
    _msg_1.content.msg.header.head_2 = PREAMBLE2;
    _msg_1.content.msg.type = PRETYPE;
    _msg_1.content.msg.length = _msg_1.length;
    _msg_1.content.msg.sum_check = 0;
    for (int16_t i = 0; i < _msg_1.length-1; i ++) {
        _msg_1.content.msg.sum_check += _msg_1.content.data[i];
    }
}
