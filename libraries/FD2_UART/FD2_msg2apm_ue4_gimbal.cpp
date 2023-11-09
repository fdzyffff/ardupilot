#include "FD2_msg2apm_ue4_gimbal.h"

FD2_msg2apm_ue4_gimbal::FD2_msg2apm_ue4_gimbal(void)
{
    _enable = false;
    _msg_1.need_send = false;
    _msg_1.updated = false;
}

void FD2_msg2apm_ue4_gimbal::parse(uint8_t temp)
{
    switch (_msg.msg_state)
    {
        default:
        case FD2UART_msg_parser::FD2UART_PREAMBLE1:
            _msg.read = 0;
            _msg.sum_check = 0;
            //_msg.sum_check += temp;
            _msg.data[0] = temp;
            if (temp == PREAMBLE1) {
                _msg.msg_state = FD2UART_msg_parser::FD2UART_PREAMBLE2;
            }
            break;
        case FD2UART_msg_parser::FD2UART_PREAMBLE2:
            if (temp == PREAMBLE2)
            {
                _msg.length = _msg_1.length;
                _msg.read = 2;
                //_msg.sum_check += temp;
                _msg.msg_state = FD2UART_msg_parser::FD2UART_DATA;
                _msg.data[1] = temp;
            }
            else
            {
                _msg.msg_state = FD2UART_msg_parser::FD2UART_PREAMBLE1;
            }
            break;
        case FD2UART_msg_parser::FD2UART_DATA:
            if (_msg.read >= sizeof(_msg.data)-1) {
                _msg.msg_state = FD2UART_msg_parser::FD2UART_PREAMBLE1;
                break;
            }
            _msg.data[_msg.read] = temp;
            _msg.read++;
            _msg.sum_check += temp;

            if (_msg.read >= (_msg.length - 1))
            {
                _msg.msg_state = FD2UART_msg_parser::FD2UART_SUM;
            }
            break;
        case FD2UART_msg_parser::FD2UART_SUM:
            _msg.data[_msg.read] = temp;
            _msg.read++;

            if (_msg.sum_check == temp)
            {
                process_message();
            }
            _msg.msg_state = FD2UART_msg_parser::FD2UART_PREAMBLE1;
            break;
    }
}

void FD2_msg2apm_ue4_gimbal::process_message(void)
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

void FD2_msg2apm_ue4_gimbal::swap_message(void)
{
    // swap_message_sub(_msg_1.content.data[4-1] , _msg_1.content.data[5-1] , _msg_1.content.data[6-1] , _msg_1.content.data[7-1]);
    // swap_message_sub(_msg_1.content.data[8-1] , _msg_1.content.data[9-1] , _msg_1.content.data[10-1], _msg_1.content.data[11-1]);
    // swap_message_sub(_msg_1.content.data[12-1], _msg_1.content.data[13-1], _msg_1.content.data[14-1], _msg_1.content.data[15-1]);
}
