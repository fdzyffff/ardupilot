#include "FD1_msg_INS.h"
#include <GCS_MAVLink/GCS.h>

FD1_msg_INS::FD1_msg_INS(void)
{
    _enable = false;
    _msg_1.need_send = false;
    _msg_1.updated = false;
}

void FD1_msg_INS::parse(uint8_t temp)
{
    // gcs().send_text(MAV_SEVERITY_INFO, "State: %d, Byte: %d",_msg.msg_state, temp);
    switch (_msg.msg_state)
    {
        default:
        case FD1UART_msg_parser::FD1UART_PREAMBLE1:
            _msg.read = 0;
            _msg.xor_check = 0;
            _msg.xor_check = _msg.xor_check ^ temp;
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
                _msg.xor_check = _msg.xor_check ^ temp;
                _msg.xor_check = 0;
                _msg.msg_state = FD1UART_msg_parser::FD1UART_DATA;
                _msg.data[1] = temp;
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
            _msg.xor_check = _msg.xor_check ^ temp;

            if (_msg.read >= (_msg.length - 1))
            {
                _msg.msg_state = FD1UART_msg_parser::FD1UART_SUM;
            }
            break;
        case FD1UART_msg_parser::FD1UART_SUM:
            _msg.data[_msg.read] = temp;
            _msg.read++;

            if (temp == _msg.xor_check)
            {
                process_message();
            }
            // gcs().send_text(MAV_SEVERITY_INFO, "temp: %d, _msg.xor_check: %d",temp, _msg.xor_check);
            _msg.msg_state = FD1UART_msg_parser::FD1UART_PREAMBLE1;
            break;
    }
}

void FD1_msg_INS::process_message(void)
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

void FD1_msg_INS::swap_message(void)
{
    // swap_message_sub(_msg_1.content.data[7-1] , _msg_1.content.data[8-1] );
    // swap_message_sub(_msg_1.content.data[9-1] , _msg_1.content.data[10-1] );
    // swap_message_sub(_msg_1.content.data[4-1] , _msg_1.content.data[5-1] , _msg_1.content.data[6-1] , _msg_1.content.data[7-1]);
    // swap_message_sub(_msg_1.content.data[8-1] , _msg_1.content.data[9-1] , _msg_1.content.data[10-1], _msg_1.content.data[11-1]);
    // swap_message_sub(_msg_1.content.data[12-1], _msg_1.content.data[13-1], _msg_1.content.data[14-1], _msg_1.content.data[15-1]);
    // swap_message_sub(_msg_1.content.data[16-1], _msg_1.content.data[17-1], _msg_1.content.data[18-1], _msg_1.content.data[19-1]);
    // swap_message_sub(_msg_1.content.data[20-1], _msg_1.content.data[21-1], _msg_1.content.data[22-1], _msg_1.content.data[23-1]);
    // swap_message_sub(_msg_1.content.data[24-1], _msg_1.content.data[25-1], _msg_1.content.data[26-1], _msg_1.content.data[27-1]);
}
