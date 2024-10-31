#include "FD_CAM_CMD.h"
// #include <GCS_MAVLink/GCS.h>

FD_CAM_CMD::FD_CAM_CMD(void)
{
    _enable = false;
    _msg_1.need_send = false;
    _msg_1.updated = false;
}

void FD_CAM_CMD::parse(uint8_t temp)
{
    // gcs().send_text(MAV_SEVERITY_INFO, "State: %d, Byte: %d",_msg.msg_state, temp);
    switch (_msg.msg_state)
    {
        default:
        case FD1UART_msg_parser::FD1UART_PREAMBLE1:
            _msg.read = 0;
            _msg.sum_check = 0;
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
                _msg.length = FD_CAM_CMD_LEN;
                _msg.msg_state = FD1UART_msg_parser::FD1UART_DATA;
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
            _msg.sum_check += temp;
            _msg.read++;

            if (_msg.read >= (_msg.length - 1))
            {
                _msg.msg_state = FD1UART_msg_parser::FD1UART_SUM;
            }
            break;
        case FD1UART_msg_parser::FD1UART_SUM:
            _msg.data[_msg.read] = temp;
            _msg.read++;

            // gcs().send_text(MAV_SEVERITY_INFO, "sum: %d, sum_in: %d",_msg.sum_check, temp);

            if (_msg.sum_check == temp)
            {
                process_message();
            }
            _msg.msg_state = FD1UART_msg_parser::FD1UART_PREAMBLE1;
            break;
    }
}

void FD_CAM_CMD::process_message(void)
{
    int16_t i = 0;

    _msg_1.length = _msg.length;
    for (i = 0; i <= _msg_1.length-1; i ++) {
        _msg_1.content.data[i] = _msg.data[i];
    }
    swap_message();
    _msg_1.updated = true;
    _msg_1.need_send = false;
    _msg_1.print = true;
}

void FD_CAM_CMD::make_sum()
{
    _msg_1.content.msg.header.head_1 = FD_CAM_CMD::PREAMBLE1;
    _msg_1.content.msg.header.head_2 = FD_CAM_CMD::PREAMBLE2;
    _msg_1.length = FD_CAM_CMD_LEN;
    _msg_1.content.msg.sum_check = 0;
    for (int8_t i = 0; i < _msg_1.length - 1; i++) {
        _msg_1.content.msg.sum_check += _msg_1.content.data[i];
    }
}

void FD_CAM_CMD::swap_message(void)
{
    ;
}
