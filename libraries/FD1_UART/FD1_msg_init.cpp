#include "FD1_msg_init.h"
// #include <GCS_MAVLink/GCS.h>

FD1_msg_init::FD1_msg_init(void)
{
    _enable = false;
    _msg_1.need_send = false;
    _msg_1.updated = false;
}

void FD1_msg_init::parse(uint8_t temp)
{
    switch (_msg.msg_state)
    {
        default:
        case FD1UART_msg_parser::FD1UART_PREAMBLE1:
            _msg.read = 0;
            _msg.sum_check = 0;
            //_msg.sum_check += temp;
            _msg.data[0] = temp;
            if (temp == PREAMBLE1) {
                _msg.msg_state = FD1UART_msg_parser::FD1UART_PREAMBLE2;
            }
            break;
        case FD1UART_msg_parser::FD1UART_PREAMBLE2:
            if (temp == PREAMBLE2)
            {
                _msg.read = 2;
                //_msg.sum_check += temp;
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
            if (_msg.read == 2) {
                if (temp != FRAMETYPE) {
                    _msg.msg_state = FD1UART_msg_parser::FD1UART_PREAMBLE1;
                    break;
                }
            }
            if (_msg.read == 7) {
                if (temp != FRAMEID) {
                    _msg.msg_state = FD1UART_msg_parser::FD1UART_PREAMBLE1;
                    break;
                }
            }
            _msg.data[_msg.read] = temp;
            _msg.read++;
            _msg.sum_check += temp;

            if (_msg.read >= (_msg.length - 2))
            {
                _msg.msg_state = FD1UART_msg_parser::FD1UART_SUM;
            }
            break;
        case FD1UART_msg_parser::FD1UART_SUM:
            _msg.data[_msg.read] = temp;
            _msg.read++;
            if (_msg.read >= _msg.length && _msg.sum_check == (((uint16_t)_msg.data[_msg.read-2] << 8) | _msg.data[_msg.read-1])) 
            {
                process_message();
            }
            _msg.msg_state = FD1UART_msg_parser::FD1UART_PREAMBLE1;
            break;
    }
}

void FD1_msg_init::process_message(void)
{
    int16_t i = 0;
    for (i = 0; i < _msg.length; i ++) {
        _msg_1.content.data[i] = _msg.data[i];
    }
    swap_message();
    _msg_1.updated = true;
    _msg_1.need_send = false;
    _msg_1.print = true;
}

bool FD1_msg_init::sum_check(void)
{
    int16_t i = 0;
    _msg_1.content.msg.sum_check = 0;
    for (i = 0; i < _msg.length - 2; i ++) {
        _msg_1.content.msg.sum_check += _msg_1.content.data[i];
    }
    return true;
}

void FD1_msg_init::swap_message(void)
{
    // _msg_1.content.msg.sub_msg.msg_40.sub_t1.plane_lat = swap_message_int32_t(_msg_1.content.msg.sub_msg.msg_40.sub_t1.plane_lat);
    // _msg_1.content.msg.sub_msg.msg_40.sub_t1.plane_lng = swap_message_int32_t(_msg_1.content.msg.sub_msg.msg_40.sub_t1.plane_lng);
    // _msg_1.content.msg.sub_msg.msg_40.sub_t1.plane_alt = swap_message_int16_t(_msg_1.content.msg.sub_msg.msg_40.sub_t1.plane_alt);
    // _msg_1.content.msg.sub_msg.msg_40.sub_t1.target_lat = swap_message_int32_t(_msg_1.content.msg.sub_msg.msg_40.sub_t1.target_lat);
    // _msg_1.content.msg.sub_msg.msg_40.sub_t1.target_lng = swap_message_int32_t(_msg_1.content.msg.sub_msg.msg_40.sub_t1.target_lng);
    // _msg_1.content.msg.sub_msg.msg_40.sub_t1.target_alt = swap_message_int16_t(_msg_1.content.msg.sub_msg.msg_40.sub_t1.target_alt);
    // _msg_1.content.msg.sub_msg.msg_40.sub_k1.target_x1 = swap_message_uint16_t(_msg_1.content.msg.sub_msg.msg_40.sub_k1.target_x1);
    // _msg_1.content.msg.sub_msg.msg_40.sub_k1.target_y1 = swap_message_uint16_t(_msg_1.content.msg.sub_msg.msg_40.sub_k1.target_y1);
    // _msg_1.content.msg.sub_msg.msg_40.sub_k1.target_x2 = swap_message_uint16_t(_msg_1.content.msg.sub_msg.msg_40.sub_k1.target_x2);
    // _msg_1.content.msg.sub_msg.msg_40.sub_k1.target_y2 = swap_message_uint16_t(_msg_1.content.msg.sub_msg.msg_40.sub_k1.target_y2);
    // _msg_1.content.msg.sub_msg.msg_40.sub_k1.confidence_level = swap_message_int16_t(_msg_1.content.msg.sub_msg.msg_40.sub_k1.confidence_level);
}
