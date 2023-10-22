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
                _msg.length = _msg_1.length;
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

void FD1_msg_init::process_message(void)
{
    int16_t i = 0;

    _msg_1.length = _msg.length;
    for (i = 0; i <= _msg_1.length+2; i ++) {
        _msg_1.content.data[i] = _msg.data[i];
    }
    swap_message();
    _msg_1.updated = true;
    _msg_1.need_send = false;
    _msg_1.print = true;
}

bool FD1_msg_init::sum_check(void)
{
    // int16_t i = 0;
    // uint8_t sum_check = 0;
    // _msg_1.length = _msg_1.content.msg.header.info&0b00111111;

    // if (_msg_1.length<4 || _msg_1.length>FD1_MSG_init_LEN-3) {return false;}
    // for (i = 3; i <= _msg_1.length+1; i ++) {
    //     sum_check ^= _msg_1.content.data[i];
    // }
    // _msg_1.content.data[_msg_1.length+2] = sum_check;

    // return true;
}

void FD1_msg_init::swap_message(void)
{
    // switch (_msg_1.content.msg.header.id) {
    //     case 0x30: // A1,C1,E1,S1,R1
    //         break;
    //     case 0x31: // A2,C2,E2,S2
    //         break;
    //     case 0xB1: // M
    //         swap_message_sub(_msg_1.content.data[5+9-1], _msg_1.content.data[5+10-1]);
    //         swap_message_sub(_msg_1.content.data[5+11-1], _msg_1.content.data[5+12-1]);
    //         swap_message_sub(_msg_1.content.data[5+13-1], _msg_1.content.data[5+14-1]);
    //         swap_message_sub(_msg_1.content.data[5+20-1], _msg_1.content.data[5+21-1]);
    //         swap_message_sub(_msg_1.content.data[5+23-1], _msg_1.content.data[5+24-1],_msg_1.content.data[5+25-1], _msg_1.content.data[5+26-1]);
    //         swap_message_sub(_msg_1.content.data[5+27-1], _msg_1.content.data[5+28-1],_msg_1.content.data[5+29-1], _msg_1.content.data[5+30-1]);
    //         swap_message_sub(_msg_1.content.data[5+31-1], _msg_1.content.data[5+32-1],_msg_1.content.data[5+33-1], _msg_1.content.data[5+34-1]);
    //         swap_message_sub(_msg_1.content.data[5+35-1], _msg_1.content.data[5+36-1]);
    //         swap_message_sub(_msg_1.content.data[5+37-1], _msg_1.content.data[5+38-1]);
    //         swap_message_sub(_msg_1.content.data[5+39-1], _msg_1.content.data[5+40-1]);
    //         swap_message_sub(_msg_1.content.data[5+41-1], _msg_1.content.data[5+42-1]);
    //         break;
    //     case 0x13: // X
    //         break;
    //     case 0x40: // T1,F1,B1,D1,R2
    //         _msg_1.content.msg.sub_msg.msg_40.sub_t1.plane_lat = swap_message_int32_t(_msg_1.content.msg.sub_msg.msg_40.sub_t1.plane_lat);
    //         _msg_1.content.msg.sub_msg.msg_40.sub_t1.plane_lng = swap_message_int32_t(_msg_1.content.msg.sub_msg.msg_40.sub_t1.plane_lng);
    //         _msg_1.content.msg.sub_msg.msg_40.sub_t1.plane_alt = swap_message_int16_t(_msg_1.content.msg.sub_msg.msg_40.sub_t1.plane_alt);
    //         _msg_1.content.msg.sub_msg.msg_40.sub_t1.target_lat = swap_message_int32_t(_msg_1.content.msg.sub_msg.msg_40.sub_t1.target_lat);
    //         _msg_1.content.msg.sub_msg.msg_40.sub_t1.target_lng = swap_message_int32_t(_msg_1.content.msg.sub_msg.msg_40.sub_t1.target_lng);
    //         _msg_1.content.msg.sub_msg.msg_40.sub_t1.target_alt = swap_message_int16_t(_msg_1.content.msg.sub_msg.msg_40.sub_t1.target_alt);
    //         _msg_1.content.msg.sub_msg.msg_40.sub_k1.target_x1 = swap_message_uint16_t(_msg_1.content.msg.sub_msg.msg_40.sub_k1.target_x1);
    //         _msg_1.content.msg.sub_msg.msg_40.sub_k1.target_y1 = swap_message_uint16_t(_msg_1.content.msg.sub_msg.msg_40.sub_k1.target_y1);
    //         _msg_1.content.msg.sub_msg.msg_40.sub_k1.target_x2 = swap_message_uint16_t(_msg_1.content.msg.sub_msg.msg_40.sub_k1.target_x2);
    //         _msg_1.content.msg.sub_msg.msg_40.sub_k1.target_y2 = swap_message_uint16_t(_msg_1.content.msg.sub_msg.msg_40.sub_k1.target_y2);
    //         _msg_1.content.msg.sub_msg.msg_40.sub_k1.confidence_level = swap_message_int16_t(_msg_1.content.msg.sub_msg.msg_40.sub_k1.confidence_level);
    //         break;
    //     case 0x41: // T2,F2,B2,D2
    //         break;
    //     case 0x23: // Y
    //         break;
    //     default:
    //         break;
    // }
}
