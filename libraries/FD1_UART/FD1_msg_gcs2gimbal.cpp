#include "FD1_msg_gcs2gimbal.h"
// #include <GCS_MAVLink/GCS.h>

FD1_msg_gcs2gimbal::FD1_msg_gcs2gimbal(void)
{
    _enable = false;
    _msg_1.need_send = false;
    _msg_1.updated = false;
}

void FD1_msg_gcs2gimbal::parse(uint8_t temp)
{
    switch (_msg.msg_state)
    {
        default:
        case FD1UART_msg_parser::FD1UART_PREAMBLE1:
            _msg.read = 0;
            _msg.xorsum = 0;
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
                _msg.length = FD1_MSG_GCS2GIMBAL_LEN;
                _msg.msg_state = FD1UART_msg_parser::FD1UART_DATA;
            }
            else
            {
                _msg.msg_state = FD1UART_msg_parser::FD1UART_PREAMBLE1;
            }
            break;
        case FD1UART_msg_parser::FD1UART_DATA:
            if (_msg.read >= sizeof(_msg.data)-2) {
                _msg.msg_state = FD1UART_msg_parser::FD1UART_PREAMBLE1;
                break;
            }
            _msg.data[_msg.read] = temp;
            _msg.xorsum ^= temp;
            _msg.read++;

            if (_msg.read >= (_msg.length - 2))
            {
                _msg.msg_state = FD1UART_msg_parser::FD1UART_SUM;
            }
            break;
        case FD1UART_msg_parser::FD1UART_SUM:
            _msg.data[_msg.read] = temp;
            _msg.read++;

            // gcs().send_text(MAV_SEVERITY_INFO, "sum: %d, sum_in: %d",_msg.xorsum, temp);

            if (_msg.xorsum == temp)
            {
                _msg.msg_state = FD1UART_msg_parser::FD1UART_END;
            }
            _msg.msg_state = FD1UART_msg_parser::FD1UART_PREAMBLE1;
            break;
        case FD1UART_msg_parser::FD1UART_END:
            _msg.data[_msg.read] = temp;

            // gcs().send_text(MAV_SEVERITY_INFO, "sum: %d, sum_in: %d",_msg.xorsum, temp);

            if (temp == POSTAMBLE1)
            {
                process_message();
            }
            _msg.msg_state = FD1UART_msg_parser::FD1UART_PREAMBLE1;
            break;
    }
}

void FD1_msg_gcs2gimbal::process_message(void)
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

void FD1_msg_gcs2gimbal::make_sum()
{
    _msg_1.content.msg.header.head_1 = FD1_msg_gcs2gimbal::PREAMBLE1;
    _msg_1.content.msg.header.head_2 = FD1_msg_gcs2gimbal::PREAMBLE2;
    _msg_1.content.msg.end = FD1_msg_gcs2gimbal::POSTAMBLE1;
    _msg_1.content.msg.xorsum = 0;
    for (int8_t i = 2; i < _msg_1.length - 2; i++) {
        _msg_1.content.msg.xorsum = (_msg_1.content.msg.xorsum ^ _msg_1.content.data[i]);
    }
}

void FD1_msg_gcs2gimbal::swap_message(void)
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
