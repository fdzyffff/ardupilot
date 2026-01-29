#include "FD1_msg_0XA2.h"
#include <GCS_MAVLink/GCS.h>

FD1_msg_0XA2::FD1_msg_0XA2(void)
{
    _enable = false;
    _msg_1.need_send = false;
    _msg_1.updated = false;
}

void FD1_msg_0XA2::parse(uint8_t temp)
{
    switch (_msg.msg_state)
    {
        default:
        case FD1UART_msg_parser::FD1UART_PREAMBLE1:
            _msg.read = 0;
            _msg.sum_check = 0;
            _msg.sum_check = _msg.sum_check + temp;
            _msg.data[0] = temp;
            if (temp == PREAMBLE1) {
                _msg.msg_state = FD1UART_msg_parser::FD1UART_PREAMBLE2;
            }
            break;
        case FD1UART_msg_parser::FD1UART_PREAMBLE2:
            if (temp == PREAMBLE2)
            {
                _msg.read = 2;
                _msg.sum_check = _msg.sum_check + temp;
                _msg.msg_state = FD1UART_msg_parser::FD1UART_DATA;
                _msg.data[1] = temp;
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
            if (_msg.read == 2) {
                _msg.length = temp;
            }
            if (_msg.read == 3 && temp != 0xA2) {
                _msg.msg_state = FD1UART_msg_parser::FD1UART_PREAMBLE1;
                break;
            }
            _msg.data[_msg.read] = temp;
            _msg.read++;
            _msg.sum_check = _msg.sum_check + temp;

            if (_msg.read >= (_msg.length - 2))
            {
                _msg.msg_state = FD1UART_msg_parser::FD1UART_SUM1;
            }
            break;
        case FD1UART_msg_parser::FD1UART_SUM1:
            _msg.data[_msg.read] = temp;
            _msg.read++;
            if (temp == (uint8_t)(_msg.sum_check&0xFF))
            {
                _msg.msg_state = FD1UART_msg_parser::FD1UART_SUM2;
            } else 
            {
                _msg.msg_state = FD1UART_msg_parser::FD1UART_PREAMBLE1;
            }
            break;
        case FD1UART_msg_parser::FD1UART_SUM2:
            _msg.data[_msg.read] = temp;

            // gcs().send_text(MAV_SEVERITY_INFO, "State: %d, Byte: %x - %x",_msg.msg_state, temp, _msg.sum_check);
            if (temp == (uint8_t)((_msg.sum_check>>8)&0xFF))
            {
                process_message();
            }
            // gcs().send_text(MAV_SEVERITY_INFO, "temp: %d, _msg.sum_check: %d",temp, _msg.sum_check);
            _msg.msg_state = FD1UART_msg_parser::FD1UART_PREAMBLE1;
            break;
    }
}

void FD1_msg_0XA2::process_message(void)
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

void FD1_msg_0XA2::swap_message(void)
{
    // _msg_1.content.msg.start_ms = swap_message_uint32_t(_msg_1.content.msg.start_ms);
    // _msg_1.content.msg.work_ms = swap_message_uint32_t(_msg_1.content.msg.work_ms);
    // _msg_1.content.msg.pitch_micro_deg = swap_message_int32_t(_msg_1.content.msg.pitch_micro_deg);
    // _msg_1.content.msg.roll_micro_deg = swap_message_int32_t(_msg_1.content.msg.roll_micro_deg);
    // _msg_1.content.msg.yaw_micro_deg = swap_message_int32_t(_msg_1.content.msg.yaw_micro_deg);
    // _msg_1.content.msg.lng = swap_message_int32_t(_msg_1.content.msg.lng);
    // _msg_1.content.msg.lat = swap_message_int32_t(_msg_1.content.msg.lat);
    // _msg_1.content.msg.alt_mm = swap_message_int32_t(_msg_1.content.msg.alt_mm);
    // _msg_1.content.msg.vel_e = swap_message_int32_t(_msg_1.content.msg.vel_e);
    // _msg_1.content.msg.vel_n = swap_message_int32_t(_msg_1.content.msg.vel_n);
    // _msg_1.content.msg.vel_u = swap_message_int32_t(_msg_1.content.msg.vel_u);
    // _msg_1.content.msg.rate_x_degrees = swap_message_float(_msg_1.content.msg.rate_x_degrees);
    // _msg_1.content.msg.rate_y_degrees = swap_message_float(_msg_1.content.msg.rate_y_degrees);
    // _msg_1.content.msg.rate_z_degrees = swap_message_float(_msg_1.content.msg.rate_z_degrees);
    // _msg_1.content.msg.acc_x_mss = swap_message_float(_msg_1.content.msg.acc_x_mss);
    // _msg_1.content.msg.acc_y_mss = swap_message_float(_msg_1.content.msg.acc_y_mss);
    // _msg_1.content.msg.acc_z_mss = swap_message_float(_msg_1.content.msg.acc_z_mss);
    // _msg_1.content.msg.gps_pps = swap_message_uint32_t(_msg_1.content.msg.gps_pps);
    // _msg_1.content.msg.gps_lng = swap_message_int32_t(_msg_1.content.msg.gps_lng);
    // _msg_1.content.msg.gps_lat = swap_message_int32_t(_msg_1.content.msg.gps_lat);
    // _msg_1.content.msg.gps_alt_mm = swap_message_int32_t(_msg_1.content.msg.gps_alt_mm);
    // _msg_1.content.msg.gps_vel_e_ms_o4 = swap_message_int32_t(_msg_1.content.msg.gps_vel_e_ms_o4);
    // _msg_1.content.msg.gps_vel_n_ms_o4 = swap_message_int32_t(_msg_1.content.msg.gps_vel_n_ms_o4);
    // _msg_1.content.msg.gps_vel_u_ms_o2 = swap_message_int16_t(_msg_1.content.msg.gps_vel_u_ms_o2);
    // _msg_1.content.msg.gps_numstat = swap_message_uint16_t(_msg_1.content.msg.gps_numstat);
    // _msg_1.content.msg.gps_height_error = swap_message_int16_t(_msg_1.content.msg.gps_height_error);
    // _msg_1.content.msg.gps_hdop = swap_message_uint16_t(_msg_1.content.msg.gps_hdop);
    // _msg_1.content.msg.gps_vdop = swap_message_uint16_t(_msg_1.content.msg.gps_vdop);
}

void FD1_msg_0XA2::sum_check()
{
    _msg_1.content.msg.header.head_1 = PREAMBLE1;
    _msg_1.content.msg.header.head_2 = PREAMBLE2;

    int16_t i = 0;

    _msg_1.content.msg.length = _msg_1.length;
    _msg_1.content.msg.sum_check = 0;

    for (i = 0; i < _msg_1.length - 2; i ++) {
        _msg_1.content.msg.sum_check += _msg_1.content.data[i];
    }
}
