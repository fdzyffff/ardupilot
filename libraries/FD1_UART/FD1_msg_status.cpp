#include "FD1_msg_status.h"
// #include <GCS_MAVLink/GCS.h>

FD1_msg_status::FD1_msg_status(void)
{
    _enable = false;
    _msg_1.need_send = false;
    _msg_1.updated = false;
}

void FD1_msg_status::parse(uint8_t temp)
{
    ;
}

void FD1_msg_status::process_message(void)
{
    int32_t i = 0;

    _msg_1.length = _msg.length;
    for (i = 0; i <= _msg_1.length-1; i ++) {
        _msg_1.content.data[i] = _msg.data[i];
    }
    swap_message();
    _msg_1.updated = true;
    _msg_1.need_send = false;
    _msg_1.print = true;
}

void FD1_msg_status::make_sum()
{
    _msg_1.content.msg.header.head_1 = PREAMBLE1;
    _msg_1.content.msg.header.head_2 = PREAMBLE2;
    _msg_1.content.msg.header.head_3 = PREAMBLE3;
    _msg_1.content.msg.sum1 = 0;
    _msg_1.content.msg.sum2 = 0;
    for (int32_t i = 7; i < _msg_1.length - 3; i++) {
        _msg_1.content.msg.sum1 = (_msg_1.content.msg.sum1 + _msg_1.content.data[i]);
    }
    for (int32_t i = 0; i < _msg_1.length - 2; i++) {
        _msg_1.content.msg.sum2 = (_msg_1.content.msg.sum2 + _msg_1.content.data[i]);
    }
}

void FD1_msg_status::swap_message(void)
{
    _msg_1.content.msg.length = swap_message_uint16_t(_msg_1.content.msg.length);
    _msg_1.content.msg.data_length = swap_message_uint16_t(_msg_1.content.msg.data_length);
    _msg_1.content.msg.system_time_s = swap_message_uint32_t(_msg_1.content.msg.system_time_s);
    _msg_1.content.msg.utc_year = swap_message_uint16_t(_msg_1.content.msg.utc_year);
    _msg_1.content.msg.lng_deg = swap_message_double(_msg_1.content.msg.lng_deg);
    _msg_1.content.msg.lat_deg = swap_message_double(_msg_1.content.msg.lat_deg);
    _msg_1.content.msg.alt_m = swap_message_float(_msg_1.content.msg.alt_m);
    _msg_1.content.msg.pos_n_m = swap_message_float(_msg_1.content.msg.pos_n_m);
    _msg_1.content.msg.pos_e_m = swap_message_float(_msg_1.content.msg.pos_e_m);
    _msg_1.content.msg.pos_d_m = swap_message_float(_msg_1.content.msg.pos_d_m);
    _msg_1.content.msg.vel_n_ms = swap_message_float(_msg_1.content.msg.vel_n_ms);
    _msg_1.content.msg.vel_e_ms = swap_message_float(_msg_1.content.msg.vel_e_ms);
    _msg_1.content.msg.vel_d_ms = swap_message_float(_msg_1.content.msg.vel_d_ms);
    _msg_1.content.msg.roll_deg = swap_message_float(_msg_1.content.msg.roll_deg);
    _msg_1.content.msg.pitch_deg = swap_message_float(_msg_1.content.msg.pitch_deg);
    _msg_1.content.msg.yaw_deg = swap_message_float(_msg_1.content.msg.yaw_deg);
    _msg_1.content.msg.roll_rate_degs = swap_message_float(_msg_1.content.msg.roll_rate_degs);
    _msg_1.content.msg.pitch_rate_degs = swap_message_float(_msg_1.content.msg.pitch_rate_degs);
    _msg_1.content.msg.yaw_rate_degs = swap_message_float(_msg_1.content.msg.yaw_rate_degs);
    _msg_1.content.msg.acc_x_g = swap_message_float(_msg_1.content.msg.acc_x_g);
    _msg_1.content.msg.acc_y_g = swap_message_float(_msg_1.content.msg.acc_y_g);
    _msg_1.content.msg.acc_z_g = swap_message_float(_msg_1.content.msg.acc_z_g);
    _msg_1.content.msg.airspeed = swap_message_float(_msg_1.content.msg.airspeed);
    _msg_1.content.msg.groundspeed = swap_message_float(_msg_1.content.msg.groundspeed);
    _msg_1.content.msg.baro_alt_m = swap_message_float(_msg_1.content.msg.baro_alt_m);
    _msg_1.content.msg.batt_volt = swap_message_float(_msg_1.content.msg.batt_volt);
    _msg_1.content.msg.batt_current = swap_message_float(_msg_1.content.msg.batt_current);
    _msg_1.content.msg.motor_rpm = swap_message_uint16_t(_msg_1.content.msg.motor_rpm);
    _msg_1.content.msg.sum2 = swap_message_uint16_t(_msg_1.content.msg.sum2);
}
