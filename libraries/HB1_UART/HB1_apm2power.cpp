#include "HB1_apm2power.h"
#include <GCS_MAVLink/GCS.h>

HB1_apm2power::HB1_apm2power(void)
{
    _enable = false;
    _msg_1.need_send = false;
    _msg_1.updated = false;
}

void HB1_apm2power::parse(uint8_t temp)
{
    ;
}

void HB1_apm2power::process_message(void)
{
    ;
}

void HB1_apm2power::swap_message(void)
{
    // if (_msg_1.content.msg.c[0] == 0x48 && _msg_1.content.msg.c[1] == 0x06) {
    //     swap_message_sub(_msg_1.content.msg.c[2], _msg_1.content.msg.c[3], _msg_1.content.msg.c[4], _msg_1.content.msg.c[5]);
    // }
}

// void HB1_apm2power::set_engine_throttle_control(uint16_t thr_in)
// {
//     _msg_1.content.msg.header.head_1 = HB1_apm2power::PREAMBLE1;
//     _msg_1.content.msg.byte1 = 0x10 + (uint8_t)(thr_in >> 8);
//     _msg_1.content.msg.byte2 = (uint8_t)(thr_in)&0xFF;
//     make_sum();
// }

void HB1_apm2power::set_engine_throttle_control(uint16_t thr_in)
{
    _msg_1.content.msg.header.head_1 = HB1_apm2power::PREAMBLE1;
    _msg_1.content.msg.byte1 = 0x1C + (uint8_t)(thr_in >> 8);
    _msg_1.content.msg.byte2 = (uint8_t)(thr_in)&0xFF;
    make_sum();
}

void HB1_apm2power::set_engine_start()
{
    _msg_1.content.msg.header.head_1 = HB1_apm2power::PREAMBLE1;
    _msg_1.content.msg.byte1 = 0x1C;
    _msg_1.content.msg.byte2 = 0x00;
    _msg_1.content.msg.crc = 0xA1;
    make_sum();
}

void HB1_apm2power::set_engine_stop()
{
    _msg_1.content.msg.header.head_1 = HB1_apm2power::PREAMBLE1;
    _msg_1.content.msg.byte1 = 0x18;
    _msg_1.content.msg.byte2 = 0x00;
    _msg_1.content.msg.crc = 0x9A;
    make_sum();
}

void HB1_apm2power::set_engine_emergency_stop()
{
    _msg_1.content.msg.header.head_1 = HB1_apm2power::PREAMBLE1;
    _msg_1.content.msg.byte1 = 0x14;
    _msg_1.content.msg.byte2 = 0x00;
    _msg_1.content.msg.crc = 0xD7;
}

void HB1_apm2power::set_throttle(uint8_t thr_in)
{
    // _msg_1.content.msg.header.head_1 = HB1_apm2power::PREAMBLE1;
    // _msg_1.content.msg.header.head_2 = HB1_apm2power::PREAMBLE2;
    // _msg_1.content.msg.COMM1 = 0xE3;
    // _msg_1.content.msg.COMM2 = 0xE3;
    // _msg_1.content.msg.rpm_h = 0;
    // _msg_1.content.msg.rpm_l = thr_in;
    // _msg_1.content.msg.rel_alt = 0;
    // _msg_1.content.msg.temp = 0;
    // _msg_1.content.msg.setting_flag = 0;
    // _msg_1.content.msg.airspeed = 0;
    // _msg_1.content.msg.byte_11 = 0;
    // _msg_1.content.msg.byte_22 = 0;
    // _msg_1.content.msg.sum = 0;
    // _msg_1.content.msg.xorsum = 0;
}

void HB1_apm2power::set_rpm_half(uint16_t rpm_in)
{
    // uint8_t rpm_h = ((rpm_in)>>8);
    // uint8_t rpm_l = ((rpm_in)|(0xFF<<8));
    // _msg_1.content.msg.header.head_1 = HB1_apm2power::PREAMBLE1;
    // _msg_1.content.msg.header.head_2 = HB1_apm2power::PREAMBLE2;
    // _msg_1.content.msg.COMM1 = 0xF3;
    // _msg_1.content.msg.COMM2 = 0xF3;
    // _msg_1.content.msg.rpm_h = rpm_h;
    // _msg_1.content.msg.rpm_l = rpm_l;
    // _msg_1.content.msg.rel_alt = 0;
    // _msg_1.content.msg.temp = 0;
    // _msg_1.content.msg.setting_flag = 0;
    // _msg_1.content.msg.airspeed = 0;
    // _msg_1.content.msg.byte_11 = 0;
    // _msg_1.content.msg.byte_22 = 0;
    // _msg_1.content.msg.sum = 0;
    // _msg_1.content.msg.xorsum = 0;
    // // gcs().send_text(MAV_SEVERITY_INFO, "%d, %d", rpm_h, rpm_l);
}

void HB1_apm2power::make_sum()
{
    uint8_t i = 0;
    uint8_t k = 0;
    uint8_t crc8 = 0;
    for (i = 1; i < _msg_1.length-1; i++) {
        k = _msg_1.content.data[i]^crc8;
        crc8 = 0;
        if (k & 0x01) {crc8 ^= 0x5E;}
        if (k & 0x02) {crc8 ^= 0xBC;}
        if (k & 0x04) {crc8 ^= 0x61;}
        if (k & 0x08) {crc8 ^= 0xC2;}
        if (k & 0x10) {crc8 ^= 0x9D;}
        if (k & 0x20) {crc8 ^= 0x23;}
        if (k & 0x40) {crc8 ^= 0x46;}
        if (k & 0x80) {crc8 ^= 0x8C;}
    }
    _msg_1.content.msg.crc = crc8;
}
