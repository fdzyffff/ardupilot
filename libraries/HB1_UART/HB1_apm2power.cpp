#include "HB1_apm2power.h"

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

void HB1_apm2power::set_engine_start()
{
    _msg_1.content.msg.header.head_1 = HB1_apm2power::PREAMBLE1;
    _msg_1.content.msg.header.head_2 = HB1_apm2power::PREAMBLE2;
    _msg_1.content.msg.COMM1 = 0xFC;
    _msg_1.content.msg.COMM2 = 0xFC;
    _msg_1.content.msg.rpm_h = 0;
    _msg_1.content.msg.rpm_l = 0;
    _msg_1.content.msg.rel_alt = 0;
    _msg_1.content.msg.temp = 0;
    _msg_1.content.msg.setting_flag = 0;
    _msg_1.content.msg.airspeed = 0;
    _msg_1.content.msg.byte_11 = 0;
    _msg_1.content.msg.byte_22 = 0;
    _msg_1.content.msg.sum = 0;
    _msg_1.content.msg.xorsum = 0;
}

void HB1_apm2power::set_engine_stop()
{
    _msg_1.content.msg.header.head_1 = HB1_apm2power::PREAMBLE1;
    _msg_1.content.msg.header.head_2 = HB1_apm2power::PREAMBLE2;
    _msg_1.content.msg.COMM1 = 0xF0;
    _msg_1.content.msg.COMM2 = 0xF0;
    _msg_1.content.msg.rpm_h = 0;
    _msg_1.content.msg.rpm_l = 0;
    _msg_1.content.msg.rel_alt = 0;
    _msg_1.content.msg.temp = 0;
    _msg_1.content.msg.setting_flag = 0;
    _msg_1.content.msg.airspeed = 0;
    _msg_1.content.msg.byte_11 = 0;
    _msg_1.content.msg.byte_22 = 0;
    _msg_1.content.msg.sum = 0;
    _msg_1.content.msg.xorsum = 0;
}

void HB1_apm2power::set_engine_reset()
{
    _msg_1.content.msg.header.head_1 = HB1_apm2power::PREAMBLE1;
    _msg_1.content.msg.header.head_2 = HB1_apm2power::PREAMBLE2;
    _msg_1.content.msg.COMM1 = 0xF4;
    _msg_1.content.msg.COMM2 = 0xF4;
    _msg_1.content.msg.rpm_h = 0;
    _msg_1.content.msg.rpm_l = 0;
    _msg_1.content.msg.rel_alt = 0;
    _msg_1.content.msg.temp = 0;
    _msg_1.content.msg.setting_flag = 0;
    _msg_1.content.msg.airspeed = 0;
    _msg_1.content.msg.byte_11 = 0;
    _msg_1.content.msg.byte_22 = 0;
    _msg_1.content.msg.sum = 0;
    _msg_1.content.msg.xorsum = 0;
}

void HB1_apm2power::set_throttle(uint8_t rpm_in)
{
    _msg_1.content.msg.header.head_1 = HB1_apm2power::PREAMBLE1;
    _msg_1.content.msg.header.head_2 = HB1_apm2power::PREAMBLE2;
    _msg_1.content.msg.COMM1 = 0xE3;
    _msg_1.content.msg.COMM2 = 0xE3;
    _msg_1.content.msg.rpm_h = 0;
    _msg_1.content.msg.rpm_l = rpm_in;
    _msg_1.content.msg.rel_alt = 0;
    _msg_1.content.msg.temp = 0;
    _msg_1.content.msg.setting_flag = 0;
    _msg_1.content.msg.airspeed = 0;
    _msg_1.content.msg.byte_11 = 0;
    _msg_1.content.msg.byte_22 = 0;
    _msg_1.content.msg.sum = 0;
    _msg_1.content.msg.xorsum = 0;
}

void HB1_apm2power::make_sum()
{
    _msg_1.content.msg.header.head_1 = HB1_apm2power::PREAMBLE1;
    _msg_1.content.msg.header.head_2 = HB1_apm2power::PREAMBLE2;
    _msg_1.content.msg.sum = 0;
    _msg_1.content.msg.xorsum = 0;
    for (int8_t i = 0; i < _msg_1.length - 2; i++) {
        _msg_1.content.msg.sum += _msg_1.content.data[i];
    }
    for (int8_t i = 0; i < _msg_1.length - 2; i++) {
        _msg_1.content.msg.xorsum = (_msg_1.content.msg.xorsum ^ _msg_1.content.data[i]);
    }
}
