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

void HB1_apm2power::set_rocket_on()
{
    _msg_1.content.msg.c[0] = 0x55;
    _msg_1.content.msg.c[1] = 0x55;
    _msg_1.content.msg.c[2] = 0x55;
    _msg_1.content.msg.c[3] = 0x55;
    _msg_1.content.msg.c[4] = 0x00;
    _msg_1.content.msg.c[5] = 0x00;
    _msg_1.content.msg.c[6] = 0x00;
    _msg_1.content.msg.c[7] = 0x00;
}

void HB1_apm2power::set_engine_start()
{
    _msg_1.content.msg.c[0] = 0x48;
    _msg_1.content.msg.c[1] = 0x02;
    _msg_1.content.msg.c[2] = 0x03;
    _msg_1.content.msg.c[3] = 0x2F;
    _msg_1.content.msg.c[4] = 0x00;
    _msg_1.content.msg.c[5] = 0x00;
    _msg_1.content.msg.c[6] = 0x00;
    _msg_1.content.msg.c[7] = 0x00;
}

void HB1_apm2power::set_engine_stop()
{
    _msg_1.content.msg.c[0] = 0x48;
    _msg_1.content.msg.c[1] = 0x04;
    _msg_1.content.msg.c[2] = 0x05;
    _msg_1.content.msg.c[3] = 0x2F;
    _msg_1.content.msg.c[4] = 0x00;
    _msg_1.content.msg.c[5] = 0x00;
    _msg_1.content.msg.c[6] = 0x00;
    _msg_1.content.msg.c[7] = 0x00;
}

void HB1_apm2power::set_request()
{
    _msg_1.content.msg.c[0] = 0x50;
    _msg_1.content.msg.c[1] = 0x31;
    _msg_1.content.msg.c[2] = 0x30;
    _msg_1.content.msg.c[3] = 0x2F;
    _msg_1.content.msg.c[4] = 0x00;
    _msg_1.content.msg.c[5] = 0x00;
    _msg_1.content.msg.c[6] = 0x00;
    _msg_1.content.msg.c[7] = 0x00;
}

void HB1_apm2power::set_throttle(uint32_t rpm_in)
{
    _msg_1.content.msg.c[0] = 0x48;
    _msg_1.content.msg.c[1] = 0x06;
    _msg_1.content.msg.c[2] = (rpm_in & 0x000000ff);
    _msg_1.content.msg.c[3] = (rpm_in & 0x0000ff00) >> 8;
    _msg_1.content.msg.c[4] = (rpm_in & 0x00ff0000) >> 16;
    _msg_1.content.msg.c[5] = (rpm_in & 0xff000000) >> 24;
    _msg_1.content.msg.c[6] = crc8_itu(_msg_1.content.msg.c, 6);
    _msg_1.content.msg.c[7] = 0x2F;
}

void HB1_apm2power::make_sum()
{
    _msg_1.content.msg.header.head_1 = HB1_apm2power::PREAMBLE1;
    _msg_1.content.msg.header.head_2 = HB1_apm2power::PREAMBLE2;
    _msg_1.content.msg.sum_check = 0;
    for (int8_t i = 0; i < _msg_1.length - 1; i++) {
        _msg_1.content.msg.sum_check += _msg_1.content.data[i];
    }
}

uint8_t HB1_apm2power::crc8_itu(uint8_t *data, uint8_t len)
{
    uint8_t crca = data[0];
    uint8_t crcb = 0;
    for(int i = 0; i < len; i++)
    {
        if(i == 0)
        {
            crca = crc8_itu_table[(data[i] ^ 0x00)];
            crcb = crca;
        }
        else
        {
            crca = crc8_itu_table[(data[i] ^ crcb)];
            crcb = crca;
        }
    }
    return (crcb ^ 0x55);
}
