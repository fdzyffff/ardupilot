#include "HB1_apm2rocket.h"

HB1_apm2rocket::HB1_apm2rocket(void)
{
    _enable = false;
    _msg_1.need_send = false;
    _msg_1.updated = false;
}

void HB1_apm2rocket::parse(uint8_t temp)
{
    ;
}

void HB1_apm2rocket::process_message(void)
{
    ;
}

void HB1_apm2rocket::swap_message(void)
{
    // if (_msg_1.content.msg.c[0] == 0x48 && _msg_1.content.msg.c[1] == 0x06) {
    //     swap_message_sub(_msg_1.content.msg.c[2], _msg_1.content.msg.c[3], _msg_1.content.msg.c[4], _msg_1.content.msg.c[5]);
    // }
}

void HB1_apm2rocket::set_rocket_on()
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

void HB1_apm2rocket::make_sum()
{
    _msg_1.content.msg.header.head_1 = HB1_apm2rocket::PREAMBLE1;
    _msg_1.content.msg.header.head_2 = HB1_apm2rocket::PREAMBLE2;
    _msg_1.content.msg.sum_check = 0;
    for (int8_t i = 0; i < _msg_1.length - 1; i++) {
        _msg_1.content.msg.sum_check += _msg_1.content.data[i];
    }
}

uint8_t HB1_apm2rocket::crc8_itu(uint8_t *data, uint8_t len)
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
