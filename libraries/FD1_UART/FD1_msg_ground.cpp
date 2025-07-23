#include "FD1_msg_ground.h"
// #include <GCS_MAVLink/GCS.h>

FD1_msg_ground::FD1_msg_ground(void)
{
    _enable = false;
    _msg_1.need_send = false;
    _msg_1.updated = false;
}

void FD1_msg_ground::parse(uint8_t temp)
{
    ;
}

void FD1_msg_ground::process_message(void)
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

void FD1_msg_ground::make_sum()
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

void FD1_msg_ground::swap_message(void)
{
    _msg_1.content.msg.length = swap_message_uint16_t(_msg_1.content.msg.length);
    _msg_1.content.msg.sum2 = swap_message_uint16_t(_msg_1.content.msg.sum2);
}
