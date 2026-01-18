#include "FD1_msg_reply.h"
// #include <GCS_MAVLink/GCS.h>

FD1_msg_reply::FD1_msg_reply(void)
{
    _enable = false;
    _msg_1.need_send = false;
    _msg_1.updated = false;
}

void FD1_msg_reply::parse(uint8_t temp)
{
    ;
}

void FD1_msg_reply::process_message(void)
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

void FD1_msg_reply::make_sum()
{
    _msg_1.content.msg.header.head_1 = FD1_msg_reply::PREAMBLE1;
    _msg_1.content.msg.header.head_2 = FD1_msg_reply::PREAMBLE2;
    _msg_1.content.msg.xorsum = 0;
    for (int8_t i = 0; i < _msg_1.length - 1; i++) {
        _msg_1.content.msg.xorsum = (_msg_1.content.msg.xorsum ^ _msg_1.content.data[i]);
    }
}

void FD1_msg_reply::swap_message(void)
{
    ;
}
