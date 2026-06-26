#include "FD1_msg_0x22.h"
// #include <GCS_MAVLink/GCS.h>

FD1_msg_0x22::FD1_msg_0x22(void)
{
    _enable = false;
    _msg_1.need_send = false;
    _msg_1.updated = false;
}

void FD1_msg_0x22::parse(uint8_t temp)
{
    ;
}

void FD1_msg_0x22::process_message(void)
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

void FD1_msg_0x22::make_sum()
{
    _msg_1.content.msg.header.head_1 = FD1_msg_0x22::PREAMBLE1;
    _msg_1.content.msg.header.head_2 = FD1_msg_0x22::PREAMBLE2;
    _msg_1.content.msg.xorsum = 0;
    for (int8_t i = 0; i < _msg_1.length - 1; i++) {
        _msg_1.content.msg.xorsum = (_msg_1.content.msg.xorsum ^ _msg_1.content.data[i]);
    }
}

void FD1_msg_0x22::swap_message(void)
{
    ;
}
