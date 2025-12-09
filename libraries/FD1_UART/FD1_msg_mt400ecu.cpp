#include "FD1_msg_mt400ecu.h"
// #include <GCS_MAVLink/GCS.h>

FD1_msg_mt400ecu::FD1_msg_mt400ecu(void)
{
    _enable = false;
    _msg_1.need_send = false;
    _msg_1.updated = false;
}

void FD1_msg_mt400ecu::parse(uint8_t temp)
{
    ;
}

void FD1_msg_mt400ecu::process_message(void)
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

void FD1_msg_mt400ecu::make_sum()
{
    _msg_1.content.msg.header.head_1 = FD1_msg_mt400ecu::PREAMBLE1;
    _msg_1.content.msg.end = FD1_msg_mt400ecu::POSTAMBLE1;
    _msg_1.content.msg.sum = 0;
    for (int8_t i = 1; i < 12; i++) {
        _msg_1.content.msg.sum = (_msg_1.content.msg.sum + _msg_1.content.data[i]);
    }
}

void FD1_msg_mt400ecu::swap_message(void)
{
    ;
}
