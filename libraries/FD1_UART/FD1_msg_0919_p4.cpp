#include "FD1_msg_0919_p4.h"
// #include <GCS_MAVLink/GCS.h>

FD1_msg_0919_p4::FD1_msg_0919_p4(void)
{
    _enable = false;
    _msg_1.need_send = false;
    _msg_1.updated = false;
}

void FD1_msg_0919_p4::parse(uint8_t temp)
{
    ;
}

void FD1_msg_0919_p4::process_message(void)
{
    ;
}

void FD1_msg_0919_p4::make_sum()
{
    _msg_1.content.msg.header.head_1 = FD1_msg_0919_p4::PREAMBLE1;
    _msg_1.content.msg.header.head_2 = FD1_msg_0919_p4::PREAMBLE2;
    _msg_1.content.msg.length = 39;
    _msg_1.content.msg.E_Type = 0x7D;
    _msg_1.content.data[42] = 0;
    _msg_1.content.data[43] = 0;
    _msg_1.content.msg.sum = 0;
    for (uint16_t i = 2; i < _msg_1.length + 5; i++) {
        _msg_1.content.msg.sum ^= _msg_1.content.data[i];
    }
}

void FD1_msg_0919_p4::swap_message(void)
{
    ;
}
