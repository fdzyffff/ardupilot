#include "FD1_msg_0919_p3.h"
// #include <GCS_MAVLink/GCS.h>

FD1_msg_0919_p3::FD1_msg_0919_p3(void)
{
    _enable = false;
    _msg_1.need_send = false;
    _msg_1.updated = false;
}

void FD1_msg_0919_p3::parse(uint8_t temp)
{
    ;
}

void FD1_msg_0919_p3::process_message(void)
{
    ;
}

void FD1_msg_0919_p3::make_sum()
{
    if (_msg_1.length + 6 >= FD1_MSG_0919_P3_LEN) {
        return;
    }

    _msg_1.content.msg.E_Type = 0x75;

    _msg_1.content.msg.header.head_1 = FD1_msg_0919_p3::PREAMBLE1;
    _msg_1.content.msg.header.head_2 = FD1_msg_0919_p3::PREAMBLE2;
    _msg_1.content.data[_msg_1.length+5] = 0;
    for (uint16_t i = 2; i < _msg_1.length + 5; i++) {
        _msg_1.content.data[_msg_1.length+5] ^= _msg_1.content.data[i];
    }
}

void FD1_msg_0919_p3::swap_message(void)
{
    ;
}
