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
    if (_msg_1.length >= FD1_MSG_0919_P3_LEN) {
        return;
    }
    if (_msg_1.length < 1) {
        return;
    }
    _msg_1.content.msg.header.head_1 = FD1_msg_0919_p3::PREAMBLE1;
    _msg_1.content.data[_msg_1.length-1] = 0;
    for (uint16_t i = 0; i < _msg_1.length - 1; i++) {
        _msg_1.content.data[_msg_1.length-1] ^= _msg_1.content.data[i];
    }
}

void FD1_msg_0919_p3::swap_message(void)
{
    ;
}
