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

void FD1_msg_0919_p4::make_sum()
{
    if (_msg_1.length >= FD1_MSG_0919_P4_LEN) {
        return;
    }
    if (_msg_1.length < 1) {
        return;
    }
    _msg_1.content.msg.header.head_1 = FD1_msg_0919_p4::PREAMBLE1;
    _msg_1.content.data[_msg_1.length-1] = 0;
    for (uint16_t i = 0; i < _msg_1.length - 1; i++) {
        _msg_1.content.data[_msg_1.length-1] ^= _msg_1.content.data[i];
    }
}

void FD1_msg_0919_p4::swap_message(void)
{
    ;
}
