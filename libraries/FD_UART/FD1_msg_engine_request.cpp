#include "FD1_msg_engine_request.h"
// #include <GCS_MAVLink/GCS.h>

FD1_msg_engine_request::FD1_msg_engine_request(void)
{
    _enable = false;
    _msg_1.need_send = false;
    _msg_1.updated = false;
}

void FD1_msg_engine_request::parse(uint8_t temp)
{
    ;
}

void FD1_msg_engine_request::process_message(void)
{
    ;
}

void FD1_msg_engine_request::make_sum()
{
    _msg_1.content.data[0] = 0x00;
    _msg_1.content.data[1] = 0x07;
    _msg_1.content.data[2] = 0x72;
    _msg_1.content.data[3] = 0x00;
    _msg_1.content.data[4] = 0x07;
    _msg_1.content.data[5] = 0x00;
    _msg_1.content.data[6] = 0x00;
    _msg_1.content.data[7] = 0x00;
    _msg_1.content.data[8] = 0x51;
    _msg_1.content.data[9] = 0x48;
    _msg_1.content.data[10] = 0xD7;
    _msg_1.content.data[11] = 0xA3;
    _msg_1.content.data[12] = 0x0E;
    // swap_message();
}

void FD1_msg_engine_request::swap_message(void)
{
    _msg_1.content.msg.size = swap_message_uint16_t(_msg_1.content.msg.size);
}
