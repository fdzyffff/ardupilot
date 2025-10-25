#include "FD1_msg_0919_p5.h"
// #include <GCS_MAVLink/GCS.h>

FD1_msg_0919_p5::FD1_msg_0919_p5(void)
{
    _enable = false;
    _msg_1.need_send = false;
    _msg_1.updated = false;
}

void FD1_msg_0919_p5::parse(uint8_t temp)
{
    ;
}

void FD1_msg_0919_p5::process_message(void)
{
    ;
}

void FD1_msg_0919_p5::make_sum()
{
    _msg_1.content.msg.header.head_1 = FD1_msg_0919_p5::PREAMBLE1;
    _msg_1.content.msg.header.head_2 = FD1_msg_0919_p5::PREAMBLE2;
}

void FD1_msg_0919_p5::swap_message(void)
{
    ;
}
