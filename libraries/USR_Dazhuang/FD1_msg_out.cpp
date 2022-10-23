#include "FD1_msg_out.h"

FD1_msg_out::FD1_msg_out(void)
{
    _enable = false;
    _msg_1.need_send = false;
    _msg_1.updated = false;
}

void FD1_msg_out::parse(uint8_t temp)
{
    ;
}

void FD1_msg_out::process_message(void)
{
    ;
}

void FD1_msg_out::swap_message(void)
{
    swap_message_sub(_msg_1.content.data[3-1], _msg_1.content.data[4-1]);
}
