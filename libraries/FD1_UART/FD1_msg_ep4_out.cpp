#include "FD1_msg_ep4_out.h"

FD1_msg_ep4_out::FD1_msg_ep4_out(void)
{
    _enable = false;
    _msg_1.need_send = false;
    _msg_1.updated = false;
}

void FD1_msg_ep4_out::process_message(void)
{
    ;
}
void FD1_msg_ep4_out::parse(uint8_t temp)
{
    ;
}

void FD1_msg_ep4_out::swap_message(void)
{
    swap_message_sub(_msg_1.content.data[3-1], _msg_1.content.data[4-1]);
    swap_message_sub(_msg_1.content.data[5-1], _msg_1.content.data[6-1],_msg_1.content.data[7-1], _msg_1.content.data[8-1]);
}
