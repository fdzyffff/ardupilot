#include "FD1_msg_uwb_out.h"

FD1_msg_uwb_out::FD1_msg_uwb_out(void)
{
    _enable = false;
    _msg_1.need_send = false;
    _msg_1.updated = false;
}

void FD1_msg_uwb_out::parse(uint8_t temp)
{
    ;
}

void FD1_msg_uwb_out::process_message(void)
{
    ;
}

void FD1_msg_uwb_out::swap_message(void)
{
    ;
}
