#include "HB1_apm2power.h"

HB1_apm2power::HB1_apm2power(void)
{
    _enable = false;
    _msg_1.need_send = false;
    _msg_1.updated = false;
}

void HB1_apm2power::parse(uint8_t temp)
{
    ;
}

void HB1_apm2power::process_message(void)
{
    ;
}

void HB1_apm2power::swap_message(void)
{
    swap_message_sub(_msg_1.content.data[3], _msg_1.content.data[4]);
}
