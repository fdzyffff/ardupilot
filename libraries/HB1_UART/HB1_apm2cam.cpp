#include "HB1_apm2cam.h"

HB1_apm2cam::HB1_apm2cam(void)
{
    _enable = false;
    _msg_1.need_send = false;
    _msg_1.updated = false;
}

void HB1_apm2cam::parse(uint8_t temp)
{
    ;
}

void HB1_apm2cam::process_message(void)
{
    ;
}

void HB1_apm2cam::swap_message(void)
{
    swap_message_sub(_msg_1.content.data[9], _msg_1.content.data[10]);
    swap_message_sub(_msg_1.content.data[11], _msg_1.content.data[12], _msg_1.content.data[13], _msg_1.content.data[14]);
    swap_message_sub(_msg_1.content.data[15], _msg_1.content.data[16], _msg_1.content.data[17], _msg_1.content.data[18]);
    swap_message_sub(_msg_1.content.data[19], _msg_1.content.data[20]);
    swap_message_sub(_msg_1.content.data[21], _msg_1.content.data[22]);
    swap_message_sub(_msg_1.content.data[23], _msg_1.content.data[24]);
    swap_message_sub(_msg_1.content.data[25], _msg_1.content.data[26]);
    swap_message_sub(_msg_1.content.data[27], _msg_1.content.data[28]);
    swap_message_sub(_msg_1.content.data[29], _msg_1.content.data[30]);
    swap_message_sub(_msg_1.content.data[31], _msg_1.content.data[32]);
    swap_message_sub(_msg_1.content.data[33], _msg_1.content.data[34]);
}
