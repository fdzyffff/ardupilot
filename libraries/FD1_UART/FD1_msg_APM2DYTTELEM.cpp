#include "FD1_msg_APM2DYTTELEM.h"

FD1_msg_APM2DYTTELEM::FD1_msg_APM2DYTTELEM(void)
{
    _enable = false;
    _msg_1.need_send = false;
    _msg_1.updated = false;
}

void FD1_msg_APM2DYTTELEM::parse(uint8_t temp)
{
    ;
}

void FD1_msg_APM2DYTTELEM::process_message(void)
{
    int16_t i = 0;

    for (i = 0; i < _msg_1.length; i ++) {
        _msg_1.content.data[i] = _msg.data[i];
    }
    swap_message();
    _msg_1.updated = true;
    _msg_1.need_send = false;
    _msg_1.print = true;
}

void FD1_msg_APM2DYTTELEM::swap_message(void)
{
    // swap_message_sub(_msg_1.content.data[4-1] , _msg_1.content.data[5-1] , _msg_1.content.data[6-1] , _msg_1.content.data[7-1]);
    // swap_message_sub(_msg_1.content.data[8-1] , _msg_1.content.data[9-1] , _msg_1.content.data[10-1], _msg_1.content.data[11-1]);
    // swap_message_sub(_msg_1.content.data[12-1], _msg_1.content.data[13-1], _msg_1.content.data[14-1], _msg_1.content.data[15-1]);
    // swap_message_sub(_msg_1.content.data[16-1], _msg_1.content.data[17-1], _msg_1.content.data[18-1], _msg_1.content.data[19-1]);
    // swap_message_sub(_msg_1.content.data[20-1], _msg_1.content.data[21-1], _msg_1.content.data[22-1], _msg_1.content.data[23-1]);
    // swap_message_sub(_msg_1.content.data[24-1], _msg_1.content.data[25-1], _msg_1.content.data[26-1], _msg_1.content.data[27-1]);
}

void FD1_msg_APM2DYTTELEM::sum_check(void)
{
    int16_t i = 0;

    _msg_1.content.msg.sum_check = 0;
    for (i = 0; i < _msg_1.length-1; i ++) {
        _msg_1.content.msg.sum_check += _msg_1.content.data[i];
    }
}