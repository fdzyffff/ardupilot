#include "FD_msg_SERVO_24.h"
#include <GCS_MAVLink/GCS.h>

FD_msg_SERVO_24::FD_msg_SERVO_24(void)
{
    _enable = false;
    _msg_1.need_send = false;
    _msg_1.updated = false;
}

void FD_msg_SERVO_24::parse(uint8_t temp)
{
    ;
}

void FD_msg_SERVO_24::process_message(void)
{
    ;
}

void FD_msg_SERVO_24::swap_message(void)
{
    // swap_message_sub(_msg_1.content.data[7-1] , _msg_1.content.data[8-1] );
    // swap_message_sub(_msg_1.content.data[9-1] , _msg_1.content.data[10-1] );
    // swap_message_sub(_msg_1.content.data[4-1] , _msg_1.content.data[5-1] , _msg_1.content.data[6-1] , _msg_1.content.data[7-1]);
    // swap_message_sub(_msg_1.content.data[8-1] , _msg_1.content.data[9-1] , _msg_1.content.data[10-1], _msg_1.content.data[11-1]);
    // swap_message_sub(_msg_1.content.data[12-1], _msg_1.content.data[13-1], _msg_1.content.data[14-1], _msg_1.content.data[15-1]);
    // swap_message_sub(_msg_1.content.data[16-1], _msg_1.content.data[17-1], _msg_1.content.data[18-1], _msg_1.content.data[19-1]);
    // swap_message_sub(_msg_1.content.data[20-1], _msg_1.content.data[21-1], _msg_1.content.data[22-1], _msg_1.content.data[23-1]);
    // swap_message_sub(_msg_1.content.data[24-1], _msg_1.content.data[25-1], _msg_1.content.data[26-1], _msg_1.content.data[27-1]);
}

void FD_msg_SERVO_24::sum_check(void)
{
    _msg_1.content.msg.header.head_1 = PREAMBLE1;
    _msg_1.content.msg.header.head_2 = PREAMBLE2;
    _msg_1.content.msg.header.data_id = DATA_ID;
    _msg_1.content.msg.header.data_length = DATA_LENGTH;
    _msg_1.length = _msg_1.content.msg.header.data_length + 5;
    _msg_1.content.msg.sum_check = 0;
    for (uint16_t i = 0; i < _msg_1.length - 1; i ++) {
        _msg_1.content.msg.sum_check += _msg_1.content.data[i];
    }
}
