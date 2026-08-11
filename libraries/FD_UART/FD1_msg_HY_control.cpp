#include "FD1_msg_HY_control.h"

FD1_msg_HY_control::FD1_msg_HY_control(void)
{
    _enable = false;
    _msg_1.need_send = false;
    _msg_1.updated = false;
}

void FD1_msg_HY_control::pack_auto_lock()
{
    MSG_Collection &msg = _msg_1.content.msg;
    msg.cmd0 = 0x03;
    msg.cmd1 = 0x05;
    msg.len = 0x04;
    msg.payload[0] = 0x02; // 循环自动锁定
    msg.payload[1] = 0x01; // 最接近十字中心策略
    msg.payload[2] = 0x00;
    msg.payload[3] = 0x00;
    make_sum();
}

void FD1_msg_HY_control::make_sum()
{
    MSG_Collection &msg = _msg_1.content.msg;
    msg.head_1 = 0x58;
    msg.head_2 = 0x07;

    uint8_t sum = 0;
    for (uint16_t i = 2; i < 9; i++) {
        sum = (uint8_t)(sum + _msg_1.content.data[i]);
    }
    msg.sum = sum;
    msg.end = 0x59;
    _msg_1.length = FD1_MSG_HY_CONTROL_LEN;
    _msg_1.need_send = true;
}
