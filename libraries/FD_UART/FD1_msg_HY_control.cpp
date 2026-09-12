#include "FD1_msg_HY_control.h"

FD1_msg_HY_control::FD1_msg_HY_control(void)
{
    _enable = false;
    _msg_1.need_send = false;
    _msg_1.updated = false;
}

void FD1_msg_HY_control::pack_detect(uint8_t mode)
{
    MSG_Collection &msg = _msg_1.content.msg;
    msg.cmd0 = 0x03;
    msg.cmd1 = 0x01;
    msg.len = 0x02;
    msg.payload[0] = mode;  // 0关闭/1开启目标检测/2开启+多目标跟踪
    msg.payload[1] = 0x00;  // 保留
    msg.payload[2] = 0x00;
    msg.payload[3] = 0x00;
    make_sum();
}

void FD1_msg_HY_control::pack_auto_lock(uint8_t mode, uint8_t strategy)
{
    MSG_Collection &msg = _msg_1.content.msg;
    msg.cmd0 = 0x03;
    msg.cmd1 = 0x05;
    msg.len = 0x04;
    msg.payload[0] = mode;      // 0关/1单次/2循环自动锁定
    msg.payload[1] = strategy;  // 0置信度最高(默认)/1最接近十字中心
    // msg.payload[1] = 0x01;   // 旧:固定最接近十字中心策略
    msg.payload[2] = 0x00;
    msg.payload[3] = 0x00;
    make_sum();
}

void FD1_msg_HY_control::make_sum()
{
    MSG_Collection &msg = _msg_1.content.msg;
    msg.head_1 = 0x58;
    msg.head_2 = 0x07;

    // 校验和为 CMD0 至最后一个数据字节的累加和低字节。
    // 帧总长 = len + 7 (头2 + cmd0/cmd1/len + payload + sum + end)。
    const uint16_t total_len = (uint16_t)msg.len + 7U;
    uint8_t sum = 0;
    // for (uint16_t i = 2; i < 9; i++) {   // 旧:固定按4.4.5长度(7字节)
    for (uint16_t i = 2; i < total_len - 2U; i++) {
        sum = (uint8_t)(sum + _msg_1.content.data[i]);
    }
    msg.sum = sum;
    msg.end = 0x59;
    _msg_1.length = total_len;  // 旧: FD1_MSG_HY_CONTROL_LEN 固定11
    _msg_1.need_send = true;
}
