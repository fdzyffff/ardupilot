#include "FD1_msg_HY_control.h"
#include <string.h>

FD1_msg_HY_control::FD1_msg_HY_control(void)
{
    _enable = false;
    _msg_1.need_send = false;
    _msg_1.updated = false;
}

// 通用组包骨架：清 payload -> 写 cmd/len -> make_sum
static void pack_begin(FD1_msg_HY_control::FD1UART_MSG_1 &m,
                       uint8_t cmd0, uint8_t cmd1, uint8_t len)
{
    memset(m.content.msg.payload, 0, sizeof(m.content.msg.payload));
    m.content.msg.cmd0 = cmd0;
    m.content.msg.cmd1 = cmd1;
    m.content.msg.len = len;
}

void FD1_msg_HY_control::pack_open_detection()
{
    // 03 01: byte0=2 开启目标检测+多目标跟踪(生成唯一ID, 供ID跟踪)
    pack_begin(_msg_1, 0x03, 0x01, 2);
    _msg_1.content.msg.payload[0] = 0x02;
    make_sum();
}

void FD1_msg_HY_control::pack_close_detection()
{
    // 03 01: byte0=0 关闭目标检测
    pack_begin(_msg_1, 0x03, 0x01, 2);
    _msg_1.content.msg.payload[0] = 0x00;
    make_sum();
}

void FD1_msg_HY_control::pack_detect_all_types()
{
    // 03 03: u16=0xFFFF 检测所有AI目标
    pack_begin(_msg_1, 0x03, 0x03, 4);
    _msg_1.content.msg.payload[0] = 0xFF;
    _msg_1.content.msg.payload[1] = 0xFF;
    make_sum();
}

void FD1_msg_HY_control::pack_auto_lock()
{
    // 03 05: byte0=2 循环自动锁定; byte1=1 最接近十字位置策略
    pack_begin(_msg_1, 0x03, 0x05, 4);
    _msg_1.content.msg.payload[0] = 0x02;
    _msg_1.content.msg.payload[1] = 0x01;
    make_sum();
}

void FD1_msg_HY_control::pack_stop_track()
{
    // 03 11: byte0=0 停止跟踪(波门回十字中心)
    pack_begin(_msg_1, 0x03, 0x11, 10);
    _msg_1.content.msg.payload[0] = 0x00;
    make_sum();
}

void FD1_msg_HY_control::pack_track_id(uint8_t id)
{
    // 03 11: byte0=2 ID跟踪; byte1=目标ID(1~64)
    pack_begin(_msg_1, 0x03, 0x11, 10);
    _msg_1.content.msg.payload[0] = 0x02;
    _msg_1.content.msg.payload[1] = id;
    make_sum();
}

void FD1_msg_HY_control::make_sum()
{
    _msg_1.content.msg.header.head_1 = PREAMBLE1;   // 0x58
    _msg_1.content.msg.header.head_2 = PREAMBLE2;   // 0x07

    const uint8_t len = _msg_1.content.msg.len;
    const uint16_t sum_idx = (uint16_t)5 + len;     // CHK 位置：5+len
    _msg_1.length = (uint16_t)len + 7;              // 实际帧长

    uint8_t sum = 0;
    for (uint16_t i = 2; i < sum_idx; i++) {        // CMD0 至最后一个数据字节
        sum = (uint8_t)(sum + _msg_1.content.data[i]);
    }
    _msg_1.content.data[sum_idx] = sum;
    _msg_1.content.data[sum_idx + 1] = MSG_END;     // 0x59
    _msg_1.need_send = true;
}
