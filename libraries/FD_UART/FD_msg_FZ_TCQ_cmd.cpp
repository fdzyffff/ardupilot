#include "FD_msg_FZ_TCQ_cmd.h"

FD_msg_FZ_TCQ_cmd::FD_msg_FZ_TCQ_cmd(void)
{
    _enable = false;
    _msg_1.need_send = false;
    _msg_1.updated = false;
}

void FD_msg_FZ_TCQ_cmd::pack_selftest()
{
    MSG_Collection &msg = _msg_1.content.msg;
    msg.head_1 = PREAMBLE1;
    msg.head_2 = SELFTEST_HEAD2;
    msg.value = SELFTEST_CMD;
    make_sum();
    _msg_1.need_send = true;
}

void FD_msg_FZ_TCQ_cmd::pack_zg(uint8_t zg_x2)
{
    MSG_Collection &msg = _msg_1.content.msg;
    msg.head_1 = PREAMBLE1;
    msg.head_2 = ZG_HEAD2;
    msg.value = zg_x2;
    make_sum();
    _msg_1.need_send = true;
}

void FD_msg_FZ_TCQ_cmd::make_sum()
{
    MSG_Collection &msg = _msg_1.content.msg;
    // 校验和: 前三字节累加和取低字节
    msg.sum = (uint8_t)(msg.head_1 + msg.head_2 + msg.value);
    _msg_1.length = FD_MSG_FZ_TCQ_CMD_LEN;
}
