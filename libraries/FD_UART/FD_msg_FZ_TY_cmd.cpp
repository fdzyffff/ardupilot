#include "FD_msg_FZ_TY_cmd.h"

FD_msg_FZ_TY_cmd::FD_msg_FZ_TY_cmd(void)
{
    _enable = false;
    _msg_1.need_send = false;
    _msg_1.updated = false;
}

void FD_msg_FZ_TY_cmd::pack(uint8_t cmd, uint8_t arg)
{
    MSG_Collection &msg = _msg_1.content.msg;
    msg.head_1 = HEAD_1;
    msg.dest = DEST_DEVICE;
    msg.cmd = cmd;
    // 仅装定工作模式用独立参数，其余命令 D2 = D1 冗余
    msg.arg = (cmd == CMD_SET_MODE) ? arg : cmd;
    msg.end = 0x55;
    make_sum();
    _msg_1.need_send = true;
}

void FD_msg_FZ_TY_cmd::pack_selftest()
{
    pack(CMD_SELFTEST, 0);
}

void FD_msg_FZ_TY_cmd::pack_set_mode(uint8_t mode)
{
    pack(CMD_SET_MODE, mode & 0x03U);
}

void FD_msg_FZ_TY_cmd::pack_arm1()
{
    pack(CMD_ARM1, 0);
}

void FD_msg_FZ_TY_cmd::pack_arm2()
{
    pack(CMD_ARM2, 0);
}

void FD_msg_FZ_TY_cmd::pack_fire()
{
    pack(CMD_FIRE, 0);
}

void FD_msg_FZ_TY_cmd::make_sum()
{
    MSG_Collection &msg = _msg_1.content.msg;
    // 校验和 = 目的地标识 + 数据1 + 数据2 取低字节
    msg.sum = (uint8_t)(msg.dest + msg.cmd + msg.arg);
    _msg_1.length = FD_MSG_FZ_TY_CMD_LEN;
}
