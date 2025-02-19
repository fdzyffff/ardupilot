#pragma once

#include "FD1_msg_reply.h"
#include "FD1_msg_0x11.h"
#include "FD1_msg_0x22.h"
#include "FD1_msg_0x31.h"
#include "FD1_msg_0x33.h"
#include "FD1_msg_0x36.h"
#include "FD1_msg_0x37.h"
#include "FD1_msg_attack.h"
// 命令分类    命令字    命令名称      流向
// 全程交互     0x11   飞控状态信息   飞控→任务
// 正常飞行信息 0x22    开始飞行      飞控→任务
// 控制指令     0x31    航点飞行指令  任务→飞控
//             0x33    回收/自毁指令 任务→飞控
//             0x36    进入攻击指令  任务→飞控
//             0x37    退出攻击指令  任务→飞控
