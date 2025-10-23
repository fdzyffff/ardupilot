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

#include "FD1_msg_0728_p1.h"
#include "FD1_msg_0728_p2.h"
#include "FD1_msg_0728_p3.h"


#include "FD1_msg_0919_p1.h" //载荷至无人机 循迹移动控制指令（0xC1，0xD3）。
#include "FD1_msg_0919_p2.h" //飞行控制指令（0xC1，0xE3）描述：终端按照指令进行起飞/降落。
#include "FD1_msg_0919_p3.h" //循迹移动执行成功事件（0x75）描述：循迹移动指令回执事件。
#include "FD1_msg_0919_p4.h" //飞行控制成功事件（0x7D）描述：飞行控制回执事件。
// #include "FD1_msg_0919_p5.h" //无人机心跳。

