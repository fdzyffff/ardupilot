#pragma once

#include "FD_Target.h"

#include <AP_Param/AP_Param.h>

/*
 * FD_Target_Cam —— 相机类导引头目标源基类（HUOJIAN）
 *
 * HY(慧眼)/DYT/DYT2 三种导引头共用：串口(SerialProtocol_DYT, 实例 0)获取、
 * 数据超时(CAM_TOUT)判定；帧率统计在 FD_Target_Base::handle_info() 内完成。
 * 派生类只实现各自协议的 init()/update()：把导引头给出的相机系离轴角交给
 * handle_info(p1, p2)。FD_Attack 按 UATK_CAM_USE 选一个实例挂到 target_cam 指针。
 *
 * 注意：本类不带参数表。AP_Param 只允许 3 层组嵌套，FD_Attack_Copter→FD_Attack→TC_/TD_
 * 已经到第 3 层，再嵌 FE_Target_Cam 的组会触发 "double group nesting" 启动 panic。
 * 故 target_timeout 只作为共用成员放在这里，由各派生类在自己的 var_info 里用
 * AP_GROUPINFO("TOUT", 0, <派生类>, target_timeout, 2000) 注册（参数名/序号保持不变）。
 */
class FD_Target_Cam : public FD_Target_Base {
public:
    FD_Target_Cam();

    void handle_msg(const mavlink_message_t &msg) override {}
    AP_HAL::UARTDriver *get_port() const { return _port; }

protected:
    // 找导引头串口（协议 SerialProtocol_DYT，实例 0）
    bool init_port();
    // 按 CAM_TOUT 判定数据超时：超时把 _valid 拉低
    void update_timeout(uint32_t now_ms);
    // 串口发送（端口空指针保护）
    void write_bytes(const uint8_t *data, uint16_t len);

    // CAM_TOUT 导引头数据超时 ms
    AP_Int32 target_timeout;
    AP_HAL::UARTDriver *_port;
};
