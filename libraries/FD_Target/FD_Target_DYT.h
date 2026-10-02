#pragma once

#include "FD_Target_Cam.h"

#include <AP_Param/AP_Param.h>
#include <FD_UART/FD1_msg_DYT_control.h>
#include <FD_UART/FD1_msg_DYT_telem.h>

/*
 * FD_Target_DYT —— 旧 DYT(0xEB90/0xEE16 协议) 导引头目标源
 * 暂不纳入 UATK_CAM_USE 选择（现用导引头为 DYT2，协议不同），保留类定义备用。
 * 基类 FD_Target_Cam 提供导引头串口(SerialProtocol_DYT)与数据超时(TOUT)。
 */
class FD_Target_DYT : public FD_Target_Cam {
public:
    FD_Target_DYT();

    static const struct AP_Param::GroupInfo var_info[];

    bool init() override;
    void update() override;
    void handle_msg(const mavlink_message_t &msg) override {}

private:
    void update_log();

    AP_Int32 center_time;
    AP_Int32 track_time;
    FD1_msg_DYT_control uart_msg_DYT_control;
    FD1_msg_DYT_telem uart_msg_DYT_telem;
    uint32_t last_center_ms;
    uint32_t last_track_ms;
    uint32_t last_cancel_ms;
};