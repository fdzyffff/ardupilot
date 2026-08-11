#pragma once

/*
 * FD_Target_HY — 慧眼(HY)图像跟踪板目标源
 *
 * 接收4.1.3测偏数据；可按参数周期发送4.4.5目标检测后自动锁定。
 */

#include "FD_Target.h"
#include <FD_UART/FD1_msg_HY_telem.h>
#include <FD_UART/FD1_msg_HY_control.h>

class FD_Target_HY : public FD_Target_Base {
public:
    FD_Target_HY();

    static const struct AP_Param::GroupInfo var_info[];

    bool init() override;
    void update() override;
    void handle_msg(const mavlink_message_t &msg) override {}

    AP_HAL::UARTDriver *get_port() const { return _port; }

private:
    void handle_miss_msg();

    // 中心原点像素脱靶量(右正/上正)转换为相机球面角(deg)。
    bool pixel_offset_to_spherical_deg(float offset_x, float offset_y,
                                       float &yaw_deg, float &pitch_deg) const;
    float cal_frame_angle(float pixel, float angle, float x_in) const;
    void send_control();
    void update_log_miss(int16_t yaw_cdeg, int16_t pitch_cdeg,
                         uint16_t w, uint16_t h, uint8_t status);

    AP_Int32 target_timeout;       // TC_TOUT 脱靶量超时 ms
    AP_Int8 auto_lock_enable;      // TC_AUTO 是否周期发送4.4.5
    AP_Int32 auto_lock_period_ms;  // TC_ATIME 4.4.5发送周期 ms
    AP_Float res_x;                // TC_RESX 输出图像宽 像素
    AP_Float res_y;                // TC_RESY 输出图像高 像素
    AP_Float fov_x;                // TC_FOVX 水平视场角 deg
    AP_Float fov_y;                // TC_FOVY 垂直视场角 deg

    AP_HAL::UARTDriver *_port;
    FD1_msg_HY_miss uart_msg_HY_miss;
    FD1_msg_HY_control uart_msg_HY_control;
    uint32_t last_auto_lock_ms = 0;
};
