#pragma once

#include "FD_Target.h"

#include <AP_Param/AP_Param.h>
#include <FD_UART/FD1_msg_DYT_control.h>
#include <FD_UART/FD1_msg_DYT_telem.h>

class FD_Target_DYT : public FD_Target_Base {
public:
    FD_Target_DYT();

    static const struct AP_Param::GroupInfo var_info[];

    bool init() override;
    void update() override;
    void handle_msg(const mavlink_message_t &msg) override {}
    AP_HAL::UARTDriver *get_port() const { return _port; }

private:
    void update_log();

    AP_Int32 target_timeout;
    AP_Int32 center_time;
    AP_Int32 track_time;
    AP_HAL::UARTDriver *_port;
    FD1_msg_DYT_control uart_msg_DYT_control;
    FD1_msg_DYT_telem uart_msg_DYT_telem;
    uint32_t last_center_ms;
    uint32_t last_track_ms;
    uint32_t last_cancel_ms;
};