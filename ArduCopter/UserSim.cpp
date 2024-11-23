#include "Copter.h"

void Copter::sim_init()
{
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    FD2_uart_msg_ue4.init();
    FD2_uart_msg_ue4.get_msg_ue4_ahrs().set_enable();
    FD2_uart_msg_ue4.get_msg2apm_ue4_gimbal().set_enable();
#endif
}

void Copter::sim_update()
{
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL

    FD2_uart_msg_ue4.read();

    SITL::SIM* _sitl = AP::sitl();
    if (_sitl == nullptr) {
        return;
    }
    if (_sitl) {
        const struct SITL::sitl_fdm &fdm = _sitl->state;

        FD2_msg_ue4_ahrs &tmp_msg = FD2_uart_msg_ue4.get_msg_ue4_ahrs();
        tmp_msg._msg_1.content.msg.header.head_1 = FD2_msg_ue4_ahrs::PREAMBLE1;
        tmp_msg._msg_1.content.msg.header.head_2 = FD2_msg_ue4_ahrs::PREAMBLE2;
        tmp_msg._msg_1.content.msg.vehicle_id = 0;
        tmp_msg._msg_1.content.msg.lat = fdm.latitude * 1e7;
        tmp_msg._msg_1.content.msg.lng = fdm.longitude * 1e7;
        tmp_msg._msg_1.content.msg.alt = fdm.altitude*100;
        tmp_msg._msg_1.content.msg.roll = fdm.rollDeg;
        tmp_msg._msg_1.content.msg.pitch = fdm.pitchDeg;
        tmp_msg._msg_1.content.msg.yaw = fdm.yawDeg;
        tmp_msg._msg_1.content.msg.sum_check = 0;

        for (int8_t i = 2; i < tmp_msg._msg_1.length - 1; i++) {
            tmp_msg._msg_1.content.msg.sum_check += tmp_msg._msg_1.content.data[i];
        }
        tmp_msg._msg_1.need_send = true;
    }

    FD2_uart_msg_ue4.write();

    static float last_yaw = 0.0f;
    static float last_pitch = 0.0f;
    FD2_msg2apm_ue4_gimbal &tmp_msg = FD2_uart_msg_ue4.get_msg2apm_ue4_gimbal();
    if (tmp_msg._msg_1.updated) {
        float target_x = degrees(tmp_msg._msg_1.content.msg.yaw);
        float target_y = degrees(tmp_msg._msg_1.content.msg.pitch);
        if (!is_zero(last_yaw-target_x) || !is_zero(last_pitch-target_y)) 
        {
            if (copter.uattack._cam_port_type == 1) {
                copter.uattack._UCam_ptr->handle_info_test(target_x, target_y);
            }
            last_yaw = target_x;
            last_pitch = target_y;
        }
        tmp_msg._msg_1.updated = false;   
    }
#endif
}