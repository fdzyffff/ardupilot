#include "Plane.h"

void Plane::userhook_init()
{
    uattack.init();
    ufollow.init();
    umission.init();
    sim_init();
}

void Plane::userhook_100Hz()
{
    uattack.update();
    ufollow.update();
    umission.update();
    sim_update();
}

void Plane::userhook_1Hz()
{
    if ((g2.user_print.get() & (1<<0)) && uattack.display_info_new) { // 1
        gcs().send_text(MAV_SEVERITY_WARNING, "[%d] %0.0f,%0.0f,%0.0f,%0.0f", uattack.display_info_count_log, uattack.display_info_p1, uattack.display_info_p2, uattack.display_info_p3, uattack.display_info_p4);
        uattack.display_info_new = false;
    }
    if (g2.user_print.get() & (1<<1)) { // 2
        gcs().send_text(MAV_SEVERITY_WARNING, "Corr (%0.0f,%0.0f) on:%d", uattack.get_ef_rate_info().x,uattack.get_ef_rate_info().y, uattack.is_active());
    }
    if (g2.user_print.get() & (1<<2)) { // 4
        gcs().send_text(MAV_SEVERITY_WARNING, "rpy (%0.1f,%0.1f,%0.1f)", uattack.get_target_roll_angle(), uattack.get_target_pitch_rate(), uattack.get_target_yaw_rate());
    }
    if (g2.user_print.get() & (1<<5)) { // 32
        ufollow.print();
    }

}

void Plane::sim_init()
{
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    FD2_uart_msg_ue4.init();
    FD2_uart_msg_ue4.get_msg_ue4_ahrs().set_enable();
    FD2_uart_msg_ue4.get_msg2apm_ue4_gimbal().set_enable();
#endif
}

void Plane::sim_update()
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

    FD2_msg2apm_ue4_gimbal &tmp_msg = FD2_uart_msg_ue4.get_msg2apm_ue4_gimbal();
    if (tmp_msg._msg_1.updated) {
        float target_x = degrees(tmp_msg._msg_1.content.msg.yaw);
        float target_y = degrees(tmp_msg._msg_1.content.msg.pitch);
        if (plane.uattack._cam_port_type == 1) {
            plane.uattack._UCam_ptr->handle_info(target_x, target_y);
        }
        tmp_msg._msg_1.updated = false;   
    }
#endif
}