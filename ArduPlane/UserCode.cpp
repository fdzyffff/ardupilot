#include "Plane.h"

void Plane::userhook_init()
{
    ;
    // FD1_msg_ts &tmp_msg = ts_ctrl.uart_msg_ts.get_msg_ts_in();
    // tmp_msg._msg_1.content.msg.sub_msg.msg_40.sub_t1.plane_lat = 1123123;
}

void Plane::userhook_100Hz()
{
    ;
}

void Plane::userhook_1Hz()
{
    // static uint32_t a1 = 1000;
    // union PACKED{
    //     uint32_t v = a1;
    //     uint8_t data[4];
    // } a;
    // // a.v = a1;
    // uint8_t tmp;
    // tmp = a.data[0];
    // a.data[0] = a.data[3];
    // a.data[3] = tmp;
    // tmp = a.data[1];
    // a.data[1] = a.data[2];
    // a.data[2] = tmp;
    // a1 = a.v;

    // gcs().send_text(MAV_SEVERITY_INFO, "%d", a1);

    // FD1_msg_ts &tmp_msg = ts_ctrl.uart_msg_ts.get_msg_ts_in();
    // tmp_msg._msg_1.content.msg.sub_msg.msg_40.sub_t1.plane_lat = tmp_msg.swap_message_int32_t(tmp_msg._msg_1.content.msg.sub_msg.msg_40.sub_t1.plane_lat);
    // gcs().send_text(MAV_SEVERITY_INFO, "%d", tmp_msg._msg_1.content.msg.sub_msg.msg_40.sub_t1.plane_lat);
}

// void Plane::sim_init()
// {
// #if CONFIG_HAL_BOARD == HAL_BOARD_SITL
//     FD2_uart_msg_ue4.init();
//     FD2_uart_msg_ue4.get_msg_ue4_ahrs().set_enable();
//     FD2_uart_msg_ue4.get_msg2apm_ue4_gimbal().set_enable();
// #endif
// }

// void Plane::sim_update()
// {
// #if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    
//     FD2_uart_msg_ue4.read();

//     SITL::SIM* _sitl = AP::sitl();
//     if (_sitl == nullptr) {
//         return;
//     }
//     if (_sitl) {
//         const struct SITL::sitl_fdm &fdm = _sitl->state;

//         FD2_msg_ue4_ahrs &tmp_msg = FD2_uart_msg_ue4.get_msg_ue4_ahrs();
//         tmp_msg._msg_1.content.msg.header.head_1 = FD2_msg_ue4_ahrs::PREAMBLE1;
//         tmp_msg._msg_1.content.msg.header.head_2 = FD2_msg_ue4_ahrs::PREAMBLE2;
//         tmp_msg._msg_1.content.msg.vehicle_id = 0;
//         tmp_msg._msg_1.content.msg.lat = fdm.latitude * 1e7;
//         tmp_msg._msg_1.content.msg.lng = fdm.longitude * 1e7;
//         tmp_msg._msg_1.content.msg.alt = fdm.altitude*100;
//         tmp_msg._msg_1.content.msg.roll = fdm.rollDeg;
//         tmp_msg._msg_1.content.msg.pitch = fdm.pitchDeg;
//         tmp_msg._msg_1.content.msg.yaw = fdm.yawDeg;
//         tmp_msg._msg_1.content.msg.sum_check = 0;

//         for (int8_t i = 2; i < tmp_msg._msg_1.length - 1; i++) {
//             tmp_msg._msg_1.content.msg.sum_check += tmp_msg._msg_1.content.data[i];
//         }
//         tmp_msg._msg_1.need_send = true;
//     }

//     FD2_uart_msg_ue4.write();

//     FD2_msg2apm_ue4_gimbal &tmp_msg = FD2_uart_msg_ue4.get_msg2apm_ue4_gimbal();
//     if (tmp_msg._msg_1.updated) {
//         float target_x = degrees(tmp_msg._msg_1.content.msg.yaw);
//         float target_y = degrees(tmp_msg._msg_1.content.msg.pitch);
//         if (plane.uattack._cam_port_type == 1) {
//             plane.uattack._UCam_ptr->handle_info(target_x, target_y);
//         }
//         tmp_msg._msg_1.updated = false;   
//     }
// #endif
// }