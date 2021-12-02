#include "Copter.h"

#ifdef USERHOOK_INIT
void Copter::userhook_init()
{
    // put your initialisation code here
    // this will be called once at start-up
    userhook_init_cam();
}
#endif

#ifdef USERHOOK_FASTLOOP
void Copter::userhook_FastLoop()
{
    // put your 100Hz code here
}
#endif

#ifdef USERHOOK_50HZLOOP
void Copter::userhook_50Hz()
{
    // put your 50Hz code here
    userhook_50Hz_update_uart();
    userhook_50Hz_update_cam();
}
#endif

#ifdef USERHOOK_MEDIUMLOOP
void Copter::userhook_MediumLoop()
{
    // put your 10Hz code here
    Ucam_Log_Write_UCamTarget();
}
#endif

#ifdef USERHOOK_SLOWLOOP
void Copter::userhook_SlowLoop()
{
    // put your 1Hz code here
    if ((g2.user_parameters.cam_print.get() & (1<<0)) && Ucam.display_info_new) { // 1
        gcs().send_text(MAV_SEVERITY_WARNING, "Raw (%0.0f,%0.0f) on:%0.0f", Ucam.display_info_p1, Ucam.display_info_p2, Ucam.display_info_p3);
        Ucam.display_info_new = false;
    }
    if (g2.user_parameters.cam_print.get() & (1<<1)) { // 2
        gcs().send_text(MAV_SEVERITY_WARNING, "Corr (%0.0f,%0.0f) on:%d", Ucam.get_correct_info().x,Ucam.get_correct_info().y, Ucam.is_active());
    }
    if (g2.user_parameters.cam_print.get() & (1<<2)) { // 4
        gcs().send_text(MAV_SEVERITY_WARNING, "rpy (%0.1f,%0.1f,%0.1f)", Ucam.get_target_roll_angle()*0.01f, Ucam.get_target_pitch_rate()*0.01f, Ucam.get_target_yaw_rate()*0.01f);
    }
    if (g2.user_parameters.cam_print.get() & (1<<3)) { // 8
        gcs().send_text(MAV_SEVERITY_WARNING, "Track Angle (%0.1f)", Ucam.get_current_angle_deg());
    }
}
#endif

#ifdef USERHOOK_SUPERSLOWLOOP
void Copter::userhook_SuperSlowLoop()
{
    // put your 1Hz code here
}
#endif

#ifdef USERHOOK_AUXSWITCH
void Copter::userhook_auxSwitch1(uint8_t ch_flag)
{
    // put your aux switch #1 handler here (CHx_OPT = 47)
}

void Copter::userhook_auxSwitch2(uint8_t ch_flag)
{
    // put your aux switch #2 handler here (CHx_OPT = 48)
}

void Copter::userhook_auxSwitch3(uint8_t ch_flag)
{
    // put your aux switch #3 handler here (CHx_OPT = 49)
}
#endif

void Copter::userhook_init_cam() {
    FD1_uart_msg_cam.init();
    FD1_uart_msg_cam.get_msg_cam_in().set_enable();
}

void Copter::userhook_50Hz_update_uart() {
    if (FD1_uart_msg_cam.initialized()) {
        FD1_uart_msg_cam.read();
    }
    // test mode
    if (g2.user_parameters.uart_cam_test != 0) {
        FD1_uart_cam_test_send();
    } else {
        FD1_uart_cam_handle();
    }
    FD1_uart_msg_cam.write();

}

void Copter::userhook_50Hz_update_cam() {
    Ucam.update();
}

void Copter::FD1_uart_cam_handle() {
    FD1_msg_cam_in &tmp_msg = FD1_uart_msg_cam.get_msg_cam_in();
    if (tmp_msg._msg_1.updated) {
        Ucam.handle_info((float)tmp_msg._msg_1.content.msg.cam_x_in, (float)tmp_msg._msg_1.content.msg.cam_y_in, (tmp_msg._msg_1.content.msg.cam_id==0x10));
    }
}

void Copter::FD1_uart_cam_test_send() {
    FD1_msg_cam_in &tmp_msg = FD1_uart_msg_cam.get_msg_cam_in();
    tmp_msg._msg_1.need_send = true;
    tmp_msg._msg_1.content.msg.cam_id = 0x10;
    tmp_msg._msg_1.content.msg.cam_x_in = 10;
    tmp_msg._msg_1.content.msg.cam_y_in = 15;
    tmp_msg._msg_1.content.msg.sum_check = 0;
    for (int8_t i = 0; i < tmp_msg._msg_1.length - 1; i++) {
        tmp_msg._msg_1.content.msg.sum_check += tmp_msg._msg_1.content.data[i];
    }
}
