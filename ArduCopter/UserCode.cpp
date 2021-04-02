#include "Copter.h"

#ifdef USERHOOK_INIT
void Copter::userhook_init()
{
    // put your initialisation code here
    // this will be called once at start-up
}
#endif

#ifdef USERHOOK_FASTLOOP
void Copter::userhook_FastLoop()
{
    FD1_uart_update();
    // put your 100Hz code here
    if (g2.user_parameters.usr_hil_mode != 0) {
        compass.setHIL(0, ahrs.roll, ahrs.pitch, FD1_hil.yaw_rad);
        compass.setHIL(1, ahrs.roll, ahrs.pitch, FD1_hil.yaw_rad);
        compass.setHIL(2, ahrs.roll, ahrs.pitch, FD1_hil.yaw_rad);
    }
    if (!FD1_hil.healthy) {
        if (control_mode == Mode::Number::MYVEL || control_mode == Mode::Number::MYATT) {
            set_mode(Mode::Number::STABILIZE, ModeReason::TOY_MODE);
            gcs().send_text(MAV_SEVERITY_INFO, "#FS land");
        }
    }
}
#endif

#ifdef USERHOOK_50HZLOOP
void Copter::userhook_50Hz()
{
    // put your 50Hz code here
}
#endif

#ifdef USERHOOK_MEDIUMLOOP
void Copter::userhook_MediumLoop()
{
    // put your 10Hz code here
}
#endif

#ifdef USERHOOK_SLOWLOOP
void Copter::userhook_SlowLoop()
{
    // put your 3.3Hz code here
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


void Copter::FD1_uart_init() {
    FD1_uart_msg_hil.init();
    FD1_uart_msg_hil.get_msg_hil_in().set_enable();

    FD1_hil.healthy = false;
    FD1_hil.last_update_ms = 0;
    FD1_hil.yaw_rad = 0;
    FD1_hil.vel_x_cms = 0;
    FD1_hil.vel_y_cms = 0;
    FD1_hil.ctrl_vel_x_cms = 0.0f;
    FD1_hil.ctrl_vel_y_cms = 0.0f;
    FD1_hil.ctrl_vel_z_cms = 0.0f;
    FD1_hil.ctrl_roll_cd = 0.0f;
    FD1_hil.ctrl_pitch_cd = 0.0f;
    FD1_hil.ctrl_yaw_cd = 0.0f;
    FD1_hil.ctrl_yaw_rate_cd = 0.0f;
}

void Copter::FD1_uart_update() {
    if (FD1_uart_msg_hil.initialized()) {
        FD1_uart_msg_hil.read();
    }
    // test mode
    if (g2.user_parameters.usr_hil_test != 0) {
        FD1_uart_hil_test_send();
    } else {
        FD1_uart_hil_handle();
        FD1_uart_hil_send();
    }
    FD1_uart_msg_hil.write();

    FD1_hil.healthy = (millis() - FD1_hil.last_update_ms < 1000);
}

void Copter::FD1_uart_hil_handle() {
    static uint8_t last_control_mode = 0;
    FD1_msg_hil_in &tmp_msg = FD1_uart_msg_hil.get_msg_hil_in();
    if (tmp_msg._msg_1.updated) {
        FD1_hil.last_update_ms = millis();
        if (last_control_mode != tmp_msg._msg_1.content.msg.ctrl_mode) {
            FD1_hil.ctrl_mode = tmp_msg._msg_1.content.msg.ctrl_mode;
            if (FD1_hil.ctrl_mode == 1 || FD1_hil.ctrl_mode == 2) {
                set_mode(Mode::Number::MYVEL, ModeReason::TOY_MODE);
            }
            if (FD1_hil.ctrl_mode == 3 || FD1_hil.ctrl_mode == 4) {
                set_mode(Mode::Number::MYATT, ModeReason::TOY_MODE);
            }
        }
        switch (FD1_hil.ctrl_mode) {
            case 1:
                FD1_hil.ctrl_roll_cd = constrain_float((float)tmp_msg._msg_1.content.msg.ctrl_1, -4500.f, 4500.f);
                FD1_hil.ctrl_pitch_cd = constrain_float((float)tmp_msg._msg_1.content.msg.ctrl_2, -4500.f, 4500.f);
                FD1_hil.ctrl_vel_z_cms = (float)tmp_msg._msg_1.content.msg.ctrl_3;
                FD1_hil.ctrl_yaw_cd = (float)tmp_msg._msg_1.content.msg.ctrl_4;
                break;
            case 2:
                FD1_hil.ctrl_roll_cd = constrain_float((float)tmp_msg._msg_1.content.msg.ctrl_1, -4500.f, 4500.f);
                FD1_hil.ctrl_pitch_cd = constrain_float((float)tmp_msg._msg_1.content.msg.ctrl_2, -4500.f, 4500.f);
                FD1_hil.ctrl_vel_z_cms = (float)tmp_msg._msg_1.content.msg.ctrl_3;
                FD1_hil.ctrl_yaw_rate_cd = (float)tmp_msg._msg_1.content.msg.ctrl_4;
                break;
            case 3:
                FD1_hil.ctrl_vel_y_cms = (float)tmp_msg._msg_1.content.msg.ctrl_1;
                FD1_hil.ctrl_vel_x_cms = (float)tmp_msg._msg_1.content.msg.ctrl_2;
                FD1_hil.ctrl_vel_z_cms = (float)tmp_msg._msg_1.content.msg.ctrl_3;
                FD1_hil.ctrl_yaw_cd = (float)tmp_msg._msg_1.content.msg.ctrl_4;
                break;
            case 4:
                FD1_hil.ctrl_vel_y_cms = (float)tmp_msg._msg_1.content.msg.ctrl_1;
                FD1_hil.ctrl_vel_x_cms = (float)tmp_msg._msg_1.content.msg.ctrl_2;
                FD1_hil.ctrl_vel_z_cms = (float)tmp_msg._msg_1.content.msg.ctrl_3;
                FD1_hil.ctrl_yaw_rate_cd = (float)tmp_msg._msg_1.content.msg.ctrl_4;
                break;
            default:
                break;
        }
        FD1_hil.yaw_rad = radians(0.01f*(float)tmp_msg._msg_1.content.msg.angle_yaw_cd);
        FD1_hil.vel_x_cms = (float)tmp_msg._msg_1.content.msg.vel_x_cms;
        FD1_hil.vel_y_cms = (float)tmp_msg._msg_1.content.msg.vel_y_cms;
        tmp_msg._msg_1.updated = false;
    }
}

void Copter::FD1_uart_hil_send() {
    FD1_msg_hil_out &tmp_msg = FD1_uart_msg_hil.get_msg_hil_out();
    tmp_msg._msg_1.need_send = true;
    tmp_msg._msg_1.content.msg.header.head_1 = FD1_msg_hil_out::PREAMBLE1;
    tmp_msg._msg_1.content.msg.header.head_2 = FD1_msg_hil_out::PREAMBLE2;
    tmp_msg._msg_1.content.msg.ctrl_mode = FD1_hil.ctrl_mode;
    tmp_msg._msg_1.content.msg.gyrox = 0;
    tmp_msg._msg_1.content.msg.gyroy = 0;
    tmp_msg._msg_1.content.msg.gyroz = 0;
    tmp_msg._msg_1.content.msg.accx = 0;
    tmp_msg._msg_1.content.msg.accy = 0;
    tmp_msg._msg_1.content.msg.accz = 0;
    tmp_msg._msg_1.content.msg.eulerx = ahrs.roll_sensor;
    tmp_msg._msg_1.content.msg.eulery = ahrs.pitch_sensor;
    tmp_msg._msg_1.content.msg.eulerz = ahrs.yaw_sensor;
    tmp_msg._msg_1.content.msg.height_rel_home = 0;
    tmp_msg._msg_1.content.msg.velz = 0;
    for (int8_t i = 0; i < tmp_msg._msg_1.length - 1; i++) {
        tmp_msg._msg_1.content.msg.sum_check += tmp_msg._msg_1.content.data[i];
    }
    tmp_msg._msg_1.content.msg.end = FD1_msg_hil_out::POSTAMBLE;
}


void Copter::FD1_uart_hil_test_send() {
    FD1_msg_hil_in &tmp_msg = FD1_uart_msg_hil.get_msg_hil_test();
    tmp_msg._msg_1.need_send = true;
    tmp_msg._msg_1.content.msg.header.head_1 = FD1_msg_hil_in::PREAMBLE1;
    tmp_msg._msg_1.content.msg.header.head_2 = FD1_msg_hil_in::PREAMBLE2;
    tmp_msg._msg_1.content.msg.ctrl_mode = 1;
    tmp_msg._msg_1.content.msg.ctrl_1 = ahrs.roll_sensor;
    tmp_msg._msg_1.content.msg.ctrl_2 = ahrs.pitch_sensor;
    tmp_msg._msg_1.content.msg.ctrl_3 = 0;
    tmp_msg._msg_1.content.msg.ctrl_4 = ahrs.yaw_sensor;
    tmp_msg._msg_1.content.msg.angle_yaw_cd = wrap_180_cd(ahrs.yaw_sensor);
    tmp_msg._msg_1.content.msg.vel_x_cms = 0;
    tmp_msg._msg_1.content.msg.vel_y_cms = 0;
    for (int8_t i = 0; i < tmp_msg._msg_1.length - 1; i++) {
        tmp_msg._msg_1.content.msg.sum_check += tmp_msg._msg_1.content.data[i];
    }
    tmp_msg._msg_1.content.msg.end = FD1_msg_hil_in::POSTAMBLE;
}
