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
    if (g2.user_parameters.usr_hil_compass != 0) {
        compass.setHIL(0, ahrs.roll, ahrs.pitch, FD1_hil.yaw_rad);
        compass.setHIL(1, ahrs.roll, ahrs.pitch, FD1_hil.yaw_rad);
        compass.setHIL(2, ahrs.roll, ahrs.pitch, FD1_hil.yaw_rad);
    }

    if (!FD1_hil.healthy) {
        if (control_mode == Mode::Number::MYVEL || control_mode == Mode::Number::MYATT) {
            set_mode(Mode::Number::ALT_HOLD, ModeReason::TOY_MODE);
            // FD1_hil.ctrl_mode = 1;
            gcs().send_text(MAV_SEVERITY_INFO, "#FS TIMEOUT");
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
    FD1_msg_hil_out &tmp_msg = FD1_uart_msg_hil.get_msg_hil_out();
    if (g2.user_parameters.usr_print != 0) {
        gcs().send_text(MAV_SEVERITY_INFO, "gyro (%d, %d, %d)",tmp_msg._msg_1.content.msg.gyrox, tmp_msg._msg_1.content.msg.gyroy, tmp_msg._msg_1.content.msg.gyroz);
        gcs().send_text(MAV_SEVERITY_INFO, "acc (%d, %d, %d)",tmp_msg._msg_1.content.msg.accx, tmp_msg._msg_1.content.msg.accy, tmp_msg._msg_1.content.msg.accz);
        gcs().send_text(MAV_SEVERITY_INFO, "ctrl_mode %d (%d)",FD1_hil.ctrl_mode, FD1_hil.healthy);
        gcs().send_text(MAV_SEVERITY_INFO, "ctrl_rpy %0.0f, %0.0f, %0.0f",FD1_hil.ctrl_roll_cd , FD1_hil.ctrl_pitch_cd, FD1_hil.ctrl_yaw_rate_cd);
        gcs().send_text(MAV_SEVERITY_INFO, "ctrl_vel %0.0f, %0.0f, %0.0f",FD1_hil.ctrl_vel_x_cms , FD1_hil.ctrl_vel_y_cms, FD1_hil.ctrl_vel_z_cms);
        gcs().send_text(MAV_SEVERITY_INFO, "info %0.2f (%0.1f, %0.1f)",FD1_hil.yaw_rad, FD1_hil.vel_x_cms, FD1_hil.vel_y_cms);
    }
}
#endif

#ifdef USERHOOK_AUXSWITCH
void Copter::userhook_auxSwitch1(uint8_t ch_flag)
{
    // put your aux switch #1 handler here (CHx_OPT = 47)
    if (ch_flag == 0) {FD1_hil.ctrl_mode = 1;}
    if (ch_flag == 1) {FD1_hil.ctrl_mode = 2;}
    if (ch_flag == 2) {FD1_hil.ctrl_mode = 3;}
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

    FD1_hil.ctrl_mode = 1;
    FD1_hil.scene_mode = 1;
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
    static int16_t last_control_mode = 0;
    FD1_msg_hil_in &tmp_msg = FD1_uart_msg_hil.get_msg_hil_in();
    if (tmp_msg._msg_1.updated) {
        FD1_hil.last_update_ms = millis();
        if (last_control_mode != FD1_hil.ctrl_mode) {
            if (FD1_hil.ctrl_mode == tmp_msg._msg_1.content.msg.ctrl_mode
                || FD1_hil.ctrl_mode == 1) {
                // if (FD1_hil.ctrl_mode == 1 || FD1_hil.ctrl_mode == 2) {
                //     set_mode(Mode::Number::MYVEL, ModeReason::TOY_MODE);
                // }
                if (FD1_hil.ctrl_mode == 1) {
                    set_mode(Mode::Number::ALT_HOLD, ModeReason::TOY_MODE);
                    gcs().send_text(MAV_SEVERITY_INFO, "#MODE %d",FD1_hil.ctrl_mode);
                    last_control_mode = FD1_hil.ctrl_mode;
                }
                if (FD1_hil.ctrl_mode == 2 || FD1_hil.ctrl_mode == 3) {
                    set_mode(Mode::Number::MYATT, ModeReason::TOY_MODE);
                    gcs().send_text(MAV_SEVERITY_INFO, "#MODE %d",FD1_hil.ctrl_mode);
                    last_control_mode = FD1_hil.ctrl_mode;
                }
            }
        }
        FD1_hil.scene_mode = tmp_msg._msg_1.content.msg.scene_mode;
        FD1_get_ctrl_in(tmp_msg._msg_1.content.msg.ctrl_mode,
                            tmp_msg._msg_1.content.msg.scene_mode,
                            constrain_float((float)tmp_msg._msg_1.content.msg.ctrl_roll_cd, -4500.f, 4500.f),
                            constrain_float((float)tmp_msg._msg_1.content.msg.ctrl_pitch_cd, -4500.f, 4500.f),
                            (float)tmp_msg._msg_1.content.msg.ctrl_yaw_cd,
                            degrees((float)tmp_msg._msg_1.content.msg.ctrl_yaw_rate_crads),
                            (float)tmp_msg._msg_1.content.msg.ctrl_z_vel_cms);
        FD1_hil.ctrl_yaw_cd = (float)tmp_msg._msg_1.content.msg.ctrl_yaw_cd;
        FD1_hil.ctrl_yaw_rate_cd = degrees((float)tmp_msg._msg_1.content.msg.ctrl_yaw_rate_crads);
        FD1_hil.ctrl_vel_x_cms = (float)tmp_msg._msg_1.content.msg.ctrl_x_vel_cms;
        FD1_hil.ctrl_vel_y_cms = (float)tmp_msg._msg_1.content.msg.ctrl_y_vel_cms;

        FD1_hil.vel_x_cms = (float)tmp_msg._msg_1.content.msg.vel_x_cms;
        FD1_hil.vel_y_cms = (float)tmp_msg._msg_1.content.msg.vel_y_cms;
        Vector3f temp_vel;
        if (ahrs.get_velocity_NED(temp_vel) && g2.user_parameters.usr_hil_vel.get() == 0) {
            FD1_hil.vel_x_cms = 100.f*(float)temp_vel.x;
            FD1_hil.vel_y_cms = 100.f*(float)temp_vel.y;
        }
        tmp_msg._msg_1.updated = false;
    }
}

void Copter::FD1_uart_hil_send() {
    FD1_msg_hil_out &tmp_msg = FD1_uart_msg_hil.get_msg_hil_out();
    Vector3f tmp_gyro = ahrs.get_gyro();
    Matrix3f M_ned_to_body = Matrix3f(ahrs.get_rotation_body_to_ned()).transposed();
    Vector3f tmp_acc = M_ned_to_body * ahrs.get_accel_ef();
    tmp_msg._msg_1.need_send = true;
    tmp_msg._msg_1.content.msg.sum_check = 0;
    tmp_msg._msg_1.content.msg.header.head_1 = FD1_msg_hil_out::PREAMBLE1;
    tmp_msg._msg_1.content.msg.header.head_2 = FD1_msg_hil_out::PREAMBLE2;
    tmp_msg._msg_1.content.msg.ctrl_mode = FD1_hil.ctrl_mode;
    tmp_msg._msg_1.content.msg.gyrox = (int16_t)constrain_float(tmp_gyro.x*100.f, -30000.f, 30000.f);
    tmp_msg._msg_1.content.msg.gyroy = (int16_t)constrain_float(tmp_gyro.y*100.f, -30000.f, 30000.f);
    tmp_msg._msg_1.content.msg.gyroz = (int16_t)constrain_float(tmp_gyro.z*100.f, -30000.f, 30000.f);
    tmp_msg._msg_1.content.msg.accx = (int16_t)constrain_float(tmp_acc.x*100.f, -30000.f, 30000.f);
    tmp_msg._msg_1.content.msg.accy = (int16_t)constrain_float(tmp_acc.y*100.f, -30000.f, 30000.f);
    tmp_msg._msg_1.content.msg.accz = (int16_t)constrain_float(tmp_acc.z*100.f, -30000.f, 30000.f);
    tmp_msg._msg_1.content.msg.eulerx = ahrs.roll_sensor;
    tmp_msg._msg_1.content.msg.eulery = ahrs.pitch_sensor;
    tmp_msg._msg_1.content.msg.eulerz = ahrs.yaw_sensor;
    tmp_msg._msg_1.content.msg.height_rel_home = 0;
    tmp_msg._msg_1.content.msg.velz = 0;

    for (int8_t i = 0; i < tmp_msg._msg_1.length - 2; i++) {
        tmp_msg._msg_1.content.msg.sum_check += tmp_msg._msg_1.content.data[i];
    }
    tmp_msg._msg_1.content.msg.end = FD1_msg_hil_out::POSTAMBLE;
}

void Copter::FD1_uart_hil_test_send() {
    FD1_msg_hil_in &tmp_msg = FD1_uart_msg_hil.get_msg_hil_test();
    tmp_msg._msg_1.need_send = true;
    tmp_msg._msg_1.content.msg.sum_check = 0;
    tmp_msg._msg_1.content.msg.header.head_1 = FD1_msg_hil_in::PREAMBLE1;
    tmp_msg._msg_1.content.msg.header.head_2 = FD1_msg_hil_in::PREAMBLE2;
    tmp_msg._msg_1.content.msg.scene_mode = 1;
    tmp_msg._msg_1.content.msg.ctrl_mode = 2;
    tmp_msg._msg_1.content.msg.ctrl_pitch_cd = ahrs.pitch_sensor;
    tmp_msg._msg_1.content.msg.ctrl_roll_cd = ahrs.roll_sensor;
    tmp_msg._msg_1.content.msg.ctrl_x_vel_cms = -(float)ahrs.pitch_sensor/4.5f;
    tmp_msg._msg_1.content.msg.ctrl_y_vel_cms = (float)ahrs.roll_sensor/4.5f;
    tmp_msg._msg_1.content.msg.ctrl_z_vel_cms = 10;
    tmp_msg._msg_1.content.msg.ctrl_yaw_cd = ahrs.yaw_sensor;
    tmp_msg._msg_1.content.msg.ctrl_yaw_rate_crads = (int16_t)constrain_float(ahrs.get_yaw_rate_earth()*100.f, -30000.f, 30000.f);
    tmp_msg._msg_1.content.msg.angle_yaw_cd = wrap_180_cd(ahrs.yaw_sensor);
    tmp_msg._msg_1.content.msg.vel_x_cms = 0;
    tmp_msg._msg_1.content.msg.vel_y_cms = 0;
    for (int8_t i = 0; i < tmp_msg._msg_1.length - 2; i++) {
        tmp_msg._msg_1.content.msg.sum_check += tmp_msg._msg_1.content.data[i];
    }
    tmp_msg._msg_1.content.msg.end = FD1_msg_hil_in::POSTAMBLE;
}

void Copter::FD1_get_ctrl_in(int16_t ctrl_mode, int16_t scene_mode, float ctrl_roll_cd, float ctrl_pitch_cd, float ctrl_yaw_cd, float ctrl_yaw_rate_crads, float ctrl_vel_z_cms) {
    // get pilot desired lean angles
    //float pilot_roll = channel_roll->get_control_in();
    float pilot_pitch = channel_pitch->get_control_in();
    float pilot_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());
    float pilot_climb_rate = get_pilot_desired_climb_rate(channel_throttle->get_control_in());

    if (ctrl_mode==1) {
        if (scene_mode == 1) {
            FD1_hil.ctrl_roll_cd = ctrl_roll_cd;
            FD1_hil.ctrl_pitch_cd = pilot_pitch;
            FD1_hil.ctrl_yaw_rate_cd = pilot_yaw_rate;
            FD1_hil.ctrl_vel_z_cms = pilot_climb_rate;
        } else {
            FD1_hil.ctrl_roll_cd = ctrl_roll_cd;
            FD1_hil.ctrl_pitch_cd = pilot_pitch;
            FD1_hil.ctrl_yaw_rate_cd = ctrl_yaw_rate_crads;
            FD1_hil.ctrl_vel_z_cms = ctrl_vel_z_cms;
        }
    } else if (ctrl_mode==2) {
        if (scene_mode == 1) {
            FD1_hil.ctrl_roll_cd = ctrl_roll_cd;
            FD1_hil.ctrl_pitch_cd = ctrl_pitch_cd;
            FD1_hil.ctrl_yaw_rate_cd = ctrl_yaw_rate_crads;
            FD1_hil.ctrl_vel_z_cms = pilot_climb_rate;
        } else {
            FD1_hil.ctrl_roll_cd = ctrl_roll_cd;
            FD1_hil.ctrl_pitch_cd = ctrl_pitch_cd;
            FD1_hil.ctrl_yaw_rate_cd = ctrl_yaw_rate_crads;
            FD1_hil.ctrl_vel_z_cms = ctrl_vel_z_cms;
        }
    }
}
