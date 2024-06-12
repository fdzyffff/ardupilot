#include "Copter.h"

#ifdef USERHOOK_INIT
void Copter::userhook_init()
{
    // put your initialisation code here
    // this will be called once at start-up

    FD_uart_msg.init();
}
#endif

void Copter::userhook_SuperLoop()
{
#ifdef USERHOOK_FASTLOOP
    FD_send_raw_imu();
    FD_handle_message();
#endif
}

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
void Copter::userhook_auxSwitch1(const RC_Channel::AuxSwitchPos ch_flag)
{
    // put your aux switch #1 handler here (CHx_OPT = 47)
}

void Copter::userhook_auxSwitch2(const RC_Channel::AuxSwitchPos ch_flag)
{
    // put your aux switch #2 handler here (CHx_OPT = 48)
}

void Copter::userhook_auxSwitch3(const RC_Channel::AuxSwitchPos ch_flag)
{
    // put your aux switch #3 handler here (CHx_OPT = 49)
}
#endif

void Copter::FD_send_raw_imu()
{
    if (!FD_uart_msg.initialized()) {return;}
    mavlink_status_t *chan0_status = mavlink_get_channel_status(MAVLINK_COMM_0);
    uint8_t saved_seq = chan0_status->current_tx_seq;
    uint8_t saved_flags = chan0_status->flags;
    chan0_status->flags |= MAVLINK_STATUS_FLAG_OUT_MAVLINK1;
    //chan0_status->current_tx_seq = FD1_mav.mavlink.seq;

    mavlink_message_t msg;
    uint16_t len;

#if AP_INERTIALSENSOR_ENABLED
    const Vector3f &accel = ins.get_accel(0);
    const Vector3f &gyro = ins.get_gyro(0);
    Vector3f mag;
#if AP_COMPASS_ENABLED
    if (compass.get_count() >= 1) {
        mag = compass.get_field(0);
    }
#endif
    mavlink_raw_imu_t packet;
    packet.time_usec = AP_HAL::micros64();
    packet.xacc = accel.x * 1000.0f / GRAVITY_MSS;
    packet.yacc = accel.y * 1000.0f / GRAVITY_MSS;
    packet.zacc = accel.z * 1000.0f / GRAVITY_MSS;
    packet.xgyro = gyro.x * 1000.0f;
    packet.ygyro = gyro.y * 1000.0f;
    packet.zgyro = gyro.z * 1000.0f;
    packet.xmag = mag.x;
    packet.ymag = mag.y;
    packet.zmag = mag.z;
    packet.id = 0;  // we use SCALED_IMU and SCALED_IMU2 for other IMUs;
    packet.temperature = int16_t(ins.get_temperature(0)*100);

    len = mavlink_msg_raw_imu_encode(copter.g.sysid_this_mav,
                                        0,
                                        &msg, &packet);
    FD_uart_msg.get_port()->write(&msg.magic, 2);
    FD_uart_msg.get_port()->write(&msg.magic+4, 4);
    FD_uart_msg.get_port()->write(&msg.magic+10, len-6);

    chan0_status->current_tx_seq = saved_seq;
    chan0_status->flags = saved_flags;
#endif
}

void Copter::FD_handle_message() 
{
    if (!FD_uart_msg.initialized()) {return;}

    // static uint32_t last_update_ms = millis();
    // uint32_t tnow = millis();
    // static uint32_t pk0_count = 0;
    // static uint32_t pk1_count = 0;
    // static uint32_t pk2_count = 0;
    while (FD_uart_msg.get_port()->available()>0) {
        uint8_t temp = FD_uart_msg.get_port()->read();
        // if (temp == 0xFE) pk0_count++;
        mavlink_message_t msg;
        mavlink_status_t status;
        uint8_t ret = mavlink_frame_char_buffer(&mavlink.rxmsg, &mavlink.status, temp, &msg, &status);

        if (ret == MAVLINK_FRAMING_OK) {
            gcs().send_text(MAV_SEVERITY_INFO, "MSG");
            for (uint8_t i=0; i<gcs().num_gcs(); i++) {
                gcs().chan(i)->handle_message_fd(msg);
                break;
            }
        }
    }
}
