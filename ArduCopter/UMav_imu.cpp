#include "Copter.h"

void UMav::send_raw_imu()
{
    if (!FD_uart_imu.initialized()) {return;}
    mavlink_status_t *chan0_status = mavlink_get_channel_status(MAVLINK_COMM_0);
    uint8_t saved_seq = chan0_status->current_tx_seq;
    uint8_t saved_flags = chan0_status->flags;
    chan0_status->flags |= MAVLINK_STATUS_FLAG_OUT_MAVLINK1;
    //chan0_status->current_tx_seq = FD1_mav.mavlink.seq;

    mavlink_message_t msg;
    uint16_t len;

#if AP_INERTIALSENSOR_ENABLED
    const Vector3f &accel = copter.ins.get_accel(0);
    const Vector3f &gyro = copter.ins.get_gyro(0);
    Vector3f mag;
#if AP_COMPASS_ENABLED
    if (copter.compass.get_count() >= 1) {
        mag = copter.compass.get_field(0);
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
    packet.temperature = int16_t(copter.ins.get_temperature(0)*100);

    len = mavlink_msg_raw_imu_encode(copter.g.sysid_this_mav,
                                        0,
                                        &msg, &packet);
    FD_uart_imu.get_port()->write(&msg.magic, 2);
    FD_uart_imu.get_port()->write(&msg.magic+4, 4);
    FD_uart_imu.get_port()->write(&msg.magic+10, len-6);

    chan0_status->current_tx_seq = saved_seq;
    chan0_status->flags = saved_flags;
#endif
}
