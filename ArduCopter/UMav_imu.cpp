#include "Copter.h"

void UMav::send_raw_imu_loop() {
    // hal.scheduler->delay(3000);
    gcs().send_text(MAV_SEVERITY_INFO, "LOOP IMURAW Start");
    while (true) {
        send_raw_imu();
    }
}

void UMav::send_raw_imu()
{
    if (!FD_uart_imu.initialized()) {
        hal.scheduler->delay(10000);
        return;
    }
    static uint32_t _last_log_ms = millis();
    static int16_t count = 0;

    float dt = (float)(millis() - _last_log_ms)*0.001f;
    if (dt > 1.0f) {
        // gcs().send_text(MAV_SEVERITY_INFO, "LOOP IMURAW %d", count);
        _last_log_ms = millis();
        float imu_rate = ((float)count)/dt;
        count = 0;
        AP::logger().WriteStreaming("UIMU",
                                    "TimeUS,rate",
                                    "s-",
                                    "F-",
                                    "Qf",
                                    AP_HAL::micros64(),
                                    (float)imu_rate);
    }


    mavlink_status_t *chan0_status = mavlink_get_channel_status(MAVLINK_COMM_0);
    uint8_t saved_seq = chan0_status->current_tx_seq;
    uint8_t saved_flags = chan0_status->flags;
    // chan0_status->flags |= MAVLINK_STATUS_FLAG_OUT_MAVLINK1;
    //chan0_status->current_tx_seq = FD1_mav.mavlink.seq;

    mavlink_message_t msg;
    uint16_t len;

#if AP_INERTIALSENSOR_ENABLED
    const Vector3f &accel = copter.ins.get_accel(0);
    const Vector3f &gyro = copter.ins.get_gyro(0);
    _imu_acc.apply(accel);
    _imu_gyro.apply(gyro);
    Vector3f mag;
#if AP_COMPASS_ENABLED
    if (copter.compass.get_count() >= 1) {
        mag = copter.compass.get_field(0);
    }
#endif
    mavlink_raw_imu_t packet;
    packet.time_usec = AP_HAL::micros64();
    packet.xacc = _imu_acc.get().x * 1000.0f / GRAVITY_MSS;
    packet.yacc = _imu_acc.get().y * 1000.0f / GRAVITY_MSS;
    packet.zacc = _imu_acc.get().z * 1000.0f / GRAVITY_MSS;
    packet.xgyro = _imu_gyro.get().x * 1000.0f;
    packet.ygyro = _imu_gyro.get().y * 1000.0f;
    packet.zgyro = _imu_gyro.get().z * 1000.0f;
    packet.xmag = mag.x;
    packet.ymag = mag.y;
    packet.zmag = mag.z;
    packet.id = 0;  // we use SCALED_IMU and SCALED_IMU2 for other IMUs;
    packet.temperature = int16_t(copter.ins.get_temperature(0)*100);

    len = mavlink_msg_raw_imu_encode(copter.g.sysid_this_mav,
                                        0,
                                        &msg, &packet);
    // FD_uart_imu.get_port()->write(&msg.magic, 2);
    // FD_uart_imu.get_port()->write(&msg.magic+4, 4);
    // FD_uart_imu.get_port()->write(&msg.magic+10, len-6);

    if (len > 0) {
        send_mav_message(FD_uart_imu.get_port(), &msg);
        count++;
    }
    
    chan0_status->current_tx_seq = saved_seq;
    chan0_status->flags = saved_flags;
#endif
    hal.scheduler->delay_microseconds(5000);
}

void UMav::send_mav_message(AP_HAL::UARTDriver* port, mavlink_message_t *msg)
{
    if (port == nullptr) {return;}

    uint8_t ck[2];

    ck[0] = (uint8_t)(msg->checksum & 0xFF);
    ck[1] = (uint8_t)(msg->checksum >> 8);
    // XXX use the right sequence here

    uint8_t header_len;
    // uint8_t signature_len;
    
    if (msg->magic == MAVLINK_STX_MAVLINK1) {
        header_len = MAVLINK_CORE_HEADER_MAVLINK1_LEN + 1;
        // signature_len = 0;
        // we can't send the structure directly as it has extra mavlink2 elements in it
        uint8_t buf[MAVLINK_CORE_HEADER_MAVLINK1_LEN + 1];
        buf[0] = msg->magic;
        buf[1] = msg->len;
        buf[2] = msg->seq;
        buf[3] = msg->sysid;
        buf[4] = msg->compid;
        buf[5] = msg->msgid & 0xFF;
        port->write(buf, header_len);
    } else {
        header_len = MAVLINK_CORE_HEADER_LEN + 1;
        // signature_len = (msg->incompat_flags & MAVLINK_IFLAG_SIGNED)?MAVLINK_SIGNATURE_BLOCK_LEN:0;
        uint8_t buf[MAVLINK_CORE_HEADER_LEN + 1];
        buf[0] = msg->magic;
        buf[1] = msg->len;
        buf[2] = msg->incompat_flags;
        buf[3] = msg->compat_flags;
        buf[4] = msg->seq;
        buf[5] = msg->sysid;
        buf[6] = msg->compid;
        buf[7] = msg->msgid & 0xFF;
        buf[8] = (msg->msgid >> 8) & 0xFF;
        buf[9] = (msg->msgid >> 16) & 0xFF;
        port->write(buf, header_len);
    }

    port->write((uint8_t *)_MAV_PAYLOAD(msg), msg->len);
    port->write((uint8_t *)ck, 2);
}
