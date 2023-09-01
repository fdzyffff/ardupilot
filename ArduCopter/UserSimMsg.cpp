#include "Copter.h"

UserSimMsg::UserSimMsg(enum AP_SerialManager::SerialProtocol protocol):
    _protocol(protocol)
{
    _port = NULL;
    _initialized = false;
}

void UserSimMsg::init()
{
    const AP_SerialManager &serial_manager = AP::serialmanager();

    // check for protocol configured for a serial port - only the first serial port with one of these protocols will then run (cannot have FrSky on multiple serial ports)
    if ((_port = serial_manager.find_serial(_protocol, 0))) {
        _initialized = true;
    }
}

void UserSimMsg::handle_msg(const mavlink_message_t &msg)
{
    if (!_initialized) {return;}
        // gcs().send_text(MAV_SEVERITY_INFO, "%d -> %d" , msg.sysid, msg.msgid);
    ;
}

void UserSimMsg::update()
{
    if (!_initialized) {return;}
    read_uart();
    send_mav();
}

void UserSimMsg::read_uart() 
{
    static uint32_t last_update_ms = millis();
    uint32_t tnow = millis();
    // static uint32_t pk0_count = 0;
    // static uint32_t pk1_count = 0;
    // static uint32_t pk2_count = 0;
    mavlink_status_t *chan0_status = mavlink_get_channel_status(MAVLINK_COMM_0);
    uint8_t saved_seq = chan0_status->current_tx_seq;
    uint8_t saved_flags = chan0_status->flags;
    //chan0_status->flags |= MAVLINK_STATUS_FLAG_OUT_MAVLINK1;
    while (_port->available()>0) {
        uint8_t temp = _port->read();
        // if (temp == 0xFE) pk0_count++;
        mavlink_message_t msg;
        mavlink_status_t status;
        uint8_t ret = mavlink_frame_char_buffer(&mavlink.rxmsg, &mavlink.status, temp, &msg, &status);
        if (ret == MAVLINK_FRAMING_OK) {
            switch (msg.msgid) {
                case MAVLINK_MSG_ID_HEARTBEAT: {
                    ;
                    break;
                }
                default:
                    break;
            }
        }
    }
    chan0_status->current_tx_seq = saved_seq;
    chan0_status->flags = saved_flags;
    if (tnow - last_update_ms > 1000) {
        //gcs().send_text(MAV_SEVERITY_INFO, "raw: %d, att: %d, arspd: %d", pk0_count, pk1_count, pk2_count);
        last_update_ms = tnow;
        // pk0_count = 0;
        // pk1_count = 0;
        // pk2_count = 0;
    }
}

void UserSimMsg::send_mav()
{
    mavlink_status_t *chan0_status = mavlink_get_channel_status(MAVLINK_COMM_0);
    uint8_t saved_seq = chan0_status->current_tx_seq;
    uint8_t saved_flags = chan0_status->flags;
    // chan0_status->flags |= MAVLINK_STATUS_FLAG_OUT_MAVLINK1;
    //chan0_status->current_tx_seq = FD1_mav.mavlink.seq;

    mavlink_message_t msg;
    // uint16_t len;

    // msg attitude
    mavlink_attitude_t attitude_packet;
    const AP_AHRS &ahrs = AP::ahrs();
    const Vector3f omega = ahrs.get_gyro();
    attitude_packet.time_boot_ms = millis();
    attitude_packet.roll = ahrs.roll;
    attitude_packet.pitch = ahrs.roll;
    attitude_packet.yaw = ahrs.pitch;
    attitude_packet.rollspeed = ahrs.yaw;
    attitude_packet.pitchspeed = omega.x;
    attitude_packet.pitchspeed = omega.y;
    attitude_packet.yawspeed = omega.z;

    UNUSED_RESULT(mavlink_msg_attitude_encode(copter.g.sysid_this_mav,
                                        0,
                                        &msg, &attitude_packet));

    send_msg(&msg);

    // msg position
    mavlink_global_position_int_t global_position_int_packet;
    Location global_position_current_loc;
    UNUSED_RESULT(ahrs.get_location(global_position_current_loc)); // return value ignored; we send stale data

    Vector3f vel;
    if (!ahrs.get_velocity_NED(vel)) {
        vel.zero();
    }

    float posD;
    ahrs.get_relative_position_D_home(posD);
    posD *= -1000.0f; // change from down to up and metres to millimeters

    global_position_int_packet.time_boot_ms = millis();
    global_position_int_packet.lat          = global_position_current_loc.lat;                 // in 1E7 degrees
    global_position_int_packet.lon          = global_position_current_loc.lng;                 // in 1E7 degrees
    global_position_int_packet.alt          = global_position_current_loc.alt * 10UL;          // millimeters above ground/sea level
    global_position_int_packet.relative_alt = posD;                                            // millimeters above home
    global_position_int_packet.vx           = vel.x * 100;                                     // X speed cm/s (+ve North)
    global_position_int_packet.vy           = vel.y * 100;                                     // Y speed cm/s (+ve East)
    global_position_int_packet.vz           = vel.z * 100;                                     // Z speed cm/s (+ve Down)
    global_position_int_packet.hdg          = ahrs.yaw_sensor;                                 // compass heading in 1/100 degree

    UNUSED_RESULT(mavlink_msg_global_position_int_encode(copter.g.sysid_this_mav,
                                        0,
                                        &msg, &global_position_int_packet));
    send_msg(&msg);

    chan0_status->current_tx_seq = saved_seq;
    chan0_status->flags = saved_flags;
}

void UserSimMsg::send_msg(mavlink_message_t *msg)
{
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
            _port->write(buf, header_len);
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
            _port->write(buf, header_len);
        }

    _port->write((uint8_t *)_MAV_PAYLOAD(msg), msg->len);
    _port->write((uint8_t *)ck, 2);
}
