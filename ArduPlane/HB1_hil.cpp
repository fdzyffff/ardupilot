#include "Plane.h"

void Plane::FD1_mav_init()
{
    // if (!FD1_mav.initialized()) {
    //     FD1_mav.init();
    //     gcs().send_text(MAV_SEVERITY_INFO, "FD1_MAV init");
    // } else {}
}

void Plane::FD1_mav_read()
{
    // static uint32_t last_update_ms = millis();
    // uint32_t tnow = millis();
    // static uint32_t pk0_count = 0;
    // static uint32_t pk1_count = 0;
    // static uint32_t pk2_count = 0;
    // if (!FD1_mav.initialized()) {return;}
    // mavlink_status_t *chan0_status = mavlink_get_channel_status(MAVLINK_COMM_0);
    // uint8_t saved_seq = chan0_status->current_tx_seq;
    // uint8_t saved_flags = chan0_status->flags;
    // //chan0_status->flags |= MAVLINK_STATUS_FLAG_OUT_MAVLINK1;
    // while (FD1_mav.get_port()->available()>0) {
    //     uint8_t temp = FD1_mav.get_port()->read();
    //     if (temp == 0xFE) pk0_count++;
    //     mavlink_message_t msg;
    //     mavlink_status_t status;
    //     uint8_t ret = mavlink_frame_char_buffer(&FD1_mav.mavlink.rxmsg, &FD1_mav.mavlink.status, temp, &msg, &status);
    //     if (ret == MAVLINK_FRAMING_OK || ret == MAVLINK_FRAMING_BAD_CRC) {
    //         switch (msg.msgid) {
    //             case MAVLINK_MSG_ID_HEARTBEAT: {
    //                 mavlink_heartbeat_t pkt;
    //                 mavlink_msg_heartbeat_decode(&msg, &pkt);
    //                 //gcs().send_text(MAV_SEVERITY_INFO, "type=%u autopilot=%u base_mode=0x%x\n", pkt.type, pkt.autopilot, pkt.base_mode);
    //                 break;
    //             }
    //             case MAVLINK_MSG_ID_HIL_STATE: {
    //                 //gcs().send_text(MAV_SEVERITY_INFO, "lalala");
    //                 if (plane.g.hil_mode != 1) {
    //                     break;
    //                 }
    //                 pk1_count++;

    //                 mavlink_hil_state_t packet;
    //                 mavlink_msg_hil_state_decode(&msg, &packet);
            
    //                 // set gps hil sensor
    //                 const Location loc{packet.lat, packet.lon, packet.alt/10, Location::AltFrame::ABSOLUTE};
    //                 Vector3f vel(packet.vx, packet.vy, packet.vz);
    //                 vel *= 0.01f;
    
    //                 // sanity check location
    //                 if (!check_latlng(packet.lat, packet.lon) || (packet.lat==0&&packet.lon==0)) {
    //                     // setup airspeed pressure based on 3D speed, no wind
    //                     // plane.airspeed.setHIL(sq(vel.length()) / 2.0f + 2013);
            
    //                     plane.gps.setHIL(0, AP_GPS::NO_FIX,
    //                                     tnow,
    //                                     loc, vel, 10, 0);

    //                 } else {
    //                     // setup airspeed pressure based on 3D speed, no wind
    //                     // plane.airspeed.setHIL(sq(vel.length()) / 2.0f + 2013);
            
    //                     plane.gps.setHIL(0, AP_GPS::GPS_OK_FIX_3D,
    //                                     tnow,
    //                                     loc, vel, 10, 0);
    //                 }

    //                 // rad/sec
    //                 Vector3f gyros;
    //                 gyros.x = packet.rollspeed;
    //                 gyros.y = packet.pitchspeed;
    //                 gyros.z = packet.yawspeed;

    //                 // m/s/s
    //                 Vector3f accels;
    //                 accels.x = packet.xacc * GRAVITY_MSS*0.001f;
    //                 accels.y = packet.yacc * GRAVITY_MSS*0.001f;
    //                 accels.z = packet.zacc * GRAVITY_MSS*0.001f;

    //                 Vector3f tmp_gravity = Vector3f(0.f,0.f,-GRAVITY_MSS);
    //                 Matrix3f tmp_ned_to_body;
    //                 tmp_ned_to_body.from_euler(packet.roll, packet.pitch, packet.yaw);
    //                 tmp_ned_to_body.transpose();
    //                 tmp_gravity = tmp_ned_to_body*tmp_gravity;
    //                 accels = accels+tmp_gravity;

    //                 plane.ins.set_gyro(0, gyros);
    //                 plane.ins.set_accel(0, accels);

    //                 plane.barometer.setHIL(packet.alt*0.001f);
    //                 plane.compass.setHIL(0, packet.roll, packet.pitch, packet.yaw);
    //                 plane.compass.setHIL(1, packet.roll, packet.pitch, packet.yaw);

    //                 // cope with DCM getting badly off due to HIL lag
    //                 if (plane.g.hil_err_limit > 0 &&
    //                     (fabsf(packet.roll - plane.ahrs.roll) > ToRad(plane.g.hil_err_limit) ||
    //                     fabsf(packet.pitch - plane.ahrs.pitch) > ToRad(plane.g.hil_err_limit) ||
    //                     wrap_PI(fabsf(packet.yaw - plane.ahrs.yaw)) > ToRad(plane.g.hil_err_limit))) {
    //                     plane.ahrs.reset_attitude(packet.roll, packet.pitch, packet.yaw);
    //                 }
    //                 // plane.ahrs.reset_attitude(packet.roll, packet.pitch, packet.yaw);
    //                 // if (tnow - last_update_ms > 2000) {
    //                 //     gcs().send_text(MAV_SEVERITY_INFO, "att: %0.2f, %0.2f %0.2f", packet.roll, packet.pitch, packet.yaw);
    //                 //     gcs().send_text(MAV_SEVERITY_INFO, "gyro: %0.2f, %0.2f %0.2f", gyros.x, gyros.y, gyros.z);
    //                 //     gcs().send_text(MAV_SEVERITY_INFO, "acc: %0.2f, %0.2f %0.2f", accels.x, accels.y, accels.z);
    //                 //     gcs().send_text(MAV_SEVERITY_INFO, "pos: %d, %d, %d", packet.lat, packet.lon, packet.alt/10);
    //                 //     gcs().send_text(MAV_SEVERITY_INFO, "vel: %0.2f, %0.2f %0.2f", vel.x, vel.y, vel.z);
    //                 //     last_update_ms = tnow;
    //                 // }

    //                 break;
    //             }
    //             case MAVLINK_MSG_ID_COMMAND_LONG: {
    //                 if (plane.g.hil_mode != 1) {
    //                     break;
    //                 }
    //                 pk2_count++;
    //                 mavlink_command_long_t packet;
    //                 mavlink_msg_command_long_decode(&msg, &packet); 
    //                 switch(packet.command) {
    //                     case MAV_CMD_USER_1:
    //                         plane.airspeed.setHIL(sq(packet.param1) / 2.0f + 2013);
    //                         plane.airspeed.set_HIL(packet.param1);
    //                         plane.barometer.setHIL_EAS2TAS(packet.param2);
    //                         // if (tnow - last_update_ms > 2000) {
    //                         //     gcs().send_text(MAV_SEVERITY_INFO, "arspd: %0.2f, %0.2f", packet.param1, plane.airspeed.get_airspeed());
    //                         //     last_update_ms = tnow;
    //                         // }
    //                         break;
    //                     default:
    //                         break;
    //                 }

    //                 break;
    //             }
    //             default:
    //                 break;
    //         }
    //     }
    // }
    // chan0_status->current_tx_seq = saved_seq;
    // chan0_status->flags = saved_flags;
    // if (tnow - last_update_ms > 1000) {
    //     //gcs().send_text(MAV_SEVERITY_INFO, "raw: %d, att: %d, arspd: %d", pk0_count, pk1_count, pk2_count);
    //     last_update_ms = tnow;
    //     pk0_count = 0;
    //     pk1_count = 0;
    //     pk2_count = 0;
    // }
}


void Plane::FD1_mav_send()
{
//     if (!FD1_mav.initialized()) {return;}
//     mavlink_status_t *chan0_status = mavlink_get_channel_status(MAVLINK_COMM_0);
//     uint8_t saved_seq = chan0_status->current_tx_seq;
//     uint8_t saved_flags = chan0_status->flags;
//     chan0_status->flags |= MAVLINK_STATUS_FLAG_OUT_MAVLINK1;
//     //chan0_status->current_tx_seq = FD1_mav.mavlink.seq;

//     mavlink_message_t msg;
//     uint16_t len;
//     mavlink_rc_channels_scaled_t packet;
//     packet.time_boot_ms = millis();
//     packet.chan1_scaled = 10000 * (SRV_Channels::get_output_scaled(SRV_Channel::k_aileron) / 4500.0f);
//     packet.chan2_scaled = 10000 * (SRV_Channels::get_output_scaled(SRV_Channel::k_elevator) / 4500.0f);
//     packet.chan3_scaled = 10000 * (SRV_Channels::get_output_scaled(SRV_Channel::k_throttle) / 100.0f);
//     packet.chan4_scaled = 10000 * (SRV_Channels::get_output_scaled(SRV_Channel::k_rudder) / 4500.0f);
//     packet.chan5_scaled = 0;
//     packet.chan6_scaled = 0;
//     packet.chan7_scaled = 10000 * (constrain_float(SRV_Channels::get_output_norm(SRV_Channel::k_launcher_HB1), -1.f, 1.f));
//     packet.chan8_scaled = 0;
//     packet.port = 0;
//     packet.rssi = 100;

//     len = mavlink_msg_rc_channels_scaled_encode(g.sysid_this_mav,
//                                         0,
//                                         &msg, &packet);

//     FD1_mav.get_port()->write(&msg.magic, 2);
//     FD1_mav.get_port()->write(&msg.magic+4, 4);
//     FD1_mav.get_port()->write(&msg.magic+10, len-6);

//     mavlink_heartbeat_t heartbeat = {0};
//     heartbeat.type = 1;
//     heartbeat.autopilot = 2;
//     heartbeat.base_mode = 3;
//     heartbeat.system_status = 4;
//     heartbeat.mavlink_version = 0;
//     heartbeat.custom_mode = 1;

//     /*
//      save and restore sequence number for chan0, as it is used by
//      generated encode functions
//     */
//     len = mavlink_msg_heartbeat_encode(5,
//                                        8,
//                                        &msg, &heartbeat);

// //    gcs().send_text(MAV_SEVERITY_INFO, "ck %d  %d (%d)",(uint8_t)(msg.checksum & 0xFF), (uint8_t)(msg.checksum >> 8), len);


//     // FD1_mav.get_port()->write(&msg.magic, 2);
//     // FD1_mav.get_port()->write(&msg.magic+4, 4);
//     // FD1_mav.get_port()->write(&msg.magic+10, len-6);

//     chan0_status->current_tx_seq = saved_seq;
//     chan0_status->flags = saved_flags;
}