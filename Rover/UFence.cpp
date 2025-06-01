/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.
   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/


#include "Rover.h"

UFence::UFence()
{
    ;
}

// initialise
void UFence::init()
{
    // 加速度估计器变量定义
    detect = 0.0f;
    tgt_pose = Vector2f(0.0f, 0.0f);
    tgt_accel_est = Vector2f(0.0f, 0.0f);
    tgt_last_ms = 0;
    // 分布式运动观测器参数设置
    cp = 0.01f;
    gp = 0.01f;
    ca = 0.01f;
    ga = 0.02f;
    R = 15.0f;

    thisuav_id = rover.g.sysid_this_mav.get();

    // 分布式运动观测器变量定义
    dot_tgt_pose_obs = Vector2f(0.0f, 0.0f);  // 对目标位置的分布式观测值的导数，在论文的Section3.B中记录为\dot{\hat{p}}_{d,i}
    tgt_pose_obs = Vector2f(0.0f, 0.0f);   // 对目标位置的分布式观测值，在论文的Section3.B中记录为\hat{p}_{d,i}
    dot_tgt_accel_obs = Vector2f(0.0f, 0.0f);  // 对目标加速度的分布式观测值的导数，在论文的Section3.B中记录为\dot{\hat{a}}_{d,i}
    tgt_accel_obs = Vector2f(0.0f, 0.0f);  // 对目标加速度的分布式观测值，在论文的Section3.B中记录为\hat{a}_{d,i}
    con_pose = Vector2f(0.0f, 0.0f);   // 对目标位置的分布式观测值的一致性误差，在论文的Section3.B中的（17）中记录为\omega_i
    con_accel = Vector2f(0.0f, 0.0f);   // 对目标加速度的分布式观测值的一致性误差，在论文的Section3.B中的（18）中记录为\ksi_i
    current_position = Vector2f(0.0f, 0.0f);
    dot_hattheta = 0.0f;
    hattheta = 0.0f;  // 自适应估计项\hat{\theta}
    dot_hatksi = 0.0f;
    hatksi = 0.0f;  // 自适应估计项\hat{\ksi}

    // 无标签目标包围控制器参数设置      
    c1 = 3.f/2.f;
    c2 = 3.f/4.f;
    c3 = 1.f/7.f;
    kphi = 2.f;
    d = 3.5f;

    // 无标签目标包围控制器变量定义
    xypose = Vector2f(0.0f, 0.0f);  // 从无人机位置中抽取仅需要的x与y轴两方向位置
    xy_tgt_pose_obs = Vector2f(0.0f, 0.0f);  // 从self.tgt_pose_obs中抽取的仅需要的x与y轴两方向信息
    xy_tgt_accel_obs = Vector2f(0.0f, 0.0f);  // 从self.tgt_accel_obs中抽取的仅需要的x与y轴两方向信息
    hatk = Vector2f(0.0f, 0.0f);   // 速度补偿项\hat{k}_i，在公式（36）中定义
    dot_hatk = Vector2f(0.0f, 0.0f);   // 速度补偿项\hat{k}_i的导数，在公式（36）中定义
    relapose = Vector2f(0.0f, 0.0f);  // 计算无人机间相对位置向量时使用的辅助向量
    repulsionsolo = Vector2f(0.0f, 0.0f);   // 计算无人机间斥力的辅助向量
    repulsiontotal = Vector2f(0.0f, 0.0f);   // 计算无人机与其他无人机间斥力综合的辅助向量
    attract = Vector2f(0.0f, 0.0f);   // 目标对无人机的斥力
    cmd_accel_enu = Vector2f(0.0f, 0.0f);
    cmd_vel_enu = Vector2f(0.0f, 0.0f);
}

// update 
void UFence::update()
{
    const bool position_ok = rover.ekf_position_ok();
    if (position_ok) {
        Vector3f temp_vel;
        if (rover.ahrs.get_velocity_NED(temp_vel)) {
            ;
        }
        _accel_x.update(temp_vel.x, millis());
        _accel_y.update(temp_vel.y, millis());
    }
    update_mavlink();
    update_log();

    for (uint8_t i_uav = 0; i_uav < UFENCE_UAV_NUM; i_uav++) {
        otheruav[i_uav].update();
    }
}

void UFence::update_mavlink() {
    static uint32_t _last_send_ms = millis();
    uint16_t mask = GCS_MAVLINK::active_channel_mask() | GCS_MAVLINK::streaming_channel_mask();
    if (millis() - _last_send_ms > 100) {//30 Hz
        _last_send_ms = millis();
        for (uint8_t i=0; i<gcs().num_gcs(); i++) {
            mavlink_channel_t channel = (mavlink_channel_t)(MAVLINK_COMM_0 + i);
            if (mask & (1U<<i)) {
                if (comm_get_txspace(channel) >= GCS_MAVLINK::packet_overhead_chan(channel) + 255) {
                    send_mavlink(channel);
                }
            }
        }
    }
}

void UFence::send_mavlink(mavlink_channel_t chan) {
    const bool position_ok = rover.ekf_position_ok();
    if (!position_ok) {return;}

    // static uint32_t _last_mav_ms = millis();
    // if (millis() - _last_mav_ms > 3000) {
    //     _last_mav_ms = millis();
    //     gcs().send_text(MAV_SEVERITY_INFO, "JSFence mavlink_msg_jsfencing_send");
    // }

    Vector3f temp_vel;
    if (rover.ahrs.get_velocity_NED(temp_vel)) {
        ;
    }
    mavlink_msg_jsfencing_send(
                                chan,
                                rover.current_loc.lat,//current_lat,
                                rover.current_loc.lng,//current_lng,
                                rover.current_loc.lat, //tgt_pose_obs_lat,
                                rover.current_loc.lng, //tgt_pose_obs_lng,
                                temp_vel.x, //tgt_vel_obs_x,
                                temp_vel.y, //tgt_vel_obs_y,
                                _accel_x.slope()*1000.f, //tgt_accel_obs_x,
                                _accel_y.slope()*1000.f, //tgt_accel_obs_y,
                                temp_vel.x, //tgt_vel_obs_x,
                                temp_vel.y, //tgt_vel_obs_y,
                                2);
}

void UFence::handle_message(const mavlink_message_t &msg) {
    // skip our own messages
    if (msg.sysid == rover.g.sysid_this_mav.get()) {
        return;
    }

    switch (msg.msgid) {
        case MAVLINK_MSG_ID_JSFENCING: {
            // gcs().send_text(MAV_SEVERITY_INFO, "get msg from %d", msg.sysid);
            // decode message
            mavlink_jsfencing_t packet;
            mavlink_msg_jsfencing_decode(&msg, &packet);
            if (packet.role == 1) {
                // skip out-of-predefined messages
                if (!(0 < msg.sysid && msg.sysid <= UFENCE_UAV_NUM)) {
                    return;
                }
                handle_message_uav(msg.sysid, packet);
            }
            break;
        }
        default:
            break;
    }
}

void UFence::handle_message_uav(uint16_t msg_sysid, mavlink_jsfencing_t &packet) {
    const bool position_ok = rover.ekf_position_ok();
    if (!position_ok) {return;}
    otheruav[msg_sysid-1].id = msg_sysid;
    otheruav[msg_sysid-1].current_loc.lat = packet.current_lat;
    otheruav[msg_sysid-1].current_loc.lng = packet.current_lng;
    otheruav[msg_sysid-1].tgt_pose_obs_loc.lat = packet.tgt_pose_obs_lat;
    otheruav[msg_sysid-1].tgt_pose_obs_loc.lng = packet.tgt_pose_obs_lng;
    otheruav[msg_sysid-1].current_vel.x = packet.current_vel_x;
    otheruav[msg_sysid-1].current_vel.y = packet.current_vel_y;
    otheruav[msg_sysid-1].tgt_accel_obs.x = packet.tgt_accel_obs_x;
    otheruav[msg_sysid-1].tgt_accel_obs.y = packet.tgt_accel_obs_y;
    otheruav[msg_sysid-1].tgt_vel_obs.x = packet.tgt_vel_obs_x;
    otheruav[msg_sysid-1].tgt_vel_obs.y = packet.tgt_vel_obs_y;
    otheruav[msg_sysid-1].last_msg_ms = millis();
    if (!otheruav[msg_sysid-1].valid) {
        otheruav[msg_sysid-1].valid = true;
        gcs().send_text(MAV_SEVERITY_INFO, "JSFence %d connect", otheruav[msg_sysid-1].id);
    }
}


void UFence::update_log() {
    const bool position_ok = rover.ekf_position_ok();
    if (!position_ok) {return;}

    static uint32_t _last_log_ms = millis();
    if (millis() - _last_log_ms > 100) {
        _last_log_ms = millis();
    } else {
        return;
    }
    Vector3f temp_vel;
    if (rover.ahrs.get_velocity_NED(temp_vel)) {
        ;
    }

    if (rover.current_loc.get_vector_xy_from_origin_NE(current_position)) {
        current_position = current_position * 0.01f;
    }
    AP::logger().WriteStreaming("JFT",
                                "TimeUS,tlat,tlng,tpx,tpy,tvx,tvy,tax,tay",
                                "s--------",
                                "F--------",
                                "Qiiffffff",
                                AP_HAL::micros64(),
                                (float)rover.current_loc.lat,
                                (float)rover.current_loc.lng,
                                (float)current_position.x,
                                (float)current_position.y,
                                (float)temp_vel.x,
                                (float)temp_vel.y,
                                (float)_accel_x.slope()*1000.f,
                                (float)_accel_y.slope()*1000.f);


    uint8_t i_uav = 0;

    i_uav = 0;
    if (otheruav[i_uav].valid) {
        Vector2f uav_pos;
        if (otheruav[i_uav].current_loc.get_vector_xy_from_origin_NE(uav_pos)) {
            uav_pos = uav_pos * 0.01f;
        }
        Vector2f tgt_pos;
        if (otheruav[i_uav].tgt_pose_obs_loc.get_vector_xy_from_origin_NE(tgt_pos)) {
            tgt_pos = tgt_pos * 0.01f;
        }
        AP::logger().WriteStreaming("JFU1",
                                    "TimeUS,lat,lng,px,py,vx,vy",
                                    "s------",
                                    "F------",
                                    "Qiiffff",
                                    AP_HAL::micros64(),
                                    otheruav[i_uav].current_loc.lat,
                                    otheruav[i_uav].current_loc.lng,
                                    (float)uav_pos.x,
                                    (float)uav_pos.y,
                                    (float)otheruav[i_uav].current_vel.x,
                                    (float)otheruav[i_uav].current_vel.y);

        AP::logger().WriteStreaming("JFT1",
                                    "TimeUS,opx,opy,ovx,ovy,oax,oay",
                                    "s------",
                                    "F------",
                                    "Qffffff",
                                    AP_HAL::micros64(),
                                    (float)tgt_pos.x,
                                    (float)tgt_pos.y,
                                    (float)otheruav[i_uav].tgt_vel_obs.x,
                                    (float)otheruav[i_uav].tgt_vel_obs.y,
                                    (float)otheruav[i_uav].tgt_accel_obs.x,
                                    (float)otheruav[i_uav].tgt_accel_obs.y);
    }

    i_uav = 1;
    if (otheruav[i_uav].valid) {
        Vector2f uav_pos;
        if (otheruav[i_uav].current_loc.get_vector_xy_from_origin_NE(uav_pos)) {
            uav_pos = uav_pos * 0.01f;
        }
        Vector2f tgt_pos;
        if (otheruav[i_uav].tgt_pose_obs_loc.get_vector_xy_from_origin_NE(tgt_pos)) {
            tgt_pos = tgt_pos * 0.01f;
        }
        AP::logger().WriteStreaming("JFU2",
                                    "TimeUS,lat,lng,px,py,vx,vy",
                                    "s------",
                                    "F------",
                                    "Qiiffff",
                                    AP_HAL::micros64(),
                                    otheruav[i_uav].current_loc.lat,
                                    otheruav[i_uav].current_loc.lng,
                                    (float)uav_pos.x,
                                    (float)uav_pos.y,
                                    (float)otheruav[i_uav].current_vel.x,
                                    (float)otheruav[i_uav].current_vel.y);

        AP::logger().WriteStreaming("JFT2",
                                    "TimeUS,opx,opy,ovx,ovy,oax,oay",
                                    "s------",
                                    "F------",
                                    "Qffffff",
                                    AP_HAL::micros64(),
                                    (float)tgt_pos.x,
                                    (float)tgt_pos.y,
                                    (float)otheruav[i_uav].tgt_vel_obs.x,
                                    (float)otheruav[i_uav].tgt_vel_obs.y,
                                    (float)otheruav[i_uav].tgt_accel_obs.x,
                                    (float)otheruav[i_uav].tgt_accel_obs.y);
    }

    i_uav = 2;
    if (otheruav[i_uav].valid) {
        Vector2f uav_pos;
        if (otheruav[i_uav].current_loc.get_vector_xy_from_origin_NE(uav_pos)) {
            uav_pos = uav_pos * 0.01f;
        }
        Vector2f tgt_pos;
        if (otheruav[i_uav].tgt_pose_obs_loc.get_vector_xy_from_origin_NE(tgt_pos)) {
            tgt_pos = tgt_pos * 0.01f;
        }
        AP::logger().WriteStreaming("JFU3",
                                    "TimeUS,lat,lng,px,py,vx,vy",
                                    "s------",
                                    "F------",
                                    "Qiiffff",
                                    AP_HAL::micros64(),
                                    otheruav[i_uav].current_loc.lat,
                                    otheruav[i_uav].current_loc.lng,
                                    (float)uav_pos.x,
                                    (float)uav_pos.y,
                                    (float)otheruav[i_uav].current_vel.x,
                                    (float)otheruav[i_uav].current_vel.y);

        AP::logger().WriteStreaming("JFT3",
                                    "TimeUS,opx,opy,ovx,ovy,oax,oay",
                                    "s------",
                                    "F------",
                                    "Qffffff",
                                    AP_HAL::micros64(),
                                    (float)tgt_pos.x,
                                    (float)tgt_pos.y,
                                    (float)otheruav[i_uav].tgt_vel_obs.x,
                                    (float)otheruav[i_uav].tgt_vel_obs.y,
                                    (float)otheruav[i_uav].tgt_accel_obs.x,
                                    (float)otheruav[i_uav].tgt_accel_obs.y);
    }

    i_uav = 3;
    if (otheruav[i_uav].valid) {
        Vector2f uav_pos;
        if (otheruav[i_uav].current_loc.get_vector_xy_from_origin_NE(uav_pos)) {
            uav_pos = uav_pos * 0.01f;
        }
        Vector2f tgt_pos;
        if (otheruav[i_uav].tgt_pose_obs_loc.get_vector_xy_from_origin_NE(tgt_pos)) {
            tgt_pos = tgt_pos * 0.01f;
        }
        AP::logger().WriteStreaming("JFU4",
                                    "TimeUS,lat,lng,px,py,vx,vy",
                                    "s------",
                                    "F------",
                                    "Qiiffff",
                                    AP_HAL::micros64(),
                                    otheruav[i_uav].current_loc.lat,
                                    otheruav[i_uav].current_loc.lng,
                                    (float)uav_pos.x,
                                    (float)uav_pos.y,
                                    (float)otheruav[i_uav].current_vel.x,
                                    (float)otheruav[i_uav].current_vel.y);

        AP::logger().WriteStreaming("JFT4",
                                    "TimeUS,opx,opy,ovx,ovy,oax,oay",
                                    "s------",
                                    "F------",
                                    "Qffffff",
                                    AP_HAL::micros64(),
                                    (float)tgt_pos.x,
                                    (float)tgt_pos.y,
                                    (float)otheruav[i_uav].tgt_vel_obs.x,
                                    (float)otheruav[i_uav].tgt_vel_obs.y,
                                    (float)otheruav[i_uav].tgt_accel_obs.x,
                                    (float)otheruav[i_uav].tgt_accel_obs.y);
    }

    i_uav = 4;
    if (otheruav[i_uav].valid) {
        Vector2f uav_pos;
        if (otheruav[i_uav].current_loc.get_vector_xy_from_origin_NE(uav_pos)) {
            uav_pos = uav_pos * 0.01f;
        }
        Vector2f tgt_pos;
        if (otheruav[i_uav].tgt_pose_obs_loc.get_vector_xy_from_origin_NE(tgt_pos)) {
            tgt_pos = tgt_pos * 0.01f;
        }
        AP::logger().WriteStreaming("JFU5",
                                    "TimeUS,lat,lng,px,py,vx,vy",
                                    "s------",
                                    "F------",
                                    "Qiiffff",
                                    AP_HAL::micros64(),
                                    otheruav[i_uav].current_loc.lat,
                                    otheruav[i_uav].current_loc.lng,
                                    (float)uav_pos.x,
                                    (float)uav_pos.y,
                                    (float)otheruav[i_uav].current_vel.x,
                                    (float)otheruav[i_uav].current_vel.y);

        AP::logger().WriteStreaming("JFT5",
                                    "TimeUS,opx,opy,ovx,ovy,oax,oay",
                                    "s------",
                                    "F------",
                                    "Qffffff",
                                    AP_HAL::micros64(),
                                    (float)tgt_pos.x,
                                    (float)tgt_pos.y,
                                    (float)otheruav[i_uav].tgt_vel_obs.x,
                                    (float)otheruav[i_uav].tgt_vel_obs.y,
                                    (float)otheruav[i_uav].tgt_accel_obs.x,
                                    (float)otheruav[i_uav].tgt_accel_obs.y);
    }
}

void UFence::uav_status::init() {
    valid = false;
}

void UFence::uav_status::update() {
    if (millis() - last_msg_ms > 2000) {
        if (valid == true) {
            gcs().send_text(MAV_SEVERITY_INFO, "JSFence %d lost", id);
        }
        valid = false;
    }
}
