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


#include "Copter.h"

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
    // 分布式运动观测器参数设置
    cp = 0.01f;
    gp = 0.01f;
    ca = 0.01f;
    ga = 0.02f;
    R = 15.0f;

    thisuav_id = copter.g.sysid_this_mav.get();

    // 分布式运动观测器变量定义
    dot_tgt_pose_obs = Vector2f(0.0f, 0.0f);  // 对目标位置的分布式观测值的导数，在论文的Section3.B中记录为\dot{\hat{p}}_{d,i}
    tgt_pose_obs = Vector2f(0.0f, 0.0f);   // 对目标位置的分布式观测值，在论文的Section3.B中记录为\hat{p}_{d,i}
    dot_tgt_accel_obs = Vector2f(0.0f, 0.0f);  // 对目标加速度的分布式观测值的导数，在论文的Section3.B中记录为\dot{\hat{a}}_{d,i}
    tgt_accel_obs = Vector2f(0.0f, 0.0f);  // 对目标加速度的分布式观测值，在论文的Section3.B中记录为\hat{a}_{d,i}
    con_pose = Vector2f(0.0f, 0.0f);   // 对目标位置的分布式观测值的一致性误差，在论文的Section3.B中的（17）中记录为\omega_i
    con_accel = Vector2f(0.0f, 0.0f);   // 对目标加速度的分布式观测值的一致性误差，在论文的Section3.B中的（18）中记录为\ksi_i
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
    distance = 0.0   // 计算无人机间距离向量时使用的辅助向量
    repulsionsolo = Vector2f(0.0f, 0.0f);   // 计算无人机间斥力的辅助向量
    repulsiontotal = Vector2f(0.0f, 0.0f);   // 计算无人机与其他无人机间斥力综合的辅助向量
    attract = Vector2f(0.0f, 0.0f);   // 目标对无人机的斥力
    cmd_accel_enu = Vector2f(0.0f, 0.0f);
    cmd_vel_enu = Vector2f(0.0f, 0.0f);
}

// update 
void UFence::update()
{
    if (!copter.position_ok()) {return;}
    // 1. 目标加速度估计（输出为self.tgt_accel_est，用于目标运动分布式估计器）
    // 1.1 目标探测状态设置：根据距离判断是否可以获取目标位置
    relapose = current_position - tgt_pose;
    distance = relapose.length();
    if (distance < 15.0f) {
        detect = 1.0f;
    } else {
        detect = 0.0f;
    }

    // 2. 目标运动分布式观测器（输入为self.tgt_accel_est，输出为self.tgt_pose_obs与self.tgt_accel_obs，用于无标签目标包围控制器）
    // 2.1 基于公式（17）-（18）计算目标观测的共识误差
    // tgt_pose_obs publish
    // tgt_accel_obs publish
    con_pose = detect * (tgt_pose_obs - tgt_pose);
    con_accel = detect * (tgt_accel_obs - tgt_accel_est);
    for (uint8_t i_uav = 0; i_uav < uav_num; i_uav++) {
        if (!otheruav[i_uav].is_valid()) {continue;}
        relapose = current_position - otheruav[i_uav].position;
        float distance = relapose.length();
        if (thisuav_id != otheruav[i_uav].id && 0.2f < distance && distance < R) {
            con_pose = con_pose + tgt_pose_obs - otheruav[i_uav].tgt_pose_obs;
            con_accel = con_accel + tgt_accel_obs - otheruav[i_uav].tgt_accel_obs;
        }
    }

    // 2.2 基于公式（21）-（22）计算分布式观测器更新率以及观测值
    float temp_con_pose_length = MAX(0.0001f, con_pose.length());
    dot_tgt_pose_obs = - cp * con_pose - hattheta * con_pose/temp_con_pose_length;
    float temp_con_accel_length = MAX(0.0001f, con_accel.length());
    dot_tgt_accel_obs = - ca * con_accel - hatksi * con_accel/temp_con_accel_length;
    dot_hattheta = gp * con_pose.length();
    dot_hatksi = gp * con_accel.length();
    tgt_pose_obs = tgt_pose_obs + dot_tgt_pose_obs*dt;
    tgt_accel_obs = tgt_pose_obs + dot_tgt_accel_obs*dt;
    hattheta = hattheta + dot_hattheta*dt;
    hatksi = hatksi + dot_hatksi*dt;
        
                    
    // 3. 无标签目标包围控制器（输入为self.tgt_pose_obs与self.tgt_accel_obs，输出为速度指令self.cmd_vel_enu）
    // 3.1 无人机间斥力计算
    repulsiontotal = Vector2f(0.0f, 0.0f);
    for (uint8_t i_uav = 0; i_uav < uav_num; i_uav++)
    {
        if (!otheruav[i_uav].is_valid()) {continue;}
        relapose = current_position - otheruav[i_uav].position;
        distance = relapose.length();
        // 这部分计算斥力时加了一个保险，如果两架无人机之间距离小于self.d，则设置一个固定的较大斥力，防止无人机间发生碰撞
        distance = MAX(0.2, distance);
        if (thisuav_id != otheruav[i_uav].id && d < distance && distance < R) {
            repulsionsolo = kphi * (1.f/(distance - d) - 1.f/(R - d)) /distance * relapose;
            repulsiontotal = repulsiontotal + repulsionsolo;
        } 
    }

    // 3.2 目标对无人机的吸引力计算
    Vector2f xypose = Vector2f(current_position.x, current_position.y);
    Vector2f xy_tgt_pose_obs = Vector2f(tgt_pose_obs.x, tgt_pose_obs.y);
    Vector2f xy_tgt_accel_obs = Vector2f(tgt_accel_obs.x, tgt_accel_obs.y);
    dot_hatk = -c1*hatk - (c1*c2 - c3)/c2 * (repulsiontotal + c2*(xy_tgt_pose_obs - xypose));
    hatk = hatk + dot_hatk*dt;
    attract = c2 * (xy_tgt_pose_obs - xypose) + hatk + xy_tgt_accel_obs;

    // 3.3 计算加速度（x与y轴的最大加速度均设置为1m/s^2）
    cmd_accel_enu = attract + repulsiontotal;
    if (cmd_accel_enu.length() > 1.0f) {
        cmd_accel_enu = cmd_accel_enu/cmd_accel_enu.length();
    }
    
    // 3.4 计算并发布速度指令
    // 3.4.1 正常运行无人机的速度（x与y轴的最大速度均设置为1m/s）
    cmd_vel_enu = cmd_vel_enu + cmd_accel_enu*dt;
    if (cmd_vel_enu.length() > 1.0f) {
        cmd_vel_enu = cmd_vel_enu/cmd_vel_enu.length();
    }
}

void UFence::send_mavlink(mavlink_channel_t chan) {
    if (!copter.position_ok()) {return;}
    mavlink_msg_jsfencing_send(
                                chan,
                                copter.current_loc.lat,//current_lat,
                                copter.current_loc.lng,//current_lng,
                                tgt_pose_obs.lat, //tgt_pose_obs_lat,
                                tgt_pose_obs.lng, //tgt_pose_obs_lng,
                                tgt_accel_obs.x, //tgt_accel_obs_x,
                                tgt_accel_obs.y, //tgt_accel_obs_y,
                                copter.user_parameters.role.get());
}

void UFence::handle_message(const mavlink_message_t &msg) {
    // skip our own messages
    if (msg.sysid == g.sysid_this_mav.get()) {
        return;
    }
    // skip out-of-predefined messages
    if (0 < msg.sysid && msg.sysid <= UFENCE_UAV_NUM) {
        return;
    }

    switch (msg.msgid) {
        case MAVLINK_MSG_ID_JSFENCING: {
            // decode message
            mavlink_jsfencing_t packet;
            mavlink_msg_jsfencing_decode(&msg, &packet);
            if (packet.role == 1) {
                handle_message_uav(msg.sysid, packet);
            }
            if (packet.role == 2) {
                handle_message_target(packet);
            }
            break;
        }
        default:
            break;
    }
}

void UFence::handle_message_uav(uint16_t msg_sysid, mavlink_jsfencing_t &packet) {
    if (!copter.position_ok()) {return;}
    Location temp_loc;
    temp_loc.lat = packet.current_lat;
    temp_loc.lng = packet.current_lng;
    if (copter.current_loc.get_distance_NE(temp_loc).length() > copter.g2.user_parameters.detection_R.get()) {return;}

    otheruav[msg_sysid-1].id = msg_sysid;
    otheruav[msg_sysid-1].current_loc.lat = packet.current_lat;
    otheruav[msg_sysid-1].current_loc.lng = packet.current_lng;
    otheruav[msg_sysid-1].tgt_pose_obs_loc.lat = packet.tgt_pose_obs_lat;
    otheruav[msg_sysid-1].tgt_pose_obs_loc.lng = packet.tgt_pose_obs_lng;
    otheruav[msg_sysid-1].tgt_accel_obs.x = packet.tgt_accel_obs_x;
    otheruav[msg_sysid-1].tgt_accel_obs.y = packet.tgt_accel_obs_y;
    otheruav[msg_sysid-1].last_msg_ms = millis();
    if (!otheruav[msg_sysid-1].valid) {
        otheruav[msg_sysid-1].valid = true;
        gcs.send_text(MAV_SEVERITY_INFO, "%d connect", otheruav[msg_sysid-1].id);
    }
}

void UFence::handle_message_target(mavlink_jsfencing_t &packet) {
    if (!copter.position_ok()) {return;}
    Location temp_loc;
    temp_loc.lat = packet.current_lat;
    temp_loc.lng = packet.current_lng;
    if (copter.current_loc.get_distance_NE(temp_loc).length() > copter.g2.user_parameters.connection_R.get()) {return;}

    tgt_pose_loc.lat = packet.current_lat;
    tgt_pose_loc.lng = packet.current_lng;
    tgt_accel_est.x = packet.tgt_accel_obs_x;
    tgt_accel_est.y = packet.tgt_accel_obs_y;
}

void UFence::uav_status::init() {
    valid = false;
}

void UFence::uav_status::update() {
    if (millis() - last_msg_ms > 2000) {
        if (valid = true) {
            gcs.send_text(MAV_SEVERITY_INFO, "%d lost", id);
        }
        valid = false;
    }
}