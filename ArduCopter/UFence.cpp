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
    tgt_vel_est = Vector2f(0.0f, 0.0f);
    tgt_last_ms = 0;
    // 分布式运动观测器参数设置
    cp = 0.01f;
    gp = 0.01f;
    ca = 0.01f;
    ga = 0.01f;
    cv = 0.01f;
    gv = 0.02f;
    R = 15.0f;

    thisuav_id = copter.g.sysid_this_mav.get();

    // 分布式运动观测器变量定义
    dot_tgt_pose_obs = Vector2f(0.0f, 0.0f);  // 对目标位置的分布式观测值的导数，在论文的Section3.B中记录为\dot{\hat{p}}_{d,i}
    tgt_pose_obs = Vector2f(0.0f, 0.0f);   // 对目标位置的分布式观测值，在论文的Section3.B中记录为\hat{p}_{d,i}
    dot_tgt_accel_obs = Vector2f(0.0f, 0.0f);  // 对目标加速度的分布式观测值的导数，在论文的Section3.B中记录为\dot{\hat{a}}_{d,i}
    tgt_accel_obs = Vector2f(0.0f, 0.0f);  // 对目标加速度的分布式观测值，在论文的Section3.B中记录为\hat{a}_{d,i}
    con_pose = Vector2f(0.0f, 0.0f);   // 对目标位置的分布式观测值的一致性误差，在论文的Section3.B中的（17）中记录为\omega_i
    con_accel = Vector2f(0.0f, 0.0f);   // 对目标加速度的分布式观测值的一致性误差，在论文的Section3.B中的（18）中记录为\ksi_i
    dot_tgt_vel_obs = Vector2f(0.0f, 0.0f);  // 对目标速度的分布式观测值的导数，在论文的Section3.B中记录为\dot{\hat{p}}_{d,i}
    tgt_vel_obs = Vector2f(0.0f, 0.0f);   // 对目标速度的分布式观测值，在论文的Section3.B中记录为\hat{p}_{d,i}
    con_vel = Vector2f(0.0f, 0.0f);   // 对目标速度的分布式观测值的一致性误差，在论文的Section3.B中的（17）中记录为\omega_i
    current_position = Vector2f(0.0f, 0.0f);
    dot_hattheta = 0.0f;
    hattheta = 0.0f;  // 自适应估计项\hat{\theta}
    dot_hatksi = 0.0f;
    hatksi = 0.0f;  // 自适应估计项\hat{\ksi}
    dot_hatphi = 0.0f;
    hatphi = 0.0f;  // 自适应估计项\hat{\phi}

    // 无标签目标包围控制器参数设置      
    c1 = 3.f/2.f;
    c2 = 3.f/4.f;
    c3 = 1.f/7.f;
    kphi = 3.5f;
    d = 2.5f;

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
    cmd_vel_enu_final = Vector2f(0.0f, 0.0f);
}

// update 
void UFence::update()
{
    update_vel();
    update_mavlink();

    for (uint8_t i_uav = 0; i_uav < UFENCE_UAV_NUM; i_uav++) {
        otheruav[i_uav].update();
    }

    update_log();
}

void UFence::update_vel()
{
    bool do_print = false;
    if (do_print) {gcs().send_text(MAV_SEVERITY_INFO, "JSFence: update_vel");}  

    if (!copter.position_ok()) {return;}
    if (!copter.current_loc.get_vector_xy_from_origin_NE(current_position)) {return;}

    current_position = current_position * 0.01f;

    // 1. 目标加速度估计（输出为self.tgt_accel_est，用于目标运动分布式估计器）
    // 1.1 目标探测状态设置：根据距离判断是否可以获取目标位置

    static uint32_t last_cal_ms = millis();
    if (millis() - last_cal_ms > 5000) {
        last_cal_ms = millis();
        // gcs().send_text(MAV_SEVERITY_INFO, "JSFence: posobs x:%f, y:%f", tgt_pose_obs.x, tgt_pose_obs.y);
        // gcs().send_text(MAV_SEVERITY_INFO, "JSFence: velobs x:%f, y:%f", tgt_vel_obs.x, tgt_vel_obs.y);
        // gcs().send_text(MAV_SEVERITY_INFO, "JSFence: C x:%f, y:%f", cmd_vel_enu.x, cmd_vel_enu.y);
        // gcs().send_text(MAV_SEVERITY_INFO, "JSFence: A x:%f, y:%f", xy_tgt_accel_obs.x, xy_tgt_accel_obs.y);
    }

    if (do_print) {gcs().send_text(MAV_SEVERITY_INFO, "JSFence: update_vel 2");}  

    if (tgt_pose_obs_loc.lat == 0 || tgt_pose_obs_loc.lng == 0) {
        if (do_print) {gcs().send_text(MAV_SEVERITY_INFO, "JSFence: current_loc");}
        tgt_pose_obs_loc.lat = copter.current_loc.lat;
        tgt_pose_obs_loc.lng = copter.current_loc.lng;
        tgt_pose_obs_loc.alt = copter.current_loc.alt;
    }

    if (!tgt_pose_obs_loc.get_vector_xy_from_origin_NE(tgt_pose_obs)) {
        if (do_print) {gcs().send_text(MAV_SEVERITY_INFO, "JSFence: no tgt_pos");}
        return;
    }

    tgt_pose_obs = tgt_pose_obs*0.01f;


    static uint32_t last_update_ms = millis();
    float dt = constrain_float((float)(millis() - last_update_ms) * 0.001f, 0.0f, 1.0f);
    last_update_ms = millis();

    if (do_print) {gcs().send_text(MAV_SEVERITY_INFO, "JSFence: dt: %f", dt);}

    if (millis() - tgt_last_ms < 2000) {
        float vel_dt = constrain_float((float)(millis() - tgt_last_ms) * 0.001f, 0.0f, 2.0f);
        if (tgt_pose_loc.get_vector_xy_from_origin_NE(tgt_pose)) {
            tgt_pose = tgt_pose * 0.01f + tgt_vel_est*(vel_dt+0.2f);
            detect = 1.0f;
        } else {
            detect = 0.0f;
        }
    } else {
        if (!is_zero(detect)) {
            tgt_accel_obs = Vector2f(0.0f, 0.0f);
            tgt_vel_obs = Vector2f(0.0f, 0.0f);
        }
        detect = 0.0f;
    }

    // 2. 目标运动分布式观测器（输入为self.tgt_accel_est，输出为self.tgt_pose_obs与self.tgt_accel_obs，用于无标签目标包围控制器）
    // 2.1 基于公式（17）-（18）计算目标观测的共识误差
    // tgt_pose_obs publish
    // tgt_accel_obs publish
    con_pose =  (tgt_pose_obs - tgt_pose) * detect;
    con_accel = (tgt_accel_obs - tgt_accel_est) * detect;
    con_vel = (tgt_vel_obs - tgt_vel_est) * detect;
    // if (do_print) {gcs().send_text(MAV_SEVERITY_INFO, "JSFence: conpos 1");}
    for (uint8_t i_uav = 0; i_uav < UFENCE_UAV_NUM; i_uav++) {
        if (!otheruav[i_uav].is_valid()) {continue;}
        relapose = copter.current_loc.get_distance_NE(otheruav[i_uav].current_loc);
        float distance = relapose.length();
        if (thisuav_id != otheruav[i_uav].id && 0.2f < distance && distance < R) {
            if (otheruav[i_uav].tgt_pose_obs_loc.get_vector_xy_from_origin_NE(otheruav[i_uav].tgt_pose_obs)) {
                otheruav[i_uav].tgt_pose_obs = otheruav[i_uav].tgt_pose_obs * 0.01f;
                con_pose = con_pose + tgt_pose_obs - otheruav[i_uav].tgt_pose_obs;
                con_accel = con_accel + tgt_accel_obs - otheruav[i_uav].tgt_accel_obs;
                con_vel = con_vel + tgt_vel_obs - otheruav[i_uav].tgt_vel_obs;
            }
        }
    }
    if (do_print) {gcs().send_text(MAV_SEVERITY_INFO, "JSFence: conpos 2");}

    // 2.2 基于公式（21）-（22）计算分布式观测器更新率以及观测值
    float temp_con_pose_length = MAX(0.0001f, con_pose.length());
    dot_tgt_pose_obs = con_pose*(-cp) - (con_pose*hattheta/temp_con_pose_length);
    float temp_con_accel_length = MAX(0.0001f, con_accel.length());
    dot_tgt_accel_obs = con_accel*(-ca) - (con_accel*hatksi/temp_con_accel_length);
    dot_hattheta = gp * con_pose.length();
    dot_hatksi = ga * con_accel.length();
    tgt_pose_obs = tgt_pose_obs + dot_tgt_pose_obs*dt;
    tgt_pose_obs_loc = Location(Vector3f(tgt_pose_obs.x*100.f, tgt_pose_obs.y*100.f, 0.0f), Location::AltFrame::ABSOLUTE);
    tgt_accel_obs = tgt_accel_obs + dot_tgt_accel_obs*dt;
    hattheta = hattheta + dot_hattheta*dt;
    hatksi = hatksi + dot_hatksi*dt;

    float temp_con_vel_length = MAX(0.0001f, con_vel.length());
    dot_tgt_vel_obs = con_vel*(-cv) - (con_vel*hatphi/temp_con_vel_length);
    dot_hatphi = gv * con_vel.length();
    tgt_vel_obs = tgt_vel_obs + dot_tgt_vel_obs*dt;
    hatphi = hatphi + dot_hatphi*dt;


    if (do_print) {gcs().send_text(MAV_SEVERITY_INFO, "JSFence: hatksi");} 
                    
    // 3. 无标签目标包围控制器（输入为self.tgt_pose_obs与self.tgt_accel_obs，输出为速度指令self.cmd_vel_enu）
    // 3.1 无人机间斥力计算
    repulsiontotal = Vector2f(0.0f, 0.0f);
    for (uint8_t i_uav = 0; i_uav < UFENCE_UAV_NUM; i_uav++)
    {
        if (!otheruav[i_uav].is_valid()) {continue;}
        relapose = otheruav[i_uav].current_loc.get_distance_NE(copter.current_loc);//注意方向
        float distance = relapose.length();
        // 这部分计算斥力时加了一个保险，如果两架无人机之间距离小于self.d，则设置一个固定的较大斥力，防止无人机间发生碰撞
        distance = MAX(d, distance);
        if (thisuav_id != otheruav[i_uav].id && d < distance && distance < R) {
            repulsionsolo = relapose * (kphi * (1.f/(distance - d) - 1.f/(R - d)) /distance);
            repulsiontotal = repulsiontotal + repulsionsolo;
        } 
    }
    // if (do_print) {gcs().send_text(MAV_SEVERITY_INFO, "JSFence: repulsiontotal");}  

    // 3.2 目标对无人机的吸引力计算
    xypose = Vector2f(current_position.x, current_position.y);
    xy_tgt_pose_obs = Vector2f(tgt_pose_obs.x, tgt_pose_obs.y);
    xy_tgt_accel_obs = Vector2f(tgt_accel_obs.x, tgt_accel_obs.y);
    dot_hatk = hatk*(-c1) - (repulsiontotal + (xy_tgt_pose_obs - xypose) * c2)*((c1*c2 - c3)/c2);
    hatk = hatk + dot_hatk*dt;
    attract = (xy_tgt_pose_obs - xypose)*(c2) + hatk + xy_tgt_accel_obs;

    if (do_print) {gcs().send_text(MAV_SEVERITY_INFO, "JSFence: attract");}  

    // // 3.3 计算加速度（x与y轴的最大加速度均设置为1m/s^2）
    cmd_accel_enu = attract + repulsiontotal;
    if (cmd_accel_enu.length() > 2.0f) {
        cmd_accel_enu = cmd_accel_enu/cmd_accel_enu.length();
    }

    // 3.4 计算并发布速度指令
    // 3.4.1 正常运行无人机的速度（x与y轴的最大速度均设置为1m/s）


    Vector2f brake_accel_enu;
    brake_accel_enu = cmd_vel_enu * (-0.15f);

    cmd_vel_enu = cmd_vel_enu + cmd_accel_enu*dt + brake_accel_enu * dt;// + tgt_vel_obs;
    if (cmd_vel_enu.length() > 2.0f) {
        cmd_vel_enu = cmd_vel_enu/cmd_vel_enu.length();
    }
    cmd_vel_enu_final = (cmd_vel_enu);//+tgt_vel_obs);

    
    if (do_print) {gcs().send_text(MAV_SEVERITY_INFO, "JSFence: cmd_accel_enu");}  
}

Vector2f& UFence::get_cmd_vel_enu() {
    return cmd_vel_enu_final;
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
    if (!copter.position_ok()) {return;}
    Vector3f temp_vel;
    if (copter.ahrs.get_velocity_NED(temp_vel)) {
        ;
    }
    mavlink_msg_jsfencing_send(
                                chan,
                                copter.current_loc.lat,//current_lat,
                                copter.current_loc.lng,//current_lng,
                                tgt_pose_obs_loc.lat, //tgt_pose_obs_lat,
                                tgt_pose_obs_loc.lng, //tgt_pose_obs_lng,
                                temp_vel.x,
                                temp_vel.y,
                                tgt_accel_obs.x, //tgt_accel_obs_x,
                                tgt_accel_obs.y, //tgt_accel_obs_y,
                                tgt_vel_obs.x, //tgt_vel_obs_x,
                                tgt_vel_obs.y, //tgt_vel_obs_y,
                                copter.g2.user_parameters.role.get());
}

void UFence::handle_message(const mavlink_message_t &msg) {
    // skip our own messages
    if (msg.sysid == copter.g.sysid_this_mav.get()) {
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
    if (copter.current_loc.get_distance_NE(temp_loc).length() > copter.g2.user_parameters.connection_R.get()) {return;}

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

void UFence::handle_message_target(mavlink_jsfencing_t &packet) {
    if (!copter.position_ok()) {return;}
    Location temp_loc;
    temp_loc.lat = packet.current_lat;
    temp_loc.lng = packet.current_lng;
    if (copter.current_loc.get_distance_NE(temp_loc).length() > copter.g2.user_parameters.detection_R.get()) {return;}

    tgt_pose_loc.lat = packet.current_lat;
    tgt_pose_loc.lng = packet.current_lng;
    tgt_accel_est.x = packet.tgt_accel_obs_x;
    tgt_accel_est.y = packet.tgt_accel_obs_y;
    tgt_vel_est.x = packet.tgt_vel_obs_x;
    tgt_vel_est.y = packet.tgt_vel_obs_y;

    tgt_last_ms = millis();
}


void UFence::update_log() {
    if (!copter.position_ok()) {return;}
    static uint32_t _last_log_ms = millis();
    if (millis() - _last_log_ms > 100) {
        _last_log_ms = millis();
    } else {
        return;
    }

    AP::logger().WriteStreaming("JFN1",
                                "TimeUS,tlat,tlng,tvx,tvy,tax,tay",
                                "s------",
                                "F------",
                                "Qiiffff",
                                AP_HAL::micros64(),
                                tgt_pose_loc.lat,
                                tgt_pose_loc.lng,
                                (float)tgt_vel_est.x,
                                (float)tgt_vel_est.y,
                                (float)tgt_accel_est.x,
                                (float)tgt_accel_est.y);

    AP::logger().WriteStreaming("JFN2",
                                "TimeUS,olat,olng,ovx,ovy,oax,oay",
                                "s------",
                                "F------",
                                "Qiiffff",
                                AP_HAL::micros64(),
                                tgt_pose_obs_loc.lat,
                                tgt_pose_obs_loc.lng,
                                (float)tgt_vel_obs.x,
                                (float)tgt_vel_obs.y,
                                (float)tgt_accel_obs.x,
                                (float)tgt_accel_obs.y);

    AP::logger().WriteStreaming("JFN3",
                                "TimeUS,tpx,tpy,cvx,cvy,cax,cay",
                                "s------",
                                "F------",
                                "Qffffff",
                                AP_HAL::micros64(),
                                (float)tgt_pose.x,
                                (float)tgt_pose.y,
                                (float)cmd_vel_enu.x,
                                (float)cmd_vel_enu.y,
                                (float)cmd_accel_enu.x,
                                (float)cmd_accel_enu.y);

    AP::logger().WriteStreaming("JFN4",
                                "TimeUS,tpox,tpoy,theta,ksi,phi,atkx,atky",
                                "s-------",
                                "F-------",
                                "Qfffffff",
                                AP_HAL::micros64(),
                                (float)tgt_pose_obs.x,
                                (float)tgt_pose_obs.y,
                                (float)hattheta,
                                (float)hatksi,
                                (float)hatphi,
                                (float)hatk.x,
                                (float)hatk.y);

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