#pragma once

#define UFENCE_UAV_NUM 5

class UFence {

public:

    class uav_status {
    public:
        void init();
        bool is_valid() {return valid;}
        void update();
        uint16_t id = 0;
        Location current_loc;
        Location tgt_pose_obs_loc;
        Vector2f tgt_pose_obs;
        Vector2f tgt_accel_obs;
        Vector2f tgt_vel_obs;
        uint32_t last_msg_ms;
        bool valid;
    };

    uav_status otheruav[UFENCE_UAV_NUM];

    // constructor, destructor
    UFence();

    // initialise
    void init();

    void update();
    void update_vel();
    void update_mavlink(); 

    void handle_message(const mavlink_message_t &msg);
    void handle_message_uav(uint16_t msg_sysid, mavlink_jsfencing_t &packet);
    void handle_message_target(mavlink_jsfencing_t &packet);
    void send_mavlink(mavlink_channel_t chan);

    void update_log();

    Vector2f &get_cmd_vel_enu();

    Vector2f cmd_accel_enu;
    Vector2f cmd_vel_enu;
    Vector2f cmd_vel_enu_final;

private:

    float detect;
    Vector2f tgt_pose;
    Vector2f tgt_accel_est;
    Vector2f tgt_vel_est;
    Location tgt_pose_obs_loc;
    Location tgt_pose_loc;
    float tgt_last_ms;
    // 分布式运动观测器参数设置
    float cp;
    float gp;
    float ca;
    float ga;
    float cv;
    float gv;
    float R;

    uint16_t thisuav_id;

    // 分布式运动观测器变量定义
    Vector2f dot_tgt_pose_obs;  // 对目标位置的分布式观测值的导数，在论文的Section3.B中记录为\dot{\hat{p}}_{d,i}
    Vector2f tgt_pose_obs;   // 对目标位置的分布式观测值，在论文的Section3.B中记录为\hat{p}_{d,i}
    Vector2f dot_tgt_accel_obs;  // 对目标加速度的分布式观测值的导数，在论文的Section3.B中记录为\dot{\hat{a}}_{d,i}
    Vector2f tgt_accel_obs;  // 对目标加速度的分布式观测值，在论文的Section3.B中记录为\hat{a}_{d,i}
    Vector2f con_pose;   // 对目标位置的分布式观测值的一致性误差，在论文的Section3.B中的（17）中记录为\omega_i
    Vector2f con_accel;   // 对目标加速度的分布式观测值的一致性误差，在论文的Section3.B中的（18）中记录为\ksi_i
    Vector2f tgt_vel_obs;
    Vector2f dot_tgt_vel_obs;
    Vector2f con_vel;
    Vector2f current_position;
    float dot_hattheta;
    float hattheta;  // 自适应估计项\hat{\theta}
    float dot_hatksi;
    float hatksi;  // 自适应估计项\hat{\ksi}
    float dot_hatphi;
    float hatphi;  // 自适应估计项\hat{\ksi}

    // 无标签目标包围控制器参数设置      
    float c1;
    float c2;
    float c3;
    float kphi;
    float d;

    // 无标签目标包围控制器变量定义
    Vector2f xypose;  // 从无人机位置中抽取仅需要的x与y轴两方向位置
    Vector2f xy_tgt_pose_obs;  // 从self.tgt_pose_obs中抽取的仅需要的x与y轴两方向信息
    Vector2f xy_tgt_accel_obs;  // 从self.tgt_accel_obs中抽取的仅需要的x与y轴两方向信息
    Vector2f hatk;   // 速度补偿项\hat{k}_i，在公式（36）中定义
    Vector2f dot_hatk;   // 速度补偿项\hat{k}_i的导数，在公式（36）中定义
    Vector2f relapose;  // 计算无人机间相对位置向量时使用的辅助向量
    Vector2f repulsionsolo;   // 计算无人机间斥力的辅助向量
    Vector2f repulsiontotal;   // 计算无人机与其他无人机间斥力综合的辅助向量
    Vector2f attract;   // 目标对无人机的斥力

};
