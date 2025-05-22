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
        Vector2f tgt_accel_obs;
        uint32_t last_msg_ms;
        bool valid;
    };

    uav_status otheruav[UFENCE_UAV_NUM];

    // constructor, destructor
    UFence();

    // initialise
    void init();

    void handle_message(const mavlink_message_t msg);

    void update();

private:

    

};
