#pragma once

#include "Ugroup.h"

class UGround {

public:

    // constructor, destructor
    UGround();

    // initialise
    void init();

    // return true if smart_rtl is usable (it may become unusable if the user took off without GPS lock or the path became too long)
    bool is_active() const { return _active; }

    void handle_info(int16_t p1, float p2, float p3, float p4);

    void do_cmd(int16_t cmd);
    void refresh_cmd();

    void update();
    void state_update();

    bool is_leader() {return _is_leader;}
    void is_leader(bool b) {_is_leader = b;}

    float get_group_distance() {return _group_distance;}
    int16_t get_group_id() {return _group_id;}
    Vector3f get_offset(int8_t id_A, int8_t id_B, float distance);
    void set_up_group( int16_t group_id, float distance, float direction);


    void set_cruise_yaw_middle_cd(float yaw_middle_cd) {_yaw_middle_cd = yaw_middle_cd;}
    void update_group_yaw(float dt);
    void set_up_follow(int8_t sender_id, Vector3f target_postion, Vector3f target_velocity, float target_heading);
    void set_up_dest(float lat_in, float lng_in);
    void refresh_dest();
    void set_up_alt(float target_alt);
    void set_up_alt_offset(float target_alt);
    void set_up_search_dist(float search_dist);
    void clean_follow();
    float get_final_target_alt();

    int16_t get_state_num();
    Vector3f get_dest_loc_vec() {return _dest_loc_vec;}
    Vector3f get_follow_vel_vec() {return _follow_vel_vec;}
    Vector3f get_follow_loc_vec() {return _follow_loc_vec;}
    Vector3f get_search_dest();
    Vector3f get_assemble_dest();
    bool dest_pos_update() {return _new_dist;}
    void dest_pos_update(bool b) {_new_dist = b;}
    float get_follow_yaw_cd() {return _follow_yaw_cd;}
    float get_lockon_yaw_rate();
    float get_lockon_yaw_rate(float yaw_middle_cd);
    float get_lockon_yaw() { return _yaw_middle_cd;}

    bool do_takeoff(); // takeoff
    bool do_fly();     // fly
    bool do_search();  // search
    bool do_assemble();// assemble
    bool do_lockon();  // lock on target
    bool do_attack();  // attack
    bool do_fs1();     // failsafe type1
    bool do_pause();   // brake
    bool do_arm();     // arm


    int16_t _group_id;
    int16_t _position_id;
    uint32_t _last_update_ms;
    float _yaw_middle_cd;

private:

    Vector2f raw_info;
    Vector2f correct_info;
    Vector3f _follow_loc_vec;
    Vector3f _follow_vel_vec;
    Vector3f _dest_loc_vec;
    Vector3f _raw_dest_loc_vec;
    Vector3f _leader_loc_vec;
    float _follow_yaw_cd;
    float _group_target_yaw;
    float _group_current_yaw;
    float _group_distance;
    bool _active;
    bool _is_leader;
    bool _new_dist;
    int16_t _cmd;
    uint32_t _state_timer_ms;
    uint32_t _last_state_update_ms;
    int16_t _leader_id;
    float _dist_to_target;
    float _bearing_to_target;
    float _gcs_target_alt_offset;

    my_group_1_t my_group1;
    my_group_2_t my_group2;
    my_group_1_assemble_t my_group1_assemble;
};

