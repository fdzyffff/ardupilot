#pragma once

#include "Ugroup.h"

class UGround {

public:

    enum UGCS_state_t {
        UGCS_None            = 0,
        UGCS_Takeoff         = 1,
        UGCS_Standby1        = 2,
        UGCS_Fly             = 3,
        UGCS_Standby2        = 4,
        UGCS_Curise          = 5,
        UGCS_Assemble        = 6,
        UGCS_Lockon          = 7,
        UGCS_Attack          = 8,
        UGCS_FS1             = 10,
    };

    // constructor, destructor
    UGround();

    // initialise
    void init();

    // return true if smart_rtl is usable (it may become unusable if the user took off without GPS lock or the path became too long)
    bool is_active() const { return _active; }

    void handle_info(int16_t cmd, int16_t position_id, int16_t group_id, int16_t free=0);

    void do_cmd(int16_t cmd, bool force_set = false);
    void set_state(UGCS_state_t new_state, bool force_set = false);
    void refresh_cmd();
    UGCS_state_t get_state() {return _state;}
    void update();
    void state_update();

    bool is_leader() {return _is_leader;}
    void is_leader(bool b) {_is_leader = b;}
    bool is_lockon() {return (_state == UGCS_Lockon);}
    float get_distance() {return _distance;}
    int16_t get_group_id() {return _group_id;}
    Vector3f get_offset_position() {return _offset_position;}
    Vector3f get_dest_loc_vec() {return _dest_loc_vec;}

    void set_up_offset(int8_t sender_id, Vector3f target_postion, Vector3f target_velocity, float target_heading);

    int16_t get_state_num();
    float get_dest_yaw_cd() {return _dest_yaw_cd;}

    int16_t _group_id;
    int16_t _position_id;
    uint32_t _last_update_ms;

private:

    UGCS_state_t _state;
    Vector2f raw_info;
    Vector2f correct_info;
    Vector3f _dest_loc_vec;
    Vector3f _dest_vel_vec;
    Vector3f _offset_position;
    float _dest_yaw_cd;
    float _distance;
    bool _active;
    bool _is_leader;
    int16_t _cmd;
    uint32_t _state_timer_ms;
    uint32_t _last_state_update_ms;

    my_group_1_t my_group1;
    my_group_2_t my_group2;
};

