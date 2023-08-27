#pragma once
#include "UGroup.h"

class UFollow {

public:

    // constructor, destructor
    UFollow();
    // initialise
    void init();
    bool is_active() const { return _active; }
    void handle_my_follow_msg(const mavlink_message_t &msg);
    void update();
    void get_target_pos(Location &loc);
    float get_target_vel() {return _target_vel;}
    float get_target_bearing() {return _target_bearing;}
    void print();

    int16_t _leader_id;
    UGroup ugroup;
private:

	bool _active;
	Location _raw_target_loc;
	uint32_t _last_update_ms;
	float _target_bearing; // degree
	float _target_vel;
};

