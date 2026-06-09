#pragma once

#include <FD_UART/FD_UART.h>

class UMission {

public:

    // constructor, destructor
    UMission();

    void init();
    void update();
    void check_alive();
    void update_log();
    void handle_message(const mavlink_message_t &msg);
    void set_target_loc(Location& loc_in);
    bool have_target_loc() {return _alive;}
    Location get_target_loc() {return _target_loc;}

private:

    bool _alive;
    LowPassFilterVector3f _target_pos{1.0};
    uint32_t _last_log_ms;
    uint32_t _last_target_update_ms;
    Location _target_loc;
};
