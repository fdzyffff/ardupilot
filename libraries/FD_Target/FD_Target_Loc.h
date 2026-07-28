#pragma once

#include "FD_Target.h"

#include <AP_Param/AP_Param.h>

class FD_Target_Loc : public FD_Target_Base {
public:
    FD_Target_Loc();

    static const struct AP_Param::GroupInfo var_info[];

    bool init() override;
    void update() override;
    void handle_msg(const mavlink_message_t &msg) override;
    void handle_info_test(float p1, float p2);
    void set_target_loc(Location &loc_in);
    Location &get_target_loc() override;

    Location _current_loc;

    AP_Int32 target_timeout;
    AP_Float target_distout;
    AP_Float nav_radius;
    AP_Int8 use_external_loc;

private:
    uint32_t _last_target_ms;
    uint32_t last_update_ms;
};