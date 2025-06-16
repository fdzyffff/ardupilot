#pragma once

#include <AP_HAL/AP_HAL.h>
#include <FD1_UART/FD1_UART.h>

class UBIM {

public:

    friend class Copter;
    friend class ModeAttack;

    // constructor, destructor
    UBIM();

    // var_info for holding Parameter information
    // static const struct AP_Param::GroupInfo var_info[];

    void init();
    void update_log();
    void update();
    void update_msg_cmd();
    void update_msg_send();
    bool switch_back_to_wp();
    bool switch_hover();
    bool switch_unlock();
    bool switch_manual();
    bool switch_land();
    bool cmd_add_wp();
    bool cmd_set_pos();
    bool cmd_set_speed();
    bool cmd_set_alt();
    bool cmd_set_yaw();
    bool cmd_set_pos_offset();

    bool uav_unlock;
    bool uav_manual;
    uint8_t _plat_switch_cmd;
    uint8_t _plat_switch_act;
    uint8_t _plat_input_cmd;
    uint8_t _plat_input_act;

private:

    FD1_UART uart_bim{AP_SerialManager::SerialProtocol_BIM};
};
