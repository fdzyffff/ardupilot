#pragma once

#include <FD1_UART/FD1_UART.h>
#include <SRV_Channel/SRV_Channel.h>
#include <AP_SerialManager/AP_SerialManager.h>      // Serial manager library

class UCtrl {
public:
    void init();
    void update();
    void ctrl_send();
    void msg_handle_init();
    void msg_handle_control();
    void msg_handle_mission();
    void msg_handle_guide();
    void test_ctrl_uart(uint8_t msg_id, uint8_t option);
    void test_info_send();

    void update_valid();
    void set_valid(bool v_in);
    bool valid() {return _valid;} 
    void set_target_loc(Location& loc_in); 
    Location get_target_loc() {return _target_loc;}

    void do_loiter_left();
    void do_loiter_right();
    void do_cruise_up();
    void do_cruise();
    void do_cruise_alt(float targe_alt_m);
    void do_cruise_speed(float target_speed_kmh);

    FD1_UART uart_msg_ctrl{AP_SerialManager::SerialProtocol_UCtrl};

    LowPassFilterVector3f _target_pos{1.0};

    bool _valid;
    uint32_t _last_msg_update_ms;
    Location _target_loc;
};
