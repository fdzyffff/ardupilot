#pragma once

#include <FD1_UART/FD1_UART.h>
#include <SRV_Channel/SRV_Channel.h>
#include <AP_SerialManager/AP_SerialManager.h>      // Serial manager library

class TS_ctrl_t {
public:
    void init();
    void update();
    void ts_ahrs_send();
    void ts_roi_send();
    void mission_handle_and_route();
    void ts_handle_and_route();
    void get_Time(uint8_t &year_out, uint8_t &month_out, uint8_t &day_out, uint8_t &hour_out, uint8_t &minute_out, uint8_t &second_out);
    void test_ts_uart(int16_t msg_id, uint8_t option);
    void test_ts_start();
    void test_ts_stop();
    void test_ts_update();
    void test_ts_40();
    void test_ts_AHRS();

    void valid_update();
    void set_valid(bool v_in);
    bool valid() {return _valid;} 
    void set_target_loc(Location& loc_in); 
    Location get_target_loc() {return _target_loc;}

    FD1_UART uart_msg_ts{AP_SerialManager::SerialProtocol_TS};
    FD1_UART uart_msg_mission{AP_SerialManager::SerialProtocol_MISSION};

    LowPassFilterVector3f _target_pos{1.0};

    bool _valid;
    uint32_t _last_msg_update_ms;
    Location _target_loc;

    int16_t _test_ts_id;
    bool _test_ts_running;
};
