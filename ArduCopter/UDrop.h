#pragma once
#include <AP_HAL/AP_HAL.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include "FD1_UART/FD1_UART.h"

class UDrop {
public:
    enum class UDropState{
        Idle = 1,
        Ready = 3,
        Launching = 4,
        Done = 5,
        Error = 9
    };

    void init();
    void update();
    void read_uart();
    void update_state();
    void write_uart();
    void set_state(UDrop::UDropState in_state);
    bool is_state(UDrop::UDropState in_state);
    void do_set_waypoint(Location& loc);
    void do_launch();
    void verify_launch();

    void handle_info(float p1, float p2, float p3, float p4);
    void do_test_ID1(int16_t p2, bool need_send);
    void do_test_ID2(int16_t p2, bool need_send);
    void do_test_ID6(int16_t p2, bool need_send);

    UDropState _state;

    uint32_t _last_state_ms;

	FD1_UART uart_msg_drop{AP_SerialManager::SerialProtocol_Drop};
};
