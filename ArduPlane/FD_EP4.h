#pragma once

#include <FD1_UART/FD1_UART.h>
#include <SRV_Channel/SRV_Channel.h>
#include <AP_SerialManager/AP_SerialManager.h>      // Serial manager library

class EP4_ctrl_t {
public:
    enum class EngineState{
        Stop = 0,
        Prepare = 1,
        Start = 2,
        Running = 3,
    };

    void init();
    void update();
    void set_connected(bool v_in);
    bool connected() {return _connected;}    //初始化和连接判定
    void update_state();
    void update_uart();
    void update_pwm();
    void update_connection();
    void start();
    void stop();
    void set_state(EP4_ctrl_t::EngineState in_state);
    bool is_state(EP4_ctrl_t::EngineState in_state);
    void uart_ep4_init();
    void uart_ep4_update();
    void uart_ep4_handle_and_route();
    void uart_ep4_send();

    void test_EP4_uart(uint8_t msg_id, uint8_t option);
    void test_EP4_in(uint8_t option);
    void test_EP4_out(uint8_t option);

    uint32_t _last_msg_update_ms;
    uint32_t _last_state_ms;

    FD1_UART uart_msg_ep4{AP_SerialManager::SerialProtocol_EP4};
    FD1_UART uart_msg_ep4_route{AP_SerialManager::SerialProtocol_EP4_OUT};

    float ep4_throttle_output;
    float starter_output;

private:
    bool _initialized;
    bool _connected;
    EngineState _state;
};