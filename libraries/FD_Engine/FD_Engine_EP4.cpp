#include "FD_Engine_EP4.h"

FD_Engine_EP4::FD_Engine_EP4(FD_Engine &_engine) :
    Engine_backend(_engine)
{
    ;
}

void FD_Engine_EP4::init() {
    uart_msg_ep4.init();
    uart_msg_ep4.get_msg_ep4_in().set_enable();
    uart_msg_ep4.get_msg_ep4_out().set_enable();

    uart_msg_ep4_route.init();
    uart_msg_ep4_route.get_msg_ep4_in().set_enable();

    SRV_Channels::set_range(SRV_Channel::k_throttle_EP4, 100);
    SRV_Channels::set_range(SRV_Channel::k_throttle_Starter, 100);

    _last_msg_update_ms = 0;
    _initialized = true;
}

void FD_Engine_EP4::update() {
    if (!_initialized) {return;}
    update_state();
    update_uart();
    update_pwm();
    update_connection();
}

void FD_Engine_EP4::update_connection() {
    if (_last_msg_update_ms == 0 || AP_HAL::millis() - _last_msg_update_ms > 1000) {
        set_connected(false);
    } else {
        set_connected(true);
    }
    static uint32_t _last_ms = AP_HAL::millis();
    if (connected() && is_state(EngineState::Running)) {
        FD1_msg_ep4_in &tmp_msg = uart_msg_ep4.get_msg_ep4_in();
        uint16_t rpm = tmp_msg._msg_1.content.msg.rpm;
        if (AP_HAL::millis() - _last_ms > 5000 && (rpm < 3000)) {
            gcs().send_text(MAV_SEVERITY_INFO, "Low rpm %d", rpm);
            _last_ms = AP_HAL::millis();
        }
    }
}

void FD_Engine_EP4::set_connected(bool v_in)
{
    if (v_in == _connected) {
        return;
    }
    _connected = v_in;
    if (_connected) {
        gcs().send_text(MAV_SEVERITY_INFO, "EP4 connect");
    } else {
        gcs().send_text(MAV_SEVERITY_INFO, "EP4 lost");
    }
}

void FD_Engine_EP4::update_state() {
    uint32_t delta_t = AP_HAL::millis() - _last_state_ms;
    float throttle_in = SRV_Channels::get_output_scaled(SRV_Channel::k_throttle);
    FD1_msg_ep4_in &tmp_msg = uart_msg_ep4.get_msg_ep4_in();
    uint16_t damper = tmp_msg._msg_1.content.msg.damper; //unit: 0.1%, 100% is 1000
    switch (_state) {
        default:
        case EngineState::Prepare:
            ep4_throttle_output = engine.throttle_min;
            starter_output = 0.0f;
            if (connected()) {
                if (230 < damper && damper < 270) {
                    set_state(EngineState::Start);
                }
            }
            if (delta_t > 2000) {
                gcs().send_text(MAV_SEVERITY_INFO, "TimeOUT, force start!");
                set_state(EngineState::Start);
            }
            break;
        case EngineState::Start:
            ep4_throttle_output = engine.throttle_min;
            starter_output = 100.0f;
            if (delta_t > 2000) {
                set_state(EngineState::Running);
            }
            break;
        case EngineState::Running:
            ep4_throttle_output = constrain_float(throttle_in, engine.throttle_min, 100.f);
            starter_output = 0.0f;
            break;
        case EngineState::Stop:
            ep4_throttle_output = 0.0f;
            starter_output = 0.0f;
            break;
    }
}

void FD_Engine_EP4::start()
{
    if (!_initialized) {
        gcs().send_text(MAV_SEVERITY_INFO, "EP4 disabled!");
        return;
    }
    if (is_state(EngineState::Prepare) || is_state(EngineState::Start)) { return;}
    FD1_msg_ep4_in &tmp_msg = uart_msg_ep4.get_msg_ep4_in();
    uint16_t rpm = tmp_msg._msg_1.content.msg.rpm;
    if (connected() ) {
        if (rpm < 1000) {
            set_state(EngineState::Prepare);
        }
    } else {
        gcs().send_text(MAV_SEVERITY_INFO, "No rpm info, force start!");
        set_state(EngineState::Prepare);
    }
}

void FD_Engine_EP4::stop()
{
    if (!_initialized) {
        gcs().send_text(MAV_SEVERITY_INFO, "EP4 disabled!");
        return;
    }
    if (is_state(EngineState::Stop)) { return; }
    set_state(EngineState::Stop);
}

void FD_Engine_EP4::set_state(FD_Engine_EP4::EngineState in_state)
{
    _state = in_state;
    _last_state_ms = AP_HAL::millis();
    switch (_state) {
        default:
        case EngineState::Start:
            gcs().send_text(MAV_SEVERITY_INFO, "EP4 Start");
            break;
        case EngineState::Running:
            gcs().send_text(MAV_SEVERITY_INFO, "EP4 Running");
            break;
        case EngineState::Stop:
            gcs().send_text(MAV_SEVERITY_INFO, "EP4 Stop");
            break;
    }
}

bool FD_Engine_EP4::is_state(FD_Engine_EP4::EngineState in_state)
{
    return (_state == in_state);
}

void FD_Engine_EP4::update_uart() {
    if (uart_msg_ep4.initialized()) {

        while (uart_msg_ep4.port_avaliable() > 0) {
            uart_msg_ep4.read();
            uart_ep4_handle_and_route();
            uart_msg_ep4.write();  
        }

        uart_ep4_send();
        uart_msg_ep4.write();
    }

    // time out check for msg from ep4, will be sent via uart (optionally) and mav
    if (uart_msg_ep4_route.get_msg_ep4_in()._msg_1.updated) {
        if (_last_msg_update_ms > 0 && AP_HAL::millis()-_last_msg_update_ms > 2000) {
            uart_msg_ep4_route.get_msg_ep4_in()._msg_1.updated = false;
        }
    }

    // route to uart
    if (uart_msg_ep4_route.initialized()) {
        uart_msg_ep4_route.write();
    }
}

void FD_Engine_EP4::update_pwm() {
    SRV_Channels::set_output_scaled(SRV_Channel::k_throttle_EP4, ep4_throttle_output);
    SRV_Channels::set_output_scaled(SRV_Channel::k_throttle_Starter, starter_output);
}

void FD_Engine_EP4::uart_ep4_handle_and_route() {
    FD1_msg_ep4_in &tmp_msg = uart_msg_ep4.get_msg_ep4_in();
    if (tmp_msg._msg_1.updated) {
        _last_msg_update_ms = AP_HAL::millis();

        // copy to uart_msg_ep4_route for following uart and mav uses
        memcpy(uart_msg_ep4_route.get_msg_ep4_in()._msg_1.content.data, 
            uart_msg_ep4.get_msg_ep4_in()._msg_1.content.data, 
            uart_msg_ep4.get_msg_ep4_in()._msg_1.length*sizeof(uint8_t));
        uart_msg_ep4_route.get_msg_ep4_in()._msg_1.updated = true;
        uart_msg_ep4_route.get_msg_ep4_in()._msg_1.need_send = true;

        tmp_msg._msg_1.updated = false;

        engine.status.rpm = tmp_msg._msg_1.content.msg.rpm;
        engine.status.fuel_pressure = MAX(tmp_msg._msg_1.content.msg.fuel_pressure, 0);
        engine.status.cylinder_temp1 = (uint16_t)(MAX(tmp_msg._msg_1.content.msg.cylinder_temp1, 0));
        engine.status.cylinder_temp2 = (uint16_t)(MAX(tmp_msg._msg_1.content.msg.cylinder_temp2, 0));
        engine.status.venting_temp1 = (uint16_t)(MAX(tmp_msg._msg_1.content.msg.venting_temp1, 0));
        engine.status.venting_temp2 = (uint16_t)(MAX(tmp_msg._msg_1.content.msg.venting_temp2, 0));
    }
}

void FD_Engine_EP4::uart_ep4_send() {
    FD1_msg_ep4_out &tmp_msg = uart_msg_ep4.get_msg_ep4_out();
    static uint8_t n_count = 0;
    tmp_msg._msg_1.content.msg.sum_check = 0;
    tmp_msg._msg_1.content.msg.damper = (uint16_t)(ep4_throttle_output*10.0f);
    tmp_msg._msg_1.content.msg.empty1 = 0;
    tmp_msg._msg_1.content.msg.count = n_count;
    tmp_msg._msg_1.content.msg.header.head_1 = FD1_msg_ep4_out::PREAMBLE1;
    tmp_msg._msg_1.content.msg.header.head_2 = FD1_msg_ep4_out::PREAMBLE2;

    for (int8_t i = 2; i < tmp_msg._msg_1.length - 1; i++) {
        tmp_msg._msg_1.content.msg.sum_check += tmp_msg._msg_1.content.data[i];
    }
    tmp_msg._msg_1.need_send = true;
    n_count++;
}
