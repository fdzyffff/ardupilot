#include "FD_Engine_LUTAN.h"

FD_Engine_LUTAN::FD_Engine_LUTAN(FD_Engine &_engine) :
    Engine_backend(_engine)
{
    ;
}

void FD_Engine_LUTAN::init() {
    uart_msg_lutan.init();
    uart_msg_lutan.get_msg_lutan_in().set_enable();
    uart_msg_lutan.get_msg_lutan_out().set_enable();

    uart_msg_lutan_route.init();
    uart_msg_lutan_route.get_msg_lutan_in().set_enable();

    SRV_Channels::set_range(SRV_Channel::k_throttle_EP4, 100);
    SRV_Channels::set_range(SRV_Channel::k_throttle_Starter, 100);

    _last_msg_update_ms = 0;
    _initialized = true;
}

void FD_Engine_LUTAN::update() {
    if (!_initialized) {return;}
    update_state();
    update_uart();
    update_pwm();
    update_connection();
}

void FD_Engine_LUTAN::update_connection() {
    if (_last_msg_update_ms == 0 || AP_HAL::millis() - _last_msg_update_ms > 1000) {
        set_connected(false);
    } else {
        set_connected(true);
    }
    static uint32_t _last_ms = AP_HAL::millis();
    if (connected() && is_state(EngineState::Running)) {
        FD1_msg_lutan_in &tmp_msg = uart_msg_lutan.get_msg_lutan_in();
        uint16_t rpm = tmp_msg._msg_1.content.msg.rpm;
        if (AP_HAL::millis() - _last_ms > 5000 && (rpm < 3000)) {
            gcs().send_text(MAV_SEVERITY_INFO, "Low rpm %d", rpm);
            _last_ms = AP_HAL::millis();
        }
    }
}

void FD_Engine_LUTAN::set_connected(bool v_in)
{
    if (v_in == _connected) {
        return;
    }
    _connected = v_in;
    if (_connected) {
        gcs().send_text(MAV_SEVERITY_INFO, "LUTAN connect");
    } else {
        gcs().send_text(MAV_SEVERITY_INFO, "LUTAN lost");
    }
}

void FD_Engine_LUTAN::update_state() {
    uint32_t delta_t = AP_HAL::millis() - _last_state_ms;
    float throttle_in = SRV_Channels::get_output_scaled(SRV_Channel::k_throttle);
    FD1_msg_lutan_in &tmp_msg = uart_msg_lutan.get_msg_lutan_in();
    uint16_t damper = tmp_msg._msg_1.content.msg.tps; //unit: 0.1%, 100% is 1000
    switch (_state) {
        default:
        case EngineState::Prepare:
            throttle_output = engine.throttle_min;
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
            throttle_output = engine.throttle_min;
            starter_output = 100.0f;
            if (delta_t > 2000) {
                set_state(EngineState::Running);
            }
            break;
        case EngineState::Running:
            throttle_output = constrain_float(throttle_in, engine.throttle_min, 100.f);
            starter_output = 0.0f;
            break;
        case EngineState::Stop:
            throttle_output = 0.0f;
            starter_output = 0.0f;
            break;
    }
}

void FD_Engine_LUTAN::start()
{
    if (!_initialized) {
        gcs().send_text(MAV_SEVERITY_INFO, "LUTAN disabled!");
        return;
    }
    if (is_state(EngineState::Prepare) || is_state(EngineState::Start)) { return;}
    FD1_msg_lutan_in &tmp_msg = uart_msg_lutan.get_msg_lutan_in();
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

void FD_Engine_LUTAN::stop()
{
    if (!_initialized) {
        gcs().send_text(MAV_SEVERITY_INFO, "LUTAN disabled!");
        return;
    }
    if (is_state(EngineState::Stop)) { return; }
    set_state(EngineState::Stop);
}

void FD_Engine_LUTAN::set_state(FD_Engine_LUTAN::EngineState in_state)
{
    _state = in_state;
    _last_state_ms = AP_HAL::millis();
    switch (_state) {
        default:
        case EngineState::Start:
            gcs().send_text(MAV_SEVERITY_INFO, "LUTAN Start");
            break;
        case EngineState::Running:
            gcs().send_text(MAV_SEVERITY_INFO, "LUTAN Running");
            break;
        case EngineState::Stop:
            gcs().send_text(MAV_SEVERITY_INFO, "LUTAN Stop");
            break;
    }
}

bool FD_Engine_LUTAN::is_state(FD_Engine_LUTAN::EngineState in_state)
{
    return (_state == in_state);
}

void FD_Engine_LUTAN::update_uart() {
    if (uart_msg_lutan.initialized()) {

        while (uart_msg_lutan.port_avaliable() > 0) {
            uart_msg_lutan.read();
            uart_lutan_handle_and_route();
            uart_msg_lutan.write();  
        }
        uart_lutan_send();

        uart_msg_lutan.write();
    }

    // time out check for msg from ep4, will be sent via uart (optionally) and mav
    if (uart_msg_lutan_route.get_msg_lutan_in()._msg_1.updated) {
        if (_last_msg_update_ms > 0 && AP_HAL::millis()-_last_msg_update_ms > 2000) {
            uart_msg_lutan_route.get_msg_lutan_in()._msg_1.updated = false;
        }
    }

    // route to uart
    if (uart_msg_lutan_route.initialized()) {
        uart_msg_lutan_route.write();
    }
}

void FD_Engine_LUTAN::update_pwm() {
    SRV_Channels::set_output_scaled(SRV_Channel::k_throttle_EP4, throttle_output);
    SRV_Channels::set_output_scaled(SRV_Channel::k_throttle_Starter, starter_output);
}

void FD_Engine_LUTAN::uart_lutan_handle_and_route() {
    FD1_msg_lutan_in &tmp_msg = uart_msg_lutan.get_msg_lutan_in();
    if (tmp_msg._msg_1.updated) {
        _last_msg_update_ms = AP_HAL::millis();

        // copy to uart_msg_lutan_route for following uart and mav uses
        memcpy(uart_msg_lutan_route.get_msg_lutan_in()._msg_1.content.data, 
            uart_msg_lutan.get_msg_lutan_in()._msg_1.content.data, 
            uart_msg_lutan.get_msg_lutan_in()._msg_1.length*sizeof(uint8_t));
        uart_msg_lutan_route.get_msg_lutan_in()._msg_1.updated = true;
        uart_msg_lutan_route.get_msg_lutan_in()._msg_1.need_send = true;

        tmp_msg._msg_1.updated = false;

        engine.status.rpm = tmp_msg._msg_1.content.msg.rpm;
        // engine.status.fuel_pressure = MAX(tmp_msg._msg_1.content.msg.fuel_pressure, 0);
        engine.status.cylinder_temp1 = (uint16_t)(MAX(tmp_msg._msg_1.content.msg.coolant, 0));
        // engine.status.cylinder_temp2 = (uint16_t)(MAX(tmp_msg._msg_1.content.msg.cylinder_temp2, 0));
        // engine.status.venting_temp1 = (uint16_t)(MAX(tmp_msg._msg_1.content.msg.venting_temp1, 0));
        // engine.status.venting_temp2 = (uint16_t)(MAX(tmp_msg._msg_1.content.msg.venting_temp2, 0));
    }
}

void FD_Engine_LUTAN::uart_lutan_send() {
    FD1_msg_lutan_out &tmp_msg = uart_msg_lutan.get_msg_lutan_out();
    static uint8_t n_count = 0;
    tmp_msg._msg_1.content.data[0] = 0x00;
    tmp_msg._msg_1.content.data[1] = 0x07;
    tmp_msg._msg_1.content.data[2] = 0x72;
    tmp_msg._msg_1.content.data[3] = 0x00;
    tmp_msg._msg_1.content.data[4] = 0x07;
    tmp_msg._msg_1.content.data[5] = 0x00;
    tmp_msg._msg_1.content.data[6] = 0x00;
    tmp_msg._msg_1.content.data[7] = 0x00;
    tmp_msg._msg_1.content.data[8] = 0x51;
    tmp_msg._msg_1.content.data[9] = 0x48;
    tmp_msg._msg_1.content.data[10] = 0xd7;
    tmp_msg._msg_1.content.data[11] = 0xa3;
    tmp_msg._msg_1.content.data[12] = 0x0e;

    tmp_msg._msg_1.need_send = true;
    n_count++;
}
