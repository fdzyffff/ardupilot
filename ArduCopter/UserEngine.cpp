#include "Copter.h"

#define USERENGINE_THR_TRIM 1150
#define USERENGINE_THR_LOWEST 980
#define USERENGINE_THR_HIGHEST 2000

void UserEngine::Init(SRV_Channel::Aux_servo_function_t in_srv_function, uint8_t id_in)
{
    set_state(EngineState::Brake_2);
    _connected = false;
    _output = USERENGINE_THR_TRIM;
    _srv_function = in_srv_function;
    _id = id_in;
}

void UserEngine::Set_msg(FD_engine* engine_in)
{
    _msg = engine_in;
    _msg->set_enable();
    _msg->PREAMBLE_ID = _id;
}

FD_engine* UserEngine::Get_msg()
{
    return _msg;
}

void UserEngine::Update()
{
    // sainty_check
    if (copter.g2.user_parameters.thr_low.get() < 1000) {
        copter.g2.user_parameters.thr_low.set(1000);
    }

    if (copter.g2.user_parameters.thr_low.get() > 1200) {
        copter.g2.user_parameters.thr_low.set(1200);
    }
    update_uart();
    update_state();
}

void UserEngine::update_uart() 
{
    if (_msg == nullptr) {
        set_connected(false);
        return;
    }
    if (_msg->_msg_1.updated) {
        _msg_state.last_ms = millis();
        _msg->_msg_1.updated = false;
        _msg->_msg_1.need_send = true;
    }
    if (_msg_state.last_ms == 0 || millis() - _msg_state.last_ms > 5000) {
        set_connected(false);
    } else {
        set_connected(true);
    }
}

void UserEngine::set_connected(bool v_in)
{
    if (v_in == _connected) {
        return;
    }
    _connected = v_in;
    if (_connected) {
        gcs().send_text(MAV_SEVERITY_INFO, "[%d] connect", _id);
    } else {
        gcs().send_text(MAV_SEVERITY_INFO, "[%d] lost", _id);
    }
}

void UserEngine::update_state()
{
    uint32_t delta_t = millis() - _last_state_ms;

    switch (_state) {
        default:
            _output = copter.g2.user_parameters.thr_low;
            break;
        case EngineState::Boost_1:
            _output = USERENGINE_THR_LOWEST; // lowest, 980
            if (delta_t > 1000) {
                set_state(EngineState::Boost_2);
            }
            break;
        case EngineState::Boost_2:
            _output = copter.g2.user_parameters.thr_low; // normal low, 1150
            if (delta_t > 1000) {
                set_state(EngineState::Boost_3);
            }
            break;
        case EngineState::Boost_3:
            _output = USERENGINE_THR_HIGHEST; // highest, 1950
            if (delta_t > 1000) {
                set_state(EngineState::Boost_4);
            }
            break;
        case EngineState::Boost_4:
            _output = copter.g2.user_parameters.thr_low; // normal low, 1150
            if (delta_t > 1000) {
                set_state(EngineState::Running);
            }
            break;
        case EngineState::Brake_1:
            _output = copter.g2.user_parameters.thr_low; // normal low, 1150
            if (delta_t > 5000) {
                set_state(EngineState::Brake_2);
            }
            break;
        case EngineState::Brake_2:
            _output = USERENGINE_THR_LOWEST; // lowest, 980
            // if (delta_t > 3000 && !connected()) {
            //     set_state(EngineState::Normal);
            // }
            break;
    }
}

uint16_t UserEngine::get_output()
{
    return _output;
}

void UserEngine::boost()
{
    if (connected()) {
        if (is_state(EngineState::Running)) { 
            return;
        }
    }
    set_state(EngineState::Boost_1);
}

void UserEngine::brake()
{
    if (connected()) {
        if (is_state(EngineState::Brake_1) || is_state(EngineState::Brake_2)) { 
            return;
        }
    }
    set_state(EngineState::Brake_1);
}

void UserEngine::set_state(UserEngine::EngineState in_state)
{
    _state = in_state;
    _last_state_ms = millis();
}

bool UserEngine::is_state(UserEngine::EngineState in_state)
{
    return (_state == in_state);
}

bool UserEngine::can_override()
{
    bool ret = true;
    switch (_state) {
        default:
        case EngineState::Running:
            ret = true;
            break;
        case EngineState::Normal:
        case EngineState::Boost_1:
        case EngineState::Boost_2:
        case EngineState::Boost_3:
        case EngineState::Boost_4:
        case EngineState::Brake_1:
        case EngineState::Brake_2:
            ret = false;
            break;
    }
    return ret;
}
