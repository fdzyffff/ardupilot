#include "Copter.h"

#define USERENGINE_THR_TRIM 1150
#define USERENGINE_THR_LOWEST 980
#define USERENGINE_THR_HIGHEST 2000

void UserEngine::Init(AP_SerialManager::SerialProtocol in_protocol, SRV_Channel::Aux_servo_function_t in_srv_function)
{
    if (_uart == nullptr) {
        _uart = new FD_UART(in_protocol);
    }
    set_state(EngineState::Brake_2);
    _connected = false;
    _output = USERENGINE_THR_TRIM;
    _srv_function = in_srv_function;
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

    _connected = false;
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
            if (delta_t > 1000 && !connected()) {
                set_state(EngineState::Boost_2);
            }
            break;
        case EngineState::Boost_2:
            _output = copter.g2.user_parameters.thr_low; // normal low, 1150
            if (delta_t > 1000 && !connected()) {
                set_state(EngineState::Boost_3);
            }
            break;
        case EngineState::Boost_3:
            _output = USERENGINE_THR_HIGHEST; // highest, 1950
            if (delta_t > 1000 && !connected()) {
                set_state(EngineState::Boost_4);
            }
            break;
        case EngineState::Boost_4:
            _output = copter.g2.user_parameters.thr_low; // normal low, 1150
            if (delta_t > 1000 && !connected()) {
                set_state(EngineState::Running);
            }
            break;
        case EngineState::Brake_1:
            _output = copter.g2.user_parameters.thr_low; // normal low, 1150
            if (delta_t > 5000 && !connected()) {
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
