#include "Copter.h"

#define USERENGINE_THR_LOWEST 1000
#define USERENGINE_THR_HIGHEST 2100

void UserBarrel::Init(SRV_Channel::Aux_servo_function_t in_srv_function)
{
    set_state(BarrelState::NORMAL);
    _srv_function = in_srv_function;
}

void UserBarrel::Update()
{
    update_state();
}

void UserBarrel::update_state()
{
    uint32_t delta_t = millis() - _last_state_ms;

    switch (_state) {
        default:
            _output = USERENGINE_THR_LOWEST;
            break;
        case BarrelState::NORMAL:
            _output = USERENGINE_THR_LOWEST; // lowest
            break;
        case BarrelState::FIRED:
            _output = USERENGINE_THR_HIGHEST; // highest
            if (delta_t > 1000) {
                set_state(BarrelState::NORMAL);
            }
            break;
    }
    SRV_Channels::set_output_pwm(_srv_function, _output);
}

uint16_t UserBarrel::get_output()
{
    return _output;
}

void UserBarrel::trigger()
{
    if (is_state(BarrelState::FIRED)) { 
        return;
    }
    set_state(BarrelState::FIRED);
}

void UserBarrel::release()
{
    if (is_state(BarrelState::NORMAL)) { 
        return;
    }
    set_state(BarrelState::NORMAL);
}

void UserBarrel::set_state(UserBarrel::BarrelState in_state)
{
    _state = in_state;
    _last_state_ms = millis();
}

bool UserBarrel::is_state(UserBarrel::BarrelState in_state)
{
    return (_state == in_state);
}

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void UserNetgun::Init()
{
    _barrels[0].Init(SRV_Channel::k_netgun_1);
    _barrels[1].Init(SRV_Channel::k_netgun_2);
    _barrel_idx = 0;
}

void UserNetgun::Update()
{
    for (uint8_t i_barrel = 0; i_barrel < NETGUN_NUM; i_barrel++) {
        _barrels[i_barrel].Update();
    }
}

void UserNetgun::Trigger()
{
    if (_barrel_idx >= NETGUN_NUM) {
        _barrel_idx = 0;
    }
    _barrels[_barrel_idx].trigger();
    gcs().send_text(MAV_SEVERITY_INFO, "Trigger Net %d", _barrel_idx);
    _barrel_idx++;
}
