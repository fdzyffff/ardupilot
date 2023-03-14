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

    SRV_Channels::set_range(SRV_Channel::k_netgun_pitch,  1000);
    _do_stab = true;
    _angle_target = 0.0f;
    _angle_stab = 0.0f;
    _angle_trim = 0.0f;
    check_param();
}

void UserNetgun::check_param()
{
    _angle_max = (float)copter.g2.user_parameters.netgun_max;
    _angle_min = (float)copter.g2.user_parameters.netgun_min;

    _angle_max = constrain_float(_angle_max, -1500.f, 13500.f);
    _angle_min = constrain_float(_angle_min, -4500.f, _angle_max - 3000.f);
    _angle_max = MAX(_angle_min, _angle_max);
}

void UserNetgun::handle_info(float p1, float p2, float p3, float p4, float p5, float p6, float p7)
{
    if ((uint16_t)p1 == 1) {
        set_trim(p2);
    }
    if ((uint16_t)p1 == 2) {
        set_target(p3);
        set_trim(p2);
    }
}

void UserNetgun::Stabilize()
{
    check_param();
    _angle_stab = -(0.0f - degrees(copter.ahrs_view->pitch)*100.f); //nose down in negative, thus need to trans
    float _output = constrain_float(_angle_target + _angle_stab + _angle_trim, _angle_min, _angle_max);
    float _output_norm = _output/(_angle_max - _angle_min)*1000.f;

    SRV_Channels::set_output_scaled(SRV_Channel::k_netgun_pitch, _output_norm);
}

void UserNetgun::set_target(float v_in) //norm 1000
{
    _angle_target = constrain_float(v_in, 0.0f, 1000.f)*9.0f;
}

void UserNetgun::set_trim(float v_in) //norm 1000
{
    _angle_trim = constrain_float(v_in, -1000.0f, 1000.f)*9.0f;
}

void UserNetgun::set_stabilize(bool v_in)
{
    _do_stab = v_in;
}

void UserNetgun::Update()
{
    Stabilize();
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
