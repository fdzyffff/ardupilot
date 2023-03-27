#include "Copter.h"

#define USERENGINE_LOWEST 1000
#define USERENGINE_HIGHEST 2100

void UserBarrel::Init(SRV_Channel::Aux_servo_function_t in_srv_function_fire, SRV_Channel::Aux_servo_function_t in_srv_function_cut)
{
    set_state(BarrelState::NORMAL);
    _srv_function[FIRE_CHANNEL] = in_srv_function_fire;
    _srv_function[CUT_CHANNEL] = in_srv_function_cut;
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
            _output[FIRE_CHANNEL] = USERENGINE_LOWEST;
            _output[CUT_CHANNEL] = 0;
            break;
        case BarrelState::NORMAL:
            _output[FIRE_CHANNEL] = USERENGINE_LOWEST; // lowest
            _output[CUT_CHANNEL] = 0;
            break;
        case BarrelState::FIRED:
            _output[FIRE_CHANNEL] = USERENGINE_HIGHEST; // highest
            _output[CUT_CHANNEL] = 0;
            if (delta_t > 1000) {
                set_state(BarrelState::NORMAL);
            }
            break;
        case BarrelState::CUT:
            _output[FIRE_CHANNEL] = USERENGINE_LOWEST;
            _output[CUT_CHANNEL] = 1000; // highest
            if (delta_t > 1000) {
                set_state(BarrelState::NORMAL);
            }
            break;
    }
    SRV_Channels::set_output_pwm(_srv_function[FIRE_CHANNEL], _output[FIRE_CHANNEL]);
    SRV_Channels::set_output_scaled(_srv_function[CUT_CHANNEL], _output[CUT_CHANNEL]);
}

uint16_t UserBarrel::get_output(uint16_t channel)
{
    if (channel != 0 && channel != 1) {
        return 0;
    }
    return _output[channel];
}

void UserBarrel::fire()
{
    if (is_state(BarrelState::NORMAL)) { 
        set_state(BarrelState::FIRED);
    }
}

void UserBarrel::cut()
{
    if (is_state(BarrelState::NORMAL)) { 
        set_state(BarrelState::CUT);
    }
}

void UserBarrel::set_state(UserBarrel::BarrelState in_state)
{
    _state = in_state;
    _last_state_ms = millis();
    update_state();
}

bool UserBarrel::is_state(UserBarrel::BarrelState in_state)
{
    return (_state == in_state);
}

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void UserNetgun::Init()
{
    _barrels[0].Init(SRV_Channel::k_netgun_fire_1, SRV_Channel::k_netgun_cut_1);
    _barrels[1].Init(SRV_Channel::k_netgun_fire_2, SRV_Channel::k_netgun_cut_2);
 
    SRV_Channels::set_range(SRV_Channel::k_netgun_pitch,  1000);
    SRV_Channels::set_range(SRV_Channel::k_netgun_cut_1,  1000);
    SRV_Channels::set_range(SRV_Channel::k_netgun_cut_2,  1000);
    _do_stab = true;
    _angle_current = 0.0f;
    _angle_target = 0.0f;
    _angle_stab = 0.0f;
    _angle_trim = 0.0f;
    _slew_rate = 0.0f;
    check_param();
}

void UserNetgun::check_param()
{
    _angle_max = (float)copter.g2.user_parameters.netgun_max;
    _angle_min = (float)copter.g2.user_parameters.netgun_min;

    _angle_max = constrain_float(_angle_max, -1500.f, 13500.f);
    _angle_min = constrain_float(_angle_min, -4500.f, _angle_max - 3000.f);
    _angle_max = MAX(_angle_min, _angle_max);
    _slew_rate = constrain_float((float)copter.g2.user_parameters.netgun_slew, 500.f, 9000.f);
}

void UserNetgun::handle_info(float p1, float p2, float p3, float p4, float p5, float p6, float p7)
{
    if ((uint16_t)p1 == 1) {
        Fire((uint16_t)p2);
    }
    if ((uint16_t)p1 == 2) {
        copter.set_mode(Mode::Number::LOCKON, ModeReason::GCS_COMMAND);
    }
    if ((uint16_t)p1 == 3) {
        set_trim(p2);
    }
    if ((uint16_t)p1 == 4) {
        Cut((uint16_t)p2);
    }
    if ((uint16_t)p1 == 10) {
        set_target(p3);
        set_trim(p2);
    }
}

void UserNetgun::Stabilize()
{
    check_param();
    _angle_current = constrain_float(_angle_target - _angle_current, -_slew_rate, _slew_rate) + _angle_current;
    _angle_stab = (degrees(copter.ahrs_view->pitch)*100.f); //nose down in negative, thus need to trans
    float _output = constrain_float(_angle_current + _angle_stab - _angle_trim, _angle_min, _angle_max);
    float _output_norm = _output/(_angle_max - _angle_min)*1000.f;

    SRV_Channels::set_output_scaled(SRV_Channel::k_netgun_pitch, _output_norm);
}

void UserNetgun::set_target(float v_in) //norm 1000
{
    check_param();
    _angle_target = constrain_float(v_in, 0.0f, 1000.f)*(_angle_max - _angle_min);
}

void UserNetgun::set_target_angle(float angle_in) // cd
{
    check_param();
    _angle_target = constrain_float(angle_in, _angle_min, _angle_max);
}

void UserNetgun::set_trim(float v_in) //norm 1000
{
    check_param();
    _angle_trim = constrain_float(v_in, -1000.0f, 1000.f)*0.001*(_angle_max - _angle_min);
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

void UserNetgun::Fire(uint16_t channel)
{
    for(uint8_t i_barrel = 0; i_barrel < NETGUN_NUM; i_barrel++) {
        if (channel == 0 || channel == (i_barrel+1)) {
            _barrels[i_barrel].fire();
            gcs().send_text(MAV_SEVERITY_INFO, "Fire Net %d", i_barrel);
        }
    }
}

void UserNetgun::Cut(uint16_t channel)
{
    for(uint8_t i_barrel = 0; i_barrel < NETGUN_NUM; i_barrel++) {
        if (channel == 0 || channel == (i_barrel+1)) {
            _barrels[i_barrel].cut();
            gcs().send_text(MAV_SEVERITY_INFO, "Cut Net %d", i_barrel);
        }
    }
}
