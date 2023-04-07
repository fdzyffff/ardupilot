#include "Copter.h"

void UserEngines::Init()
{
    _engine[0].Init(SRV_Channel::k_engine_1, 1);
    _engine[1].Init(SRV_Channel::k_engine_2, 2);
    _engine[2].Init(SRV_Channel::k_engine_3, 3);
    _engine[3].Init(SRV_Channel::k_engine_4, 4);
    _engine[4].Init(SRV_Channel::k_engine_5, 5);
    _engine[5].Init(SRV_Channel::k_engine_6, 6);
    _engine[6].Init(SRV_Channel::k_engine_7, 7);

    set_state(UserEnginesState::None);
    copter.ap.motor_interlock_switch = false;
    need_send = false;

    Set_port(AP_SerialManager::SerialProtocol_Engines);
}

void UserEngines::Set_port(AP_SerialManager::SerialProtocol in_protocol)
{
    if (_uart == nullptr) {
        _uart = new FD_UART(in_protocol);
        _uart->init();
        if (_uart->initialized()) {
            _engine[0].Set_msg(&(_uart->get_msg_engine(0)));
            _engine[1].Set_msg(&(_uart->get_msg_engine(1)));
            _engine[2].Set_msg(&(_uart->get_msg_engine(2)));
            _engine[3].Set_msg(&(_uart->get_msg_engine(3)));
            _engine[4].Set_msg(&(_uart->get_msg_engine(4)));
            _engine[5].Set_msg(&(_uart->get_msg_engine(5)));
            _engine[6].Set_msg(&(_uart->get_msg_engine(6)));
        }
    }
}

void UserEngines::Update()
{
    update_uart();
    update_engine();
    update_state();
    update_output();
}

void UserEngines::update_uart()
{
    _uart->read();
}

void UserEngines::set_state(UserEngines::UserEnginesState in_state)
{
    if (_state == in_state) {return;}
    _state = in_state;
    _last_state_ms = millis();
    switch (_state) {
        case UserEnginesState::Start:
            for (uint8_t i_engine = 0; i_engine < ENGINE_NUM; i_engine++) {
                _engine[i_engine].boost();
            }
            break;
        case UserEnginesState::Stop:
            for (uint8_t i_engine = 0; i_engine < ENGINE_NUM; i_engine++) {
                _engine[i_engine].brake();
            }
            break;
        default:
            break;
    }
}

bool UserEngines::is_state(UserEngines::UserEnginesState in_state)
{
    return (_state == in_state);
}

void UserEngines::update_engine()
{
    for (uint8_t i_engine = 0; i_engine < ENGINE_NUM; i_engine++) {
        _engine[i_engine].Update();
    }
    // must put behind engine.Update() to set need_send flag true
    if (_uart->initialized()) {
        engine_msg_pack();
    }
}

void UserEngines::update_state()
{
    uint32_t tnow = millis();
    if (_state == UserEnginesState::Start)
    {
        if (tnow - _last_state_ms < 5000) {
            for (uint8_t i_engine = 0; i_engine < ENGINE_NUM; i_engine++) {
                if (!_engine[i_engine].is_state(UserEngine::EngineState::Running))
                {
                    copter.ap.motor_interlock_switch = false;
                    return;
                }
            }
            copter.ap.motor_interlock_switch = true; //allow output to motors
        } else {
            set_state(UserEnginesState::None);
        }
    }

    if (_state == UserEnginesState::Stop)
    {
        copter.ap.motor_interlock_switch = false;
        if (tnow - _last_state_ms > 5000) {
            set_state(UserEnginesState::None);
        }
    }
}

void UserEngines::update_output() // call at 400 Hz
{
    for (uint8_t i_engine = 0; i_engine < ENGINE_NUM; i_engine++) {
        if (_engine[i_engine].can_override() && copter.ap.motor_interlock_switch) {
            float delta_throttle = 2000 - copter.g2.user_parameters.thr_low.get();
            _output[i_engine] = constrain_int16(copter.g2.user_parameters.thr_low + (int16_t)(copter.motors->get_throttle_out()*delta_throttle), copter.g2.user_parameters.thr_low, 2000);
        } else {
            _output[i_engine] = _engine[i_engine].get_output(); // the engine is in boost or brake procedures, output are pre-set in time order with uart state feedback
        }
        SRV_Channels::set_output_pwm(_engine[i_engine].get_srv_function(), _output[i_engine]);
    }
}

void UserEngines::engine_msg_pack()
{
    for (uint8_t i_engine = 0; i_engine < ENGINE_NUM; i_engine++) {
        uint8_t *temp_data = _engine[i_engine].Get_msg()->_msg_1.content.data;
        memcpy((engine_msg+i_engine*5), temp_data+2, 5);
        need_send |= _engine[i_engine].Get_msg()->_msg_1.need_send;
        _engine[i_engine].Get_msg()->_msg_1.need_send = false;
    }
    // if (_uart->initialized() && need_send) {
    //     _uart->get_port()->write(engine_msg, sizeof(engine_msg));
    // }
}
