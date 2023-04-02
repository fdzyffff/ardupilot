#include "Copter.h"

void UserEngines::Init()
{
	_engine[0].Init(AP_SerialManager::SerialProtocol_Engine_1, SRV_Channel::k_engine_1);
	_engine[1].Init(AP_SerialManager::SerialProtocol_Engine_2, SRV_Channel::k_engine_2);
	_engine[2].Init(AP_SerialManager::SerialProtocol_Engine_3, SRV_Channel::k_engine_3);
	_engine[3].Init(AP_SerialManager::SerialProtocol_Engine_4, SRV_Channel::k_engine_4);
	_engine[4].Init(AP_SerialManager::SerialProtocol_Engine_5, SRV_Channel::k_engine_5);
	_engine[5].Init(AP_SerialManager::SerialProtocol_Engine_6, SRV_Channel::k_engine_6);
	_engine[6].Init(AP_SerialManager::SerialProtocol_Engine_7, SRV_Channel::k_engine_7);

	set_state(UserEnginesState::None);
	copter.ap.motor_interlock_switch = false;
}

void UserEngines::Update()
{
	update_state();
	update_output();

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

void UserEngines::update_state()
{
	for (uint8_t i_engine = 0; i_engine < ENGINE_NUM; i_engine++) {
		_engine[i_engine].Update();
	}

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
