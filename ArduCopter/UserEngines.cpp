#include "Copter.h"

UserEngines::update_state()
{
	for (uint8_t i_engine = 0; i_engine < ENGINE_NUM; i_engine++) {
		_engine[i_engine].update();
	}

	uint32_t tnow = millis();
	if (_state == UserEnginesState::Start)
	{
		if (tnow - last_state_ms < 5000) {
			bool all_start = true;
			for (uint8_t i_engine = 0; i_engine < ENGINE_NUM; i_engine++) {
				if (!_engine[i_engine].IsState(UserEngine::EngineState::Running))
				{
					all_start = false;
					break;
				}
			}
			copter.ap.motor_interlock_switch = true;
		} else {
			_state = UserEnginesState::None;
		}

	}

	if (_state == UserEnginesState::Stop)
	{
		copter.ap.motor_interlock_switch = false;
		if (tnow - last_state_ms > 5000) {
			_state = UserEnginesState::None;
		}
	}
}

UserEngines::set_state(UserEnginesState in_state)
{
	_state = in_state;
	switch (_state) {
		case Start:
			for (uint8_t i_engine = 0; i_engine < ENGINE_NUM; i_engine++) {
				if (!_engine[i_engine].IsState(UserEngine::EngineState::Running)) 
				{
					if (_engine[i_engine].thrPos == 0) {
						_engine[i_engine].Boost()
					}
				}
			}
			break;
		default:
			break;
	}
}

UserEngines::update_output()
{
	switch (_state) {
		case Start:
			for (uint8_t i_engine = 0; i_engine < ENGINE_NUM; i_engine++) {
				if (!_engine[i_engine].IsState(UserEngine::EngineState::Running)) 
				{
					if (_engine[i_engine].thrPos == 0) {
						_output[i_engine] = 1000; //pwm
					} else {
						_output[i_engine] = _engine[i_engine].get_output();
					}
				}
			}
			break;
		case Stop:
			for (uint8_t i_engine = 0; i_engine < ENGINE_NUM; i_engine++) {
				if (_engine[i_engine].IsState(UserEngine::EngineState::Running)) 
				{
					_output[i_engine] = _engine[i_engine].get_output_min();
				}
			}
			break;
		}
		default:
			for (uint8_t i_engine = 0; i_engine < ENGINE_NUM; i_engine++) {
				_output[i_engine] = 1000 + (int16_t)(copter.motors->get_throttle_out()*1000.0f);
			}
			break;

}
