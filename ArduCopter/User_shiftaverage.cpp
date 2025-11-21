#include "Copter.h"

void User_shiftaverage::init(float step_ms, uint8_t data_max)
{
    _step_ms = step_ms;

    if (_step_ms < 5.0f ) {
        _step_ms = 5.0f;
    }
    _data_max = MIN(data_max, NUM_MY_DATAMAX);
    _data_max = MAX(data_max, 1);
    _idx = 0;
    _data_length = 0;
    _last_push_ms = 0;
    _active = true;
}

void User_shiftaverage::push(float value)
{
    if (!_active) {return;}
    if (millis() - _last_push_ms > (uint32_t)_step_ms) {
        _last_push_ms = millis();
        _idx++;
        _data_length++;
        if (_idx >= _data_max) {
            _idx = 0;
        }
        if (_data_length >= _data_max) {
            _data_length = _data_max;
        }
        _data[_idx] = value;
    }
}

float User_shiftaverage::get()
{
    if (!_active) {return 0.0f;}
    if (_data_length == 0) {
        return 0.0f;
    }

    float sum = 0.0f;
    for (uint8_t i_data = 0; i_data < _data_length; i_data++) {
        sum += _data[i_data];
    }
    float ret = sum/(float)_data_length;
    return ret;
}
