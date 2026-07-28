#include "UDelay.h"

UDelay::UDelay() :
    _write_index(0),
    _sample_count(0)
{
}

void UDelay::init()
{
    _write_index = 0;
    _sample_count = 0;
    for (uint16_t i = 0; i < BUFFER_SIZE; i++) {
        _buffer[i].value.zero();
        _buffer[i].time_ms = 0;
        _buffer[i].valid = false;
    }
}

void UDelay::push(const Vector3f &value)
{
    _buffer[_write_index].value = value;
    _buffer[_write_index].time_ms = AP_HAL::millis();
    _buffer[_write_index].valid = true;

    _write_index++;
    if (_write_index >= BUFFER_SIZE) {
        _write_index = 0;
    }
    if (_sample_count < BUFFER_SIZE) {
        _sample_count++;
    }
}

bool UDelay::get_idx(uint16_t step, Vector3f &value) const
{
    if ((step >= BUFFER_SIZE) || (step >= _sample_count)) {
        return false;
    }

    const uint16_t newest_index = (_write_index == 0) ? (BUFFER_SIZE - 1) : (_write_index - 1);
    const uint16_t read_index = (newest_index >= step) ?
        (newest_index - step) :
        (BUFFER_SIZE + newest_index - step);

    if (!_buffer[read_index].valid) {
        return false;
    }

    const uint32_t sample_age_ms = AP_HAL::millis() - _buffer[read_index].time_ms;
    if (sample_age_ms > 2500U) {
        return false;
    }

    value = _buffer[read_index].value;
    return true;
}
