#include "Plane.h"

UDelay::UDelay()
{
    ;
}
    
void UDelay::init()
{
    _idx = 0;
    for (uint16_t i = 0; i < UDELAY_BUFFER; i++) {
        _buffer[i].roll = 0.0f;
        _buffer[i].pitch = 0.0f;
        _buffer[i].yaw = 0.0f;
        _buffer[i].time_ms = 0;
    }
}

void UDelay::push()
{
    _idx += 1;
    if (_idx >= UDELAY_BUFFER) {
        _idx = 0;
    }
    _buffer[_idx].roll = plane.ahrs.roll;
    _buffer[_idx].pitch = plane.ahrs.pitch;
    _buffer[_idx].yaw = plane.ahrs.yaw;
    _buffer[_idx].time_ms = millis();
}

bool UDelay::get_idx(uint16_t step, float &roll, float &pitch, float &yaw) 
{
    uint16_t this_idx = 0;
    if (_idx >= step) {
        this_idx = _idx - step;
    } else {
        this_idx = UDELAY_BUFFER + _idx - step;
    }
    roll = _buffer[this_idx].roll;
    pitch = _buffer[this_idx].pitch;
    yaw = _buffer[this_idx].yaw;
    // gcs().send_text(MAV_SEVERITY_INFO, "%d", (millis()-_buffer[this_idx].time_ms));
    return true;
}
