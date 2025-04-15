#include "FD_Uartpass_buffer.h"

void FD_Uartpass_buffer::set_active()
{
    if (!_active) {
        gcs().send_text(MAV_SEVERITY_INFO, "Fwd chan(%d) connect", _id);
    }
    _active = true;
    _last_active_ms = millis();
}

void FD_Uartpass_buffer::update() 
{
    uint32_t tnow = millis();
    if (tnow - _last_active_ms > 3000) {
        if (_active) {
            gcs().send_text(MAV_SEVERITY_INFO, "Fwd chan(%d) disconnect", _id);
            _active = false;
        }
    }
}

//  - - - data_idx - - -
//  a a a 0        0 0 0
uint16_t FD_Uartpass_buffer::get_data(uint8_t (&data)[NUM_MY_DATALEN])
{
    // gcs().send_text(MAV_SEVERITY_INFO, "data_idx %d", data_idx);
    uint16_t avaliable_data_len = MIN(data_idx, NUM_MY_DATALEN-1);
    memcpy(data, _data, avaliable_data_len);
    data_idx -= (avaliable_data_len);
    // re-organize data, move rest data to 0 positon
    for (uint8_t i=0; i<data_idx; i++) {
        _data[i] = _data[i+avaliable_data_len];
    }
    return avaliable_data_len;
} 

void FD_Uartpass_buffer::push(uint8_t c)
{
    if (!_active) {return;}
    static uint32_t _last_log_ms = 0;
    if (data_idx < NUM_MY_DATALEN-1) {
        _data[data_idx] = c;
        data_idx++;
    } else {
        uint32_t tnow = millis();
        if (tnow - _last_log_ms > 5000) {
            gcs().send_text(MAV_SEVERITY_INFO, "Fwd chan(%d) is full", _id);
            _last_log_ms = tnow;
        }
    }
}