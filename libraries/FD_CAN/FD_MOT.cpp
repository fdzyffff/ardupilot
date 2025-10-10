#include "FD_MOT.h"

#include <AP_AHRS/AP_AHRS.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Logger/AP_Logger.h>
#include <GCS_MAVLink/GCS.h>
#include <SRV_Channel/SRV_Channel.h>

extern const AP_HAL::HAL &hal;

FD_MOT::FD_MOT(FD_CAN_1 *frotend) {
    _frotend_ptr = frotend;
}

void FD_MOT::handle_info(AP_HAL::CANFrame &in_frame, bool do_print) {
    if (in_frame.id == (0x80+status.id)) {
        status.mode_out = in_frame.data[0]&0b00000111;
        uint16_t tmp_rpm_out = (uint16_t)in_frame.data[1] | (uint16_t)(in_frame.data[2]<<8);
        status.rpm_out = (int16_t)(constrain_int32((int32_t)tmp_rpm_out-30000, -32767, 32767));
    }
    if (in_frame.id == (0x200+status.id)) {
        status.mode_out = in_frame.data[0]&0b00000111;
        uint16_t tmp_pitch_out = (uint16_t)in_frame.data[1] | (uint16_t)(in_frame.data[2]<<8);
        status.pitch_out = constrain_float(((float)tmp_pitch_out)/65535.f - 90.f, -90.f, 90.f);
    }
}

void FD_MOT::set_id(uint8_t id_in)
{
    status.id = id_in;
}

void FD_MOT::set_mode(uint8_t mode_in)
{
    status.mode_in = mode_in;
}

void FD_MOT::set_rpm(int16_t rpm_in)
{
    status.rpm_in = rpm_in;
}

void FD_MOT::set_pitch(float pitch_in)
{
    status.pitch_in = pitch_in;
}

void FD_MOT::update()
{
    update_cmd();
}

void FD_MOT::update_cmd()
{
    // send mot cmd
    {
        if (AP_HAL::millis() - status.last_mot_ms > 20) {
            status.last_mot_ms = AP_HAL::millis();
            uint16_t tmp_rpm = (constrain_int32((int32_t)(status.rpm_in + 30000), 0, 65535));
            _data[0] = status.mode_in&0b00000111;
            _data[1] = 0x00;
            _data[2] = 0x00;
            _data[3] = (uint8_t)(tmp_rpm&0xff);
            _data[4] = (uint8_t)((tmp_rpm>>8)&0xff);
            _data[5] = 0x00;
            _data[6] = (uint8_t)((status.send_count++)<<4);
            _data[7] = 0x00;
            sumcheck();
            send_cmd(0x70+status.id, _data);
        }
    }
    // send pitch cmd
    {
        if (AP_HAL::millis() - status.last_pitch_ms > 50) {
            status.last_pitch_ms = AP_HAL::millis();
            uint16_t tmp_pitch = (uint16_t)(constrain_float(status.pitch_in + 90.f, 0.f, 180.f) * (65535.f-1.0f)/180.f);
            _data[0] = (uint8_t)(tmp_pitch&0xff);
            _data[1] = (uint8_t)((tmp_pitch>>8)&0xff);
            _data[2] = 0x00;
            _data[3] = 0x00;
            _data[4] = 0x00;
            _data[5] = 0x00;
            _data[6] = 0x00;
            _data[7] = 0x00;
            send_cmd(0x400+status.id, _data);
        }
    }
}

void FD_MOT::sumcheck()
{
    uint8_t i, j;
    uint8_t u8_crc8;
    uint8_t u8_poly;
    u8_crc8 = 0xFF;
    u8_poly = 0x1D;
    for(i=0; i<7; i++) {
        u8_crc8 ^= _data[i];
        for(j=0;j<8;j++) {
            if (u8_crc8&0x80) {
                u8_crc8 = (u8_crc8<<1)^u8_poly;
            } else {
                u8_crc8<<=1;
            }
        } 
    }
    u8_crc8 ^= (uint8_t)0xFF;
    _data[7] = u8_crc8;
}

void FD_MOT::send_cmd(uint32_t id, uint8_t *data) {
    if (_frotend_ptr == nullptr) {return;}
    const uint8_t data_length = 8;
    AP_HAL::CANFrame txFrame{};
    memcpy(txFrame.data, data, data_length);
    txFrame.id = id;
    txFrame.dlc = 8;
    uint64_t timeout = AP_HAL::micros64() + 10000ULL;
    _frotend_ptr->write_frame(txFrame, timeout);
}
