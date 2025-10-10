#include "FD_SERVO.h"

#include <AP_AHRS/AP_AHRS.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Logger/AP_Logger.h>
#include <GCS_MAVLink/GCS.h>
#include <SRV_Channel/SRV_Channel.h>

extern const AP_HAL::HAL &hal;

FD_SERVO::FD_SERVO(FD_CAN_1 *frotend) {
    _frotend_ptr = frotend;
    status.brake_confirm = false;
}

void FD_SERVO::handle_info(AP_HAL::CANFrame &in_frame, bool do_print) {
    // if (in_frame.id == (0x580+status.id)) {
    //     if (in_frame.data[0] == 0x60
    //      && in_frame.data[1] == 0x00
    //      && in_frame.data[2] == 0x30
    //      && in_frame.data[3] == 0x00
    //      && in_frame.data[4] == 0x00
    //      && in_frame.data[5] == 0x00
    //      && in_frame.data[6] == 0x00
    //      && in_frame.data[7] == 0x00) {
    //         status.brake_confirm = true;
    //     }
    // }
    if (in_frame.id == (0x480+status.id)) {
        status.AngleFb = (uint16_t)in_frame.data[0] | (uint16_t)(in_frame.data[1]<<8);
        status.AngleCtrl = (uint16_t)in_frame.data[2] | (uint16_t)(in_frame.data[3]<<8);
        status.Current = in_frame.data[4];
        status.Voltage = in_frame.data[5];
        status.SelfCheckState = (uint16_t)in_frame.data[6] | (uint16_t)(in_frame.data[7]<<8);
        if (status.SelfCheckState & 0b0000001000000000) {
            status.brake = true;
        }
        if (status.SelfCheckState & 0b0000000100000000) {
            status.brake = false;
        }
        if (status.target_brake == status.brake){
            status.brake_confirm = true;
        }
        //gcs().send_text(MAV_SEVERITY_INFO, "ID:%lx AngleFb:%u AngleCtrl:%u", in_frame.id,status.AngleFb,status.AngleCtrl);
    }

}

void FD_SERVO::set_pos(float pos_in)
{
    status.pos = constrain_float(pos_in, -45.0f, 45.0f);//////
    status.last_pos = status.pos;
    status.last_pos_ms = AP_HAL::millis();
}

void FD_SERVO::set_brake(bool brake_in)
{
    status.target_brake = brake_in;
    status.brake_confirm = false;
}

void FD_SERVO::enable_brake(bool enable)
{
    status.have_brake = enable;
    if (!status.have_brake) {
        if (!status.brake_confirm || status.target_brake) {
            status.target_brake = false;
            status.brake_confirm = false;
        }
    }
}

bool FD_SERVO::get_brake()
{
    return status.brake;
}

void FD_SERVO::set_id(uint8_t id_in)
{
    status.id = id_in;
}

void FD_SERVO::update()
{
    if (status.have_brake) {
        update_cmd_brake();
    } else {
        update_cmd_nobrake();
    }
}

void FD_SERVO::update_cmd_nobrake()
{
    // send confirm
    if (!status.brake_confirm) {
        if (AP_HAL::millis() - status.last_brake_ms > 100) {
            status.last_brake_ms = AP_HAL::millis();
            _data[0] = 0x22;
            _data[1] = 0x0A;
            _data[2] = 0x30;
            _data[3] = 0x00;
            _data[4] = status.target_brake?0x00:0x01;
            _data[5] = 0x00;
            _data[6] = 0x00;
            _data[7] = 0x00;
            send_cmd(0x600+status.id, _data);
        }
    }

    // set position
    {
        if (AP_HAL::millis() - status.last_send_pos_ms > 5) {
            status.last_send_pos_ms = AP_HAL::millis();
            int16_t tmp_pos = (int16_t)(status.pos*100.f);
            _data[0] = 0x22;
            _data[1] = 0x03;
            _data[2] = 0x60;
            _data[3] = 0x00;
            _data[4] = (uint8_t)(tmp_pos&0xff);
            _data[5] = (uint8_t)((tmp_pos>>8)&0xff);
            _data[6] = 0x00;
            _data[7] = 0x00;
            send_cmd(0x600+status.id, _data);
        }
    }

    // ask status
    {
        if (AP_HAL::millis() - status.last_ask_status_ms > 100) {
            status.last_ask_status_ms = AP_HAL::millis();
            _data[0] = 0x40;
            _data[1] = 0x01;
            _data[2] = 0x60;
            _data[3] = 0x00;
            _data[4] = 0x00;
            _data[5] = 0x00;
            _data[6] = 0x00;
            _data[7] = 0x00;
            send_cmd(0x500+status.id, _data);
        }
    }

}

void FD_SERVO::update_cmd_brake()
{
    // send confirm
    if (!status.brake_confirm) {
        if (AP_HAL::millis() - status.last_brake_ms > 100) {
            status.last_brake_ms = AP_HAL::millis();
            _data[0] = 0x22;
            _data[1] = 0x0A;
            _data[2] = 0x30;
            _data[3] = 0x00;
            _data[4] = status.target_brake?0x00:0x01;
            _data[5] = 0x00;
            _data[6] = 0x00;
            _data[7] = 0x00;
            send_cmd(0x600+status.id, _data);
        }
        // set position
        if (AP_HAL::millis() - status.last_send_pos_ms > 5) {
            status.last_send_pos_ms = AP_HAL::millis();
            int16_t tmp_pos = (int16_t)(status.pos*100.f);
            _data[0] = 0x22;
            _data[1] = 0x03;
            _data[2] = 0x60;
            _data[3] = 0x00;
            _data[4] = (uint8_t)(tmp_pos&0xff);
            _data[5] = (uint8_t)((tmp_pos>>8)&0xff);
            _data[6] = 0x00;
            _data[7] = 0x00;
            send_cmd(0x600+status.id, _data);
        }
    } else {
        if (!status.brake) {
            if (AP_HAL::millis() - status.last_send_pos_ms > 5) {
                status.last_send_pos_ms = AP_HAL::millis();
                int16_t tmp_pos = (int16_t)(status.pos*100.f);
                _data[0] = 0x22;
                _data[1] = 0x03;
                _data[2] = 0x60;
                _data[3] = 0x00;
                _data[4] = (uint8_t)(tmp_pos&0xff);
                _data[5] = (uint8_t)((tmp_pos>>8)&0xff);
                _data[6] = 0x00;
                _data[7] = 0x00;
                send_cmd(0x600+status.id, _data);
            }
        }
    }

    // ask status
    {
        if (AP_HAL::millis() - status.last_ask_status_ms > 100) {
            status.last_ask_status_ms = AP_HAL::millis();
            _data[0] = 0x40;
            _data[1] = 0x01;
            _data[2] = 0x60;
            _data[3] = 0x00;
            _data[4] = 0x00;
            _data[5] = 0x00;
            _data[6] = 0x00;
            _data[7] = 0x00;
            send_cmd(0x500+status.id, _data);
        }
    }

}

void FD_SERVO::send_cmd(uint32_t id, uint8_t *data) {
    if (_frotend_ptr == nullptr) {return;}
    const uint8_t data_length = 8;
    AP_HAL::CANFrame txFrame{};
    memcpy(txFrame.data, data, data_length);
    txFrame.id = id;
    txFrame.dlc = 8;
    uint64_t timeout = AP_HAL::micros64() + 10000ULL;
    _frotend_ptr->write_frame(txFrame, timeout);
}
