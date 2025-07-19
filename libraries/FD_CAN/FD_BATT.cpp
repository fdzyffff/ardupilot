#include "FD_BATT.h"

#include <AP_AHRS/AP_AHRS.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Logger/AP_Logger.h>
#include <GCS_MAVLink/GCS.h>
#include <SRV_Channel/SRV_Channel.h>

extern const AP_HAL::HAL &hal;


FD_BATT::FD_BATT(FD_CAN *frotend) {
    _frotend_ptr = frotend;
}

void FD_BATT::handle_info(AP_HAL::CANFrame &in_frame, bool do_print) {
    if (in_frame.id != (0x580+status.id)) {
        return;
    }
    if (in_frame.data[0] == 0x60
     && in_frame.data[1] == 0x00
     && in_frame.data[2] == 0x30
     && in_frame.data[3] == 0x00
     && in_frame.data[4] == 0x00
     && in_frame.data[5] == 0x00
     && in_frame.data[6] == 0x00
     && in_frame.data[7] == 0x00) {
        status.brake_confirm = true;
    }
    if (in_frame.data[0] == 0x60
     && in_frame.data[1] == 0x00
     && in_frame.data[2] == 0x30
     && in_frame.data[3] == 0x00
     && in_frame.data[4] == 0x00
     && in_frame.data[5] == 0x00
     && in_frame.data[6] == 0x00
     && in_frame.data[7] == 0x00) {
        status.brake_confirm = true;
    }

}

void FD_BATT::void set_pos(float pos_in)
{
    status.pos = constrain_float(pos_in, -45.0f, 45.0f);
}

void FD_BATT::void set_brake(bool brake_in)
{
    status.brake = brake_in;
    status.brake_confirm = false;
}

void FD_BATT::void set_id(uint8_t id_in)
{
    status.id = id_in;
}


void FD_BATT::void update_cmd()
{
    // send confirm
    if (!status.brake_confirm) {
        if (millis() - status.last_brake_ms > 100) {
            status.last_brake_ms = millis();
            _data[0] = 0x22;
            _data[1] = 0x0A;
            _data[2] = 0x30;
            _data[3] = 0x00;
            _data[4] = status.brake?0x00:0x01;
            _data[5] = 0x00;
            _data[6] = 0x00;
            _data[7] = 0x00;
            send_cmd(0x600+status.id);
        }
    }

    // set position
    {
        if (millis() - status.last_set_pos_ms > 100) {
            status.last_set_pos_ms = millis();
            int16_t tmp_pos = (int16_t)(status.pos*100.f);
            _data[0] = 0x22;
            _data[1] = 0x03;
            _data[2] = 0x60;
            _data[3] = 0x00;
            _data[4] = (uint8_t)(tmp_pos&0xff);
            _data[5] = (uint8_t)((tmp_pos>>8)&0xff);
            _data[6] = 0x00;
            _data[7] = 0x00;
            send_cmd(0x600+status.id);
        }
    }

    // ask status
    {
        if (millis() - status.last_ask_status_ms > 100) {
            status.last_ask_status_ms = millis();
            int16_t tmp_pos = (int16_t)(status.pos*100.f);
            _data[0] = 0x40;
            _data[1] = 0x01;
            _data[2] = 0x60;
            _data[3] = 0x00;
            _data[4] = 0x00;
            _data[5] = 0x00;
            _data[6] = 0x00;
            _data[7] = 0x00;
            send_cmd(0x600+status.id);
            status.ask_send = true;
        }
    }

}

void FD_BATT::send_cmd(uint32_t id, uint8_t *data) {
    if (_frotend_ptr == nullptr) {return;}
    const uint8_t data_length = 8;
    AP_HAL::CANFrame txFrame{};
    memcpy(txFrame.data, data, data_length);
    txFrame.id = id;
    _frotend_ptr->write_frame(txFrame, 0);
}
