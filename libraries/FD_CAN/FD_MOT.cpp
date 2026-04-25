#include "FD_MOT.h"

#include <AP_AHRS/AP_AHRS.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Logger/AP_Logger.h>
#include <GCS_MAVLink/GCS.h>
#include <SRV_Channel/SRV_Channel.h>
#include <FD_DATA/FD_DATA.h>

extern const AP_HAL::HAL &hal;

FD_MOT::FD_MOT(FD_CAN *frotend) {
    _frotend_ptr = frotend;
}

void FD_MOT::handle_info(AP_HAL::CANFrame &in_frame, bool do_print) {
    bool have_rpm = false;
    uint16_t group = 0;
    uint16_t order = 0;
    if (!get_rpm_group(group, order)) {
        return;
    }
    if (in_frame.id == ((0x189D271C + ((group - 1) * 0x10000))| AP_HAL::CANFrame::FlagEFF)) {
        switch(order) {
        case 1:
            status.rpm = (uint16_t)in_frame.data[1] | (uint16_t)in_frame.data[0] << 8 ; // 0~2000
            have_rpm = true;
            break;
        case 2:
            status.rpm = (uint16_t)in_frame.data[3] | (uint16_t)in_frame.data[2] << 8 ; // 0~2000
            have_rpm = true;
            break;
        case 3:
            status.rpm = (uint16_t)in_frame.data[5] | (uint16_t)in_frame.data[4] << 8 ; // 0~2000
            have_rpm = true;
            break;
        case 4:
            status.rpm = (uint16_t)in_frame.data[7] | (uint16_t)in_frame.data[6] << 8 ; // 0~2000
            have_rpm = true;
            break;
        default:
            break;
        }
    }

    if (have_rpm && status.id <= 16) {
        status.last_rpm_ms = AP_HAL::millis();
        // AP::fd_data().motor_rpm_packet.motor_rpm[status.id-1] = status.rpm;
    }

    bool have_temp = false;
    if (in_frame.id == ((0x18A1271C + ((group - 1) * 0x10000))| AP_HAL::CANFrame::FlagEFF)) {
        switch(order) {
        case 1:
            status.temp = (uint16_t)in_frame.data[1] | (uint16_t)in_frame.data[0] << 8 ; // 0~2000
            have_temp = true;
            break;
        case 2:
            status.temp = (uint16_t)in_frame.data[3] | (uint16_t)in_frame.data[2] << 8 ; // 0~2000
            have_temp = true;
            break;
        case 3:
            status.temp = (uint16_t)in_frame.data[5] | (uint16_t)in_frame.data[4] << 8 ; // 0~2000
            have_temp = true;
            break;
        case 4:
            status.temp = (uint16_t)in_frame.data[7] | (uint16_t)in_frame.data[6] << 8 ; // 0~2000
            have_temp = true;
            break;
        default:
            break;
        }
    }

    if (have_temp && status.id <= 16) {
        status.last_temp_ms = AP_HAL::millis();
        // AP::fd_data().motor_rpm_packet.motor_temp[status.id-1] = status.temp;
    }

}

bool FD_MOT::get_rpm_group(uint16_t &group, uint16_t &order)
{
    bool ret = false;
    if ((1<=status.id) && (status.id<=4)) {
        group = 1;
        ret = true;
    }
    if ((5<=status.id) && (status.id<=8)) {
        group = 2;
        ret = true;
    }
    if ((9<=status.id) && (status.id<=12)) {
        group = 3;
        ret = true;
    }
    if ((13<=status.id) && (status.id<=16)) {
        group = 4;
        ret = true;
    }

    order = status.id % 4;
    if (order == 0) {
        order = 4;
    }

    return ret;
}

bool FD_MOT::get_temp_group(uint16_t &group, uint16_t &order)
{
    return get_rpm_group(group, order);
}

bool FD_MOT::get_throttle_group(uint16_t &group, uint16_t &order)
{
    bool ret = false;
    if ((1<=status.id) && (status.id<=2)) {
        group = 1;
        ret = true;
    }
    if ((3<=status.id) && (status.id<=4)) {
        group = 2;
        ret = true;
    }
    if ((5<=status.id) && (status.id<=6)) {
        group = 3;
        ret = true;
    }
    if ((7<=status.id) && (status.id<=8)) {
        group = 4;
        ret = true;
    }
    if ((9<=status.id) && (status.id<=10)) {
        group = 5;
        ret = true;
    }
    if ((11<=status.id) && (status.id<=12)) {
        group = 6;
        ret = true;
    }
    if ((13<=status.id) && (status.id<=14)) {
        group = 7;
        ret = true;
    }
    if ((15<=status.id) && (status.id<=16)) {
        group = 8;
        ret = true;
    }

    order = status.id % 2;
    if (order == 0) {
        order = 2;
    }

    return ret;
}

void FD_MOT::set_id(uint8_t id_in)
{
    status.id = id_in;
    uint16_t group = 0;
    uint16_t order = 0;
    if (!get_throttle_group(group, order)) {
        gcs().send_text(MAV_SEVERITY_INFO, "CAN MOT: no thr group/order for id %d", status.id);
    }
    if (!get_rpm_group(group, order)) {
        gcs().send_text(MAV_SEVERITY_INFO, "CAN MOT: no rpm group/order for id %d", status.id);
    }
}

void FD_MOT::set_pwm(uint16_t pwm_in) // 1000~2000
{
    status.thr_in = pwm_in;
}

void FD_MOT::update()
{
    update_cmd();
    update_status();
}

void FD_MOT::update_status()
{
    // send mot cmd
    {
        if (AP_HAL::millis() - status.last_status_ms > 500) {
            status.last_status_ms = AP_HAL::millis();
        }
    }
}

void FD_MOT::update_cmd()
{
    // send mot cmd
    {
        if (AP_HAL::millis() - status.last_mot_ms > 20) {
            status.last_mot_ms = AP_HAL::millis();
            uint16_t tmp_thr = status.thr_in;

            _data[0] = 0xFF;
            _data[1] = 0xFF;
            _data[2] = 0xFF;
            _data[3] = 0xFF;
            _data[4] = 0xFF;
            _data[5] = 0xFF;
            _data[6] = 0xFF;
            _data[7] = 0xFF;

            uint16_t group = 0;
            uint16_t order = 0;
            if (!get_throttle_group(group, order)) {
                return;
            }

            _data[0] = (uint8_t)((tmp_thr>>8)&0xff);
            _data[1] = (uint8_t)(tmp_thr&0xff);
            _data[2] = (uint8_t)((tmp_thr>>8)&0xff);
            _data[3] = (uint8_t)(tmp_thr&0xff);
            // switch(order) {
            // case 1:
            //     _data[0] = (uint8_t)((tmp_thr>>8)&0xff);
            //     _data[1] = (uint8_t)(tmp_thr&0xff);
            //     break;
            // case 2:
            //     _data[2] = (uint8_t)((tmp_thr>>8)&0xff);
            //     _data[3] = (uint8_t)(tmp_thr&0xff);
            //     break;
            // case 3:
            //     _data[4] = (uint8_t)((tmp_thr>>8)&0xff);
            //     _data[5] = (uint8_t)(tmp_thr&0xff);
            //     break;
            // case 4:
            //     _data[6] = (uint8_t)((tmp_thr>>8)&0xff);
            //     _data[7] = (uint8_t)(tmp_thr&0xff);
            //     break;
            // default:
            //     break;
            // }

            uint32_t target_addr = 0x14651C27 + (group * 0x10000);
            send_cmd(target_addr | AP_HAL::CANFrame::FlagEFF, _data);
            // send_cmd(target_addr, _data);
        }
    }
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
