#include "FD_MOT.h"

#include <AP_AHRS/AP_AHRS.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Logger/AP_Logger.h>
#include <SRV_Channel/SRV_Channel.h>
#include <FD_DATA/FD_DATA.h>

extern const AP_HAL::HAL &hal;

FD_MOT::FD_MOT(FD_CAN *frotend) {
    _frotend_ptr = frotend;
    init();
}


void FD_MOT::init()
{
    for (uint8_t i_mot = 0; i_mot < FD_CAN_MAX_MOT_NUM; i_mot++) {
        ;
    }
    // status[i_mot].id = i_mot + 1;
}

void FD_MOT::handle_info(AP_HAL::CANFrame &in_frame, bool do_print) 
{
    {
        uint32_t address = 0x189D271C;
        if (in_frame.id == (address | AP_HAL::CANFrame::FlagEFF)) {
            status[0].rpm1 = (uint16_t)in_frame.data[1] | (uint16_t)in_frame.data[0] << 8 ; // 0~2000
            status[0].rpm2 = (uint16_t)in_frame.data[3] | (uint16_t)in_frame.data[2] << 8 ; // 0~2000
            status[0].last_rpm_ms = AP_HAL::millis();
            status[1].rpm1 = (uint16_t)in_frame.data[5] | (uint16_t)in_frame.data[4] << 8 ; // 0~2000
            status[1].rpm2 = (uint16_t)in_frame.data[7] | (uint16_t)in_frame.data[6] << 8 ; // 0~2000
            status[1].last_rpm_ms = AP_HAL::millis();
        }
    }
    {
        uint32_t address = 0x189E271C;
        if (in_frame.id == (address | AP_HAL::CANFrame::FlagEFF)) {
            status[2].rpm1 = (uint16_t)in_frame.data[1] | (uint16_t)in_frame.data[0] << 8 ; // 0~2000
            status[2].rpm2 = (uint16_t)in_frame.data[3] | (uint16_t)in_frame.data[2] << 8 ; // 0~2000
            status[2].last_rpm_ms = AP_HAL::millis();
            status[3].rpm1 = (uint16_t)in_frame.data[5] | (uint16_t)in_frame.data[4] << 8 ; // 0~2000
            status[3].rpm2 = (uint16_t)in_frame.data[7] | (uint16_t)in_frame.data[6] << 8 ; // 0~2000
            status[3].last_rpm_ms = AP_HAL::millis();
        }
    }
    {
        uint32_t address = 0x189F271C;
        if (in_frame.id == (address | AP_HAL::CANFrame::FlagEFF)) {
            status[4].rpm1 = (uint16_t)in_frame.data[1] | (uint16_t)in_frame.data[0] << 8 ; // 0~2000
            status[4].rpm2 = (uint16_t)in_frame.data[3] | (uint16_t)in_frame.data[2] << 8 ; // 0~2000
            status[4].last_rpm_ms = AP_HAL::millis();
            status[5].rpm1 = (uint16_t)in_frame.data[5] | (uint16_t)in_frame.data[4] << 8 ; // 0~2000
            status[5].rpm2 = (uint16_t)in_frame.data[7] | (uint16_t)in_frame.data[6] << 8 ; // 0~2000
            status[5].last_rpm_ms = AP_HAL::millis();
        }
    }
    {
        uint32_t address = 0x18A0271C;
        if (in_frame.id == (address | AP_HAL::CANFrame::FlagEFF)) {
            status[6].rpm1 = (uint16_t)in_frame.data[1] | (uint16_t)in_frame.data[0] << 8 ; // 0~2000
            status[6].rpm2 = (uint16_t)in_frame.data[3] | (uint16_t)in_frame.data[2] << 8 ; // 0~2000
            status[6].last_rpm_ms = AP_HAL::millis();
            status[7].rpm1 = (uint16_t)in_frame.data[5] | (uint16_t)in_frame.data[4] << 8 ; // 0~2000
            status[7].rpm2 = (uint16_t)in_frame.data[7] | (uint16_t)in_frame.data[6] << 8 ; // 0~2000
            status[7].last_rpm_ms = AP_HAL::millis();
        }
    }

    // TEMP
    {
        uint32_t address = 0x18A1271C;
        if (in_frame.id == (address | AP_HAL::CANFrame::FlagEFF)) {
            status[0].temp1 = (uint16_t)in_frame.data[1] | (uint16_t)in_frame.data[0] << 8 ; // 0~2000
            status[0].temp2 = (uint16_t)in_frame.data[3] | (uint16_t)in_frame.data[2] << 8 ; // 0~2000
            status[0].last_temp_ms = AP_HAL::millis();
            status[1].temp1 = (uint16_t)in_frame.data[5] | (uint16_t)in_frame.data[4] << 8 ; // 0~2000
            status[1].temp2 = (uint16_t)in_frame.data[7] | (uint16_t)in_frame.data[6] << 8 ; // 0~2000
            status[1].last_temp_ms = AP_HAL::millis();
        }
    }
    {
        uint32_t address = 0x18A2271C;
        if (in_frame.id == (address | AP_HAL::CANFrame::FlagEFF)) {
            status[2].temp1 = (uint16_t)in_frame.data[1] | (uint16_t)in_frame.data[0] << 8 ; // 0~2000
            status[2].temp2 = (uint16_t)in_frame.data[3] | (uint16_t)in_frame.data[2] << 8 ; // 0~2000
            status[2].last_temp_ms = AP_HAL::millis();
            status[3].temp1 = (uint16_t)in_frame.data[5] | (uint16_t)in_frame.data[4] << 8 ; // 0~2000
            status[3].temp2 = (uint16_t)in_frame.data[7] | (uint16_t)in_frame.data[6] << 8 ; // 0~2000
            status[3].last_temp_ms = AP_HAL::millis();
        }
    }
    {
        uint32_t address = 0x18A3271C;
        if (in_frame.id == (address | AP_HAL::CANFrame::FlagEFF)) {
            status[4].temp1 = (uint16_t)in_frame.data[1] | (uint16_t)in_frame.data[0] << 8 ; // 0~2000
            status[4].temp2 = (uint16_t)in_frame.data[3] | (uint16_t)in_frame.data[2] << 8 ; // 0~2000
            status[4].last_temp_ms = AP_HAL::millis();
            status[5].temp1 = (uint16_t)in_frame.data[5] | (uint16_t)in_frame.data[4] << 8 ; // 0~2000
            status[5].temp2 = (uint16_t)in_frame.data[7] | (uint16_t)in_frame.data[6] << 8 ; // 0~2000
            status[5].last_temp_ms = AP_HAL::millis();
        }
    }
    {
        uint32_t address = 0x18A4271C;
        if (in_frame.id == (address | AP_HAL::CANFrame::FlagEFF)) {
            status[6].temp1 = (uint16_t)in_frame.data[1] | (uint16_t)in_frame.data[0] << 8 ; // 0~2000
            status[6].temp2 = (uint16_t)in_frame.data[3] | (uint16_t)in_frame.data[2] << 8 ; // 0~2000
            status[6].last_temp_ms = AP_HAL::millis();
            status[7].temp1 = (uint16_t)in_frame.data[5] | (uint16_t)in_frame.data[4] << 8 ; // 0~2000
            status[7].temp2 = (uint16_t)in_frame.data[7] | (uint16_t)in_frame.data[6] << 8 ; // 0~2000
            status[7].last_rpm_ms = AP_HAL::millis();
        }
    }
}

bool FD_MOT::get_throttle_address(uint8_t i_mot, uint32_t &address)
{
    bool ret = false;
    uint8_t id = i_mot+1;
    if ((1<=id) && (id<=FD_CAN_MAX_MOT_NUM)) {
        switch (id) {
            case 1:
                address = 0x14661C27;
                break;
            case 2:
                address = 0x14671C27;
                break;
            case 3:
                address = 0x14681C27;
                break;
            case 4:
                address = 0x14691C27;
                break;
            case 5:
                address = 0x14761C27;
                break;
            case 6:
                address = 0x14771C27;
                break;
            case 7:
                address = 0x14781C27;
                break;
            case 8:
                address = 0x14791C27;
                break;
        }
        ret = true;
    } else {
        address = 0;
        ret = false;
    }
    return ret;
}

void FD_MOT::set_pwm(uint8_t id_in, uint16_t pwm_in) // 1000~2000
{
    if (id_in < FD_CAN_MAX_MOT_NUM) {
        status[id_in].thr_in = pwm_in;
        status[id_in].have_thr = true;
    }
}

void FD_MOT::update()
{
    update_cmd();
    update_status();
}

void FD_MOT::update_status()
{
    // send mot cmd
    uint32_t t_now = AP_HAL::millis();
    for (uint8_t i_mot = 0; i_mot < FD_CAN_MAX_MOT_NUM; i_mot++) 
    {
        if (t_now - status[i_mot].last_status_ms > 500) {
            status[i_mot].last_status_ms = t_now;
        }

        if (t_now - status[i_mot].last_rpm_ms < 1000) {
            AP::fd_data().hxts_can_mot_info_packet.RPM[i_mot*2] = status[i_mot].rpm1;
            AP::fd_data().hxts_can_mot_info_packet.RPM[i_mot*2 + 1] = status[i_mot].rpm2;
        } else {
            AP::fd_data().hxts_can_mot_info_packet.RPM[i_mot*2] = 0xFFFF;
            AP::fd_data().hxts_can_mot_info_packet.RPM[i_mot*2 + 1] = 0xFFFF;
        }

        if (t_now - status[i_mot].last_rpm_ms < 1000) {
            AP::fd_data().hxts_can_mot_info_packet.TEMP[i_mot*2] = status[i_mot].temp1;
            AP::fd_data().hxts_can_mot_info_packet.TEMP[i_mot*2 + 1] = status[i_mot].temp2;
        } else {
            AP::fd_data().hxts_can_mot_info_packet.TEMP[i_mot*2] = 0xFFFF;
            AP::fd_data().hxts_can_mot_info_packet.TEMP[i_mot*2 + 1] = 0xFFFF;
        }
    }
}

void FD_MOT::update_cmd()
{
    // send mot cmd
    for (uint8_t i_mot = 0; i_mot < FD_CAN_MAX_MOT_NUM; i_mot++) 
    {
        if (AP_HAL::millis() - status[i_mot].last_mot_ms > 20) {
            status[i_mot].last_mot_ms = AP_HAL::millis();

            if (status[i_mot].have_thr) {
                status[i_mot].have_thr = false;
                uint16_t tmp_thr = status[i_mot].thr_in;

                _data[0] = 0xFF;
                _data[1] = 0xFF;
                _data[2] = 0xFF;
                _data[3] = 0xFF;
                _data[4] = 0xFF;
                _data[5] = 0xFF;
                _data[6] = 0xFF;
                _data[7] = 0xFF;

                uint32_t address = 0;
                if (!get_throttle_address(i_mot, address)) {
                    return;
                }

                _data[0] = (uint8_t)((tmp_thr>>8)&0xff);
                _data[1] = (uint8_t)(tmp_thr&0xff);
                _data[2] = (uint8_t)((tmp_thr>>8)&0xff);
                _data[3] = (uint8_t)(tmp_thr&0xff);

                uint32_t target_addr = address | AP_HAL::CANFrame::FlagEFF;
                send_cmd(target_addr, _data);
            }
        } else {
            continue;
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
