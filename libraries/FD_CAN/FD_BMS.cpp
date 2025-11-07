#include "FD_BMS.h"

#include <AP_AHRS/AP_AHRS.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Logger/AP_Logger.h>
#include <GCS_MAVLink/GCS.h>
#include <SRV_Channel/SRV_Channel.h>
#include <AP_HAL/CANIface.h>

#include <FD1_DATA/FD1_DATA.h>

extern const AP_HAL::HAL &hal;


FD_BMS::FD_BMS(FD_CAN_2 *frotend) {
    _frotend_ptr = frotend;
}

void FD_BMS::handle_info(AP_HAL::CANFrame &in_frame, bool do_print) {
    if (in_frame.id == (0x1801FFF4)) {
        uint16_t voltage = (uint16_t)in_frame.data[0] | (uint16_t)(in_frame.data[1]<<8);
        uint16_t soc = (uint16_t)in_frame.data[4] | (uint16_t)(in_frame.data[5]<<8);
        status.voltage = voltage * 0.1;
        status.soc = soc * 0.1;
        AP::fd1_data().set_bms_vol(status.voltage);
        AP::fd1_data().set_bms_soc(status.soc);
    }
    if (in_frame.id == (0x1804FFF4)) {
        uint16_t temperature = (uint16_t)in_frame.data[0] | (uint16_t)(in_frame.data[1]<<8);
        status.temperature = temperature-50.f;
        AP::fd1_data().set_bms_tem(status.temperature);
    }
    if (in_frame.id == (0x180DFFF4)) {
        for (int i = 0; i < 8; i++) {
            status.error1[i] = in_frame.data[i];
        }
        AP::fd1_data().set_bms_error1(status.error1);

        char bms_error1_str[17] = {0}; // 16字符 + 结束符
        // for (int i = 0; i < 8; i++) {
        //     uint8_t byte = status.error1[i];
        //     uint8_t high = (byte >> 4) & 0x0F;
        //     bms_error1_str[2*i] = (high < 10) ? ('0' + high) : ('A' + high - 10);
        //     uint8_t low = byte & 0x0F;
        //     bms_error1_str[2*i + 1] = (low < 10) ? ('0' + low) : ('A' + low - 10);
        // }
        AP::fd1_data().set_bms_error1_char(bms_error1_str);
    }
    if (in_frame.id == (0x180EFFF4)) {
        for (int i = 0; i < 8; i++) {
            status.error2[i] = in_frame.data[i];
        }
        AP::fd1_data().set_bms_error2(status.error2);

        char bms_error2_str[17] = {0}; // 16字符 + 结束符
        // for (int i = 0; i < 8; i++) {
        //     uint8_t byte = status.error2[i];
        //     uint8_t high = (byte >> 4) & 0x0F;
        //     bms_error2_str[2*i] = (high < 10) ? ('0' + high) : ('A' + high - 10);
        //     uint8_t low = byte & 0x0F;
        //     bms_error2_str[2*i + 1] = (low < 10) ? ('0' + low) : ('A' + low - 10);
        // }
        AP::fd1_data().set_bms_error2_char(bms_error2_str);
    }
}

void FD_BMS::do_power_on()
{
    _data[0] = 0x01;
    _data[1] = 0x01;
    _data[2] = 0x01;
    _data[3] = 0x01;
    _data[4] = 0x00;
    _data[5] = 0x00;
    _data[6] = 0x00;
    _data[7] = 0x00;
    send_cmd(0x1801F4E5, _data);
}

void FD_BMS::do_power_off()
{
    _data[0] = 0x02;
    _data[1] = 0x02;
    _data[2] = 0x02;
    _data[3] = 0x02;
    _data[4] = 0x00;
    _data[5] = 0x00;
    _data[6] = 0x00;
    _data[7] = 0x00;
    send_cmd(0x1801F4E5, _data);
}

void FD_BMS::do_power_on(uint8_t id)
{
    _data[0] = 0x00;
    _data[1] = 0x00;
    _data[2] = 0x00;
    _data[3] = 0x00;
    _data[4] = 0x00;
    _data[5] = 0x00;
    _data[6] = 0x00;
    _data[7] = 0x00;

    if (id == 1) {
        _data[3] = 0x01;
    }else if(id == 2){
        _data[2] = 0x01;
    }else if(id == 3){
        _data[0] = 0x01;
    }else if(id == 4){
        _data[1] = 0x01;
    }

    send_cmd(0x1801F4E5, _data);
}

void FD_BMS::do_power_off(uint8_t id)
{
    _data[0] = 0x00;
    _data[1] = 0x00;
    _data[2] = 0x00;
    _data[3] = 0x00;
    _data[4] = 0x00;
    _data[5] = 0x00;
    _data[6] = 0x00;
    _data[7] = 0x00;

    if (id == 1) {
        _data[3] = 0x02;
    }else if(id == 2){
        _data[2] = 0x02;
    }else if(id == 3){
        _data[0] = 0x02;
    }else if(id == 4){
        _data[1] = 0x02;
    }

    send_cmd(0x1801F4E5, _data);
}

void FD_BMS::update()
{
    update_cmd();
}

void FD_BMS::update_cmd()
{
    ;
}

void FD_BMS::send_cmd(uint32_t id, uint8_t *data) {
    if (_frotend_ptr == nullptr) {return;}
    const uint8_t data_length = 8;
    AP_HAL::CANFrame txFrame{};
    memcpy(txFrame.data, data, data_length);
    txFrame.id = id | AP_HAL::CANFrame::FlagEFF;//.扩展帧
    txFrame.dlc = 8;
    gcs().send_text(MAV_SEVERITY_INFO, "id 0x%lx", txFrame.id);
    uint64_t timeout = AP_HAL::micros64() + 10000ULL;
    _frotend_ptr->write_frame(txFrame, timeout);
}
