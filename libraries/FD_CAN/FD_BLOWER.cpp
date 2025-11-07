#include "FD_BLOWER.h"

#include <AP_AHRS/AP_AHRS.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Logger/AP_Logger.h>
#include <GCS_MAVLink/GCS.h>
#include <SRV_Channel/SRV_Channel.h>

#include <FD1_DATA/FD1_DATA.h>

extern const AP_HAL::HAL &hal;


FD_BLOWER::FD_BLOWER(FD_CAN_2 *frotend) {
    _frotend_ptr = frotend;
}

void FD_BLOWER::handle_info(AP_HAL::CANFrame &in_frame, bool do_print) {
    if (in_frame.id == (0x500E)) {
        uint16_t p_rec = (uint16_t)in_frame.data[0] | (uint16_t)(in_frame.data[1]<<8);
        status.p = p_rec * 0.1;
        AP::fd1_data().set_blower_p(status.p);
    }
    if (in_frame.id == (0x5022)) {
        status.temperature = (uint16_t)in_frame.data[0] | (uint16_t)(in_frame.data[1]<<8);
        AP::fd1_data().set_blower_tem(status.temperature);
    }
    if (in_frame.id == (0xFC0D)) {
        for (int i = 0; i < 8; i++) {
            status.error[i] = in_frame.data[i];
        }
        AP::fd1_data().set_blower_error(status.error);

        char blower_error_str[17] = {0}; // 16字符 + 结束符
        // for (int i = 0; i < 8; i++) {
        //     uint8_t byte = status.error[i];
        //     uint8_t high = (byte >> 4) & 0x0F;
        //     blower_error_str[2*i] = (high < 10) ? ('0' + high) : ('A' + high - 10);
        //     uint8_t low = byte & 0x0F;
        //     blower_error_str[2*i + 1] = (low < 10) ? ('0' + low) : ('A' + low - 10);
        // }
        AP::fd1_data().set_blower_error_char(blower_error_str);
    }
}

void FD_BLOWER::do_power_on()
{
    _data[0] = 0x00;
    _data[1] = 0x00;
    _data[2] = 0x00;
    _data[3] = 0x00;
    _data[4] = 0x01;
    _data[5] = 0x00;
    _data[6] = 0x00;
    _data[7] = 0x00;
    send_cmd(0x1801F4E5, _data);
}

void FD_BLOWER::do_power_off()
{
    _data[0] = 0x00;
    _data[1] = 0x00;
    _data[2] = 0x00;
    _data[3] = 0x00;
    _data[4] = 0x02;
    _data[5] = 0x00;
    _data[6] = 0x00;
    _data[7] = 0x00;
    send_cmd(0x1801F4E5, _data);
}

void FD_BLOWER::do_on()
{
    _data[0] = 0x01;
    _data[1] = 0x00;
    _data[2] = 0x00;
    _data[3] = 0x00;
    _data[4] = 0x00;
    _data[5] = 0x00;
    _data[6] = 0x00;
    _data[7] = 0x00;
    send_cmd(0x7000, _data);
}

void FD_BLOWER::do_off()
{
    _data[0] = 0x05;
    _data[1] = 0x00;
    _data[2] = 0x00;
    _data[3] = 0x00;
    _data[4] = 0x00;
    _data[5] = 0x00;
    _data[6] = 0x00;
    _data[7] = 0x00;
    send_cmd(0x7000, _data);
}

void FD_BLOWER::update()
{
    update_cmd();
}

void FD_BLOWER::update_cmd()
{
    ;
}

void FD_BLOWER::send_cmd(uint32_t id, uint8_t *data) {
    if (_frotend_ptr == nullptr) {return;}
    const uint8_t data_length = 8;
    AP_HAL::CANFrame txFrame{};
    memcpy(txFrame.data, data, data_length);
    txFrame.id = id | AP_HAL::CANFrame::FlagEFF;//.扩展帧
    txFrame.dlc = 8;
    uint64_t timeout = AP_HAL::micros64() + 10000ULL;
    _frotend_ptr->write_frame(txFrame, timeout);
}
