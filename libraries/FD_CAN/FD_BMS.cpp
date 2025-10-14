#include "FD_BMS.h"

#include <AP_AHRS/AP_AHRS.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Logger/AP_Logger.h>
#include <GCS_MAVLink/GCS.h>
#include <SRV_Channel/SRV_Channel.h>
#include <AP_HAL/CANIface.h>

extern const AP_HAL::HAL &hal;


FD_BMS::FD_BMS(FD_CAN_2 *frotend) {
    _frotend_ptr = frotend;
}

void FD_BMS::handle_info(AP_HAL::CANFrame &in_frame, bool do_print) {
    ;
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
