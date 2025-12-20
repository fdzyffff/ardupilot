#include "FD_BMS.h"

#include <AP_AHRS/AP_AHRS.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Logger/AP_Logger.h>
#include <GCS_MAVLink/GCS.h>
#include <SRV_Channel/SRV_Channel.h>

extern const AP_HAL::HAL &hal;

FD_BMS::FD_BMS(FD_CAN *frotend) {
    _frotend_ptr = frotend;
}

void FD_BMS::handle_info(AP_HAL::CANFrame &in_frame, bool do_print) {
    if (in_frame.id == (0x18102701| AP_HAL::CANFrame::FlagEFF)) {
        status.SOC = (uint8_t)in_frame.data[0];
        status.SOH = (uint8_t)in_frame.data[1];
        status.Volt = (float)((uint16_t)in_frame.data[2] << 8 | (uint16_t)in_frame.data[3]) * 0.1f;
        status.Curr = (float)((uint16_t)in_frame.data[4] << 8 | (uint16_t)in_frame.data[5]) * 0.1f;

        AP::fd_data().hxts_hy_bms_c1_packet.SOC = status.SOC;
        AP::fd_data().hxts_hy_bms_c1_packet.SOH = status.SOH;
        AP::fd_data().hxts_hy_bms_c1_packet.Volt = (uint16_t)(status.Volt*10.0f);
        AP::fd_data().hxts_hy_bms_c1_packet.Current = (uint16_t)(status.Curr*10.0f);
    }

    if (in_frame.id == (0x18112701| AP_HAL::CANFrame::FlagEFF)) {
        status.SOC = (uint8_t)in_frame.data[0];
        status.SOH = (uint8_t)in_frame.data[1];
        status.Volt = (float)((uint16_t)in_frame.data[2] << 8 | (uint16_t)in_frame.data[3]) * 0.1f;
        status.Curr = (float)((uint16_t)in_frame.data[4] << 8 | (uint16_t)in_frame.data[5]) * 0.1f;

        AP::fd_data().hxts_hy_bms_c1_packet.HighestVolt = (uint16_t)in_frame.data[0] << 8 | (uint16_t)in_frame.data[1];
        AP::fd_data().hxts_hy_bms_c1_packet.HighestCell = (uint16_t)in_frame.data[2] << 8 | (uint16_t)in_frame.data[3];
        AP::fd_data().hxts_hy_bms_c1_packet.LowestVolt = (uint16_t)in_frame.data[4] << 8 | (uint16_t)in_frame.data[5];
        AP::fd_data().hxts_hy_bms_c1_packet.LowestCell = (uint16_t)in_frame.data[6] << 8 | (uint16_t)in_frame.data[7];
    }

    if (in_frame.id == (0x18152701| AP_HAL::CANFrame::FlagEFF)) {
        status.allow_charge           = in_frame.data[0];
        status.allow_discharge        = in_frame.data[1];
        status.error_charge_level     = in_frame.data[2];
        status.error_charge_code      = in_frame.data[3];
        status.error_discharge_level  = in_frame.data[4];
        status.error_discharge_code   = in_frame.data[5];
        status.batter_status          = in_frame.data[6];
        status.other_error_code       = in_frame.data[7];

        AP::fd_data().hxts_hy_bms_c2_packet.ChargeFlag      = status.allow_charge;
        AP::fd_data().hxts_hy_bms_c2_packet.DischargeFlag   = status.allow_discharge;
        AP::fd_data().hxts_hy_bms_c2_packet.ChargeError     = status.error_charge_level;
        AP::fd_data().hxts_hy_bms_c2_packet.ChargeECode     = status.error_charge_code;
        AP::fd_data().hxts_hy_bms_c2_packet.DischargeError  = status.error_discharge_level;
        AP::fd_data().hxts_hy_bms_c2_packet.DischargeECode  = status.error_discharge_code;
        AP::fd_data().hxts_hy_bms_c2_packet.ChargeStatus    = status.batter_status;
        AP::fd_data().hxts_hy_bms_c2_packet.ErrorCode       = status.other_error_code;
    }
}

void FD_BMS::update()
{
    ;
}

void FD_BMS::set_switch(uint8_t switch_in)
{
    // send mot cmd
    {
        if (switch_in == 0) {
            _data[0] = 0x02;
            _data[1] = 0x00;
            _data[2] = 0x00;
            _data[3] = 0x00;
            _data[4] = 0x00;
            _data[5] = 0x00;
            _data[6] = 0x00;
            _data[7] = 0x00;

            uint32_t target_addr = 0x102;
            send_cmd(target_addr, _data);
        }

        if (switch_in == 1) {
            _data[0] = 0x02;
            _data[1] = 0x01;
            _data[2] = 0x00;
            _data[3] = 0x00;
            _data[4] = 0x00;
            _data[5] = 0x00;
            _data[6] = 0x00;
            _data[7] = 0x00;

            uint32_t target_addr = 0x102;
            send_cmd(target_addr, _data);
        }
    }
}

void FD_BMS::send_cmd(uint32_t id, uint8_t *data) {
    if (_frotend_ptr == nullptr) {return;}
    const uint8_t data_length = 8;
    AP_HAL::CANFrame txFrame{};
    memcpy(txFrame.data, data, data_length);
    txFrame.id = id;
    txFrame.dlc = 8;
    uint64_t timeout = AP_HAL::micros64() + 10000ULL;
    _frotend_ptr->write_frame(txFrame, timeout);
}
