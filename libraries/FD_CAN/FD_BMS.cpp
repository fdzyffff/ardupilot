#include "FD_BMS.h"

#include <AP_AHRS/AP_AHRS.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Logger/AP_Logger.h>
#include <GCS_MAVLink/GCS.h>
#include <SRV_Channel/SRV_Channel.h>

extern const AP_HAL::HAL &hal;

// 大端取值：CAN 报文里所有 16 位量都是 H/L 顺序
static uint16_t fd_bms_be16(const uint8_t *p)
{
    return (uint16_t)(((uint16_t)p[0] << 8) | (uint16_t)p[1]);
}

FD_BMS::FD_BMS(FD_CAN *frotend) {
    _frotend_ptr = frotend;
}

// 说明：本工程飞控侧不做任何换算（缩放/偏移），CAN 原始码原样写入 MAVLink 报文字段，
//       全部换算由地面站完成。
void FD_BMS::handle_info(AP_HAL::CANFrame &in_frame, bool do_print) {
    // ------------------------------------------------------------------
    // 0x1810 电池组基础信息报文：SOC / SOH / 总压(0.1V) / 总电流(0.1A, 偏移 -3200A)
    // ------------------------------------------------------------------
    if (in_frame.id == (0x18102701| AP_HAL::CANFrame::FlagEFF)) {
        status.SOC = (uint8_t)in_frame.data[0];
        status.SOH = (uint8_t)in_frame.data[1];

        AP::fd_data().hxts_hy_bms_c1_packet.SOC = in_frame.data[0];
        AP::fd_data().hxts_hy_bms_c1_packet.SOH = in_frame.data[1];
        AP::fd_data().hxts_hy_bms_c1_packet.Volt = fd_bms_be16(&in_frame.data[2]);
        AP::fd_data().hxts_hy_bms_c1_packet.Current = fd_bms_be16(&in_frame.data[4]);
        AP::fd_data().status.bms_c1_updated = true;
        _allow_log_1 = true;
        if (do_print) {
            gcs().send_text(MAV_SEVERITY_INFO, "[DBug] bms c1 update");
        }
    }

    // ------------------------------------------------------------------
    // 0x1811 单体及位置报文：最高/最低单体电压(0.001V) 及位置(偏移 +1)
    // ------------------------------------------------------------------
    if (in_frame.id == (0x18112701| AP_HAL::CANFrame::FlagEFF)) {
        AP::fd_data().hxts_hy_bms_c1_packet.HighestVolt = fd_bms_be16(&in_frame.data[0]);
        AP::fd_data().hxts_hy_bms_c1_packet.HighestCell = fd_bms_be16(&in_frame.data[2]);
        AP::fd_data().hxts_hy_bms_c1_packet.LowestVolt = fd_bms_be16(&in_frame.data[4]);
        AP::fd_data().hxts_hy_bms_c1_packet.LowestCell = fd_bms_be16(&in_frame.data[6]);
        AP::fd_data().status.bms_c1_updated = true;
        _allow_log_1 = true;
        if (do_print) {
            gcs().send_text(MAV_SEVERITY_INFO, "[DBug] bms c2 update");
        }
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
        AP::fd_data().status.bms_c2_updated = true;
        _allow_log_2 = true;
        if (do_print) {
            gcs().send_text(MAV_SEVERITY_INFO, "[DBug] bms c3 update");
        }
    }

    // ------------------------------------------------------------------
    // 0x1830~0x1835 单体电压报文：6 组，一组 4 节，共 24 节，分辨率 0.1V
    //   C3.Volt[0..7]   : 前 8 节（第 1、2 组）
    //   CELL.CellVolt[] : 全 24 节
    // ------------------------------------------------------------------
    const uint32_t id_1830_base = 0x18302701 | AP_HAL::CANFrame::FlagEFF;
    if (in_frame.id >= id_1830_base && in_frame.id <= (id_1830_base + 5 * 0x10000)) {
        const uint8_t grp = (uint8_t)((in_frame.id - id_1830_base) / 0x10000);
        const uint8_t base = grp * 4;
        for (uint8_t k = 0; k < 4; k++) {
            const uint16_t v = fd_bms_be16(&in_frame.data[k * 2]);
            if ((base + k) < 8) {
                AP::fd_data().hxts_hy_bms_c3_packet.Volt[base + k] = v;
            }
            AP::fd_data().hxts_hy_bms_cell_packet.CellVolt[base + k] = v;
        }
    }

    // ------------------------------------------------------------------
    // 0x1821/0x1822 分路放电电流 1~6 路（0.01A），0x1823 第 7 路充电电流（0.01A）
    // ------------------------------------------------------------------
    if (in_frame.id == (0x18212701| AP_HAL::CANFrame::FlagEFF)) {
        AP::fd_data().hxts_hy_bms_c3_packet.Current[0] = fd_bms_be16(&in_frame.data[0]);
        AP::fd_data().hxts_hy_bms_c3_packet.Current[1] = fd_bms_be16(&in_frame.data[2]);
        AP::fd_data().hxts_hy_bms_c3_packet.Current[2] = fd_bms_be16(&in_frame.data[4]);
    }

    if (in_frame.id == (0x18222701| AP_HAL::CANFrame::FlagEFF)) {
        AP::fd_data().hxts_hy_bms_c3_packet.Current[3] = fd_bms_be16(&in_frame.data[0]);
        AP::fd_data().hxts_hy_bms_c3_packet.Current[4] = fd_bms_be16(&in_frame.data[2]);
        AP::fd_data().hxts_hy_bms_c3_packet.Current[5] = fd_bms_be16(&in_frame.data[4]);
    }

    if (in_frame.id == (0x18232701| AP_HAL::CANFrame::FlagEFF)) {
        AP::fd_data().hxts_hy_bms_c3_packet.Current[6] = fd_bms_be16(&in_frame.data[0]);
    }

    // ------------------------------------------------------------------
    // 0x1880 温感温度报文：一组 4 个电芯，每个电芯 2 个温度，共 8 路（1℃，偏移 -40℃）
    // ------------------------------------------------------------------
    if (in_frame.id == (0x18802701| AP_HAL::CANFrame::FlagEFF)) {
        for (uint8_t k = 0; k < 8; k++) {
            AP::fd_data().hxts_hy_bms_temp_packet.Temp[k] = in_frame.data[k];
        }
    }

    // ------------------------------------------------------------------
    // 0x1900~0x191B 单串电压报文：4 个电芯，每个电芯 25 串，共 100 串，分辨率 0.001V
    //   索引 = (电芯号-1) * 25 + (串号-1)
    //   每组 7 帧，前 6 帧各 4 串，第 7 帧只有第 25 串有效
    // ------------------------------------------------------------------
    const uint32_t id_1900_base = 0x19002701 | AP_HAL::CANFrame::FlagEFF;
    if (in_frame.id >= id_1900_base && in_frame.id <= (id_1900_base + 27 * 0x10000)) {
        const uint8_t frame_idx = (uint8_t)((in_frame.id - id_1900_base) / 0x10000);
        const uint8_t grp = frame_idx / 7;
        const uint8_t sub = frame_idx % 7;
        const uint8_t str_base = (uint8_t)(grp * 25 + sub * 4);
        const uint8_t num = (sub == 6) ? 1 : 4;
        for (uint8_t k = 0; k < num; k++) {
            const uint8_t str_idx = (uint8_t)(str_base + k);
            const uint16_t str_volt = fd_bms_be16(&in_frame.data[k * 2]);
            // 前 50 串走 41026，后 50 串走 41027
            if (str_idx < 50) {
                AP::fd_data().hxts_hy_bms_strv_packet.StrVolt[str_idx] = str_volt;
            } else {
                AP::fd_data().hxts_hy_bms_strv2_packet.StrVolt[str_idx - 50] = str_volt;
            }
        }
    }

    update_log();
}

void FD_BMS::update()
{
    ;
}


void FD_BMS::update_log()
{
    uint32_t now_ms = AP_HAL::millis();
    if (now_ms - _last_log_ms < 200) {return;}

    _last_log_ms = now_ms;

    if (_allow_log_1) {
        AP::logger().WriteStreaming("UBS1",
                                    "TimeUS,SOC,SOH,Volt,Curr,HV,HC,LV,LC",
                                    "s--------",
                                    "F--------",
                                    "QHHHHHHHH",
                                    AP_HAL::micros64(),
                                    AP::fd_data().hxts_hy_bms_c1_packet.SOC,
                                    AP::fd_data().hxts_hy_bms_c1_packet.SOH,
                                    AP::fd_data().hxts_hy_bms_c1_packet.Volt,
                                    AP::fd_data().hxts_hy_bms_c1_packet.Current,
                                    AP::fd_data().hxts_hy_bms_c1_packet.HighestVolt,
                                    AP::fd_data().hxts_hy_bms_c1_packet.HighestCell,
                                    AP::fd_data().hxts_hy_bms_c1_packet.LowestVolt,
                                    AP::fd_data().hxts_hy_bms_c1_packet.LowestCell);
    }

    if (_allow_log_2) {
        AP::logger().WriteStreaming("UBS2",
                                    "TimeUS,CrgF,DrgF,CrgE,CrgEC,DrgE,DrEC,EC",
                                    "s--------",
                                    "F--------",
                                    "QBBBBBBBB",
                                    AP_HAL::micros64(),
                                    AP::fd_data().hxts_hy_bms_c2_packet.ChargeFlag,
                                    AP::fd_data().hxts_hy_bms_c2_packet.DischargeFlag,
                                    AP::fd_data().hxts_hy_bms_c2_packet.ChargeError,
                                    AP::fd_data().hxts_hy_bms_c2_packet.ChargeECode,
                                    AP::fd_data().hxts_hy_bms_c2_packet.DischargeError,
                                    AP::fd_data().hxts_hy_bms_c2_packet.DischargeECode,
                                    AP::fd_data().hxts_hy_bms_c2_packet.ChargeStatus,
                                    AP::fd_data().hxts_hy_bms_c2_packet.ErrorCode);
    }
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
    const uint8_t data_length = 2;
    AP_HAL::CANFrame txFrame{};
    memcpy(txFrame.data, data, data_length);
    txFrame.id = id;
    txFrame.dlc = 2;
    uint64_t timeout = AP_HAL::micros64() + 10000ULL;
    _frotend_ptr->write_frame(txFrame, timeout);
}
