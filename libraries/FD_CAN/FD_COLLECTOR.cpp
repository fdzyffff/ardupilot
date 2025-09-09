#include "FD_COLLECTOR.h"

#include <AP_AHRS/AP_AHRS.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Logger/AP_Logger.h>
#include <GCS_MAVLink/GCS.h>
#include <SRV_Channel/SRV_Channel.h>

extern const AP_HAL::HAL &hal;


FD_COLLECTOR::FD_COLLECTOR(FD_CAN *frotend) {
    _frotend_ptr = frotend;
    status.brake_confirm = false;
}

void FD_COLLECTOR::update_send()
{
    {
        if (AP_HAL::millis() - status.last_ask_status_ms > 500) {
            status.last_ask_status_ms = AP_HAL::millis();

            // send left_wheel; // 左轮速 RPM
            tmp_float_to_data.v = left_wheel;
            _data[0] = 0xFE;
            _data[1] = 0xFE;
            _data[2] = 0x01;
            _data[3] = tmp_float_to_data.dat[0];
            _data[4] = tmp_float_to_data.dat[1];
            _data[5] = tmp_float_to_data.dat[2];
            _data[6] = tmp_float_to_data.dat[3];
            _data[7] = 0xEE;
            send_cmd(0x20, _data);

            // send right_wheel; // 右轮速 RPM
            tmp_float_to_data.v = right_wheel;
            _data[0] = 0xFE;
            _data[1] = 0xFE;
            _data[2] = 0x01;
            _data[3] = tmp_float_to_data.dat[0];
            _data[4] = tmp_float_to_data.dat[1];
            _data[5] = tmp_float_to_data.dat[2];
            _data[6] = tmp_float_to_data.dat[3];
            _data[7] = 0xEE;
            send_cmd(0x20, _data);

            // send left_brake_in; // 左刹车阀输入信号 mA
            tmp_float_to_data.v = left_brake_in;
            _data[0] = 0xFE;
            _data[1] = 0xFE;
            _data[2] = 0x01;
            _data[3] = tmp_float_to_data.dat[0];
            _data[4] = tmp_float_to_data.dat[1];
            _data[5] = tmp_float_to_data.dat[2];
            _data[6] = tmp_float_to_data.dat[3];
            _data[7] = 0xEE;
            send_cmd(0x20, _data);

            // send left_brake_back; // 左刹车阀反馈信号 V
            tmp_float_to_data.v = left_brake_back;
            _data[0] = 0xFE;
            _data[1] = 0xFE;
            _data[2] = 0x01;
            _data[3] = tmp_float_to_data.dat[0];
            _data[4] = tmp_float_to_data.dat[1];
            _data[5] = tmp_float_to_data.dat[2];
            _data[6] = tmp_float_to_data.dat[3];
            _data[7] = 0xEE;
            send_cmd(0x20, _data);

            // send right_brake_in; // 右刹车阀输入信号 mA
            tmp_float_to_data.v = right_brake_in;
            _data[0] = 0xFE;
            _data[1] = 0xFE;
            _data[2] = 0x01;
            _data[3] = tmp_float_to_data.dat[0];
            _data[4] = tmp_float_to_data.dat[1];
            _data[5] = tmp_float_to_data.dat[2];
            _data[6] = tmp_float_to_data.dat[3];
            _data[7] = 0xEE;
            send_cmd(0x20, _data);

            // send right_brake_back; // 右刹车阀反馈信号 V
            tmp_float_to_data.v = right_brake_back;
            _data[0] = 0xFE;
            _data[1] = 0xFE;
            _data[2] = 0x01;
            _data[3] = tmp_float_to_data.dat[0];
            _data[4] = tmp_float_to_data.dat[1];
            _data[5] = tmp_float_to_data.dat[2];
            _data[6] = tmp_float_to_data.dat[3];
            _data[7] = 0xEE;
            send_cmd(0x20, _data);

            // send alt; // 高度 cm
            tmp_float_to_data.v = alt;
            _data[0] = 0xFE;
            _data[1] = 0xFE;
            _data[2] = 0x01;
            _data[3] = tmp_float_to_data.dat[0];
            _data[4] = tmp_float_to_data.dat[1];
            _data[5] = tmp_float_to_data.dat[2];
            _data[6] = tmp_float_to_data.dat[3];
            _data[7] = 0xEE;
            send_cmd(0x20, _data);

            // send arspd_tas; // 真空速 m/s
            tmp_float_to_data.v = arspd_tas;
            _data[0] = 0xFE;
            _data[1] = 0xFE;
            _data[2] = 0x01;
            _data[3] = tmp_float_to_data.dat[0];
            _data[4] = tmp_float_to_data.dat[1];
            _data[5] = tmp_float_to_data.dat[2];
            _data[6] = tmp_float_to_data.dat[3];
            _data[7] = 0xEE;
            send_cmd(0x20, _data);

            // send climb_rate; // 升降速度 cm/s
            tmp_float_to_data.v = climb_rate;
            _data[0] = 0xFE;
            _data[1] = 0xFE;
            _data[2] = 0x01;
            _data[3] = tmp_float_to_data.dat[0];
            _data[4] = tmp_float_to_data.dat[1];
            _data[5] = tmp_float_to_data.dat[2];
            _data[6] = tmp_float_to_data.dat[3];
            _data[7] = 0xEE;
            send_cmd(0x20, _data);

            // send aoa; // 迎角 degree
            tmp_float_to_data.v = aoa;
            _data[0] = 0xFE;
            _data[1] = 0xFE;
            _data[2] = 0x01;
            _data[3] = tmp_float_to_data.dat[0];
            _data[4] = tmp_float_to_data.dat[1];
            _data[5] = tmp_float_to_data.dat[2];
            _data[6] = tmp_float_to_data.dat[3];
            _data[7] = 0xEE;
            send_cmd(0x20, _data);

            // send ssa; // 侧滑角 degree
            tmp_float_to_data.v = ssa;
            _data[0] = 0xFE;
            _data[1] = 0xFE;
            _data[2] = 0x01;
            _data[3] = tmp_float_to_data.dat[0];
            _data[4] = tmp_float_to_data.dat[1];
            _data[5] = tmp_float_to_data.dat[2];
            _data[6] = tmp_float_to_data.dat[3];
            _data[7] = 0xEE;
            send_cmd(0x20, _data);

            // send roll; // 滚动角 degree
            tmp_float_to_data.v = roll;
            _data[0] = 0xFE;
            _data[1] = 0xFE;
            _data[2] = 0x01;
            _data[3] = tmp_float_to_data.dat[0];
            _data[4] = tmp_float_to_data.dat[1];
            _data[5] = tmp_float_to_data.dat[2];
            _data[6] = tmp_float_to_data.dat[3];
            _data[7] = 0xEE;
            send_cmd(0x20, _data);

            // send yaw; // 航向角 degree
            tmp_float_to_data.v = yaw;
            _data[0] = 0xFE;
            _data[1] = 0xFE;
            _data[2] = 0x01;
            _data[3] = tmp_float_to_data.dat[0];
            _data[4] = tmp_float_to_data.dat[1];
            _data[5] = tmp_float_to_data.dat[2];
            _data[6] = tmp_float_to_data.dat[3];
            _data[7] = 0xEE;
            send_cmd(0x20, _data);

            // send rate_x; // X轴角速度 degree/s
            tmp_float_to_data.v = rate_x;
            _data[0] = 0xFE;
            _data[1] = 0xFE;
            _data[2] = 0x01;
            _data[3] = tmp_float_to_data.dat[0];
            _data[4] = tmp_float_to_data.dat[1];
            _data[5] = tmp_float_to_data.dat[2];
            _data[6] = tmp_float_to_data.dat[3];
            _data[7] = 0xEE;
            send_cmd(0x20, _data);

            // send rate_y; // Y轴角速度 degree/s
            tmp_float_to_data.v = rate_y;
            _data[0] = 0xFE;
            _data[1] = 0xFE;
            _data[2] = 0x01;
            _data[3] = tmp_float_to_data.dat[0];
            _data[4] = tmp_float_to_data.dat[1];
            _data[5] = tmp_float_to_data.dat[2];
            _data[6] = tmp_float_to_data.dat[3];
            _data[7] = 0xEE;
            send_cmd(0x20, _data);

            // send rate_z; // Z轴角速度 degree/s
            tmp_float_to_data.v = rate_z;
            _data[0] = 0xFE;
            _data[1] = 0xFE;
            _data[2] = 0x01;
            _data[3] = tmp_float_to_data.dat[0];
            _data[4] = tmp_float_to_data.dat[1];
            _data[5] = tmp_float_to_data.dat[2];
            _data[6] = tmp_float_to_data.dat[3];
            _data[7] = 0xEE;
            send_cmd(0x20, _data);

            // send acc_x; // X轴加速度 m/s/s
            tmp_float_to_data.v = acc_x;
            _data[0] = 0xFE;
            _data[1] = 0xFE;
            _data[2] = 0x01;
            _data[3] = tmp_float_to_data.dat[0];
            _data[4] = tmp_float_to_data.dat[1];
            _data[5] = tmp_float_to_data.dat[2];
            _data[6] = tmp_float_to_data.dat[3];
            _data[7] = 0xEE;
            send_cmd(0x20, _data);

            // send acc_y; // Y轴加速度 m/s/s
            tmp_float_to_data.v = acc_y;
            _data[0] = 0xFE;
            _data[1] = 0xFE;
            _data[2] = 0x01;
            _data[3] = tmp_float_to_data.dat[0];
            _data[4] = tmp_float_to_data.dat[1];
            _data[5] = tmp_float_to_data.dat[2];
            _data[6] = tmp_float_to_data.dat[3];
            _data[7] = 0xEE;
            send_cmd(0x20, _data);

            // send acc_z; // Z轴加速度 m/s/s
            tmp_float_to_data.v = acc_z;
            _data[0] = 0xFE;
            _data[1] = 0xFE;
            _data[2] = 0x01;
            _data[3] = tmp_float_to_data.dat[0];
            _data[4] = tmp_float_to_data.dat[1];
            _data[5] = tmp_float_to_data.dat[2];
            _data[6] = tmp_float_to_data.dat[3];
            _data[7] = 0xEE;
            send_cmd(0x20, _data);

            send_utc();
        }
    }

}

void FD_COLLECTOR::send_cmd(uint32_t id, uint8_t *data) {
    if (_frotend_ptr == nullptr) {return;}
    const uint8_t data_length = 8;
    AP_HAL::CANFrame txFrame{};
    memcpy(txFrame.data, data, data_length);
    txFrame.id = id;
    txFrame.dlc = 8;
    uint64_t timeout = AP_HAL::micros64() + 10000ULL;
    _frotend_ptr->write_frame(txFrame, timeout);
}

void FD_COLLECTOR::send_utc() {
    if (_frotend_ptr == nullptr) {return;}
    AP_HAL::CANFrame txFrame{};
    tmp_uint64t_to_data.v = 0;
    txFrame.data[0] = 0xFE;
    txFrame.data[1] = 0xFE;
    txFrame.data[2] = 0x14;
    txFrame.data[3] = tmp_uint64t_to_data.data[0];
    txFrame.data[4] = tmp_uint64t_to_data.data[1];
    txFrame.data[5] = tmp_uint64t_to_data.data[2];
    txFrame.data[6] = tmp_uint64t_to_data.data[3];
    txFrame.data[7] = tmp_uint64t_to_data.data[4];
    txFrame.data[8] = tmp_uint64t_to_data.data[5];
    txFrame.data[9] = tmp_uint64t_to_data.data[6];
    txFrame.data[10] = tmp_uint64t_to_data.data[7];
    txFrame.data[11] = 0xEE;

    txFrame.id = 0x20;
    txFrame.dlc = 12;
    uint64_t timeout = AP_HAL::micros64() + 10000ULL;
    _frotend_ptr->write_frame(txFrame, timeout);
}
