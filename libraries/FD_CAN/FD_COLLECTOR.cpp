#include "FD_COLLECTOR.h"

#include <AP_AHRS/AP_AHRS.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Logger/AP_Logger.h>
#include <GCS_MAVLink/GCS.h>
#include <SRV_Channel/SRV_Channel.h>
#include <FD1_DATA/FD1_DATA.h>

// #include <AP_ExternalAHRS/AP_ExternalAHRS_config.h>
// #include <AP_ExternalAHRS/AP_ExternalAHRS_TZ605.h>
// #include <AP_Math/AP_Math.h>
// #include <AP_Math/crc.h>
// #include <AP_GPS/AP_GPS.h>
// #include <AP_Baro/AP_Baro.h>
// #include <AP_InertialSensor/AP_InertialSensor.h>
// #include <GCS_MAVLink/GCS.h>
// #include <AP_Logger/AP_Logger.h>
// #include <AP_SerialManager/AP_SerialManager.h>
// //#include <AP_HAL/utility/sparse-endian.h>
// #include <AP_Common/Bitmask.h>
// #include <AP_Vehicle/AP_Vehicle_Type.h>



extern const AP_HAL::HAL &hal;


FD_COLLECTOR::FD_COLLECTOR(FD_CAN_2 *frotend) {
    _frotend_ptr = frotend;
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
            _data[3] = tmp_float_to_data.data[0];
            _data[4] = tmp_float_to_data.data[1];
            _data[5] = tmp_float_to_data.data[2];
            _data[6] = tmp_float_to_data.data[3];
            _data[7] = 0xEE;
            send_cmd(0x20, _data);

            // send right_wheel; // 右轮速 RPM
            tmp_float_to_data.v = right_wheel;
            _data[0] = 0xFE;
            _data[1] = 0xFE;
            _data[2] = 0x02;
            _data[3] = tmp_float_to_data.data[0];
            _data[4] = tmp_float_to_data.data[1];
            _data[5] = tmp_float_to_data.data[2];
            _data[6] = tmp_float_to_data.data[3];
            _data[7] = 0xEE;
            send_cmd(0x20, _data);

            // send left_brake_in; // 左刹车阀输入信号 mA
            tmp_float_to_data.v = left_brake_in;
            _data[0] = 0xFE;
            _data[1] = 0xFE;
            _data[2] = 0x03;
            _data[3] = tmp_float_to_data.data[0];
            _data[4] = tmp_float_to_data.data[1];
            _data[5] = tmp_float_to_data.data[2];
            _data[6] = tmp_float_to_data.data[3];
            _data[7] = 0xEE;
            send_cmd(0x20, _data);

            // send left_brake_back; // 左刹车阀反馈信号 V
            tmp_float_to_data.v = left_brake_back;
            _data[0] = 0xFE;
            _data[1] = 0xFE;
            _data[2] = 0x04;
            _data[3] = tmp_float_to_data.data[0];
            _data[4] = tmp_float_to_data.data[1];
            _data[5] = tmp_float_to_data.data[2];
            _data[6] = tmp_float_to_data.data[3];
            _data[7] = 0xEE;
            send_cmd(0x20, _data);

            // send right_brake_in; // 右刹车阀输入信号 mA
            tmp_float_to_data.v = right_brake_in;
            _data[0] = 0xFE;
            _data[1] = 0xFE;
            _data[2] = 0x05;
            _data[3] = tmp_float_to_data.data[0];
            _data[4] = tmp_float_to_data.data[1];
            _data[5] = tmp_float_to_data.data[2];
            _data[6] = tmp_float_to_data.data[3];
            _data[7] = 0xEE;
            send_cmd(0x20, _data);

            // send right_brake_back; // 右刹车阀反馈信号 V
            tmp_float_to_data.v = right_brake_back;
            _data[0] = 0xFE;
            _data[1] = 0xFE;
            _data[2] = 0x06;
            _data[3] = tmp_float_to_data.data[0];
            _data[4] = tmp_float_to_data.data[1];
            _data[5] = tmp_float_to_data.data[2];
            _data[6] = tmp_float_to_data.data[3];
            _data[7] = 0xEE;
            send_cmd(0x20, _data);

            // send alt; // 高度 cm
            alt = AP::fd1_data().get_alt();
            tmp_float_to_data.v = alt;
            _data[0] = 0xFE;
            _data[1] = 0xFE;
            _data[2] = 0x07;
            _data[3] = tmp_float_to_data.data[0];
            _data[4] = tmp_float_to_data.data[1];
            _data[5] = tmp_float_to_data.data[2];
            _data[6] = tmp_float_to_data.data[3];
            _data[7] = 0xEE;
            send_cmd(0x20, _data);

            // send arspd_tas; // 真空速 m/s
            arspd_tas = AP::fd1_data().get_arspd_tas();
            tmp_float_to_data.v = arspd_tas;
            _data[0] = 0xFE;
            _data[1] = 0xFE;
            _data[2] = 0x08;
            _data[3] = tmp_float_to_data.data[0];
            _data[4] = tmp_float_to_data.data[1];
            _data[5] = tmp_float_to_data.data[2];
            _data[6] = tmp_float_to_data.data[3];
            _data[7] = 0xEE;
            send_cmd(0x20, _data);

            // send climb_rate; // 升降速度 cm/s
            climb_rate = AP::fd1_data().get_climb_rate();
            tmp_float_to_data.v = climb_rate;
            _data[0] = 0xFE;
            _data[1] = 0xFE;
            _data[2] = 0x09;
            _data[3] = tmp_float_to_data.data[0];
            _data[4] = tmp_float_to_data.data[1];
            _data[5] = tmp_float_to_data.data[2];
            _data[6] = tmp_float_to_data.data[3];
            _data[7] = 0xEE;
            send_cmd(0x20, _data);

            // send aoa; // 迎角 degree
            aoa = AP::fd1_data().get_aoa();
            tmp_float_to_data.v = aoa;
            _data[0] = 0xFE;
            _data[1] = 0xFE;
            _data[2] = 0x0A;
            _data[3] = tmp_float_to_data.data[0];
            _data[4] = tmp_float_to_data.data[1];
            _data[5] = tmp_float_to_data.data[2];
            _data[6] = tmp_float_to_data.data[3];
            _data[7] = 0xEE;
            send_cmd(0x20, _data);

            // send ssa; // 侧滑角 degree
            ssa = AP::fd1_data().get_ssa();
            tmp_float_to_data.v = ssa;
            _data[0] = 0xFE;
            _data[1] = 0xFE;
            _data[2] = 0x0B;
            _data[3] = tmp_float_to_data.data[0];
            _data[4] = tmp_float_to_data.data[1];
            _data[5] = tmp_float_to_data.data[2];
            _data[6] = tmp_float_to_data.data[3];
            _data[7] = 0xEE;
            send_cmd(0x20, _data);

            // send roll; // 滚动角 degree
            roll = AP::fd1_data().get_roll();
            tmp_float_to_data.v = roll;
            _data[0] = 0xFE;
            _data[1] = 0xFE;
            _data[2] = 0x0C;
            _data[3] = tmp_float_to_data.data[0];
            _data[4] = tmp_float_to_data.data[1];
            _data[5] = tmp_float_to_data.data[2];
            _data[6] = tmp_float_to_data.data[3];
            _data[7] = 0xEE;
            send_cmd(0x20, _data);

            // send yaw; // 航向角 degree
            yaw = AP::fd1_data().get_yaw();
            tmp_float_to_data.v = yaw;
            _data[0] = 0xFE;
            _data[1] = 0xFE;
            _data[2] = 0x0D;
            _data[3] = tmp_float_to_data.data[0];
            _data[4] = tmp_float_to_data.data[1];
            _data[5] = tmp_float_to_data.data[2];
            _data[6] = tmp_float_to_data.data[3];
            _data[7] = 0xEE;
            send_cmd(0x20, _data);

            // send rate_x; // X轴角速度 degree/s
            rate_x = AP::fd1_data().get_rate_x();
            tmp_float_to_data.v = rate_x;
            _data[0] = 0xFE;
            _data[1] = 0xFE;
            _data[2] = 0x0E;
            _data[3] = tmp_float_to_data.data[0];
            _data[4] = tmp_float_to_data.data[1];
            _data[5] = tmp_float_to_data.data[2];
            _data[6] = tmp_float_to_data.data[3];
            _data[7] = 0xEE;
            send_cmd(0x20, _data);

            // send rate_y; // Y轴角速度 degree/s
            rate_y = AP::fd1_data().get_rate_y();
            tmp_float_to_data.v = rate_y;
            _data[0] = 0xFE;
            _data[1] = 0xFE;
            _data[2] = 0x0F;
            _data[3] = tmp_float_to_data.data[0];
            _data[4] = tmp_float_to_data.data[1];
            _data[5] = tmp_float_to_data.data[2];
            _data[6] = tmp_float_to_data.data[3];
            _data[7] = 0xEE;
            send_cmd(0x20, _data);

            // send rate_z; // Z轴角速度 degree/s
            rate_z = AP::fd1_data().get_rate_z();
            tmp_float_to_data.v = rate_z;
            _data[0] = 0xFE;
            _data[1] = 0xFE;
            _data[2] = 0x10;
            _data[3] = tmp_float_to_data.data[0];
            _data[4] = tmp_float_to_data.data[1];
            _data[5] = tmp_float_to_data.data[2];
            _data[6] = tmp_float_to_data.data[3];
            _data[7] = 0xEE;
            send_cmd(0x20, _data);

            // send acc_x; // X轴加速度 m/s/s
            acc_x = AP::fd1_data().get_acc_x();
            tmp_float_to_data.v = acc_x;
            _data[0] = 0xFE;
            _data[1] = 0xFE;
            _data[2] = 0x11;
            _data[3] = tmp_float_to_data.data[0];
            _data[4] = tmp_float_to_data.data[1];
            _data[5] = tmp_float_to_data.data[2];
            _data[6] = tmp_float_to_data.data[3];
            _data[7] = 0xEE;
            send_cmd(0x20, _data);

            // send acc_y; // Y轴加速度 m/s/s
            acc_x = AP::fd1_data().get_acc_x();
            tmp_float_to_data.v = acc_y;
            _data[0] = 0xFE;
            _data[1] = 0xFE;
            _data[2] = 0x12;
            _data[3] = tmp_float_to_data.data[0];
            _data[4] = tmp_float_to_data.data[1];
            _data[5] = tmp_float_to_data.data[2];
            _data[6] = tmp_float_to_data.data[3];
            _data[7] = 0xEE;
            send_cmd(0x20, _data);

            // send acc_z; // Z轴加速度 m/s/s
            acc_x = AP::fd1_data().get_acc_x();
            tmp_float_to_data.v = acc_z;
            _data[0] = 0xFE;
            _data[1] = 0xFE;
            _data[2] = 0x13;
            _data[3] = tmp_float_to_data.data[0];
            _data[4] = tmp_float_to_data.data[1];
            _data[5] = tmp_float_to_data.data[2];
            _data[6] = tmp_float_to_data.data[3];
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
    tmp_uint32t_to_data.v = AP::fd1_data().get_gps_utc();
    txFrame.data[0] = 0xFE;
    txFrame.data[1] = 0xFE;
    txFrame.data[2] = 0x14;
    txFrame.data[3] = tmp_uint32t_to_data.data[0];
    txFrame.data[4] = tmp_uint32t_to_data.data[1];
    txFrame.data[5] = tmp_uint32t_to_data.data[2];
    txFrame.data[6] = tmp_uint32t_to_data.data[3];
    txFrame.data[7] = 0xEE;

    txFrame.id = 0x20;
    txFrame.dlc = 8;
    uint64_t timeout = AP_HAL::micros64() + 10000ULL;
    _frotend_ptr->write_frame(txFrame, timeout);
}
