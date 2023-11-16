#include "Plane.h"

void TS_ctrl_t::test_ts_uart(int16_t msg_id, uint8_t option)
{
    switch (msg_id) {
        case 1:
        case 2:
            _test_ts_id = msg_id;
            test_ts_start();
            break;
        case -1:
            _test_ts_id = msg_id;
            test_ts_stop();
            break;
        default:
            break;
    }
}

void TS_ctrl_t::test_ts_start()
{
    _test_ts_running = true;
    gcs().send_text(MAV_SEVERITY_INFO, "test_ts_start [%d]", _test_ts_id);
}

void TS_ctrl_t::test_ts_stop()
{
    _test_ts_running = false;
    gcs().send_text(MAV_SEVERITY_INFO, "test_ts_stop");
}

void TS_ctrl_t::test_ts_update()
{
    if (!_test_ts_running) {
        return;
    }
    switch (_test_ts_id) {
        case 1:
            test_ts_AHRS();
            break;
        case 2:
            test_ts_40();
            break;
        default:
            break;
    }
}


void TS_ctrl_t::test_ts_AHRS()
{
    FD1_msg_ts &tmp_msg = uart_msg_ts.get_msg_ts_in();

    if (tmp_msg._msg_1.content.msg.header.id == 0xB1) {
        gcs().send_text(MAV_SEVERITY_WARNING, "roll_angle: %d", tmp_msg._msg_1.content.msg.sub_msg.msg_m.roll_angle);
        gcs().send_text(MAV_SEVERITY_WARNING, "pitch_angle: %d", tmp_msg._msg_1.content.msg.sub_msg.msg_m.pitch_angle);
        gcs().send_text(MAV_SEVERITY_WARNING, "yaw_angle: %d", tmp_msg._msg_1.content.msg.sub_msg.msg_m.yaw_angle);
    }
    // tmp_msg._msg_1.content.msg.header.head_1 = FD1_msg_ts::PREAMBLE1;
    // tmp_msg._msg_1.content.msg.header.head_2 = FD1_msg_ts::PREAMBLE2;
    // tmp_msg._msg_1.content.msg.header.head_3 = FD1_msg_ts::PREAMBLE3;
    // tmp_msg.set_id(0xB1);

    // tmp_msg._msg_1.content.msg.msg_m.type = 0x02;
    // tmp_msg._msg_1.content.msg.msg_m.empty[0]=0;
    // tmp_msg._msg_1.content.msg.msg_m.empty[1]=0;
    // tmp_msg._msg_1.content.msg.msg_m.empty[2]=0;
    // tmp_msg._msg_1.content.msg.msg_m.empty[3]=0;
    // tmp_msg._msg_1.content.msg.msg_m.empty[4]=0;
    // tmp_msg._msg_1.content.msg.msg_m.empty[5]=0;
    // tmp_msg._msg_1.content.msg.msg_m.empty[6]=0;
    // tmp_msg._msg_1.content.msg.msg_m.roll_angle   = (int16_t)((double)(ahrs.pitch_sensor)/100.f * tmp_msg.SF_INT16);
    // tmp_msg._msg_1.content.msg.msg_m.pitch_angle  = (int16_t)((double)(ahrs.roll_sensor)/100.f * tmp_msg.SF_INT16);
    // tmp_msg._msg_1.content.msg.msg_m.yaw_angle    = (int16_t)((double)(ahrs.yaw_sensor)/100.f * tmp_msg.SF_INT16);
    // tmp_msg._msg_1.content.msg.msg_m.date[1]=0;
    // tmp_msg._msg_1.content.msg.msg_m.date[1]=year_out&0b01111111;
    // tmp_msg._msg_1.content.msg.msg_m.date[1]+=(month_out&0b00001111)<<7;
    // tmp_msg._msg_1.content.msg.msg_m.date[0]=0;
    // tmp_msg._msg_1.content.msg.msg_m.date[0]=(month_out&0b00001111)>>1;
    // tmp_msg._msg_1.content.msg.msg_m.date[0]+=(day_out&0b00011111)<<3;
    // tmp_msg._msg_1.content.msg.msg_m.time[2]=second_day%255;
    // tmp_msg._msg_1.content.msg.msg_m.time[1]=(second_day/255)%255;
    // tmp_msg._msg_1.content.msg.msg_m.time[0]=(second_day/255)/255;;
    // tmp_msg._msg_1.content.msg.msg_m.gps_heading  = (int16_t)(gps.ground_course_cd()*0.01f * tmp_msg.SF_INT16);
    // tmp_msg._msg_1.content.msg.msg_m.empty1=0;
    // tmp_msg._msg_1.content.msg.msg_m.latitude     = gps.location().latitude;
    // tmp_msg._msg_1.content.msg.msg_m.longitude    = gps.location().longitude;
    // tmp_msg._msg_1.content.msg.msg_m.alt          = gps.location().longitude*10;
    // tmp_msg._msg_1.content.msg.msg_m.gps_vel_xy   = (int16_t)(gps.ground_speed() * 100.f);
    // tmp_msg._msg_1.content.msg.msg_m.gps_hdop     = gps.get_hdop(); // already *100 when read in AP_GPS
    // tmp_msg._msg_1.content.msg.msg_m.gps_vdop     = gps.get_vdop(); // already *100 when read in AP_GPS
    // tmp_msg._msg_1.content.msg.msg_m.gps_vel_z    = (int16_t)(gps.velocity().z * 100.f);
}

void TS_ctrl_t::test_ts_40()
{
    // FD1_msg_ts &tmp_msg = uart_msg_ts.get_msg_ts_in();

    FD1_msg_ts &tmp_msg = uart_msg_ts.get_msg_ts_in();
    tmp_msg._msg_1.content.msg.header.head_1 = FD1_msg_ts::PREAMBLE1;
    tmp_msg._msg_1.content.msg.header.head_2 = FD1_msg_ts::PREAMBLE2;
    tmp_msg._msg_1.content.msg.header.id = 0x40;
    tmp_msg._msg_1.content.msg.sub_msg.msg_40.sub_k1.target_ret = 0b00100000;
    tmp_msg._msg_1.content.msg.sub_msg.msg_40.sub_f1.status = 0b00010000;
    tmp_msg._msg_1.content.msg.sub_msg.msg_40.sub_t1.target_lat = -353661132;
    tmp_msg._msg_1.content.msg.sub_msg.msg_40.sub_t1.target_lng = 1491720414;
    tmp_msg._msg_1.content.msg.sub_msg.msg_40.sub_t1.target_alt = 1;  
    tmp_msg.sum_check(); 
    tmp_msg._msg_1.updated = true;
    // tmp_msg._msg_1.need_send = true;
}
