#include "Plane.h"

void Plane::FD1_uart_ts_AHRS_test()
{
    FD1_msg_ts &tmp_msg = FD1_uart_msg_ts.get_msg_ts_in();

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
