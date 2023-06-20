#include "Plane.h"


void Plane::test_HB1_uart(uint8_t msg_id, uint8_t option)
{
    switch (msg_id) {
        case 1:
            test_HB1_uart_msg1(option);
            break;
        case 2:
            test_HB1_uart_msg2(option);
            break;
        case 3:
            test_HB1_uart_msg3(option);
            break;
        case 4:
            test_HB1_uart_msg4(option);
            break;
        case 5:
            test_HB1_uart_msg5(option);
            break;
        case 6:
            test_HB1_uart_msg6(option);
            break;
        case 7:
            test_HB1_uart_msg7(option);
            break;
        default:
            break;
    }
}

//msg_8: all 
//msg_7: apm2cam     64
//msg_6: apm2mission 32
//msg_5: apm2power   16
//msg_4: cam2mission  8
//msg_3: mission2apm  4
//msg_2: mission2cam  2
//msg_1: power2apm    1  

void Plane::test_HB1_uart_msg1(uint8_t option){
    gcs().send_text(MAV_SEVERITY_INFO, "SIM power2apm :");
    HB1_power2apm &tmp_msg = HB1_uart_power.get_msg_power2apm();
    tmp_msg._msg_1.updated = true;
    tmp_msg._msg_1.need_send = true;
    tmp_msg._msg_1.print = true;
    tmp_msg._msg_1.content.msg.header.head_1 = HB1_power2apm::PREAMBLE1;
    tmp_msg._msg_1.content.msg.header.head_2 = HB1_power2apm::PREAMBLE2;
    tmp_msg._msg_1.content.msg.sum_check = 0;

    for (int8_t i = 2; i < tmp_msg._msg_1.length; i++) {
        tmp_msg._msg_1.content.data[i] = 0;
    }
    
    tmp_msg._msg_1.content.msg.CYS350_rpm = 10000;
    tmp_msg._msg_1.content.msg.CYS350_temp = 700;
    for (int8_t i = 0; i < tmp_msg._msg_1.length - 1; i++) {
        tmp_msg._msg_1.content.msg.sum_check += tmp_msg._msg_1.content.data[i];
    }
}

void Plane::test_HB1_uart_msg2(uint8_t option){
    gcs().send_text(MAV_SEVERITY_INFO, "SIM mission2cam :");
    HB1_mission2cam &tmp_msg = HB1_uart_mission.get_msg_mission2cam();
    tmp_msg._msg_1.updated = true;
    tmp_msg._msg_1.need_send = true;
    tmp_msg._msg_1.print = true;
    tmp_msg._msg_1.content.msg.header.head_1 = HB1_mission2cam::PREAMBLE1;
    tmp_msg._msg_1.content.msg.header.head_2 = HB1_mission2cam::PREAMBLE2;
    tmp_msg._msg_1.content.msg.header.index = HB1_mission2cam::INDEX1;
    tmp_msg._msg_1.content.msg.length = tmp_msg._msg_1.length-4;
    tmp_msg._msg_1.content.msg.sum_check = 0;

    for (int8_t i = 4; i < tmp_msg._msg_1.length - 1; i++) {
        tmp_msg._msg_1.content.data[i] = 0;
    }
    tmp_msg._msg_1.content.msg.uav_id = g.sysid_this_mav;
    
    for (int8_t i = 2; i < tmp_msg._msg_1.length - 1; i++) {
        tmp_msg._msg_1.content.msg.sum_check += tmp_msg._msg_1.content.data[i];
    }
}

void Plane::test_HB1_uart_msg3(uint8_t option){
    // done in test_HB1_follwo.cpp test_HB1_mission_update_msg()
/*    gcs().send_text(MAV_SEVERITY_INFO, "SIM mission2apm :");
    HB1_mission2apm &tmp_msg = HB1_uart_mission.get_msg_mission2apm();
    tmp_msg._msg_1.updated = true;
    tmp_msg._msg_1.need_send = true;
    tmp_msg._msg_1.print = true;
    tmp_msg._msg_1.content.msg.header.head_1 = HB1_mission2apm::PREAMBLE1;
    tmp_msg._msg_1.content.msg.header.head_2 = HB1_mission2apm::PREAMBLE2;
    tmp_msg._msg_1.content.msg.header.index = HB1_mission2apm::INDEX1;
    tmp_msg._msg_1.content.msg.length = tmp_msg._msg_1.length-4;
    tmp_msg._msg_1.content.msg.sum_check = 0;

    for (int8_t i = 4; i < tmp_msg._msg_1.length - 1; i++) {
        tmp_msg._msg_1.content.data[i] = 0;
    }
    
    switch (option) {
        case 1: // Speed up
            tmp_msg._msg_1.content.msg.remote_index = 0x3A;
            break;
        case 2: // Speed up
            tmp_msg._msg_1.content.msg.remote_index = 0xA7;
            break;
        case 10: // Rocket ON
            tmp_msg._msg_1.content.msg.remote_index = 0x55;
            break;
        case 11: // EngineStart
            tmp_msg._msg_1.content.msg.remote_index = 0xA5;
            break;
        case 12: // EngineOFF
            tmp_msg._msg_1.content.msg.remote_index = 0xC6;
            break;
        case 13: // EngineFULL
            tmp_msg._msg_1.content.msg.remote_index = 0xE7;
            break;
        case 14: // EngineMID
            tmp_msg._msg_1.content.msg.remote_index = 0xB4;
            break;
        case 15: // Disarm
            tmp_msg._msg_1.content.msg.remote_index = 0xCC;
            break;
        case 16: // ServoTest
            tmp_msg._msg_1.content.msg.remote_index = 0x99;
            break;
        case 17: // ServoTest
            tmp_msg._msg_1.content.msg.remote_index = 0x99;
            break;
        case 21:
            SRV_Channels::set_output_scaled(SRV_Channel::k_launcher_HB1, 100);
            tmp_msg._msg_1.updated = false;
            tmp_msg._msg_1.need_send = false;
            tmp_msg._msg_1.print = false;
            break;
        case 22:
            SRV_Channels::set_output_scaled(SRV_Channel::k_launcher_HB1, 0);
            tmp_msg._msg_1.updated = false;
            tmp_msg._msg_1.need_send = false;
            tmp_msg._msg_1.print = false;
            break;
        default:
            break;
    }

    for (int8_t i = 2; i < tmp_msg._msg_1.length - 1; i++) {
        tmp_msg._msg_1.content.msg.sum_check += tmp_msg._msg_1.content.data[i];
    }

    tmp_msg._msg_1.content.msg.apm_deltaX = 15.f;
    tmp_msg._msg_1.content.msg.apm_deltaY = 25.f;
    tmp_msg._msg_1.content.msg.apm_deltaZ = 35.f;
    tmp_msg._msg_1.content.msg.leader_lng = (int32_t)(100.2222f * tmp_msg.SF_LL);
    tmp_msg._msg_1.content.msg.leader_lat = (int32_t)(10.33333f * tmp_msg.SF_LL);
    tmp_msg._msg_1.content.msg.leader_alt = (int16_t)(100.f * tmp_msg.SF_ALT);

    tmp_msg._msg_1.content.msg.leader_dir = (int16_t)((float)wrap_180_cd(ahrs.yaw_sensor/100) * tmp_msg.SF_ANG);*/

}

void Plane::test_HB1_uart_msg4(uint8_t option){
    gcs().send_text(MAV_SEVERITY_INFO, "SIM cam2mission :");
    HB1_cam2mission &tmp_msg = HB1_uart_cam.get_msg_cam2mission();
    tmp_msg._msg_1.updated = true;
    tmp_msg._msg_1.need_send = true;
    tmp_msg._msg_1.print = true;
    tmp_msg._msg_1.content.msg.header.head_1 = HB1_cam2mission::PREAMBLE1;
    tmp_msg._msg_1.content.msg.header.head_2 = HB1_cam2mission::PREAMBLE2;
    tmp_msg._msg_1.content.msg.header.index = HB1_cam2mission::INDEX1;
    tmp_msg._msg_1.content.msg.length = tmp_msg._msg_1.length-4;
    tmp_msg._msg_1.content.msg.sum_check = 0;

    for (int8_t i = 4; i < tmp_msg._msg_1.length - 1; i++) {
        tmp_msg._msg_1.content.data[i] = 0;
    }
    
    for (int8_t i = 2; i < tmp_msg._msg_1.length - 1; i++) {
        tmp_msg._msg_1.content.msg.sum_check += tmp_msg._msg_1.content.data[i];
    }
}

void Plane::test_HB1_uart_msg5(uint8_t option){
    gcs().send_text(MAV_SEVERITY_INFO, "SIM apm2power :");
    HB1_apm2power &tmp_msg = HB1_uart_power.get_msg_apm2power();
    tmp_msg._msg_1.updated = true;
    tmp_msg._msg_1.need_send = true;
    tmp_msg._msg_1.print = true;
    tmp_msg._msg_1.content.msg.header.head_1 = HB1_apm2power::PREAMBLE1;
    tmp_msg._msg_1.content.msg.header.head_2 = HB1_apm2power::PREAMBLE2;
    tmp_msg._msg_1.content.msg.sum_check = 0;

    tmp_msg._msg_1.content.msg.c[0] = option;
    tmp_msg._msg_1.content.msg.c[1] = option;
    
    for (int8_t i = 0; i < tmp_msg._msg_1.length - 1; i++) {
        tmp_msg._msg_1.content.msg.sum_check += tmp_msg._msg_1.content.data[i];
    }
}

void Plane::test_HB1_uart_msg6(uint8_t option){
    gcs().send_text(MAV_SEVERITY_INFO, "SIM apm2mission :");
    HB1_apm2mission &tmp_msg = HB1_uart_mission.get_msg_apm2mission();
    tmp_msg._msg_1.updated = true;
    tmp_msg._msg_1.need_send = true;
    tmp_msg._msg_1.print = true;
    tmp_msg._msg_1.content.msg.header.head_1 = HB1_apm2mission::PREAMBLE1;
    tmp_msg._msg_1.content.msg.header.head_2 = HB1_apm2mission::PREAMBLE2;
    tmp_msg._msg_1.content.msg.header.index = HB1_apm2mission::INDEX1;
    tmp_msg._msg_1.content.msg.length = tmp_msg._msg_1.length-4;
    tmp_msg._msg_1.content.msg.sum_check = 0;


    for (int8_t i = 4; i < tmp_msg._msg_1.length - 1; i++) {
        tmp_msg._msg_1.content.data[i] = 0;
    }

    tmp_msg._msg_1.content.msg.longitude = (int32_t)(125.2222f * tmp_msg.SF_LL);
    tmp_msg._msg_1.content.msg.latitude = (int32_t)(143.33333f * tmp_msg.SF_LL);
    tmp_msg._msg_1.content.msg.alt = (int16_t)(100.f * tmp_msg.SF_ALT);
    tmp_msg._msg_1.content.msg.ptich = (int16_t)((float)(ahrs.pitch_sensor/100) * tmp_msg.SF_ANG);
    tmp_msg._msg_1.content.msg.roll = (int16_t)((float)(ahrs.roll_sensor/100) * tmp_msg.SF_ANG);
    tmp_msg._msg_1.content.msg.yaw = (int16_t)((float)wrap_180_cd(ahrs.yaw_sensor/100) * tmp_msg.SF_ANG);
    tmp_msg._msg_1.content.msg.air_speed = (int16_t)(100.f * tmp_msg.SF_VEL);
    tmp_msg._msg_1.content.msg.error_code1 = 1;
    tmp_msg._msg_1.content.msg.error_code2 = 2;
    tmp_msg._msg_1.content.msg.rc_code = 5;
    tmp_msg._msg_1.content.msg.target_wp_index = mission.get_current_nav_index();
    tmp_msg._msg_1.content.msg.in_group = 1;
    tmp_msg._msg_1.content.msg.gspd = (int16_t)(15.f * tmp_msg.SF_VEL);
    tmp_msg._msg_1.content.msg.gspd_dir = (int16_t)(333.f * tmp_msg.SF_ANG);

    
    for (int8_t i = 2; i < tmp_msg._msg_1.length - 1; i++) {
        tmp_msg._msg_1.content.msg.sum_check += tmp_msg._msg_1.content.data[i];
    }
}

void Plane::test_HB1_uart_msg7(uint8_t option){
    gcs().send_text(MAV_SEVERITY_INFO, "SIM apm2cam :");
    HB1_apm2cam &tmp_msg = HB1_uart_cam.get_msg_apm2cam();
    tmp_msg._msg_1.updated = true;
    tmp_msg._msg_1.need_send = true;
    tmp_msg._msg_1.print = true;
    tmp_msg._msg_1.content.msg.header.head_1 = HB1_apm2cam::PREAMBLE1;
    tmp_msg._msg_1.content.msg.header.head_2 = HB1_apm2cam::PREAMBLE2;
    tmp_msg._msg_1.content.msg.header.index = HB1_apm2cam::INDEX1;
    tmp_msg._msg_1.content.msg.length = tmp_msg._msg_1.length-4;
    tmp_msg._msg_1.content.msg.sum_check = 0;

    for (int8_t i = 4; i < tmp_msg._msg_1.length - 1; i++) {
        tmp_msg._msg_1.content.data[i] = 0;
    }
    
    for (int8_t i = 2; i < tmp_msg._msg_1.length - 1; i++) {
        tmp_msg._msg_1.content.msg.sum_check += tmp_msg._msg_1.content.data[i];
    }
}


/*void Plane::test_HB1_uart_msg3(){
    HB1_UART::YANGUART_MSG_1 tmp_msg = HB1_uart.get_msg_1();
    tmp_msg.updated = true;
    tmp_msg.content.msg.header.head_1 = HB1_UART::PREAMBLE1;
    tmp_msg.content.msg.header.head_2 = HB1_UART::PREAMBLE2;
    tmp_msg.content.msg.header.index = HB1_UART::INDEX1;

    tmp_msg.content.msg.console_type = tmp_msg.length;
    tmp_msg.content.msg.remote_index = 1;
    tmp_msg.content.msg.remote_cmd.cmd_1.p1 = 23;
    tmp_msg.content.msg.remote_cmd.cmd_1.p2 = 33;
    tmp_msg.content.msg.remote_cmd.cmd_1.longitude = 123456;
    tmp_msg.content.msg.remote_cmd.cmd_1.latitude = 654321;
    tmp_msg.content.msg.remote_cmd.cmd_1.alt = 250;
    tmp_msg.content.msg.unused[0] = 0;
    tmp_msg.content.msg.unused[1] = 0;
    tmp_msg.content.msg.unused[2] = 0;
    tmp_msg.content.msg.unused[3] = 0;
    tmp_msg.content.msg.sum_check = 0;
    
    for (int8_t i = 2; i < tmp_msg.length - 1; i++) {
        tmp_msg.content.msg.sum_check += tmp_msg.content.data[i];
    }
}*/