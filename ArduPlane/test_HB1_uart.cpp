#include "Plane.h"


void Plane::test_HB1_uart(uint8_t msg_id)
{
    switch (msg_id) {
        case 1:
            test_HB1_uart_msg1();
            break;
        case 2:
            test_HB1_uart_msg2();
            break;
        case 3:
            test_HB1_uart_msg3();
            break;
        case 4:
            test_HB1_uart_msg4();
            break;
        case 5:
            test_HB1_uart_msg5();
            break;
        case 6:
            test_HB1_uart_msg6();
            break;
        case 7:
            test_HB1_uart_msg7();
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

void Plane::test_HB1_uart_msg1(){
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
    
    tmp_msg._msg_1.content.msg.rpm = 10000;
    tmp_msg._msg_1.content.msg.temp = 700;
    for (int8_t i = 0; i < tmp_msg._msg_1.length - 1; i++) {
        tmp_msg._msg_1.content.msg.sum_check += tmp_msg._msg_1.content.data[i];
    }
}

void Plane::test_HB1_uart_msg2(){
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

void Plane::test_HB1_uart_msg3(){
    gcs().send_text(MAV_SEVERITY_INFO, "SIM mission2apm :");
    HB1_mission2apm &tmp_msg = HB1_uart_mission.get_msg_mission2apm();
    tmp_msg._msg_1.updated = true;
    tmp_msg._msg_1.need_send = true;
    tmp_msg._msg_1.print = true;
    tmp_msg._msg_1.content.msg.header.head_1 = HB1_mission2apm::PREAMBLE1;
    tmp_msg._msg_1.content.msg.header.head_2 = HB1_mission2apm::PREAMBLE2;
    tmp_msg._msg_1.content.msg.header.index = HB1_mission2apm::INDEX1;
    tmp_msg._msg_1.content.msg.length = tmp_msg._msg_1.length-4;
    tmp_msg._msg_1.content.msg.sum_check = 0;

    static uint8_t pre_index = 0;
    for (int8_t i = 4; i < tmp_msg._msg_1.length - 1; i++) {
        tmp_msg._msg_1.content.data[i] = 0;
    }
    
    if (pre_index != 0xA5){
        tmp_msg._msg_1.content.msg.remote_index = 0xA5;
    } else {
        tmp_msg._msg_1.content.msg.remote_index = 0xC6;
    }
    pre_index = tmp_msg._msg_1.content.msg.remote_index;
    for (int8_t i = 2; i < tmp_msg._msg_1.length - 1; i++) {
        tmp_msg._msg_1.content.msg.sum_check += tmp_msg._msg_1.content.data[i];
    }
/*    tmp_msg._msg_1.content.data[0] = 0xEE;
    tmp_msg._msg_1.content.data[1] = 0x16;
    tmp_msg._msg_1.content.data[2] = 0xAA;
    tmp_msg._msg_1.content.data[3] = 0x2F;
    tmp_msg._msg_1.content.data[4] = 0x00;
    tmp_msg._msg_1.content.data[5] = 0x63;
    tmp_msg._msg_1.content.data[50] = 0x3D;*/
}

void Plane::test_HB1_uart_msg4(){
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

void Plane::test_HB1_uart_msg5(){
    gcs().send_text(MAV_SEVERITY_INFO, "SIM apm2power :");
    HB1_apm2power &tmp_msg = HB1_uart_power.get_msg_apm2power();
    tmp_msg._msg_1.updated = true;
    tmp_msg._msg_1.need_send = true;
    tmp_msg._msg_1.print = true;
    tmp_msg._msg_1.content.msg.header.head_1 = HB1_apm2power::PREAMBLE1;
    tmp_msg._msg_1.content.msg.header.head_2 = HB1_apm2power::PREAMBLE2;
    tmp_msg._msg_1.content.msg.sum_check = 0;

    for (int8_t i = 2; i < tmp_msg._msg_1.length - 1; i++) {
        tmp_msg._msg_1.content.data[i] = 0;
    }
    
    for (int8_t i = 0; i < tmp_msg._msg_1.length - 1; i++) {
        tmp_msg._msg_1.content.msg.sum_check += tmp_msg._msg_1.content.data[i];
    }
}

void Plane::test_HB1_uart_msg6(){
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
    
    for (int8_t i = 2; i < tmp_msg._msg_1.length - 1; i++) {
        tmp_msg._msg_1.content.msg.sum_check += tmp_msg._msg_1.content.data[i];
    }
}

void Plane::test_HB1_uart_msg7(){
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