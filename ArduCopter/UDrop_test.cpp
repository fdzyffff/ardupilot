#include "Copter.h"

void UDrop::handle_info(float p1, float p2, float p3, float p4)
{
    switch ((int16_t)p1) {
        case 1:
            do_test_ID1((int16_t)p2, is_zero(p3));
            gcs().send_text(MAV_SEVERITY_INFO, "Test ID 1");
            break;
        case 2:
            do_test_ID2((int16_t)p2, is_zero(p3));
            gcs().send_text(MAV_SEVERITY_INFO, "Test ID 2");
            break;
        case 6:
            do_test_ID6((int16_t)p2, is_zero(p3));
            gcs().send_text(MAV_SEVERITY_INFO, "Test ID 6");
            break;
        default:
            break;
    }
}

void UDrop::do_test_ID1(int16_t p2, bool need_send)
{
    if (!uart_msg_drop.initialized()) {return;}
    FD1_msg_ID1 &temp_msg = uart_msg_drop.get_msg_ID1();
    switch (p2) {
        case 1: 
            temp_msg._msg_1.content.msg.status = 0x01;
            break;
        case 3: 
            temp_msg._msg_1.content.msg.status = 0x03;
            break;
        case 5: 
            temp_msg._msg_1.content.msg.status = 0x05;
            break;
        case 9: 
            temp_msg._msg_1.content.msg.status = 0x09;
            break;
        default:
            temp_msg._msg_1.content.msg.status = 0x00;
            break;
    }
    temp_msg.cal_sumcheck();
    temp_msg._msg_1.updated = true;
    temp_msg._msg_1.need_send = need_send;
    uart_msg_drop.write();
}

void UDrop::do_test_ID2(int16_t p2, bool need_send)
{
    if (!uart_msg_drop.initialized()) {return;}
    FD1_msg_ID2 &temp_msg = uart_msg_drop.get_msg_ID2();
    // gcs().send_text(MAV_SEVERITY_INFO,"p2 %d", p2);
    switch (p2) {
        case 0: 
            temp_msg._msg_1.content.msg.lat = -353624276;
            temp_msg._msg_1.content.msg.lng = 1491655357;
            temp_msg._msg_1.content.msg.alt = 660;
            break;
        case 1: 
            temp_msg._msg_1.content.msg.lat = 399877618;
            temp_msg._msg_1.content.msg.lng = 1163641191;
            temp_msg._msg_1.content.msg.alt = 55;
            break;
        default:
            temp_msg._msg_1.content.msg.lat = 0;
            temp_msg._msg_1.content.msg.lng = 0;
            temp_msg._msg_1.content.msg.alt = 0;
            break;
    }
    temp_msg.cal_sumcheck();
    temp_msg._msg_1.updated = true;
    temp_msg._msg_1.need_send = need_send;
    uart_msg_drop.write();
}

void UDrop::do_test_ID6(int16_t p2, bool need_send)
{
    if (!uart_msg_drop.initialized()) {return;}
    FD1_msg_ID6 &temp_msg = uart_msg_drop.get_msg_ID6();
    temp_msg._msg_1.content.msg.launch = 1;
    temp_msg.cal_sumcheck();
    temp_msg._msg_1.updated = true;
    temp_msg._msg_1.need_send = need_send;
    uart_msg_drop.write();
}