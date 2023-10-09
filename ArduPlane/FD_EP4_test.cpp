#include "Plane.h"

void EP4_ctrl_t::test_EP4_uart(uint8_t msg_id, uint8_t option)
{
    switch (msg_id) {
        case 1:
            test_EP4_in(option);
            break;
        case 2:
            test_EP4_out(option);
            break;
        default:
            break;
    }
}

void EP4_ctrl_t::test_EP4_in(uint8_t option) {
    static uint8_t n_count = 0;
    FD1_msg_ep4_in &tmp_msg = uart_msg_ep4.get_msg_ep4_in();
    tmp_msg._msg_1.content.msg.ecu_status      = 1 ;     
    tmp_msg._msg_1.content.msg.damper          = 1 ;      //unit: 0.1%, value = real * 10
    tmp_msg._msg_1.content.msg.rpm             = 1 ;      //rpm
    tmp_msg._msg_1.content.msg.cylinder_temp1  = 1 ;      //0.1 degree
    tmp_msg._msg_1.content.msg.cylinder_temp2  = 1 ;      //0.1 degree
    tmp_msg._msg_1.content.msg.venting_temp1   = 1 ;      //1 degree
    tmp_msg._msg_1.content.msg.venting_temp2   = 1 ;      //1 degree
    tmp_msg._msg_1.content.msg.fuel_pressure   = 1 ;      //1 mbar
    tmp_msg._msg_1.content.msg.fuel_time       = 1 ;      //micro second
    tmp_msg._msg_1.content.msg.ignite_angle    = 1 ;      //0.1 degree
    tmp_msg._msg_1.content.msg.baro_temp       = 1 ;      //1 degree
    tmp_msg._msg_1.content.msg.baro_pressure   = 1 ;      //1 mbar
    tmp_msg._msg_1.content.msg.ecu_power_volt  = 1 ;      //0.1 V
    tmp_msg._msg_1.content.msg.empty1          = 1 ;      //empty byte
    tmp_msg._msg_1.content.msg.ecu_inner_volt  = 1 ;      //0.1 V
    tmp_msg._msg_1.content.msg.ecu_temp        = 1 ;      //0.1 degree
    tmp_msg._msg_1.content.msg.empty2          = 1 ;      //empty byte
    tmp_msg._msg_1.content.msg.ecu_serial      = 1 ;      //fixed value
    tmp_msg._msg_1.content.msg.warning_code    = 1 ;      //warning code
    tmp_msg._msg_1.content.msg.count           = n_count ;//0~255

    tmp_msg._msg_1.content.msg.header.head_1 = FD1_msg_ep4_in::PREAMBLE1;
    tmp_msg._msg_1.content.msg.header.head_2 = FD1_msg_ep4_in::PREAMBLE2;
    tmp_msg._msg_1.content.msg.sum_check     = 0 ;      //byte 3 to 41

    for (int8_t i = 2; i < tmp_msg._msg_1.length - 1; i++) {
        tmp_msg._msg_1.content.msg.sum_check += tmp_msg._msg_1.content.data[i];
    }
    tmp_msg._msg_1.updated = true;
    tmp_msg._msg_1.need_send = true;
    n_count++;
}

void EP4_ctrl_t::test_EP4_out(uint8_t option) {
    static uint8_t n_count = 0;

    FD1_msg_ep4_out &tmp_msg = uart_msg_ep4.get_msg_ep4_out();

    tmp_msg._msg_1.content.msg.sum_check = 0;
    tmp_msg._msg_1.content.msg.damper = 1;
    tmp_msg._msg_1.content.msg.empty1 = 0;
    tmp_msg._msg_1.content.msg.count = n_count;
    tmp_msg._msg_1.content.msg.header.head_1 = FD1_msg_ep4_out::PREAMBLE1;
    tmp_msg._msg_1.content.msg.header.head_2 = FD1_msg_ep4_out::PREAMBLE2;

    for (int8_t i = 2; i < tmp_msg._msg_1.length - 1; i++) {
        tmp_msg._msg_1.content.msg.sum_check += tmp_msg._msg_1.content.data[i];
    }
    tmp_msg._msg_1.need_send = true;
    n_count++;
}
