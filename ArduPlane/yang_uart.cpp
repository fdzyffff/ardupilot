#include "Plane.h"

void Plane::yang_uart_update()
{
    if (yang_uart.initialized()) {
        yang_uart.update();
    }
}

void Plane::yang_uart_test(uint8_t msg_id)
{
    switch (msg_id) {
        case 1:
            yang_uart_test_msg1();
            break;
        case 2:
            yang_uart_test_msg2();
            break;
        case 3:
            //yang_uart_test_msg3();
            break;
        default:
            break;
    }
}

void Plane::yang_uart_test_msg1(){
    YANG_UART::YANGUART_MSG_1 *tmp_msg = &yang_uart.get_msg_1();
    tmp_msg->updated = true;
    tmp_msg->content.msg.header.head_1 = YANG_UART::PREAMBLE1;
    tmp_msg->content.msg.header.head_2 = YANG_UART::PREAMBLE2;
    tmp_msg->content.msg.header.index = YANG_UART::INDEX1;

    tmp_msg->content.msg.console_type = tmp_msg->length;
    tmp_msg->content.msg.remote_index = 1;
    tmp_msg->content.msg.remote_cmd.cmd_1.p1 = 23;
    tmp_msg->content.msg.remote_cmd.cmd_1.p2 = 33;
    tmp_msg->content.msg.remote_cmd.cmd_1.longitude = 123456;
    tmp_msg->content.msg.remote_cmd.cmd_1.latitude = 654321;
    tmp_msg->content.msg.remote_cmd.cmd_1.alt = 250;
    tmp_msg->content.msg.unused[0] = 0;
    tmp_msg->content.msg.unused[1] = 0;
    tmp_msg->content.msg.unused[2] = 0;
    tmp_msg->content.msg.unused[3] = 0;
    tmp_msg->content.msg.sum_check = 0;
    
    for (int8_t i = 2; i < tmp_msg->length - 1; i++) {
        tmp_msg->content.msg.sum_check += tmp_msg->content.data[i];
    }
}

void Plane::yang_uart_test_msg2(){
    YANG_UART::YANGUART_MSG_1 *tmp_msg = &yang_uart.get_msg_1();
    tmp_msg->updated = true;
    tmp_msg->content.msg.header.head_1 = YANG_UART::PREAMBLE1;
    tmp_msg->content.msg.header.head_2 = YANG_UART::PREAMBLE2;
    tmp_msg->content.msg.header.index = YANG_UART::INDEX1;

    tmp_msg->content.msg.console_type = tmp_msg->length;
    tmp_msg->content.msg.remote_index = 1;
    tmp_msg->content.msg.remote_cmd.cmd_1.p1 = 23;
    tmp_msg->content.msg.remote_cmd.cmd_1.p2 = 33;
    tmp_msg->content.msg.remote_cmd.cmd_1.longitude = 123456;
    tmp_msg->content.msg.remote_cmd.cmd_1.latitude = 654321;
    tmp_msg->content.msg.remote_cmd.cmd_1.alt = 250;
    tmp_msg->content.msg.unused[0] = 0;
    tmp_msg->content.msg.unused[1] = 0;
    tmp_msg->content.msg.unused[2] = 0;
    tmp_msg->content.msg.unused[3] = 0;
    tmp_msg->content.msg.sum_check = 0;
    
    for (int8_t i = 2; i < tmp_msg->length - 1; i++) {
        tmp_msg->content.msg.sum_check += tmp_msg->content.data[i];
    }
}
