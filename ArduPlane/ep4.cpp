#include "Plane.h"

void Plane::FD1_uart_init() {
    FD1_uart_msg_ep4.init();
    FD1_uart_msg_ep4.get_msg_ep4_in().set_enable();
}