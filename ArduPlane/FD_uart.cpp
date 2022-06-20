#include "Plane.h"

void Plane::FD1_uart_init() {
    FD1_uart_ep4_init();
    FD1_uart_ts_init();
}

void Plane::FD1_uart_update() {
    FD1_uart_ep4_update();
    FD1_uart_ts_update();
}
