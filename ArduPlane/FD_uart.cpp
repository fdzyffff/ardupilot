#include "Plane.h"

void Plane::FD1_uart_init() {
    ep4_ctrl.init();
    FD1_uart_ts_init();
}

void Plane::FD1_uart_update() {
    ep4_ctrl.update();
    FD1_uart_ts_update();
}
