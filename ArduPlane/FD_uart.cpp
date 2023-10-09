#include "Plane.h"

void Plane::FD1_uart_init() {
    ep4_ctrl.init();
    ts_ctrl.init();
}

void Plane::FD1_uart_update() {
    ep4_ctrl.update();
    ts_ctrl.update();
}
