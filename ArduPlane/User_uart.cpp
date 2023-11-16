#include "Plane.h"

void Plane::FD1_uart_init() {
    engine_init();
    ts_ctrl.init();
}

void Plane::FD1_uart_update() {
    engine_update();
    ts_ctrl.update();
}
