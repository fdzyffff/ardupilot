#include "Plane.h"

void Plane::userhook_init()
{
    uart.init();
    uattack.init();
    udelay.init();
}

void Plane::userhook_100Hz()
{
    uattack.update();
    udelay.push();
}

void Plane::userhook_1Hz()
{
    AP::fd_data().update();

    uattack.do_print();
}
