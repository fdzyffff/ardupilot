#include "Plane.h"

void Plane::userhook_init()
{
    uart.init();
    uattack.init();
    udelay.init();
    uengines.init();
    uweight.init();
}

void Plane::userhook_100Hz()
{
    uattack.update();
    udelay.push();
    uengines.update();
    uweight.update();
}

void Plane::userhook_1Hz()
{
    AP::fd_data().set_is_flying(is_flying());
    AP::fd_data().update();
    // ufollow.update();

    uattack.do_print();
}
