#include "Plane.h"

void Plane::userhook_init()
{
    uart.init();
    uattack.init();
    // udelay.init();  // 已合并到 UAttack 内部
}

void Plane::userhook_100Hz()
{
    uattack.update();
    // udelay.push();  // 已合并到 UAttack 内部
}

void Plane::userhook_1Hz()
{
    AP::fd_data().set_is_flying(is_flying());
    AP::fd_data().update();
    // ufollow.update();

    uattack.do_print();
}
