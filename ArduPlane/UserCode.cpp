#include "Plane.h"

void Plane::userhook_init()
{
    uctrl.init();
}

void Plane::userhook_50Hz()
{
    ufollow.update();
    uctrl.update();
}

void Plane::userhook_1Hz()
{
    // ufollow.print();
}
