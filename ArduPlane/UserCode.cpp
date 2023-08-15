#include "Plane.h"

void Plane::userhook_50Hz()
{
    ufollow.update();
}

void Plane::userhook_1Hz()
{
    ufollow.print();
}
