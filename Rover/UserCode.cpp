#include "Rover.h"

void Rover::userhook_init()
{
    // put your initialisation code here
    // this will be called once at start-up
    ufence.init();
}

void Rover::userhook_FastLoop()
{
    // put your 100Hz code here
    ufence.update();
}