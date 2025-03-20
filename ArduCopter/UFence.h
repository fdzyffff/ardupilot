#pragma once

#include <AP_HAL/AP_HAL.h>

#define UFence_BUFFER 100
class UFence {
public:
    UFence();
    
    void init();
    void update();
    bool infence();
    bool triggered();
    void set_triggered(bool b);

private:
    bool _triggered;
    Location _last_loc;
};
