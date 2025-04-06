#pragma once

#include <AP_HAL/AP_HAL.h>

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
