#pragma once

#include <AP_HAL/AP_HAL.h>
#include <FD1_UART/FD1_UART.h>

class UBIM {

public:

    friend class Copter;
    friend class ModeAttack;

    // constructor, destructor
    UBIM();

    // var_info for holding Parameter information
    // static const struct AP_Param::GroupInfo var_info[];

    void init();
    void update();
    void update_log();


    class enum BIM_STATUS {
        STANDBY = 0,
        AUTO = 1,
        HOVER,
        MANUAL,
        LAND,
    };

    bool uav_armed;

private:

    FD1_UART uart_bim;
};
