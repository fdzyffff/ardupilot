#pragma once

#include <FD1_UART/FD1_UART.h>

class UK230 {

public:

    // constructor, destructor
    UK230();

    // initialise
    void init();

    bool is_valid() const { return _valid; }
    bool new_data() {return _new_data;}

    void read_uart();
    void get_target(int8_t tag_num, float x_in, float y_in, float z_in);

    float get_target_x_cm() {return _raw_target_cm.x;}
    float get_target_y_cm() {return _raw_target_cm.y;}
    float get_target_z_cm() {return _raw_target_cm.z;}

    void update();
    void update_valid();

    struct {
        float p1;
        float p2;
        float p3;
        float p4;
        float p11;
        float p12;
        float p13;
        float p14;
        uint16_t count;
    } display_info;

    FD1_UART FD1_uart_K230{AP_SerialManager::SerialProtocol_K230};
private:

    LowPassFilterVector3f _filter_target_cm;

    Vector3f _raw_target_cm;
    uint32_t _last_ms;
    bool _valid;
    bool _new_data;

};
