#pragma once
#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>

class USrv {

public:

    // constructor, destructor
    USrv();

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

    struct {
        float roll;
        float pitch;
        float yaw;
    } srv_in;

    void init();
    void update();
    void print();

private:

    struct {
        float smotor1;
        float smotor2;
        float smotor3;
        float smotor4;
    } srv_out;
};
