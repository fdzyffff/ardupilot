#include "FD1_message.h"

void FD1_message::swap_message_sub(uint8_t &p1, uint8_t &p2) {
    uint8_t tmp;
    tmp = p1;
    p1 = p2;
    p2 = tmp;
}

float FD1_message::swap_message_float(float a1) {
    union PACKED{
        float v;
        uint8_t data[4];
    } a;

    a.v = a1;
    uint8_t tmp;
    tmp = a.data[0];
    a.data[0] = a.data[3];
    a.data[3] = tmp;
    tmp = a.data[1];
    a.data[1] = a.data[2];
    a.data[2] = tmp;
    a1 = a.v;
    return a1;
}

double FD1_message::swap_message_double(double a1) {
    union PACKED{
        double v;
        uint8_t data[8];
    } a;

    a.v = a1;
    uint8_t tmp;
    tmp = a.data[0];
    a.data[0] = a.data[7];
    a.data[7] = tmp;
    tmp = a.data[1];
    a.data[1] = a.data[6];
    a.data[6] = tmp;
    tmp = a.data[2];
    a.data[2] = a.data[5];
    a.data[5] = tmp;
    tmp = a.data[3];
    a.data[3] = a.data[4];
    a.data[4] = tmp;
    a1 = a.v;
    return a1;
}

int32_t FD1_message::swap_message_int32_t(int32_t a1) {
    union PACKED{
        int32_t v;
        uint8_t data[4];
    } a;

    a.v = a1;
    uint8_t tmp;
    tmp = a.data[0];
    a.data[0] = a.data[3];
    a.data[3] = tmp;
    tmp = a.data[1];
    a.data[1] = a.data[2];
    a.data[2] = tmp;
    a1 = a.v;
    return a1;
}

uint32_t FD1_message::swap_message_uint32_t(uint32_t a1) {
    union PACKED{
        uint32_t v;
        uint8_t data[4];
    } a;

    a.v = a1;
    uint8_t tmp;
    tmp = a.data[0];
    a.data[0] = a.data[3];
    a.data[3] = tmp;
    tmp = a.data[1];
    a.data[1] = a.data[2];
    a.data[2] = tmp;
    a1 = a.v;
    return a1;
}

int16_t FD1_message::swap_message_int16_t(int16_t a1) {
    union PACKED{
        int16_t v;
        uint8_t data[2];
    } a;

    a.v = a1;
    uint8_t tmp;
    tmp = a.data[0];
    a.data[0] = a.data[1];
    a.data[1] = tmp;
    a1 = a.v;
    return a1;
}

uint16_t FD1_message::swap_message_uint16_t(uint16_t a1) {
    union PACKED{
        uint16_t v;
        uint8_t data[2];
    } a;

    a.v = a1;
    uint8_t tmp;
    tmp = a.data[0];
    a.data[0] = a.data[1];
    a.data[1] = tmp;
    a1 = a.v;
    return a1;
}
