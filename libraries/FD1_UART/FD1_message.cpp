#include "FD1_message.h"

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

void FD1_message::fill_int16_t(uint8_t* data, int16_t a1) {
    union PACKED{
        int16_t v;
        uint8_t data[2];
    } a;

    a.v = a1;
    data[0] = a.data[0]; 
    data[1] = a.data[1]; 
}

void FD1_message::fill_uint16_t(uint8_t* data, uint16_t a1) {
    union PACKED{
        uint16_t v;
        uint8_t data[2];
    } a;

    a.v = a1;
    data[0] = a.data[0]; 
    data[1] = a.data[1]; 
}

void FD1_message::fill_int32_t(uint8_t* data, int32_t a1) {
    union PACKED{
        int32_t v;
        uint8_t data[4];
    } a;

    a.v = a1;
    data[0] = a.data[0]; 
    data[1] = a.data[1]; 
    data[2] = a.data[2]; 
    data[3] = a.data[3]; 
}

void FD1_message::fill_uint32_t(uint8_t* data, uint32_t a1) {
    union PACKED{
        uint32_t v;
        uint8_t data[4];
    } a;

    a.v = a1;
    data[0] = a.data[0]; 
    data[1] = a.data[1]; 
    data[2] = a.data[2]; 
    data[3] = a.data[3]; 
}

void FD1_message::fill_float(uint8_t* data, float a1) {
    union PACKED{
        float v;
        uint8_t data[4];
    } a;

    a.v = a1;
    data[0] = a.data[0]; 
    data[1] = a.data[1]; 
    data[2] = a.data[2]; 
    data[3] = a.data[3]; 
}
