#include "FD1_message.h"

void FD1_message::swap_message_sub(uint8_t &p1, uint8_t &p2) {
    uint8_t tmp;
    tmp = p1;
    p1 = p2;
    p2 = tmp;
}

void FD1_message::swap_message_sub(uint8_t &p1, uint8_t &p2, uint8_t &p3, uint8_t &p4) {
    swap_message_sub(p1, p4);
    swap_message_sub(p2, p3);
}

void FD1_message::swap_message_sub2(int16_t &bytes_in) {
    // uint8_t data[2];
    // (int16_t *)data = &bytes_in;
    // swap_message_sub(data[0]. data[1]);
}

void FD1_message::swap_message_sub2(uint16_t &bytes_in)      {
    ;
}

void FD1_message::swap_message_sub4(int32_t &bytes_in) {
    ;
}

void FD1_message::swap_message_sub4(uint32_t &bytes_in) {
    ;
}

void FD1_message::swap_message_sub4(float &bytes_in) {
    ;
}