#include "FD2_message.h"

void FD2_message::swap_message_sub(uint8_t &p1, uint8_t &p2) {
    uint8_t tmp;
    tmp = p1;
    p1 = p2;
    p2 = tmp;
}
