#include "HB1_message.h"

void HB1_message::swap_message_sub(uint8_t &p1, uint8_t &p2) {
    uint8_t tmp;
    tmp = p1;
    p1 = p2;
    p2 = tmp;
}


void HB1_message::swap_message_sub(uint8_t *p1, uint8_t start_pos, uint8_t swap_num) {
    if (start_pos+swap_num >= sizeof(*p1)) {return;}
    if (swap_num == 2) {swap_message_sub(p1[start_pos], p1[start_pos+1]);}
    if (swap_num == 4) {swap_message_sub(p1[start_pos], p1[start_pos+1], p1[start_pos+2], p1[start_pos+3]);}
}