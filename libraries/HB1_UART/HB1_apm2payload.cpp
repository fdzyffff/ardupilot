#include "HB1_apm2payload.h"

HB1_apm2payload::HB1_apm2payload(void)
{
    _enable = false;
    _msg_1.need_send = false;
    _msg_1.updated = false;
}

void HB1_apm2payload::cal_sumcheck(void)
{
    _msg_1.content.data[0] = PREAMBLE1;
    _msg_1.content.data[1] = PREAMBLE2;
    _msg_1.content.msg.sum_check = 0;
    for (int i = 0; i < _msg_1.length-1; i ++) {
        _msg_1.content.msg.sum_check += _msg_1.content.data[i];
    }
}
