#include "FD_msg_DYT2_cmd.h"

#include <AP_Math/AP_Math.h>

FD_msg_DYT2_cmd::FD_msg_DYT2_cmd(void)
{
    _enable = false;
    _msg_1.need_send = false;
    _msg_1.updated = false;
    _msg_2.need_send = false;
    _msg_2.updated = false;
}

void FD_msg_DYT2_cmd::pack_set(uint8_t b, int16_t c, int16_t d)
{
    MSG_Set &msg = _msg_1.content.msg;
    msg.head_1 = PREAMBLE1;
    msg.head_2 = PREAMBLE2;
    msg.id = MSG_ID_SET;
    msg.len = FD_MSG_DYT2_CMD_SET_LEN;
    msg.b = b;
    msg.c = c;
    msg.d = d;
    _msg_1.length = FD_MSG_DYT2_CMD_SET_LEN;
    make_sum_set();
    _msg_1.need_send = true;
}

void FD_msg_DYT2_cmd::pack_center()
{
    pack_set(B_CENTER, 0, 0);
}

void FD_msg_DYT2_cmd::pack_auto_track()
{
    pack_set(B_AUTO_TRACK, 0, 0);
}

void FD_msg_DYT2_cmd::pack_report_rate(uint8_t cmd, uint8_t level)
{
    pack_set(cmd, (int16_t)level, 0);
}

void FD_msg_DYT2_cmd::pack_vehicle_state(const State &state)
{
    MSG_State &msg = _msg_2.content.msg;
    msg.head_1 = PREAMBLE1;
    msg.head_2 = PREAMBLE2;
    msg.id = MSG_ID_STATE;
    msg.len = FD_MSG_DYT2_CMD_STATE_LEN;
    msg.vel_n = (int16_t)(state.vel_n_ms * 100.0f);
    msg.vel_e = (int16_t)(state.vel_e_ms * 100.0f);
    msg.vel_d = (int16_t)(state.vel_d_ms * 100.0f);
    msg.ground_speed = (int16_t)(state.ground_speed_ms / 0.2f);
    msg.pitch = (int16_t)(state.pitch_deg * 100.0f);
    msg.roll = (int16_t)(state.roll_deg * 100.0f);
    msg.heading = (uint16_t)(wrap_360(state.heading_deg) * 100.0f);
    msg.tilt = (uint16_t)(state.tilt_deg * 10.0f);
    msg.dist_reframe = state.dist_reframe;
    msg.timestamp_ms = state.timestamp_ms;
    msg.reserved = 0;
    _msg_2.length = FD_MSG_DYT2_CMD_STATE_LEN;
    make_sum_state();
    _msg_2.need_send = true;
}

void FD_msg_DYT2_cmd::make_sum_set()
{
    MSG_Set &msg = _msg_1.content.msg;
    // 校验和: 字节1..字节N-1 累加和取低字节（sum 自己先清 0 再累加）
    msg.sum = 0;
    uint8_t sum = 0;
    for (uint16_t i = 0; i + 1 < FD_MSG_DYT2_CMD_SET_LEN; i++) {
        sum = (uint8_t)(sum + _msg_1.content.data[i]);
    }
    msg.sum = sum;
}

void FD_msg_DYT2_cmd::make_sum_state()
{
    MSG_State &msg = _msg_2.content.msg;
    msg.sum = 0;
    uint8_t sum = 0;
    for (uint16_t i = 0; i + 1 < FD_MSG_DYT2_CMD_STATE_LEN; i++) {
        sum = (uint8_t)(sum + _msg_2.content.data[i]);
    }
    msg.sum = sum;
}
