#include "FD_DATA.h"

// HUOJIAN: 自定义 MAVLink 收发实现（结构参照 Copter-4.7.0-ZFJL 的 FD_DATA_mavlink.cpp）

void FD_DATA::handle_message(mavlink_channel_t chan, const mavlink_message_t &msg)
{
    // HUOJIAN: CHASE 协议下行指令走 COMMAND_INT，由 ArduCopter/UChase 分发；
    // 本入口留给后续自定义报文扩展
    (void)chan;
    (void)msg;
}

void FD_DATA::send_chase_status(mavlink_channel_t chan)
{
    mavlink_chase_status_t pkt {};
    pkt.state = _chase_status.state;
    pkt.armed = _chase_status.armed;
    pkt.gcs_ok = _chase_status.gcs_ok;
    pkt.hb_ok = _chase_status.hb_ok;
    pkt.bbox_valid = _chase_status.bbox_valid;
    pkt.lla_valid = _chase_status.lla_valid;
    pkt.chase_valid = _chase_status.chase_valid;
    pkt.throttle = _chase_status.throttle;
    pkt.relay_state = _chase_status.relay_state;

    mavlink_msg_chase_status_send_struct(chan, &pkt);
}

void FD_DATA::send_chase_tracker(mavlink_channel_t chan)
{
    // HUOJIAN: 跟踪板状态暂无可信来源，按协议占位全 0
    mavlink_chase_tracker_t pkt {};
    mavlink_msg_chase_tracker_send_struct(chan, &pkt);
}

void FD_DATA::send_chase_fuze(mavlink_channel_t chan)
{
    // HUOJIAN: 探测器距离与引信状态由 UDetector/UPayload 提供
    mavlink_chase_fuze_t pkt {};
    pkt.det_dist1 = _chase_fuze.det_dist1;
    pkt.det_dist2 = _chase_fuze.det_dist2;
    pkt.det_dist3 = _chase_fuze.det_dist3;
    pkt.det_dist4 = _chase_fuze.det_dist4;
    pkt.det_selftest = _chase_fuze.det_selftest;
    pkt.det_zg_zd = _chase_fuze.det_zg_zd;
    pkt.det_target_count = _chase_fuze.det_target_count;
    pkt.fz_mode = _chase_fuze.fz_mode;
    pkt.fz_arm1 = _chase_fuze.fz_arm1;
    pkt.fz_arm2 = _chase_fuze.fz_arm2;
    pkt.fz_pwr_ok = _chase_fuze.fz_pwr_ok;
    pkt.fz_locked = _chase_fuze.fz_locked;
    mavlink_msg_chase_fuze_send_struct(chan, &pkt);
}
