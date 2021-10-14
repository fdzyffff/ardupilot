#include "EF_Counter.h"
#include <AP_AHRS/AP_AHRS.h>     // AHRS library
#include <AC_Fence/AC_Fence.h>         // Failsafe fence library
#include <GCS_MAVLink/GCS.h>
#include <stdio.h>

extern const AP_HAL::HAL& hal;

// table of user settable parameters
const AP_Param::GroupInfo EF_Counter::var_info[] = {

    AP_GROUPINFO("DIST",  1, EF_Counter, _update_dist, 20.f),

    AP_GROUPINFO("T0",    2,  EF_Counter, _EFGate_last_pass_time_ms[0], 0),
    AP_GROUPINFO("T1",    3,  EF_Counter, _EFGate_last_pass_time_ms[1], 0),
    AP_GROUPINFO("T2",    4,  EF_Counter, _EFGate_last_pass_time_ms[2], 0),
    AP_GROUPINFO("T3",    5,  EF_Counter, _EFGate_last_pass_time_ms[3], 0),
    AP_GROUPINFO("T4",    6,  EF_Counter, _EFGate_last_pass_time_ms[4], 0),
    AP_GROUPINFO("T5",    7,  EF_Counter, _EFGate_last_pass_time_ms[5], 0),
    AP_GROUPINFO("T6",    8,  EF_Counter, _EFGate_last_pass_time_ms[6], 0),
    AP_GROUPINFO("T7",    9,  EF_Counter, _EFGate_last_pass_time_ms[7], 0),
    AP_GROUPINFO("T8",    10, EF_Counter, _EFGate_last_pass_time_ms[8], 0),
    AP_GROUPINFO("T9",    11, EF_Counter, _EFGate_last_pass_time_ms[9], 0),
    AP_GROUPINFO("T10",   12, EF_Counter, _EFGate_last_pass_time_ms[10], 0),

    AP_GROUPEND
};

EF_Counter::EF_Counter()
{
    AP_Param::setup_object_defaults(this, var_info);
    _singleton = this;
}

void EF_Counter::update() {
    static int32_t last_update_ms = 0;
    int32_t tnow = AP_HAL::millis();

    const AP_AHRS &_ahrs = AP::ahrs();

    Vector3f new_pos;
    if (!_ahrs.get_relative_position_NED_origin(new_pos)) {
        return;
    }
    new_pos *= 100.0f; // m to cm

    if (tnow - last_update_ms > 2000) {
        _last_pos = new_pos;
    }

    float tmp_dist = (Vector2f(_last_pos.x, _last_pos.y) - Vector2f(new_pos.x, new_pos.y)).length();
    if (tmp_dist > _update_dist) {
        EFGate_update(_last_pos, new_pos);
        // gcs().send_text(MAV_SEVERITY_WARNING, "udpate %f", tmp_dist);
        _last_pos = new_pos;
    }
    last_update_ms = tnow;
}

void EF_Counter::EFGate_update(Vector3f &pos_start, Vector3f &pos_end)
{
    const AC_Fence *fence = AP::fence();
    if (fence == nullptr) {
        //gcs().send_text(MAV_SEVERITY_WARNING, "fence == nullptr");
        return;
    }

    // exit if polygon fences are not enabled
    if ((fence->get_enabled_fences() & AC_FENCE_TYPE_POLYGON) == 0) {
        //gcs().send_text(MAV_SEVERITY_WARNING, "fence_not_enable");
        return;
    }

    // iterate through inclusion polygons
    const uint8_t num_inclusion_polygons = fence->polyfence().get_inclusion_polygon_count();
    // if (num_inclusion_polygons < 2) {
    //     gcs().send_text(MAV_SEVERITY_WARNING, "fence udpate %d", num_inclusion_polygons);
    // }
    //gcs().send_text(MAV_SEVERITY_WARNING, "fence udpate %d", num_inclusion_polygons);
    uint8_t i_gate = 0; // 0 is for start line
    for (uint8_t i = 0; i < num_inclusion_polygons; i++) {
        uint16_t num_points;
        const Vector2f* lines = fence->polyfence().get_inclusion_polygon(i, num_points);
        //gcs().send_text(MAV_SEVERITY_WARNING, "Nfence udpate %d", num_points);
        if (num_points != 2) {
            // ignore exclusion polygons with less than 3 points
            continue;
        }
        uint32_t extra_time;
        if (EFGate_check(pos_start, pos_end, lines, num_points, extra_time)) {
            if (i_gate == 0 && _EFGate_last_pass_time_ms[i_gate] == 0) {
                _EFGate_last_pass_time_ms[i_gate].set_and_save(AP_HAL::millis()-extra_time);
                gcs().send_text(MAV_SEVERITY_WARNING, "Timer start");
            } else if (_EFGate_last_pass_time_ms[0] != 0 && i_gate != 0) {
                _EFGate_last_pass_time_ms[i_gate].set_and_save(AP_HAL::millis()-extra_time - _EFGate_last_pass_time_ms[0].get());
                gcs().send_text(MAV_SEVERITY_WARNING, "T%d :%0.3f", i_gate, (float)_EFGate_last_pass_time_ms[i_gate].get()*0.001f);
            }
            // gcs().send_text(MAV_SEVERITY_WARNING, "fence udpate %d", num_points);
        }
        i_gate += 1;
    }
}

void EF_Counter::EFGate_reset()
{
    for (uint8_t i_gate = 0; i_gate < EFGATE_NUM; i_gate++) {
        _EFGate_last_pass_time_ms[i_gate].set_and_save(0);
    }
}

bool EF_Counter::EFGate_check(Vector3f &pos_start, Vector3f &pos_end, const Vector2f* lines, uint16_t num_points, uint32_t &extra_time)
{
    Vector2f pos_start_xy = Vector2f(pos_start.x, pos_start.y);
    Vector2f pos_end_xy = Vector2f(pos_end.x, pos_end.y);

    // exit if there are no points
    if (lines == nullptr || num_points == 0 || num_points != 2) {
        return false;
    }
    // gcs().send_text(MAV_SEVERITY_WARNING, " ef check");

    // end points of current edge
    Vector2f gate_start = lines[0];
    Vector2f gate_end = lines[1];

    // find intersection with line segment
    Vector2f intersection;
    // gcs().send_text(MAV_SEVERITY_WARNING, " ef check %0.2f (%0.2f)", (lines[1]-lines[0]).length(), (pos_start_xy-pos_end_xy).length() );
    if (Vector2f::segment_intersection(pos_start_xy, pos_end_xy, gate_start, gate_end, intersection)) {
        // gcs().send_text(MAV_SEVERITY_WARNING, " ef check 2");
        float l1 = (pos_end_xy - intersection).length();
        float l2 = (pos_end_xy - pos_start_xy).length();
        if (is_zero(l2)) {
            extra_time = 10;
        } else {
            extra_time = (uint32_t)(10.f * l1/l2);
        }
        return true;
    }
    return false;
}

void EF_Counter::uart_send(AP_HAL::UARTDriver *_port, int16_t id) {

    _msg_uwb_out._msg_1.need_send = true;
    _msg_uwb_out._msg_1.content.msg.sum_check = 0;
    _msg_uwb_out._msg_1.content.msg.header.head_1 = FD1_msg_uwb_out::PREAMBLE1;
    _msg_uwb_out._msg_1.content.msg.header.head_2 = FD1_msg_uwb_out::PREAMBLE2;
    _msg_uwb_out._msg_1.content.msg.sys_id = id;
    _msg_uwb_out._msg_1.content.msg.t1 = _EFGate_last_pass_time_ms[1].get();
    _msg_uwb_out._msg_1.content.msg.t2 = _EFGate_last_pass_time_ms[2].get();
    _msg_uwb_out._msg_1.content.msg.t3 = _EFGate_last_pass_time_ms[3].get();
    _msg_uwb_out._msg_1.content.msg.t4 = _EFGate_last_pass_time_ms[4].get();
    _msg_uwb_out._msg_1.content.msg.t5 = _EFGate_last_pass_time_ms[5].get();
    _msg_uwb_out._msg_1.content.msg.t6 = _EFGate_last_pass_time_ms[6].get();
    _msg_uwb_out._msg_1.content.msg.t7 = _EFGate_last_pass_time_ms[7].get();
    _msg_uwb_out._msg_1.content.msg.t8 = _EFGate_last_pass_time_ms[8].get();
    _msg_uwb_out._msg_1.content.msg.t9 = _EFGate_last_pass_time_ms[9].get();
    _msg_uwb_out._msg_1.content.msg.t10 = _EFGate_last_pass_time_ms[10].get();

    _msg_uwb_out._msg_1.content.msg.sum_check = 0;
    for (int8_t i = 0; i < _msg_uwb_out._msg_1.length - 1; i++) {
        _msg_uwb_out._msg_1.content.msg.sum_check += _msg_uwb_out._msg_1.content.data[i];
    }
    _msg_uwb_out.swap_message();
    for (int8_t i = 0; i < _msg_uwb_out._msg_1.length; i ++) {
        _port->write(_msg_uwb_out._msg_1.content.data[i]);
    }
    _msg_uwb_out._msg_1.updated = false;
    _msg_uwb_out._msg_1.need_send = false;
}


// singleton instance
EF_Counter *EF_Counter::_singleton;

namespace AP {

EF_Counter &ef_counter()
{
    return *EF_Counter::get_singleton();
}

}

