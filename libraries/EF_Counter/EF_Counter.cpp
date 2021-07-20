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
    if (!_ahrs.get_relative_position_NED_home(new_pos)) {
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
        return;
    }

    // exit if polygon fences are not enabled
    if ((fence->get_enabled_fences() & AC_FENCE_TYPE_POLYGON) == 0) {
        return;
    }

    // iterate through inclusion polygons
    const uint8_t num_inclusion_polygons = fence->polyfence().get_inclusion_polygon_count();
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
            } else if (_EFGate_last_pass_time_ms[0] != 0 && _EFGate_last_pass_time_ms[i_gate] == 0) {
                _EFGate_last_pass_time_ms[i_gate].set_and_save(AP_HAL::millis()-extra_time - _EFGate_last_pass_time_ms[0].get());
                gcs().send_text(MAV_SEVERITY_WARNING, "T%d :%0.3f", i_gate, (float)_EFGate_last_pass_time_ms[i_gate].get()*0.001f);
            }
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

    // end points of current edge
    Vector2f gate_start = lines[0];
    Vector2f gate_end = lines[1];

    // find intersection with line segment
    Vector2f intersection;
    if (Vector2f::segment_intersection(pos_start_xy, pos_end_xy, gate_start, gate_end, intersection)) {
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


// singleton instance
EF_Counter *EF_Counter::_singleton;

namespace AP {

EF_Counter &ef_counter()
{
    return *EF_Counter::get_singleton();
}

}

