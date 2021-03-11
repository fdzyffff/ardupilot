
#include "Copter.h"

Vector3f my_group_1_t::get_offset(int16_t this_id, int16_t sender_id) {
    Vector3f offset_position = Vector3f(0.0f, 0.0f, 0.0f);
    offset_position.y = (float)(-sender_id+1+this_id-1) * _distance;
    return offset_position;
}

Vector3f my_group_2_t::get_offset(int16_t this_id, int16_t sender_id) {
    Vector3f offset_position = Vector3f(0.0f, 0.0f, 0.0f);
    offset_position.x = (float)(sender_id-1-this_id+1) * _distance;
    return offset_position;
}