#include "Copter.h"

Vector3f my_group_1_t::get_pos(int16_t id_in, float dist) {
    switch (id_in) {
        default:
        case 0:
            return Vector3f(0.0f,  0.0f,      0.0f);
            break;
        case 1:
            return Vector3f(0.0f, -0.5f*dist, 0.0f);
            break;
        case 2:
            return Vector3f(0.0f,  0.5f*dist, 0.0f);
            break;
        case 3:
            return Vector3f(0.0f, -1.5f*dist, 0.0f);
            break;
        case 4:
            return Vector3f(0.0f,  1.5f*dist, 0.0f);
            break;
        case 5:
            return Vector3f(0.0f, -2.5f*dist, 0.0f);
            break;
        case 6:
            return Vector3f(0.0f,  2.5f*dist, 0.0f);
            break;
        case 7:
            return Vector3f(0.0f, -3.5f*dist, 0.0f);
            break;
        case 8:
            return Vector3f(0.0f,  3.5f*dist, 0.0f);
            break;
    }
}

Vector3f my_group_1_t::get_offset(int16_t this_id, int16_t sender_id, float dist) {
    Vector3f offset_position = Vector3f(0.0f, 0.0f, 0.0f);
    if (this_id <=8 && sender_id <=8 ){
        offset_position = get_pos(this_id, dist) - get_pos(sender_id, dist);
    }
    return offset_position;
}

Vector3f my_group_1_t::get_search_dest(int16_t id, float group_dist, float search_dist) {
    Vector3f offset_position = Vector3f(search_dist, 0.0f, 0.0f);
    return offset_position;
}

Vector3f my_group_2_t::get_pos(int16_t id_in, float dist) {
    switch (id_in) {
        default:
        case 0:
            return Vector3f(0.0f,                       0.0f,                       0.0f);
            break;
        case 1:
            return Vector3f(sinf(radians(247.5f))*dist, cosf(radians(247.5f))*dist, 0.0f);
            break;
        case 2:
            return Vector3f(sinf(radians(292.5f))*dist, cosf(radians(292.5f))*dist, 0.0f);
            break;
        case 3:
            return Vector3f(sinf(radians(202.5f))*dist, cosf(radians(202.5f))*dist, 0.0f);
            break;
        case 4:
            return Vector3f(sinf(radians(337.5f))*dist, cosf(radians(337.5f))*dist, 0.0f);
            break;
        case 5:
            return Vector3f(sinf(radians(157.5f))*dist, cosf(radians(157.5f))*dist, 0.0f);
            break;
        case 6:
            return Vector3f(sinf(radians( 22.5f))*dist, cosf(radians( 22.5f))*dist, 0.0f);
            break;
        case 7:
            return Vector3f(sinf(radians(112.5f))*dist, cosf(radians(112.5f))*dist, 0.0f);
            break;
        case 8:
            return Vector3f(sinf(radians( 67.5f))*dist, cosf(radians( 67.5f))*dist, 0.0f);
            break;
    }
}

Vector3f my_group_2_t::get_offset(int16_t this_id, int16_t sender_id, float dist) {
    Vector3f offset_position = Vector3f(0.0f, 0.0f, 0.0f);
    if (this_id <=8 && sender_id <=8 ){
        offset_position = get_pos(this_id, dist) - get_pos(sender_id, dist);
    }
    return offset_position;
}

Vector3f my_group_2_t::get_search_dest(int16_t id, float group_dist, float search_dist) {
    Vector3f offset_position = get_pos(id, search_dist);

    offset_position = offset_position + get_offset(0, id, group_dist);
    return offset_position;
}


Vector3f my_group_1_assemble_t::get_pos(int16_t id_in, float dist) {
    switch (id_in) {
        default:
        case 0:
            return Vector3f(0.0f,                       0.0f,                       0.0f);
            break;
        case 1:
            return Vector3f(sinf(radians(257.5f))*dist, cosf(radians(257.5f))*dist, 0.0f);
            break;
        case 2:
            return Vector3f(sinf(radians(282.5f))*dist, cosf(radians(282.5f))*dist, 0.0f);
            break;
        case 3:
            return Vector3f(sinf(radians(232.5f))*dist, cosf(radians(232.5f))*dist, 0.0f);
            break;
        case 4:
            return Vector3f(sinf(radians(307.5f))*dist, cosf(radians(307.5f))*dist, 0.0f);
            break;
        case 5:
            return Vector3f(sinf(radians(207.5f))*dist, cosf(radians(207.5f))*dist, 0.0f);
            break;
        case 6:
            return Vector3f(sinf(radians(332.5f))*dist, cosf(radians(332.5f))*dist, 0.0f);
            break;
        case 7:
            return Vector3f(sinf(radians(182.5f))*dist, cosf(radians(182.5f))*dist, 0.0f);
            break;
        case 8:
            return Vector3f(sinf(radians(357.5f))*dist, cosf(radians(357.5f))*dist, 0.0f);
            break;
    }
}

float my_group_1_assemble_t::get_dir(int16_t id_in) {
    switch (id_in) {
        default:
        case 0:
            return 0.0f;
            break;
        case 1:
            return 12.5f;
            break;
        case 2:
            return -12.5f;
            break;
        case 3:
            return 37.5f;
            break;
        case 4:
            return -37.5f;
            break;
        case 5:
            return 62.5f;
            break;
        case 6:
            return -62.5f;
            break;
        case 7:
            return 87.5f;
            break;
        case 8:
            return -87.5f;
            break;
    }
}


Vector3f my_group_1_assemble_t::get_offset(int16_t this_id, int16_t sender_id, float dist) {
    Vector3f offset_position = Vector3f(0.0f, 0.0f, 0.0f);
    if (this_id <=8 && sender_id <=8 ){
        offset_position = get_pos(this_id, dist) - get_pos(sender_id, dist);
    }
    return offset_position;
}
