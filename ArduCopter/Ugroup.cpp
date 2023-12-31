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
