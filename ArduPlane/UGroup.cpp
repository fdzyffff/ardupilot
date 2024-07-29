#include "Plane.h"

// void UGroup::set_group(uint8_t group_type_in) {

// }

Vector3f UGroup::get_offset(int16_t this_id, int16_t sender_id, float dist) {
    int16_t group_id = plane.g2.user_group_id.get();
    switch (group_id) {
        default:
        case 1:
            {
                current_group = &my_group1;
                break;
            }
        case 2:
            {
                current_group = &my_group2;
                break;
            }
    }
    return current_group->get_offset(this_id, sender_id, dist);
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Vector3f my_group_1_t::get_pos(int16_t id_in, float dist) {
    switch (id_in) {
        default:
        case 1:
            return Vector3f(0.0f            ,  0.0f          , 0.0f);
            break;
        case 2:
            return Vector3f(-1.0f*0.71f*dist, 1.0f*0.71f*dist, 10.0f);
            break;
        case 3:
            return Vector3f(-2.0f*0.71f*dist, 2.0f*0.71f*dist, 20.0f);
            break;
        case 4:
            return Vector3f(-3.0f*0.71f*dist, 3.0f*0.71f*dist, 30.0f);
            break;
        case 5:
            return Vector3f(-4.0f*0.71f*dist, 4.0f*0.71f*dist, 40.0f);
            break;
        case 6:
            return Vector3f(-5.0f*0.71f*dist, 5.0f*0.71f*dist, 50.0f);
            break;
        case 7:
            return Vector3f(-6.0f*0.71f*dist, 6.0f*0.71f*dist, 60.0f);
            break;
        case 8:
            return Vector3f(-7.0f*0.71f*dist, 7.0f*0.71f*dist, 70.0f);
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
        case 1:
            return Vector3f(0.0f            ,  0.0f          , 0.0f);
            break;
        case 2:
            return Vector3f(-1.0f*0.71f*dist, -1.0f*0.71f*dist, 10.0f);
            break;
        case 3:
            return Vector3f(-1.0f*0.71f*dist,  1.0f*0.71f*dist, 20.0f);
            break;
        case 4:
            return Vector3f(-2.0f*0.71f*dist, -2.0f*0.71f*dist, 30.0f);
            break;
        case 5:
            return Vector3f(-2.0f*0.71f*dist,  2.0f*0.71f*dist, 40.0f);
            break;
        case 6:
            return Vector3f(-3.0f*0.71f*dist, -1.0f*0.71f*dist, 50.0f);
            break;
        case 7:
            return Vector3f(-3.0f*0.71f*dist,  1.0f*0.71f*dist, 60.0f);
            break;
        case 8:
            return Vector3f(-4.0f*0.71f*dist,  0.0f*0.71f*dist, 70.0f);
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
// ---------------2024年7月24日----------


Vector3f my_group_3_t::get_pos(int16_t id_in, float dist) 
{
    switch (id_in) {
        default:
        case 1:
            return Vector3f(0.0f  ,  0.0f  , 0.0f);
            break;

        case 2:
            return Vector3f(-1.0f*dist, -1.0f*dist, 10.0f);
            break;
        case 3:
            return Vector3f(-1.0f*dist, 0, 10.0f);
            break;
        case 4:
            return Vector3f(-1.0f*dist, 1.0f*dist, 10.0f);
            break;


        case 5:
            return Vector3f(-2.0f*dist,  -2.0f*dist, 20.0f);
            break;
        case 6:
            return Vector3f(-2.0f*dist,  -1.0f*dist, 20.0f);
            break;
        case 7:
           return Vector3f(-2.0f*dist,  0*dist, 20.0f);
            break;
        case 8:
            return Vector3f(-2.0f*dist,  1.0f*dist, 20.0f);
            break;
        case 9:
            return Vector3f(-2.0f*dist,  2.0f*dist, 20.0f);
            break;

        case 10:
            return Vector3f(-3.0f*dist,  -3.0f*dist, 30.0f);
            break;
        case 11:
            return Vector3f(-3.0f*dist,  -2.0f*dist, 30.0f);
            break;
        case 12:
            return Vector3f(-3.0f*dist,  -1.0f*dist, 30.0f);
            break;
        case 13:
            return Vector3f(-3.0f*dist,  0.0f*dist, 30.0f);
            break;
        case 14:
            return Vector3f(-3.0f*dist,  1.0f*dist, 30.0f);
            break;
        case 15:
            return Vector3f(-3.0f*dist,  2.0f*dist, 30.0f);
            break;
        case 16:
            return Vector3f(-3.0f*dist,  3.0f*dist, 30.0f);
            break;

        case 17:
            return Vector3f(-4.0f*dist,  -4.0f*dist, 40.0f);
            break;
        case 18:
            return Vector3f(-4.0f*dist,  -3.0f*dist, 40.0f);
            break;
        case 19:
            return Vector3f(-4.0f*dist,  -2.0f*dist, 40.0f);
            break;
        case 20:
            return Vector3f(-4.0f*dist,  -1.0f*dist, 40.0f);
            break;
        case 21:
            return Vector3f(-4.0f*dist,  0.0f*dist, 40.0f);
            break;   
        case 22:
            return Vector3f(-4.0f*dist,  1.0f*dist, 40.0f);
            break;  
        case 23:
            return Vector3f(-4.0f*dist,  2.0f*dist, 40.0f);
            break;
        case 24:
            return Vector3f(-4.0f*dist,  3.0f*dist, 40.0f);
            break;
        case 25:
            return Vector3f(-4.0f*dist,  4.0f*dist, 40.0f);
            break;  

        case 26:
            return Vector3f(-5.0f*dist,  -5.0f*dist, 50.0f);
            break;
        case 27:
            return Vector3f(-5.0f*dist,  -4.0f*dist, 50.0f);
            break;
        case 28:
            return Vector3f(-5.0f*dist,  -3.0f*dist, 50.0f);
            break;
        case 29:
            return Vector3f(-5.0f*dist,  -2.0f*dist, 50.0f);
            break;
        case 30:
            return Vector3f(-5.0f*dist,  -1.0f*dist, 50.0f);
            break;   
        case 31:
            return Vector3f(-5.0f*dist,  0.0f*dist, 50.0f);
            break;  
        case 32:
            return Vector3f(-5.0f*dist,  1.0f*dist, 50.0f);
            break;
        case 33:
            return Vector3f(-5.0f*dist,  2.0f*dist, 50.0f);
            break;
        case 34:
            return Vector3f(-5.0f*dist,  3.0f*dist, 50.0f);
            break;  
        case 35:
            return Vector3f(-5.0f*dist,  4.0f*dist, 50.0f);
            break;  
        case 36:
            return Vector3f(-5.0f*dist,  5.0f*dist, 50.0f);
            break;             

        case 37:
            return Vector3f(-6.0f*dist,  -4.0f*dist, 40.0f);
            break;
        case 38:
            return Vector3f(-6.0f*dist,  -3.0f*dist, 40.0f);
            break;
        case 39:
            return Vector3f(-6.0f*dist,  -2.0f*dist, 40.0f);
            break;
        case 40:
            return Vector3f(-6.0f*dist,  -1.0f*dist, 40.0f);
            break;
        case 41:
            return Vector3f(-6.0f*dist,  0.0f*dist, 40.0f);
            break;   
        case 42:
            return Vector3f(-6.0f*dist,  1.0f*dist, 40.0f);
            break;  
        case 43:
            return Vector3f(-6.0f*dist,  2.0f*dist, 40.0f);
            break;
        case 44:
            return Vector3f(-6.0f*dist,  3.0f*dist, 40.0f);
            break;
        case 45:
            return Vector3f(-6.0f*dist,  4.0f*dist, 40.0f);
            break;  

        case 46:
            return Vector3f(-7.0f*dist,  -3.0f*dist, 30.0f);
            break;
        case 47:
            return Vector3f(-7.0f*dist,  -2.0f*dist, 30.0f);
            break;
        case 48:
            return Vector3f(-7.0f*dist,  -1.0f*dist, 30.0f);
            break;
        case 49:
            return Vector3f(-7.0f*dist,  0.0f*dist, 30.0f);
            break;
        case 50:
            return Vector3f(-7.0f*dist,  1.0f*dist, 30.0f);
            break;
        case 51:
            return Vector3f(-7.0f*dist,  2.0f*dist, 30.0f);
            break;
        case 52:
            return Vector3f(-7.0f*dist,  3.0f*dist, 30.0f);
            break;

        case 53:
            return Vector3f(-8.0f*dist,  -2.0f*dist, 20.0f);
            break;
        case 54:
            return Vector3f(-8.0f*dist,  -1.0f*dist, 20.0f);
            break;
        case 55:
           return Vector3f(-8.0f*dist,  0*dist, 20.0f);
            break;
        case 56:
            return Vector3f(-8.0f*dist,  1.0f*dist, 20.0f);
            break;
        case 57:
            return Vector3f(-8.0f*dist,  2.0f*dist, 20.0f);
            break;


        case 58:
            return Vector3f(-9.0f*dist, -1.0f*dist, 10.0f);
            break;
        case 59:
            return Vector3f(-9.0f*dist, 0, 10.0f);
            break;
        case 60:
            return Vector3f(-9.0f*dist, 1.0f*dist, 10.0f);
            break;

        case 61:
            return Vector3f(10.0f*dist  ,  0.0f , 0.0f);
            break;
    }
            
}

Vector3f my_group_3_t::get_offset(int16_t this_id, int16_t sender_id, float dist) 
{
    Vector3f offset_position = Vector3f(0.0f, 0.0f, 0.0f);
    if (this_id <=61 && sender_id <=16 ){
        offset_position = get_pos(this_id, dist) - get_pos(sender_id, dist);
    }
    return offset_position;
}

Vector3f my_group_3_t::get_search_dest(int16_t id, float group_dist, float search_dist) {
    Vector3f offset_position = get_pos(id, search_dist);
    offset_position = offset_position + get_offset(0, id, group_dist);
    return offset_position;
}

//------------

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
