#include "Plane.h"

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
        case 3:
            {
                current_group = &my_group3;
                break;
            }
        case 4:
            {
                current_group = &my_group4;
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
        case 9:
            return Vector3f(-8.0f*0.71f*dist, 8.0f*0.71f*dist, 80.0f);
            break;
        case 10:
            return Vector3f(-9.0f*0.71f*dist, 9.0f*0.71f*dist, 90.0f);
            break;
        case 11:
            return Vector3f(-10.0f*0.71f*dist, 10.0f*0.71f*dist, 100.0f);
            break;
        case 12:
            return Vector3f(-11.0f*0.71f*dist, 11.0f*0.71f*dist, 10.0f);
            break;
        case 13:
            return Vector3f(-12.0f*0.71f*dist, 12.0f*0.71f*dist, 20.0f);
            break;
        case 14:
            return Vector3f(-13.0f*0.71f*dist, 13.0f*0.71f*dist, 30.0f);
            break;
        case 15:
            return Vector3f(-14.0f*0.71f*dist, 14.0f*0.71f*dist, 40.0f);
            break;
        case 16:
            return Vector3f(-15.0f*0.71f*dist, 15.0f*0.71f*dist, 50.0f);
            break;
        case 17:
            return Vector3f(-16.0f*0.71f*dist, 16.0f*0.71f*dist, 60.0f);
            break;
        case 18:
            return Vector3f(-17.0f*0.71f*dist, 17.0f*0.71f*dist, 70.0f);
            break;
        case 19:
            return Vector3f(-18.0f*0.71f*dist, 18.0f*0.71f*dist, 80.0f);
            break;
        case 20:
            return Vector3f(-19.0f*0.71f*dist, 19.0f*0.71f*dist, 90.0f);
            break;


        case 21:
            return Vector3f(-20.0f*0.71f*dist, 20.0f*0.71f*dist, 100.0f);
            break;
        case 22:
            return Vector3f(-21.0f*0.71f*dist, 21.0f*0.71f*dist, 10.0f);
            break;
        case 23:
            return Vector3f(-22.0f*0.71f*dist, 22.0f*0.71f*dist, 20.0f);
            break;
        case 24:
            return Vector3f(-23.0f*0.71f*dist, 23.0f*0.71f*dist, 30.0f);
            break;
        case 25:
            return Vector3f(-24.0f*0.71f*dist, 24.0f*0.71f*dist, 40.0f);
            break;
        case 26:
            return Vector3f(-25.0f*0.71f*dist, 25.0f*0.71f*dist, 50.0f);
            break;
        case 27:
            return Vector3f(-26.0f*0.71f*dist, 26.0f*0.71f*dist, 60.0f);
            break;
        case 28:
            return Vector3f(-27.0f*0.71f*dist, 27.0f*0.71f*dist, 70.0f);
            break;
        case 29:
            return Vector3f(-28.0f*0.71f*dist, 28.0f*0.71f*dist, 80.0f);
            break;
        case 30:
            return Vector3f(-29.0f*0.71f*dist, 29.0f*0.71f*dist, 90.0f);
            break;

        case 31:
            return Vector3f(-30.0f*0.71f*dist, 30.0f*0.71f*dist, 100.0f);
            break;
        case 32:
            return Vector3f(-31.0f*0.71f*dist, 31.0f*0.71f*dist, 10.0f);
            break;
        case 33:
            return Vector3f(-32.0f*0.71f*dist, 32.0f*0.71f*dist, 20.0f);
            break;
        case 34:
            return Vector3f(-33.0f*0.71f*dist, 33.0f*0.71f*dist, 30.0f);
            break;
        case 35:
            return Vector3f(-34.0f*0.71f*dist, 34.0f*0.71f*dist, 40.0f);
            break;
        case 36:
            return Vector3f(-35.0f*0.71f*dist, 35.0f*0.71f*dist, 50.0f);
            break;
        case 37:
            return Vector3f(-36.0f*0.71f*dist, 36.0f*0.71f*dist, 60.0f);
            break;
        case 38:
            return Vector3f(-37.0f*0.71f*dist, 37.0f*0.71f*dist, 70.0f);
            break;
        case 39:
            return Vector3f(-38.0f*0.71f*dist, 38.0f*0.71f*dist, 80.0f);
            break;
        case 40:
            return Vector3f(-39.0f*0.71f*dist, 39.0f*0.71f*dist, 90.0f);
            break;
        case 41:
            return Vector3f(-40.0f*0.71f*dist, 40.0f*0.71f*dist, 100.0f);
            break;
        case 42:
            return Vector3f(-41.0f*0.71f*dist, 41.0f*0.71f*dist, 10.0f);
            break;
        case 43:
            return Vector3f(-42.0f*0.71f*dist, 42.0f*0.71f*dist, 20.0f);
            break;
        case 44:
            return Vector3f(-43.0f*0.71f*dist, 43.0f*0.71f*dist, 30.0f);
            break;
        case 45:
            return Vector3f(-44.0f*0.71f*dist, 44.0f*0.71f*dist, 40.0f);
            break;
        case 46:
            return Vector3f(-45.0f*0.71f*dist, 45.0f*0.71f*dist, 50.0f);
            break;
        case 47:
            return Vector3f(-46.0f*0.71f*dist, 46.0f*0.71f*dist, 60.0f);
            break;
        case 48:
            return Vector3f(-47.0f*0.71f*dist, 47.0f*0.71f*dist, 70.0f);
            break;
        case 49:
            return Vector3f(-48.0f*0.71f*dist, 48.0f*0.71f*dist, 80.0f);
            break;
        case 50:
            return Vector3f(-49.0f*0.71f*dist, 49.0f*0.71f*dist, 90.0f);
            break;

    }
}

Vector3f my_group_1_t::get_offset(int16_t this_id, int16_t sender_id, float dist) {
    Vector3f offset_position = Vector3f(0.0f, 0.0f, 0.0f);
    if (this_id <=60 && sender_id <=60 ){
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
// 第二排    
        case 2:
            return Vector3f(-1.0f*0.71f*dist, -1.0f*0.71f*dist, 15.0f);
            break;
        case 3:
            return Vector3f(-1.0f*0.71f*dist,  1.0f*0.71f*dist, 25.0f);
            break;
// 第三排  
        case 4:
            return Vector3f(-2.0f*0.71f*dist, -2.0f*0.71f*dist, 30.0f);
            break;
        case 5:
            return Vector3f(-2.0f*0.71f*dist,  0, 40.0f);
            break;
        case 6:
            return Vector3f(-2.0f*0.71f*dist, 2.0f*0.71f*dist, 50.0f);
            break;
// 第四排  
        case 7:
            return Vector3f(-3.0f*0.71f*dist,  -3.0f*0.71f*dist, 45.0f);
            break;
        case 8:
            return Vector3f(-3.0f*0.71f*dist,  -1.0f*0.71f*dist, 55.0f);
            break;
        case 9:
            return Vector3f(-3.0f*0.71f*dist,  1.0f*0.71f*dist, 65.0f);
            break;
        case 10:
            return Vector3f(-3.0f*0.71f*dist,  3.0f*0.71f*dist, 75.0f);
            break;
// 第五排 
        case 11:
            return Vector3f(-4.0f*0.71f*dist,  -4.0f*0.71f*dist, 60.0f);
            break;
        case 12:
            return Vector3f(-4.0f*0.71f*dist,  -2.0f*0.71f*dist, 70.0f);
            break;
        case 13:
            return Vector3f(-4.0f*0.71f*dist,  0, 80.0f);
            break;
        case 14:
            return Vector3f(-4.0f*0.71f*dist,  2.0f*0.71f*dist, 90.0f);
            break;
        case 15:
            return Vector3f(-4.0f*0.71f*dist,  4.0f*0.71f*dist, 100.0f);
            break;
// 第六排  16 21 

        case 16:
            return Vector3f(-5.0f*0.71f*dist,  -5.0f*0.71f*dist, 75.0f);
            break;
        case 17:
            return Vector3f(-5.0f*0.71f*dist,  -3.0f*0.71f*dist, 85.0f);
            break;
        case 18:
            return Vector3f(-5.0f*0.71f*dist,  -1.0f*0.71f*dist, 95.0f);
            break;
        case 19:
            return Vector3f(-5.0f*0.71f*dist,  1.0f*0.71f*dist, 105.0f);
            break;
        case 20:
            return Vector3f(-5.0f*0.71f*dist,  3.0f*0.71f*dist, 115.0f);
            break;
        case 21:
            return Vector3f(-5.0f*0.71f*dist,  5.0f*0.71f*dist, 125.0f);
            break;
// 第七排 22 28
        case 22:
            return Vector3f(-6.0f*0.71f*dist,  -6.0f*0.71f*dist, 90.0f);
            break;
        case 23:
            return Vector3f(-6.0f*0.71f*dist,  -4.0f*0.71f*dist, 100.0f);
            break;
        case 24:
            return Vector3f(-6.0f*0.71f*dist,  -2.0f*0.71f*dist, 110.0f);
            break;
        case 25:
            return Vector3f(-6.0f*0.71f*dist,  0.0f*0.71f*dist, 120.0f);
            break;
        case 26:
            return Vector3f(-6.0f*0.71f*dist,  2.0f*0.71f*dist, 130.0f);
            break;
        case 27:
            return Vector3f(-6.0f*0.71f*dist,  4.0f*0.71f*dist, 140.0f);
            break;
        case 28:
            return Vector3f(-6.0f*0.71f*dist,  6.0f*0.71f*dist, 150.0f);
            break;
// 第八排 29 36
        case 29:
            return Vector3f(-7.0f*0.71f*dist,  -7.0f*0.71f*dist, 105.0f);
            break;
        case 30:
            return Vector3f(-7.0f*0.71f*dist,  -5.0f*0.71f*dist, 115.0f);
            break;
        case 31:
            return Vector3f(-7.0f*0.71f*dist,  -3.0f*0.71f*dist, 125.0f);
            break;
        case 32:
            return Vector3f(-7.0f*0.71f*dist,  -1.0f*0.71f*dist, 135.0f);
            break;
        case 33:
            return Vector3f(-7.0f*0.71f*dist,  1.0f*0.71f*dist, 145.0f);
            break;
        case 34:
            return Vector3f(-7.0f*0.71f*dist,  3.0f*0.71f*dist, 155.0f);
            break;
        case 35:
            return Vector3f(-7.0f*0.71f*dist,  5.0f*0.71f*dist, 165.0f);
            break;
        case 36:
            return Vector3f(-7.0f*0.71f*dist,  7.0f*0.71f*dist, 175.0f);
            break;
// 第九排 37 45
        case 37:
            return Vector3f(-8.0f*0.71f*dist,  -8.0f*0.71f*dist, 120.0f);
            break;
        case 38:
            return Vector3f(-8.0f*0.71f*dist,  -6.0f*0.71f*dist, 130.0f);
            break;
        case 39:
            return Vector3f(-8.0f*0.71f*dist,  -4.0f*0.71f*dist, 140.0f);
            break;
        case 40:
            return Vector3f(-8.0f*0.71f*dist,  -2.0f*0.71f*dist, 150.0f);
            break;
        case 41:
            return Vector3f(-8.0f*0.71f*dist,  0.0f*0.71f*dist, 160.0f);
            break;
        case 42:
            return Vector3f(-8.0f*0.71f*dist,  2.0f*0.71f*dist, 170.0f);
            break;
        case 43:
            return Vector3f(-8.0f*0.71f*dist,  4.0f*0.71f*dist, 180.0f);
            break;
        case 44:
            return Vector3f(-8.0f*0.71f*dist,  6.0f*0.71f*dist, 190.0f);
            break;
        case 45:
            return Vector3f(-8.0f*0.71f*dist,  8.0f*0.71f*dist, 200.0f);
            break;            
// 第十排 46 55
        case 46:
            return Vector3f(-9.0f*0.71f*dist,  -9.0f*0.71f*dist, 135.0f);
            break;
        case 47:
            return Vector3f(-9.0f*0.71f*dist,  -7.0f*0.71f*dist, 145.0f);
            break;
        case 48:
            return Vector3f(-9.0f*0.71f*dist,  -5.0f*0.71f*dist, 155.0f);
            break;
        case 49:
            return Vector3f(-9.0f*0.71f*dist,  -3.0f*0.71f*dist, 165.0f);
            break;
        case 50:
            return Vector3f(-9.0f*0.71f*dist,  -1.0f*0.71f*dist, 175.0f);
            break;
        case 51:
            return Vector3f(-9.0f*0.71f*dist,  1.0f*0.71f*dist, 185.0f);
            break;
        case 52:
            return Vector3f(-9.0f*0.71f*dist,  3.0f*0.71f*dist, 195.0f);
            break;
        case 53:
            return Vector3f(-9.0f*0.71f*dist,  5.0f*0.71f*dist, 205.0f);
            break;
        case 54:
            return Vector3f(-9.0f*0.71f*dist,  7.0f*0.71f*dist, 215.0f);
            break;
        case 55:
            return Vector3f(-9.0f*0.71f*dist,  9.0f*0.71f*dist, 225.0f);
            break;
    }
}

Vector3f my_group_2_t::get_offset(int16_t this_id, int16_t sender_id, float dist) {
    Vector3f offset_position = Vector3f(0.0f, 0.0f, 0.0f);
    if (this_id <=60 && sender_id <=60 ){
        offset_position = get_pos(this_id, dist) - get_pos(sender_id, dist);
    }
    return offset_position;
}

Vector3f my_group_2_t::get_search_dest(int16_t id, float group_dist, float search_dist) {
    Vector3f offset_position = get_pos(id, search_dist);

    offset_position = offset_position + get_offset(0, id, group_dist);
    return offset_position;
}

// ---------------2024年7月24日------2024年10月20日----

Vector3f my_group_3_t::get_pos(int16_t id_in, float dist) 
{
   if(id_in >= 1 && id_in <= 11)
   {
        return Vector3f(-1.0f*(id_in-1)*dist,  0.0f  ,(id_in-1)*10.0f);
   }
   else if (id_in >= 12 && id_in <= 21)
   {
        return Vector3f(-1.0f*(id_in-1)*dist,  0.0f  ,(id_in-11)*10.0f);    
   }
    else if (id_in >= 22 && id_in <= 31)
   {
        return Vector3f(-1.0f*(id_in-1)*dist,  0.0f  ,(id_in-21)*10.0f);    
   }
    else if (id_in >= 32 && id_in <= 41)
   {
        return Vector3f(-1.0f*(id_in-1)*dist,  0.0f  ,(id_in-31)*10.0f);    
   }
    else if (id_in >= 42 && id_in <= 51)
   {
        return Vector3f(-1.0f*(id_in-1)*dist,  0.0f  ,(id_in-41)*10.0f);    
   }
   else
   {
        return Vector3f(0.0f,0.0f,100.0f);
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



Vector3f my_group_4_t::get_pos(int16_t id_in, float dist) 
{
    if (id_in >= 1 && id_in <= 8) {
        return Vector3f(0.0f * 1.0f * dist, (id_in - 1) * 1.0f * dist,
                        (id_in - 1) * 10);

    } else if (id_in >= 9 && id_in <= 16) {
        return Vector3f(-1.0f * 1.0f * dist, (id_in - 9) * 1.0f * dist,
                        (id_in - 9) * 10 + 15);

    } else if (id_in >= 17 && id_in <= 24) {
        return Vector3f(-2.0f * 1.0f * dist, (id_in - 17) * 1.0f * dist,
                        (id_in - 17) * 10 + 30);

    } else if (id_in >= 25 && id_in <= 32) {
        return Vector3f(-3.0f * 1.0f * dist, (id_in - 25) * 1.0f * dist,
                        (id_in - 25) * 10 + 45);

    } else if (id_in >= 33 && id_in <= 40) {
        return Vector3f(-4.0f * 1.0f * dist, (id_in - 33) * 1.0f * dist,
                        (id_in - 33) * 10 + 60);

    } else if (id_in >= 41 && id_in <= 48) {
        return Vector3f(-5.0f * 1.0f * dist, (id_in - 41) * 1.0f * dist,
                        (id_in - 41) * 10 + 75);

    } else if (id_in >= 49 && id_in <= 56) {
        return Vector3f(-6.0f * 1.0f * dist, (id_in - 49) * 1.0f * dist,
                        (id_in - 49) * 10 + 90);
    }else{
        return Vector3f(0.0f,0.0f,100.0f);
    }


}

Vector3f my_group_4_t::get_offset(int16_t this_id, int16_t sender_id, float dist) 
{
    Vector3f offset_position = Vector3f(0.0f, 0.0f, 0.0f);
    if (this_id <=60 && sender_id <=60 ){
        offset_position = get_pos(this_id, dist) - get_pos(sender_id, dist);
    }
    return offset_position;
}

Vector3f my_group_4_t::get_search_dest(int16_t id, float group_dist, float search_dist) {
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
