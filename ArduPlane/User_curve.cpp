#include "Plane.h"

// yaw, rudder
float Plane::get_Y_curve() {
    return get_curve(g2.curve_y_s1, 
                    g2.curve_y_f1, 
                    g2.curve_y_s2, 
                    g2.curve_y_f2);
}

// steering
float Plane::get_R_curve() {
    return get_curve(g2.curve_r_s1, 
                    g2.curve_r_f1, 
                    g2.curve_r_s2, 
                    g2.curve_r_f2);
}

// roll, aileron
float Plane::get_A_curve() {
    return get_curve(g2.curve_a_s1, 
                    g2.curve_a_f1, 
                    g2.curve_a_s2, 
                    g2.curve_a_f2);
}

// pitch, elevator
float Plane::get_P_curve() {
    return get_curve(g2.curve_p_s1, 
                    g2.curve_p_f1, 
                    g2.curve_p_s2, 
                    g2.curve_p_f2);
}

// yaw, throttle
float Plane::get_T_curve() {
    return get_curve(g2.curve_t_s1, 
                    g2.curve_t_f1, 
                    g2.curve_t_s2, 
                    g2.curve_t_f2);
}


float Plane::get_curve_speed() {
    Vector3f vel;
    if (ahrs.get_velocity_NED(vel) && AP::gps().status() >= AP_GPS::GPS_OK_FIX_2D){
        return vel.xy().length();
    }
    else {
        return 0.0f;
    }
}

/*
_________(s1,f1)
--------\
---------\
----------\
-----------\
------------\
------(s2,f2)\_____________
*/
float Plane::get_curve(float s1_in, float f1_in, float s2_in, float f2_in) {
    float speed = get_curve_speed();
    float s1 = constrain_float(s1_in, 2.0f, 20.0f);
    float f1 = constrain_float(f1_in, -1.0f, 1.0f);
    float s2 = constrain_float(s2_in, 2.0f, 20.0f);
    float f2 = constrain_float(f2_in, -1.0f, 1.0f);
    float f_out = 0.0f;
    if (s1 >= s2) {
        s1 = s2 - 1.0f;
    }
    if (speed >= s2) {
        f_out = f2;
    } else if (speed <= s1) {
        f_out = f1;
    } else {
        f_out = (speed - s1) * (f2 - f1)/(s2 - s1) + f1;
    }
    f_out = constrain_float(f_out, -1.0f, 1.0f);
    return f_out;
}

bool Plane::curve_enable() {
    if (g2.curve_enable == 0) {
        return false;
    }
    return true;
}
