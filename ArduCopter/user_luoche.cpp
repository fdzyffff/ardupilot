#include "Copter.h"

void Copter::luoche_init(void)
{
    luoche_msg_follow.last_update_ms = 0; 
    luoche_msg_follow.valid = false;
    luoche_msg_follow.raw_x = 0.0f;
    luoche_msg_follow.raw_y = 0.0f;
    luoche_msg_follow.corr_x = 0.0f;
    luoche_msg_follow.corr_y = 0.0f;
    luoche_msg_follow.out.x = 0.0f;
    luoche_msg_follow.out.y = 0.0f;
    luoche_msg_follow.out.z = 0.0f;
    luoche_state = luoche_NONE;
    luoche_state_yaw_cd = 0.0f;
}

void Copter::luoche_sanity_check_para(){
    g2.user_parameters.cam1_xlength.set(constrain_float(g2.user_parameters.cam1_xlength, 60.0f, 1080.f));
    g2.user_parameters.cam1_angle.set(constrain_float(g2.user_parameters.cam1_angle, 20.0f, 80.0f));
}

void Copter::luoche_follow_handle(float input_x, float input_y)
{    
    if (g2.user_parameters.luoche_info_level >= 2) {
        gcs().send_text(MAV_SEVERITY_INFO, "C1 in X:%0.2f Y:%0.2f", input_x, input_y);
    }
    if (is_zero(input_x) && is_zero(input_y)) {
        return;
    }
    luoche_msg_follow.raw_x = input_x;
    luoche_msg_follow.raw_y = input_y;
    luoche_msg_follow.last_update_ms = millis();
    luoche_msg_follow.valid = true;
    Matrix3f tmp_m;

    float pixel_z = g2.user_parameters.cam1_xlength*0.5f/tanf(radians( g2.user_parameters.cam1_angle *0.5f));
    Vector3f tmp_input = Vector3f(input_x,input_y,pixel_z);

    tmp_m.from_euler(ahrs_view->roll, ahrs_view->pitch, 0.0f);
    luoche_msg_follow.out = tmp_m*tmp_input;
    luoche_msg_follow.corr_x = luoche_msg_follow.out.y;
    luoche_msg_follow.corr_y = -luoche_msg_follow.out.z;

    luoche_msg_follow.out = tmp_m*tmp_input;

    if (g2.user_parameters.luoche_info_level >= 2) {
        gcs().send_text(MAV_SEVERITY_INFO, "C1 raw X:%0.2f Y:%0.2f", luoche_msg_follow.raw_x, luoche_msg_follow.raw_y);
        gcs().send_text(MAV_SEVERITY_INFO, "C1 cor X:%0.2f Y:%0.2f", luoche_msg_follow.corr_x, luoche_msg_follow.corr_y);
    }
    if (g2.user_parameters.luoche_info_level >= 1) {
        gcs().send_text(MAV_SEVERITY_INFO, "C1 out X:%0.2f Y:%0.2f Z:%0.2f", luoche_msg_follow.out.x, luoche_msg_follow.out.y, luoche_msg_follow.out.z);
    }
}

void Copter::luoche_status_update()
{
    luoche_sanity_check_para();
    if (!flightmode->in_guided_mode()) {
        luoche_state = luoche_NONE;
        return;
    }

    uint32_t tnow_ms = millis();
    if (g2.user_parameters.luoche_cam1_timeout != 0 && (tnow_ms - luoche_msg_follow.last_update_ms > (uint32_t)g2.user_parameters.luoche_cam1_timeout)) {
        luoche_msg_follow.valid = false;
    }
    //bool luoche_sonar_valid = true;
    bool luoche_valid = luoche_msg_follow.valid;// && luoche_sonar_valid;

    switch (luoche_state){
        case luoche_NONE:
        {
            if (copter.inertial_nav.get_altitude() < g2.user_parameters.luoche_follow_alt){
                luoche_set_state(luoche_TAKEOFF);
            } else {
                luoche_set_state(luoche_FOLLOWING);
            }
        }
            break;
        case luoche_TAKEOFF:
        {
            if (copter.wp_nav->reached_wp_destination()){
                luoche_set_state(luoche_FOLLOWING);
            }
        }
            break;
        case luoche_FOLLOWING:
        {
            if (luoche_valid)
            {
                luoche_set_state(luoche_LANDING);
            }
        }
            break;
        case luoche_LANDING:
        {
            if (luoche_valid)
            {
                if (luoche_final_trigger()) {
                    luoche_set_state(luoche_FINAL);
                }
            } else {
                luoche_set_state(luoche_FOLLOWING);
            }
        }
            break;
        default:
            break;
    }

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    static float last_vx;
    static float last_vy;
    float v_up = -constrain_float(g2.user_parameters.luoche_land_speed, 50.0f, 500.f);

    bool use_yaw = true;
    float yaw_cd = luoche_state_yaw_cd;
    bool use_yaw_rate = false;
    float yaw_rate_cds = 0.0f;
    bool yaw_relative = false;

    switch (luoche_state){
        case luoche_NONE:
        {
            ;
        }
            break;
        case luoche_TAKEOFF:
        {
            ;
        }
            break;
        case luoche_FOLLOWING:
        {
            luoche_msg_follow.valid = false;;
        }
            break;
        case luoche_LANDING:
        {
            last_vx = luoche_get_speed(luoche_msg_follow.out.x, luoche_msg_follow.out.z);
            last_vy = luoche_get_speed(luoche_msg_follow.out.y, luoche_msg_follow.out.z);
            mode_guided.set_velocity(Vector3f(last_vx, last_vy, v_up), use_yaw, yaw_cd, use_yaw_rate, yaw_rate_cds, yaw_relative);
        }
            break;
        case luoche_FINAL:
        {
            mode_guided.set_velocity(Vector3f(last_vx, last_vx, v_up), use_yaw, yaw_cd, use_yaw_rate, yaw_rate_cds, yaw_relative);
        }
            break;
        default:
            break;
    }
}

bool Copter::luoche_final_trigger() {
    if ( rangefinder_alt_ok() ) {
        if (rangefinder_state.alt_cm < g2.user_parameters.luoche_final_alt) {
            return true;
        }
    }
    return false;
}

void Copter::luoche_set_state(luoche_state_t new_state) {
    luoche_state = new_state;
    gcs().send_text(MAV_SEVERITY_INFO, "Yaw at :%0.2f", luoche_state_yaw_cd);
    
    switch (luoche_state){
        case luoche_NONE:
        {
            ;
        }
            break;
        case luoche_TAKEOFF:
        {
            flightmode->do_user_takeoff(g2.user_parameters.luoche_follow_alt, true);
            if (g2.user_parameters.luoche_info_level >= 1) {gcs().send_text(MAV_SEVERITY_INFO, "TakeOff at :%0.2f", g2.user_parameters.luoche_follow_alt.get());}
        }
            break;
        case luoche_FOLLOWING:
        {
            copter.mode_guided.pos_control_start();
            if (g2.user_parameters.luoche_info_level >= 1) {gcs().send_text(MAV_SEVERITY_INFO, "Start Following");}
        }
            break;
        case luoche_LANDING:
        {
            copter.mode_guided.vel_control_start();
            if (g2.user_parameters.luoche_info_level >= 1) {gcs().send_text(MAV_SEVERITY_INFO, "Landing :%0.2f cm/s", g2.user_parameters.luoche_land_speed.get());}
        }
            break;
        case luoche_FINAL:
        {
            if (g2.user_parameters.luoche_info_level >= 1) {gcs().send_text(MAV_SEVERITY_INFO, "Final stage at :%0.2f cm", g2.user_parameters.luoche_final_alt.get());}
        }
            break;
        default:
            break;
    }
}

float Copter::luoche_get_speed(float input_x, float input_z){
    float cam_rad = 0.5f * radians(g2.user_parameters.cam1_angle);
    //float cam_length = 0.5f * g2.user_parameters.cam1_xlength;
    float tan_cam_rad = constrain_float(tanf(cam_rad), 1.0, 15.0f);
    float tmp_tan = 0.0f;
    if (is_zero(input_z) || input_z < 0.0f) {
        if (input_x > 0.0f) {
            tmp_tan = tan_cam_rad;
        } else {
            tmp_tan = -tan_cam_rad;
        }
    } else {
        tmp_tan = constrain_float(input_x/input_z, -cam_rad, cam_rad);
    }

    float vel_max = g2.user_parameters.luoche_xy_speed;
    if (g2.user_parameters.vel_channel-1 > 5) {
        int16_t tune_in = rc().get_radio_in(g2.user_parameters.vel_channel-1) - 1000;
        float vel_factor = constrain_float((float)tune_in / 1000.0f, 0.0f, 1.0f);
        vel_max = vel_max * vel_factor;
    }

    float v_out = vel_max * constrain_float( tmp_tan / tan_cam_rad * g2.user_parameters.luoche_vel_factor, -1.0f, 1.0f);
    return v_out;
}