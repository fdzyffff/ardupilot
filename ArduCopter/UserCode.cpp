#include "Copter.h"

#ifdef USERHOOK_INIT
void Copter::userhook_init()
{
    // put your initialisation code here
    // this will be called once at start-up
    genren_init();
}
#endif

#ifdef USERHOOK_FASTLOOP
void Copter::userhook_FastLoop()
{
    // put your 100Hz code here
}
#endif

#ifdef USERHOOK_50HZLOOP
void Copter::userhook_50Hz()
{
    // put your 50Hz code here
    genren_status_update();

    bool use_yaw = false;
    float yaw_cd = 0.0f;
    bool use_yaw_rate = false;
    float yaw_rate_cds = 0.0f;
    bool yaw_relative = false;

    float v_foward = 0.0f;
    float v_right = 0.0f;
    float v_down = 0.0f;

    if (genren_msg_follow.valid) {
        genren_state = genren_FOLLOWING;
    } else {
        genren_state = genren_STANDBY;
    }

    switch(genren_state){
        case genren_STANDBY:
            v_foward = 0.0f;
            v_right = 0.0f;
            v_down = 0.0f;
            use_yaw = false;
            yaw_cd = 0.0f;
            use_yaw_rate = true;
            yaw_rate_cds = g2.user_parameters.genren_follow_default_yawrate;
            yaw_relative = false;
            break;
        case genren_FOLLOWING:{
            v_right = 0.0f;
            yaw_cd = degrees(wrap_2PI(atan2f(genren_msg_follow.out.y, genren_msg_follow.out.x))) * 100.f;
            float curr_yaw_cd = float(ahrs_view->yaw_sensor);
            float forward_factor = 1.0f - fabsf(constrain_float(wrap_180_cd(curr_yaw_cd - yaw_cd)/12000.f, -1.0, 1.0f));
            v_foward = g2.user_parameters.genren_follow_vel * forward_factor;
            if (genren_msg_avoid.valid) {
                float right_factor = genren_msg_avoid.corr_x/(constrain_float(g2.user_parameters.cam2_xlength, 60.0f, 1080.f)*0.25f);
                right_factor = constrain_float(right_factor, -1.0f, 1.0f);
                right_factor = 1.0f - fabsf(right_factor);
                if (genren_msg_avoid.corr_x < 0) {
                    v_right = g2.user_parameters.genren_follow_vel * right_factor;
                } else {
                    v_right = -g2.user_parameters.genren_follow_vel * right_factor;
                }
            }
            v_down = 0.0f;
            use_yaw = true;
            use_yaw_rate = false;
            yaw_rate_cds = 0.0f;
            yaw_relative = false;
            }
            break;
    } 
    if (flightmode->in_guided_mode()) {
        float vx = (v_foward*ahrs.cos_yaw() - v_right*ahrs.sin_yaw());
        float vy = (v_foward*ahrs.sin_yaw() + v_right*ahrs.cos_yaw());
        mode_guided.set_velocity(Vector3f(vx, vy, v_down), use_yaw, yaw_cd, use_yaw_rate, yaw_rate_cds, yaw_relative);
    }    
}
#endif

#ifdef USERHOOK_MEDIUMLOOP
void Copter::userhook_MediumLoop()
{
    // put your 10Hz code here
}
#endif

#ifdef USERHOOK_SLOWLOOP
void Copter::userhook_SlowLoop()
{
    // put your 3.3Hz code here
}
#endif

#ifdef USERHOOK_SUPERSLOWLOOP
void Copter::userhook_SuperSlowLoop()
{
    // put your 1Hz code here
}
#endif

#ifdef USERHOOK_AUXSWITCH
void Copter::userhook_auxSwitch1(uint8_t ch_flag)
{
    // put your aux switch #1 handler here (CHx_OPT = 47)
}

void Copter::userhook_auxSwitch2(uint8_t ch_flag)
{
    // put your aux switch #2 handler here (CHx_OPT = 48)
}

void Copter::userhook_auxSwitch3(uint8_t ch_flag)
{
    // put your aux switch #3 handler here (CHx_OPT = 49)
}
#endif
