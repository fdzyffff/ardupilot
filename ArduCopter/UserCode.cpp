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

    
    float vel_max = g2.user_parameters.genren_follow_vel;
    if (g2.user_parameters.vel_channel-1 > 5) {
        int16_t tune_in = rc().get_radio_in(g2.user_parameters.vel_channel-1) - 1000;
        float vel_factor = constrain_float((float)tune_in / 1000.0f, 0.0f, 1.0f);
        vel_max = vel_max * vel_factor;
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
            float target_yaw_cd = degrees(wrap_2PI(atan2f(genren_msg_follow.out.y, genren_msg_follow.out.x))) * 100.f;
            float curr_yaw_cd = float(ahrs_view->yaw_sensor);
            float forward_factor = 1.0f;
            if (g2.user_parameters.vel_corr_enable) {
                forward_factor = 1.0f - fabsf(constrain_float(wrap_180_cd(curr_yaw_cd - target_yaw_cd)/18000.f, 0.0f, 1.0f));
            }
            v_foward = vel_max * forward_factor;
            if (genren_msg_avoid.valid) {
                float right_factor = 0.0f;
                right_factor = 1.5f*genren_msg_avoid.corr_x/(constrain_float(g2.user_parameters.cam2_xlength, 60.0f, 1080.f)*0.5f);
                right_factor = constrain_float(right_factor, -2.0f*fabsf(forward_factor), 2.0f*fabsf(forward_factor));
                right_factor = constrain_float(right_factor, -1.0f, 1.0f);

                v_right = vel_max * right_factor;
            }
            float yaw_rate_tc = MAX(0.1f, fabsf(g2.user_parameters.genren_follow_yawrate_max));
            yaw_cd = 0.0f;
            v_down = 0.0f;
            use_yaw = false;
            use_yaw_rate = true;
            yaw_rate_cds = wrap_180_cd(target_yaw_cd - attitude_control->get_att_target_euler_cd().z)/yaw_rate_tc;
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

    Log_Write_GenrenTarget();
    Log_Write_GenrenAvoid();
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
