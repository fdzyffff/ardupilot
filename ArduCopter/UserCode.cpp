#include "Copter.h"

#ifdef USERHOOK_INIT
void Copter::userhook_init()
{
    // put your initialisation code here
    // this will be called once at start-up
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
    icd_a1_in_update();

    //25Hz
    static bool update_c1_out = false;
    if (update_c1_out) {
    	icd_c1_out_update();
    	update_c1_out = false;
    } else {
    	update_c1_out = true;
    }

    user_update_gps_dir();
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

void Copter::user_update_gps_dir()
{
    static uint32_t last_dir_update_time = millis();

    if (last_dir_update_time - millis() > 2000) {
        if (g2.rtk_yaw_out_enable == 1 || g2.rtk_yaw_out_enable == 3) {
            gcs_send_text_fmt(MAV_SEVERITY_WARNING, "heading: %0.2f , dev: %0.2f ", gps.get_heading_deg(), gps.get_heading_dev_deg());
        }
        
        last_dir_update_time = millis();
    }
}

float Copter::user_corr_yaw_rtk_cd(float input_target_yaw_cd) {

    static uint32_t last_dir_update_time = millis();
    static float global_yaw_rtk_offset_cd = 0.0f;

    float copter_target_yaw_cd = yaw_look_at_heading;//attitude_control->get_att_target_euler_cd().z;
    float copter_current_yaw_cd = ahrs.yaw_sensor;
    if (last_dir_update_time - millis() > 5000) {
        if (fabsf(wrap_180_cd(copter_target_yaw_cd - copter_current_yaw_cd)) < 100.0f && gps.get_heading_dev_deg() > 0.01f && gps.get_heading_dev_deg() < 2.0f) {
            global_yaw_rtk_offset_cd = wrap_180_cd(gps.get_heading_deg()*100.0f - copter_current_yaw_cd);
            if (g2.rtk_yaw_out_enable == 2 || g2.rtk_yaw_out_enable == 3) {
                gcs_send_text_fmt(MAV_SEVERITY_WARNING, "corrected: %0.2f, final: %0.2f", global_yaw_rtk_offset_cd, wrap_360_cd(copter_target_yaw_cd + global_yaw_rtk_offset_cd));
            }
            last_dir_update_time = millis();
        }
    }
    float final_yaw_cd = wrap_360_cd(copter_target_yaw_cd + global_yaw_rtk_offset_cd);
    return final_yaw_cd;
}
