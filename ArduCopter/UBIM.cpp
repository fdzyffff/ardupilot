#include "Copter.h"


UBIM::UBIM()
{
    // AP_Param::setup_object_defaults(this, var_info);
}

// initialise
void UBIM::init()
{
    if (uart_bim.init()) {
        uart_bim.get_msg_BIMCMD().set_enable(true);
        uart_bim.get_msg_BIMSTATUS().set_enable(true);
        gcs().send_text(MAV_SEVERITY_INFO, "BIM INIT");
    } else {
        gcs().send_text(MAV_SEVERITY_WARNING, "Err: BIM INIT FAIL");
    }
}


void UBIM::update_log() {
    // AP::logger().WriteStreaming("UATK",
    //                             "TimeUS,bfx,bfy,efx,efy,efrx,efry,tpth,trll,tyaw",
    //                             "s---------",
    //                             "F---------",
    //                             "Qfffffffff",
    //                             AP_HAL::micros64(),
    //                             (float)bf_info.x,
    //                             (float)bf_info.y,
    //                             (float)ef_info.x,
    //                             (float)ef_info.y,
    //                             (float)ef_rate_info.x,
    //                             (float)ef_rate_info.y,
    //                             (float)_target_pitch_rate,
    //                             (float)_target_roll_angle,
    //                             (float)_target_yaw_rate);

    // AP::logger().WriteStreaming("UAT2",
    //                             "TimeUS,angt,angm,agrt,agrm",
    //                             "s----",
    //                             "F----",
    //                             "Qffff",
    //                             AP_HAL::micros64(),
    //                             (float)_attack_angle_target,
    //                             (float)_attack_angle_measure,
    //                             (float)_attack_angle_rate_target,
    //                             (float)_attack_angle_rate_measure);

    // AP::logger().WriteStreaming("UATH",
    //                             "TimeUS,target,actual,ff,P,I,D,srate,dmod",
    //                             "s--------",
    //                             "F--------",
    //                             "Qffffffff",
    //                             AP_HAL::micros64(),
    //                             (float)attack_throttle_pid.get_pid_info().target,
    //                             (float)attack_throttle_pid.get_pid_info().actual,
    //                             (float)attack_throttle_pid.get_pid_info().FF,
    //                             (float)attack_throttle_pid.get_pid_info().P,
    //                             (float)attack_throttle_pid.get_pid_info().I,
    //                             (float)attack_throttle_pid.get_pid_info().D,
    //                             (float)attack_throttle_pid.get_pid_info().slew_rate,
    //                             (float)attack_throttle_pid.get_pid_info().Dmod);

    // AP::logger().WriteStreaming("UARL",
    //                             "TimeUS,target,actual,ff,P,I,D,srate,dmod",
    //                             "s--------",
    //                             "F--------",
    //                             "Qffffffff",
    //                             AP_HAL::micros64(),
    //                             (float)attack_roll_pid.get_pid_info().target,
    //                             (float)attack_roll_pid.get_pid_info().actual,
    //                             (float)attack_roll_pid.get_pid_info().FF,
    //                             (float)attack_roll_pid.get_pid_info().P,
    //                             (float)attack_roll_pid.get_pid_info().I,
    //                             (float)attack_roll_pid.get_pid_info().D,
    //                             (float)attack_roll_pid.get_pid_info().slew_rate,
    //                             (float)attack_roll_pid.get_pid_info().Dmod);

}
// called at 100 Hz

void UBIM::update()
{
    // for log purpose
    static uint32_t last_count_ms = millis();
    uint32_t tnow_ms = millis();
    if (tnow_ms - last_count_ms > 1000) {
        display_info.count_log = display_info.count;
        display_info.count = 0;
        last_count_ms = tnow_ms;
        //update filter cutoff HZ in flight
        _yaw_sample_filter.set_cutoff_frequency(30.f, filt_yaw_hz.get());
        _pitch_sample_filter.set_cutoff_frequency(30.f, filt_pithc_hz.get());
    }

    while (uart_bim.port_avaliable() > 0) {
        uint8_t temp = uart_bim.read_byte();
        uart_bim.parse(temp);

        if (uart_bim.get_msg_BIMCMD()._msg_1.updated) {
            FD1_msg_BIMCMD &tmp_msg = uart_bim.get_msg_BIMCMD();
            uint8_t plat_switch_cmd = tmp_msg._msg_1.content.msg.plat_switch_cmd[0];
            uint8_t plat_input_cmd = tmp_msg._msg_1.content.msg.plat_input_cmd[0];
            switch (plat_switch_cmd) {
                case 0xA0:
                    {
                        switch_back_to_wp();
                        break;
                    }
                case 0x28:
                    {
                        switch_hover();
                        break;
                    }
                case 0x40:
                    {
                        switch_arm();
                        break;
                    }
                case 0x42:
                    {
                        switch_manual();
                        break;
                    }
                case 0x44:
                    {
                        switch_land();
                        break;
                    }
                default:
                    break;
            }

            switch (plat_switch_cmd) {
                case :
                    {
                        break;
                    }
                default:
                    break;
            }

        }
    }

    if (AP_HAL::millis() - _last_post > 1000) {
        _last_post = AP_HAL::millis();
        update_log();
        // GCS_SEND_TEXT(MAV_SEVERITY_INFO, "PORT IN : %x", b);
    }
}
