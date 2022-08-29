#include "Copter.h"

#ifdef USERHOOK_INIT
void Copter::userhook_init()
{
    // put your initialisation code here
    // this will be called once at start-up
    mocap_stat.n_count = 0;
    mocap_stat.last_update_ms = 0;
    mocap_stat.x = 0.0f;
    mocap_stat.y = 0.0f;
    mocap_stat.z = 0.0f;

    Ucam.init();
    Ugcs.init();
    Upayload.init();
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
    Ucam.update();
    Ugcs.update();
    Upayload.update();
}
#endif

#ifdef USERHOOK_MEDIUMLOOP
void Copter::userhook_MediumLoop()
{
    // put your 10Hz code here
    Ugcs_Log_Write_UCamTarget();
    // Ugcs_Log_Write_Mocap();
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
    userhook_SuperSlowLoop_print();
    userhook_SuperSlowLoop_telemsetup();
    userhook_SuperSlowLoop_setgpsorigin();
    // userhook_SuperSlowLoop_gcsfeedback();

    userhook_SuperSlowLoop_mocap_update();
}
#endif

#ifdef USERHOOK_AUXSWITCH
void Copter::userhook_auxSwitch1(const RC_Channel::AuxSwitchPos ch_flag)
{
    // put your aux switch #1 handler here (CHx_OPT = 47)
    switch (ch_flag) {
        case RC_Channel::AuxSwitchPos::HIGH:
            copter.set_mode(Mode::Number::ATTACK_ANGLE, ModeReason::UNKNOWN);
            break;
        case RC_Channel::AuxSwitchPos::MIDDLE:
            // nothing
            break;
        case RC_Channel::AuxSwitchPos::LOW:
            copter.set_mode(Mode::Number::STABILIZE, ModeReason::UNKNOWN);
            break;
    }
}

void Copter::userhook_auxSwitch2(const RC_Channel::AuxSwitchPos ch_flag)
{
    // put your aux switch #2 handler here (CHx_OPT = 48)
    switch (ch_flag) {
        case RC_Channel::AuxSwitchPos::HIGH:
            copter.Upayload.set_state(UPayload::payload_parse);
            gcs().send_text(MAV_SEVERITY_WARNING, "Payload FIRE");
            break;
        case RC_Channel::AuxSwitchPos::MIDDLE:
            // nothing
            break;
        case RC_Channel::AuxSwitchPos::LOW:
            copter.Upayload.set_state(UPayload::payload_none);
            gcs().send_text(MAV_SEVERITY_WARNING, "Payload NONE");
            break;
    }
}

void Copter::userhook_auxSwitch3(const RC_Channel::AuxSwitchPos ch_flag)
{
    // put your aux switch #3 handler here (CHx_OPT = 49)
}
#endif

void Copter::userhook_SuperSlowLoop_print() {
    if ((g2.user_parameters.cam_print.get() & (1<<0)) && Ucam.display_info_new) { // 1
        gcs().send_text(MAV_SEVERITY_WARNING, "[%d] %0.0f,%0.0f,%0.0f,%0.0f", Ucam.display_info_count, Ucam.display_info_p1, Ucam.display_info_p2, Ucam.display_info_p3, Ucam.display_info_p4);
        Ucam.display_info_new = false;
    }
    if (g2.user_parameters.cam_print.get() & (1<<1)) { // 2
        gcs().send_text(MAV_SEVERITY_WARNING, "Corr (%0.0f,%0.0f) on:%d", Ucam.get_correct_info().x,Ucam.get_correct_info().y, Ucam.is_active());
    }
    if (g2.user_parameters.cam_print.get() & (1<<2)) { // 4
        gcs().send_text(MAV_SEVERITY_WARNING, "rpy (%0.1f,%0.1f,%0.1f)", Ucam.get_target_roll_angle()*0.01f, Ucam.get_target_pitch_rate()*0.01f, Ucam.get_target_yaw_rate()*0.01f);
    }
    if (g2.user_parameters.cam_print.get() & (1<<3)) { // 8
        gcs().send_text(MAV_SEVERITY_WARNING, "TA (%0.1f), q(%0.1f)", Ucam.get_current_angle_deg(),  Ucam.get_q_rate_cds()*0.01f);
    }
    if (g2.user_parameters.cam_print.get() & (1<<4)) { // 16
        gcs().send_text(MAV_SEVERITY_WARNING, "Mocap [%d] [%0.1f, %0.1f, %0.1f]", mocap_stat.n_count, mocap_stat.x, mocap_stat.y, mocap_stat.z);
    }


    Ucam.display_info_count = 0;
}

void Copter::userhook_SuperSlowLoop_telemsetup() {
    static uint32_t last_update = millis();
    static int16_t pre_power = 0;
    static bool tele_set = false;
    if (tele_set && (pre_power != g2.user_parameters.tele_power.get()) ) {
        tele_set = false;
        last_update = millis();
    }
    if (!tele_set && (millis() - last_update > 1000)) {

        for (uint8_t i=0; i<MAVLINK_COMM_NUM_BUFFERS; i++) {
            AP_HAL::UARTDriver *_port;
            if ((_port = serial_manager.find_serial(AP_SerialManager::SerialProtocol_MAVLink, i))) {
                //gcs().send_text(MAV_SEVERITY_WARNING, "port : %d", i);
                _port->write(0x41);
                _port->write(0x54);
                _port->write(0x2B);
                _port->write(0x54);
                _port->write(0x58);
                _port->write(0x50);
                _port->write(0x57);
                _port->write(0x52);
                _port->write(0x3D);
                char power[10] = {0};
                //sprintf(power, "%d", g2.user_parameters.tele_power.get());
                //itoa(g2.user_parameters.tele_power.get(), power, 16);
                hal.util->snprintf(power, sizeof(power), "%d", g2.user_parameters.tele_power.get());

                //_port->write(0x31);
                //_port->write(0x30);
                for (uint8_t i_char = 0; i_char < 10; i_char++) {
                    if (power[i_char] != 0) {
                        _port->write(power[i_char]);
                    }
                }

                _port->write("\n");
            }
        }

        if (millis() - last_update > 10000) {
            tele_set = true;
            last_update = millis();
            pre_power = g2.user_parameters.tele_power.get();
        }
    }
}

void Copter::userhook_SuperSlowLoop_gcsfeedback() {
    uint16_t mask = GCS_MAVLINK::active_channel_mask() | GCS_MAVLINK::streaming_channel_mask();

    for (uint8_t i=0; i<MAVLINK_COMM_NUM_BUFFERS; i++) {
        if (mask & (1U<<i)) {
            mavlink_channel_t channel = (mavlink_channel_t)(MAVLINK_COMM_0 + i);
            if (comm_get_txspace(channel) >= 56 +
               GCS_MAVLINK::packet_overhead_chan(channel)) {
            // gcs().send_text(MAV_SEVERITY_WARNING, "channel %d", i);
            mavlink_msg_command_long_send(
                channel,
                0,
                0,
                MAV_CMD_USER_1,
                0,
                0.f,
                0.f, 
                0.f, 
                0.f,
                0.f, 
                0.f, 0.f);
            }
        }
    }
}

void Copter::userhook_SuperSlowLoop_setgpsorigin() {
    static uint32_t last_update = millis();
    if (!copter.ahrs.initialised()) {
        last_update = millis();
        return;
    }
    if (mocap_stat.n_count == 0) {last_update = millis();}

    Location ekf_origin;
    if (ahrs.get_origin(ekf_origin)) {
        return;
    }

    if (g2.user_parameters.ekf_origin_latitude.get() == 0 || g2.user_parameters.ekf_origin_longitude.get() == 0) {
        return;
    }

    if (millis() - last_update > 10000 && mocap_stat.n_count > 1) {
        Location ekf_origin_new;
        ekf_origin_new.lat = g2.user_parameters.ekf_origin_latitude.get();
        ekf_origin_new.lng = g2.user_parameters.ekf_origin_longitude.get();
        ekf_origin_new.alt = g2.user_parameters.ekf_origin_alt.get();
        if (ahrs.set_origin(ekf_origin_new)) {
            gcs().send_text(MAV_SEVERITY_WARNING, "EKF origin set up");

            gcs().send_text(MAV_SEVERITY_WARNING, "alt: %f", (float)ekf_origin_new.alt);
            gcs().send_text(MAV_SEVERITY_WARNING, "lng: %f", (float)ekf_origin_new.lng);
            gcs().send_text(MAV_SEVERITY_WARNING, "lat: %f", (float)ekf_origin_new.lat);
            ahrs.Log_Write_Home_And_Origin();
            // ahrs.reset();
            // if (set_home(ekf_origin_new, false)) {
            //     return;
            // }
        } else {
            gcs().send_text(MAV_SEVERITY_WARNING, "FAILED: EKF origin set up");
        }
    }

}

void Copter::userhook_SuperSlowLoop_mocap_update() {
    mocap_stat.n_count = 0;
    mocap_stat.last_update_ms = millis();
}

void Copter::send_my_micro_image(mavlink_channel_t chan, mavlink_my_micro_image_t* my_micro_image) {
        // mavlink_channel_t new_chan = MAVLINK_COMM_0;
        // if (chan == MAVLINK_COMM_0) {new_chan = MAVLINK_COMM_1;}
        // if (chan == MAVLINK_COMM_1) {new_chan = MAVLINK_COMM_2;}
        // if (chan == MAVLINK_COMM_2) {new_chan = MAVLINK_COMM_3;}
        // mavlink_msg_my_micro_image_send(new_chan,
        //     my_micro_image->st_row_idx,
        //     my_micro_image->st_packed_col_idx,
        //     my_micro_image->info,
        //     my_micro_image->data); 

    uint16_t mask = GCS_MAVLINK::active_channel_mask() | GCS_MAVLINK::streaming_channel_mask();
    //gcs().send_text(MAV_SEVERITY_WARNING, "Mask %d", mask);
    // don't send on the incoming channel. This should only matter if
    // the routing table is full
    // mask &= ~(1U<<(chan-MAVLINK_COMM_0));
    //gcs().send_text(MAV_SEVERITY_WARNING, "Mask %d", mask);
    // send on the remaining channels
    for (uint8_t i=0; i<MAVLINK_COMM_NUM_BUFFERS; i++) {
        if (mask & (1U<<i)) {
            mavlink_channel_t channel = (mavlink_channel_t)(MAVLINK_COMM_0 + i);
            if (comm_get_txspace(channel) >= 128 +
               GCS_MAVLINK::packet_overhead_chan(channel)) {
            // gcs().send_text(MAV_SEVERITY_WARNING, "channel %d", i);
            mavlink_msg_my_micro_image_send(channel,
                my_micro_image->st_row_idx,
                my_micro_image->st_packed_col_idx,
                my_micro_image->info,
                my_micro_image->data); 
            }
        }
    }
}
