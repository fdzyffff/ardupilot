#include "Copter.h"

UMav::UMav()
{
    ;
}

void UMav::init()
{
    FD_uart_imu.init();
    FD_uart_bsq.init();

    _imu_gyro.set_cutoff_frequency(400.f, copter.g2.user_parameters.filt_gyro_hz.get());
    _imu_acc.set_cutoff_frequency(400.f, copter.g2.user_parameters.filt_acc_hz.get());

    gcs().send_text(MAV_SEVERITY_INFO, "UMAV INIT");


    // start calls to loop in separate thread
    if (!hal.scheduler->thread_create(
            FUNCTOR_BIND_MEMBER(&UMav::send_raw_imu_loop, void), "IMURAW", 2048, AP_HAL::Scheduler::PRIORITY_SPI, 0)) {
            gcs().send_text(MAV_SEVERITY_INFO, "IMURAW: couldn't create thread\n\r");
    } else {
        gcs().send_text(MAV_SEVERITY_INFO, "IMURAW: create thread\n\r");
    }
}

void UMav::update()
{
    // send_raw_imu();
    read_bsq_message();
    
    trans_status.update();
    trans_selfcheck.update();
    trans_target.update();
    trans_mission.update();
    trans_relay_positon.update();
}

void UMav::send_all()
{
    // send_do_selfcheck();
    // send_target();
    // send_mission();
    // send_relay_position();
    send_apm_status();
}


void UMav::handle_info_test(int16_t p1)
{
    // if (p1 == 400) {
    //     gcs().send_text(MAV_SEVERITY_INFO, "Try send 400");
    //     send_do_selfcheck();
    // }
    // if (p1 == 401) {
    //     gcs().send_text(MAV_SEVERITY_INFO, "Try send 401");
    //     send_target();
    // }
    // if (p1 == 402) {
    //     gcs().send_text(MAV_SEVERITY_INFO, "Try send 402");
    //     send_status();
    // }
    // if (p1 == 403) {
    //     gcs().send_text(MAV_SEVERITY_INFO, "Try send 403");
    //     send_selfcheck_result();
    // }
    // if (p1 == 404) {
    //     gcs().send_text(MAV_SEVERITY_INFO, "Try send 404");
    //     send_target_result();
    // }
    // if (p1 == 405) {
    //     gcs().send_text(MAV_SEVERITY_INFO, "Try send 405");
    //     send_mission();
    // }
    // if (p1 == 406) {
    //     gcs().send_text(MAV_SEVERITY_INFO, "Try send 406");
    //     send_mission_result();
    // }
    // if (p1 == 407) {
    //     gcs().send_text(MAV_SEVERITY_INFO, "Try send 407");
    //     send_relay_position();
    // }
    // if (p1 == 408) {
    //     gcs().send_text(MAV_SEVERITY_INFO, "Try send 408");
    //     send_relay_position_result();
    // }
    // if (p1 == 413) {
    //     gcs().send_text(MAV_SEVERITY_INFO, "Try send 413");
    //     send_apm_status();
    // }
}
