#pragma once

// #include "Copter.h"
#include <FD1_UART/FD1_UART.h>
#include <AP_SerialManager/AP_SerialManager.h>

class UserNacelle {
public:
    // UserNacelle() {};

    void Init();
    void Update();

    FD1_UART FD1_uart_msg_nacelle{AP_SerialManager::SerialProtocol_Nacelle};
    FD1_UART FD1_uart_msg_gcs{AP_SerialManager::SerialProtocol_GCS};

    void nacelle_init();
    void nacelle_update();
    void nacelle_send();
    void gcs_handle_and_route();
    void nacelle_handle_and_route();
    void nacelle_AHRS_test();
    void nacelle_read(uint8_t temp);
    void nacelle_update_status();
    void gcs_read(uint8_t temp);
    void print_info();
    void handle_msg(const mavlink_message_t &msg);

    bool valid();
    float get_yaw();
    float get_pitch();

    struct status_t {
        uint32_t last_msg_ms;
        float pitch_angle;
        float yaw_angle;
        float yaw_angle_body;
    } status;

    struct user_statistic {
        int16_t nacelle_byte_count;
        int16_t gcs_byte_count;
        int16_t nacelle_valid_byte_count;
        int16_t gcs_valid_byte_count;
    } user_stat;

private:
    ;
};
