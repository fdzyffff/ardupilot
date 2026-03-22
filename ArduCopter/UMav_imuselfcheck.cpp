#include "Copter.h"

void UMav::read_imusc_message() 
{
    if (!FD_uart_selfcheck.initialized()) {return;}

    static uint32_t _last_log_ms = millis();
    static int16_t count = 0;
    static uint32_t _last_receive_ms = millis();
    while (FD_uart_selfcheck.get_port()->available()>0) {
        uint8_t temp = FD_uart_selfcheck.get_port()->read();
        // if (temp == 0xFE) pk0_count++;
        mavlink_message_t msg;
        mavlink_status_t status;
        uint8_t ret = mavlink_frame_char_buffer(&mavlink.rxmsg, &mavlink.status, temp, &msg, &status);

        if (ret >= MAVLINK_FRAMING_OK) {
            if (msg.msgid == MAVLINK_MSG_ID_RAW_IMU) {
                count++;

                uint32_t dt_ms = (millis() - _last_receive_ms);

                if (copter.g2.user_parameters.fast_log.get() == 1) {
                    AP::logger().WriteStreaming("CMSC",
                                                    "TimeUS,dt",
                                                    "s-",
                                                    "F-",
                                                    "Qf",
                                                    AP_HAL::micros64(),
                                                    (float)dt_ms);
                }
                _last_receive_ms = millis();
            }
        }
    }

    float dt = (float)(millis() - _last_log_ms)*0.001f;
    if (dt > 1.0f) {
        // gcs().send_text(MAV_SEVERITY_INFO, "LOOP IMURAW %d", count);
        _last_log_ms = millis();
        float imu_rate = ((float)count)/dt;
        count = 0;
        AP::logger().WriteStreaming("UMSC",
                                    "TimeUS,rate",
                                    "s-",
                                    "F-",
                                    "Qf",
                                    AP_HAL::micros64(),
                                    (float)imu_rate);
    }

}
