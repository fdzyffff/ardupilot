#pragma once

#include "AP_ExternalAHRS_config.h"

#if AP_EXTERNAL_AHRS_FS982_ENABLED

#include "AP_ExternalAHRS_backend.h"

class AP_ExternalAHRS_FS982 : public AP_ExternalAHRS_backend {

public:
    AP_ExternalAHRS_FS982(AP_ExternalAHRS *frontend, AP_ExternalAHRS::state_t &state);

    // get serial port number, -1 for not enabled
    int8_t get_port(void) const override;

    // accessors for AP_AHRS
    bool healthy(void) const override;
    bool initialised(void) const override;
    bool pre_arm_check(char *failure_msg, uint8_t failure_msg_len) const override;
    void get_filter_status(nav_filter_status &status) const override;
    void send_status_report(class GCS_MAVLINK &link) const override;

    // check for new data
    void update() override {
        check_uart();
    }

    // Get model/type name
    const char* get_name() const override {
        return "FS982";
    }

    AP_ExternalAHRS::gps_data_message_t gps_data;
    AP_ExternalAHRS::ins_data_message_t ins_data;

    uint16_t buffer_ofs;
    uint8_t buffer[256];

private:
    AP_HAL::UARTDriver *uart;
    int8_t port_num;
    uint32_t baudrate;
    bool setup_complete;

    void update_thread();
    bool check_uart();
    void parse_msg();

    struct PACKED NAV_t
    {
        uint8_t header1;
        uint8_t header2;
        uint16_t ID;
        uint16_t data_length;
        uint32_t GNSS_tow_ms;  // GNSS周内表
        uint16_t GNSS_week;  // GNSS周技术
        int32_t latitude;
        int32_t longitude;
        int32_t altitude_mm;
        float velocity_north_m_s;
        float velocity_east_m_s;
        float velocity_down_m_s;
        float roll_deg;
        float pitch_deg;
        float yaw_deg;
        float GNSS_yaw_deg;
        float rsv0;  // 预留
        float acc_x_g;  // 注意加速度单位为：g
        float acc_y_g;
        float acc_z_g;
        float gyro_x_deg_s;
        float gyro_y_deg_s;
        float gyro_z_deg_s;
        float temperature_dc;
        uint8_t fix_type;  // 定位状态
        uint8_t sat_num;  // 参与定位卫星数
        uint8_t RTK_delay;  // RTK差分延时
        uint8_t GNSS_yaw_status;  // 双天线定向状态，50表示已经定向，其他表示未定向
        uint16_t hdop_cm;  // 定位因子，组合导航初始化后有效
        uint16_t INS_status;  // 组合导航状态
        uint32_t rsv1;  // 预留
        uint32_t rsv2;
    };

    union PACKED msgbuffer
    {
        NAV_t nav;
        uint8_t bytes[256];
    } nav_msg;

    uint8_t decode_state = 0;  // 解帧状态
    uint16_t frame_data_counter = 0;  // 帧数据计数器
    uint32_t received_crc;  // 接收到的CRC值
    uint32_t calculated_crc;  // 计算得到的CRC值

    uint32_t crc_crc32(uint32_t crc, const uint8_t *buf, uint32_t size);

    uint32_t last_nav_msg_ms = 0;

    bool GNSS_has_fixed_once = false;  // 本次上电后GNSS已经定过一次（用于GNSS失锁后惯导控制逻辑）
};

#endif  // AP_EXTERNAL_AHRS_FS982_ENABLED

