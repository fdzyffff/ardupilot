#include "HB1_message.h"

#define HB1_MSG_INSAPM_LEN 122
class HB1_ins2apm : public HB1_message{
public:
    struct PACKED HB1_msg_header {
        uint8_t head_1;
        uint8_t head_2;
    };

    // message structure
    struct PACKED MSG_Command_1 {
        HB1_msg_header header;
        uint8_t  length;
        uint8_t  id;
        uint32_t counter;
        uint8_t  state;  // bit7: baro; bit6: acc; bit5: gyro; bit4: compass; bit3: compass calibration (0: OK, 1: need calibration); bit2~bit0: ahrs (0: init, 1: OK, 2: err)
        float    pitch;  // rad
        float    roll;
        float    yaw;  // rad, -180~180
        float    yaw_gps;
        float    pitch_rate;  // rad/s
        float    roll_rate;
        float    yaw_rate;
        int32_t  longitude;  // 1e-7d
        int32_t  latitude;
        int32_t  alt_baro;  // cm
        int32_t  alt_gps;  // cm
        int32_t  alt_ekf;  // cm
        float    vel_n;  // m/s, NED
        float    vel_e;  // m/s, NED
        float    vel_d;  // m/s, NED
        float    airspeed;  // m/s, not used
        float    acc_n;  // m/s^2
        float    acc_e;
        float    acc_d;  
        uint8_t  satellite_count;
        uint16_t hdop;  // 0.01m
        uint16_t vdop;  // 0.01m
        uint8_t  gps_status;  // 0: NO_GPS, 1: NO_FIX, 2: GPS_FIX_2D, 3: GPS_FIX_3D, 4: GPS_FIX_3D_DGPS, 5: GPS_FIX_3D_RTK_FLOAT, 6: GPS_FIX_3D_RTK_FIXED
        uint8_t  gps_hh;  // gps hour
        uint8_t  gps_mm;  // gps minite
        uint8_t  gps_ss;  // gps second
        int8_t   temperature;  // centi-degree
        int16_t  hdt; // gpsyaw, 0.1 degree, 0~360
        int16_t  hdt_dev; // 0.1 degree
        uint8_t  sensor_used;  // bit 0&1: acc, bit 2&3: gyro, bit 4&5: compass, bit6&7: GPS
        uint8_t  gps0_dt;  // sample time, unit: 100ms
        uint8_t  gps1_dt;
        float    gps_vn;  // m/s, NED
        float    gps_ve;  // m/s, NED
        float    gps_vd;  // m/s, NED
        uint16_t gps_ms;  // gps time millis
        uint8_t  gps_day;  // gps time day
        uint16_t gps_week;
        uint8_t  ahrs_state;  // 0x00: standy, 0x10: rough OK, 0x20: precise OK, 0x30: combined navigation, 0x31: inertial navigation
        uint16_t  sum_check;  // data[0] ~ data[N-3]
    };

    union PACKED Content_1 {
        MSG_Command_1 msg;
        uint8_t data[HB1_MSG_INSAPM_LEN];
    };

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // message structure
    struct PACKED HB1_UART_MSG_1 {
        bool print;
        bool updated;
        bool need_send;
        Content_1 content;
    };

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    struct PACKED HB1_UART_msg_parser
    {
        enum
        {
            HB1_UART_PREAMBLE1 = 0,
            HB1_UART_PREAMBLE2,
            HB1_UART_LENGTH,
            HB1_UART_ID,
            HB1_UART_DATA,
            HB1_UART_SUM1,
            HB1_UART_SUM2
        } msg_state;

        uint16_t length;
        uint16_t read_idx;
        uint16_t sum_check;
        uint8_t data[HB1_MSG_INSAPM_LEN];
    } _msg;

    HB1_ins2apm();
    
    /* Do not allow copies */
    HB1_ins2apm(const HB1_ins2apm &other) = delete;
    HB1_ins2apm &operator=(const HB1_ins2apm&) = delete;

    static const uint8_t PREAMBLE1 = 0xEB;
    static const uint8_t PREAMBLE2 = 0x90;

    void process_message(void) override;
    void parse(uint8_t temp) override;
    void cal_sumcheck() override {};
    void swap_message() override {};

    HB1_UART_MSG_1 _msg_1;
};
