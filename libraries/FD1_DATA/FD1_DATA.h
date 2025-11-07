#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_SerialManager/AP_SerialManager.h>


class FD1_DATA
{

public:
    FD1_DATA();

    /* Do not allow copies */
    FD1_DATA(const FD1_DATA &other) = delete;
    FD1_DATA &operator=(const FD1_DATA&) = delete;

    static FD1_DATA *get_singleton() {
        return _singleton;
    }

    // yaw in degrees if available
    bool get_yaw_deg(float &yaw_deg, float &accuracy_deg);
    void set_yaw_deg(float yaw_deg, float accuracy_deg);

    float get_alt(){return alt;}
    void set_alt(float alt_in){alt = alt_in;}
    float get_arspd_tas(){return arspd_tas;}
    void set_arspd_tas(float arspd_tas_in){arspd_tas = arspd_tas_in;}
    float get_climb_rate(){return climb_rate;}
    void set_climb_rate(float climb_rate_in){climb_rate = climb_rate_in;}
    float get_aoa(){return aoa;}
    void set_aoa(float aoa_in){aoa = aoa_in;}
    float get_ssa(){return ssa;}
    void set_ssa(float ssa_in){ssa = ssa_in;}
    float get_roll(){return roll;}
    void set_roll(float roll_in){roll = roll_in;}
    float get_yaw(){return yaw;}
    void set_yaw(float yaw_in){yaw = yaw_in;}
    float get_rate_x(){return rate_x;}
    void set_rate_x(float rate_x_in){rate_x = rate_x_in;}
    float get_rate_y(){return rate_y;}
    void set_rate_y(float rate_y_in){rate_y = rate_y_in;}
    float get_rate_z(){return rate_z;}
    void set_rate_z(float rate_z_in){rate_z = rate_z_in;}
    float get_acc_x(){return acc_x;}
    void set_acc_x(float acc_x_in){acc_x = acc_x_in;}
    float get_acc_y(){return acc_y;}
    void set_acc_y(float acc_y_in){acc_y = acc_y_in;}
    float get_acc_z(){return acc_z;}
    void set_acc_z(float acc_z_in){acc_z = acc_z_in;}
    float get_gps_utc(){return gps_utc;}
    void set_gps_utc(uint32_t gps_utc_in){gps_utc = gps_utc_in;}

    int16_t get_mot1_rpm(){return mot1_rpm;}
    void set_mot1_rpm(int16_t mot1_rpm_in){mot1_rpm = mot1_rpm_in;}
    int16_t get_mot2_rpm(){return mot2_rpm;}
    void set_mot2_rpm(int16_t mot2_rpm_in){mot2_rpm = mot2_rpm_in;}
    int16_t get_mot3_rpm(){return mot3_rpm;}
    void set_mot3_rpm(int16_t mot3_rpm_in){mot3_rpm = mot3_rpm_in;}
    int16_t get_mot4_rpm(){return mot4_rpm;}
    void set_mot4_rpm(int16_t mot4_rpm_in){mot4_rpm = mot4_rpm_in;}

    uint16_t get_mot1_temperature(){return mot1_temperature;}
    void set_mot1_temperature(uint16_t mot1_temperature_in){mot1_temperature = mot1_temperature_in;}
    uint16_t get_mot2_temperature(){return mot2_temperature;}
    void set_mot2_temperature(uint16_t mot2_temperature_in){mot2_temperature = mot2_temperature_in;}
    uint16_t get_mot3_temperature(){return mot3_temperature;}
    void set_mot3_temperature(uint16_t mot3_temperature_in){mot3_temperature = mot3_temperature_in;}
    uint16_t get_mot4_temperature(){return mot4_temperature;}
    void set_mot4_temperature(uint16_t mot4_temperature_in){mot4_temperature = mot4_temperature_in;}

    uint16_t get_controller1_temperature(){return controller1_temperature;}
    void set_controller1_temperature(uint16_t controller1_temperature_in){controller1_temperature = controller1_temperature_in;}
    uint16_t get_controller2_temperature(){return controller2_temperature;}
    void set_controller2_temperature(uint16_t controller2_temperature_in){controller2_temperature = controller2_temperature_in;}
    uint16_t get_controller3_temperature(){return controller3_temperature;}
    void set_controller3_temperature(uint16_t controller3_temperature_in){controller3_temperature = controller3_temperature_in;}
    uint16_t get_controller4_temperature(){return controller4_temperature;}
    void set_controller4_temperature(uint16_t controller4_temperature_in){controller4_temperature = controller4_temperature_in;}

    float get_propeller1_angle(){return propeller1_angle;}
    void set_propeller1_angle(float propeller1_angle_in){propeller1_angle = propeller1_angle_in;}
    float get_propeller2_angle(){return propeller2_angle;}
    void set_propeller2_angle(float propeller2_angle_in){propeller2_angle = propeller2_angle_in;}
    float get_propeller3_angle(){return propeller3_angle;}
    void set_propeller3_angle(float propeller3_angle_in){propeller3_angle = propeller3_angle_in;}
    float get_propeller4_angle(){return propeller4_angle;}
    void set_propeller4_angle(float propeller4_angle_in){propeller4_angle = propeller4_angle_in;}

    uint16_t get_mot1_error(){return mot1_error;}
    void set_mot1_error(uint16_t mot1_error_in){mot1_error = mot1_error_in;}
    uint16_t get_mot2_error(){return mot2_error;}
    void set_mot2_error(uint16_t mot2_error_in){mot2_error = mot2_error_in;}
    uint16_t get_mot3_error(){return mot3_error;}
    void set_mot3_error(uint16_t mot3_error_in){mot3_error = mot3_error_in;}
    uint16_t get_mot4_error(){return mot4_error;}
    void set_mot4_error(uint16_t mot4_error_in){mot4_error = mot4_error_in;}

    uint16_t get_propeller1_error(){return propeller1_error;}
    void set_propeller1_error(uint16_t propeller1_error_in){propeller1_error = propeller1_error_in;}
    uint16_t get_propeller2_error(){return propeller2_error;}
    void set_propeller2_error(uint16_t propeller2_error_in){propeller2_error = propeller2_error_in;}
    uint16_t get_propeller3_error(){return propeller3_error;}
    void set_propeller3_error(uint16_t propeller3_error_in){propeller3_error = propeller3_error_in;}
    uint16_t get_propeller4_error(){return propeller4_error;}
    void set_propeller4_error(uint16_t propeller4_error_in){propeller4_error = propeller4_error_in;}

    uint16_t get_bms_vol(){return bms_vol;}
    void set_bms_vol(uint16_t bms_vol_in){bms_vol = bms_vol_in;}
    uint16_t get_bms_soc(){return bms_soc;}
    void set_bms_soc(uint16_t bms_soc_in){bms_soc = bms_soc_in;}
    uint16_t get_bms_tem(){return bms_tem;}
    void set_bms_tem(uint16_t bms_tem_in){bms_tem = bms_tem_in;}
    uint16_t get_blower_p(){return blower_p;}
    void set_blower_p(uint16_t blower_p_in){blower_p = blower_p_in;}
    uint16_t get_blower_tem(){return blower_tem;}
    void set_blower_tem(uint16_t blower_tem_in){blower_tem = blower_tem_in;}
    
    uint8_t get_bms_error1_byte(int index) {
        if (index >= 0 && index < 8) {
            return bms_error1[index];
        }
        return 0;
    }
    char* get_bms_error1_char() {
        return bms_error1_str;
    }
    void set_bms_error1(uint8_t bms_error1_in[8]) {
    for (int i = 0; i < 8; i++) {
        bms_error1[i] = bms_error1_in[i];
        }
    }
    void set_bms_error1_char(char* error_str) {
        for (int i = 0; i < 16; i++) {
            bms_error1_str[i] = error_str[i];
        }
        bms_error1_str[16] = '\0';
    }
    

    uint8_t get_bms_error2_byte(int index) {
        if (index >= 0 && index < 8) {
            return bms_error2[index];
        }
        return 0;
    }
    char* get_bms_error2_char() {
        return bms_error2_str;
    }
    void set_bms_error2(uint8_t bms_error2_in[8]) {
    for (int i = 0; i < 8; i++) {
        bms_error2[i] = bms_error2_in[i];
        }
    }
    void set_bms_error2_char(char* error_str) {
        for (int i = 0; i < 16; i++) {
            bms_error2_str[i] = error_str[i];
        }
        bms_error2_str[16] = '\0';
    }

    uint8_t get_blower_error_byte(int index) {
        if (index >= 0 && index < 8) {
            return blower_error[index];
        }
        return 0;
    }
    char* get_blower_error_char() {
        return blower_error_str;
    }
    void set_blower_error(uint8_t blower_error_in[8]) {
    for (int i = 0; i < 8; i++) {
        blower_error[i] = blower_error_in[i];
        }
    }
    void set_blower_error_char(char* error_str) {
        for (int i = 0; i < 16; i++) {
            blower_error_str[i] = error_str[i];
        }
        blower_error_str[16] = '\0';
    }



    
private:
    static FD1_DATA *_singleton;
    bool _new_data;
    float _yaw_deg;
    float _accuracy_deg;
    float alt; // 高度 cm
    float arspd_tas; // 真空速 m/s
    float climb_rate; // 升降速度 cm/s
    float aoa; // 迎角 degree
    float ssa; // 侧滑角 degree
    float roll; // 滚动角 degree
    float yaw; // 航向角 degree
    float rate_x; // X轴角速度 degree/s
    float rate_y; // Y轴角速度 degree/s
    float rate_z; // Z轴角速度 degree/s
    float acc_x; // X轴加速度 m/s/s
    float acc_y; // Y轴加速度 m/s/s
    float acc_z; // Z轴加速度 m/s/s
    uint32_t gps_utc; // 卫星时间 s

    int16_t mot1_rpm;
    int16_t mot2_rpm;
    int16_t mot3_rpm;
    int16_t mot4_rpm;
    uint16_t mot1_temperature;
    uint16_t mot2_temperature;
    uint16_t mot3_temperature;
    uint16_t mot4_temperature;
    uint16_t controller1_temperature;
    uint16_t controller2_temperature;
    uint16_t controller3_temperature;
    uint16_t controller4_temperature;
    float propeller1_angle;
    float propeller2_angle;
    float propeller3_angle;
    float propeller4_angle;
    uint16_t mot1_error;
    uint16_t mot2_error;
    uint16_t mot3_error;
    uint16_t mot4_error;
    uint16_t propeller1_error;
    uint16_t propeller2_error;
    uint16_t propeller3_error;
    uint16_t propeller4_error;
    uint16_t bms_vol;
    uint16_t bms_soc;
    uint16_t bms_tem;
    uint16_t blower_p;
    uint16_t blower_tem;
    uint8_t bms_error1[8];
    uint8_t bms_error2[8];
    uint8_t blower_error[8];
    char bms_error1_str[17];
    char bms_error2_str[17];
    char blower_error_str[17];
};


namespace AP {
    FD1_DATA &fd1_data();
};
