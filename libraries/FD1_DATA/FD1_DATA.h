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
    uint32_t get_gps_utc(){return gps_utc;}
    void set_gps_utc(uint32_t gps_utc_in){gps_utc = gps_utc_in;}
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
};


namespace AP {
    FD1_DATA &fd1_data();
};
