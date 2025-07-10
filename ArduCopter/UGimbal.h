#pragma once

#include <AP_HAL/AP_HAL.h>
#include <FD_Gimbal/FD_Gimbal.h>

class UGimbal {

public:

    friend class Copter;
    friend class ModeMission;

    // constructor, destructor
    UGimbal();

    enum class Gimbal_State {
        Ahead = 0,
        Search,
        Lock,
    };

    // var_info for holding Parameter information
    static const struct AP_Param::GroupInfo var_info[];

    void init();
    void init_gimbal();
    const Vector2f& get_bf_info();
    const Vector2f& get_ef_info();
    const Vector2f& get_ef_rate_info();
    void update();
    void update_gimbal_control();
    void gimbal_ret_update();
    void gimbal_control_update();
    void handle_info_final(float p1, float p2);
    void do_gimbal_attitude_control(float target_gimbal_pitch, float target_gimbal_yaw);
    void update_gimbal_pitch_rate(float target_gimbal_pitch, float dt);
    void update_gimbal_yaw_rate(float target_gimbal_yaw, float dt);
    void handle_gimbal_msg(const mavlink_message_t &msg);
    void update_log();
    void set_state(Gimbal_State state_in);

    float get_gimbal_pitch_rate() {return _gimbal_pitch_rate;}
    float get_gimbal_yaw_rate() {return _gimbal_yaw_rate;}

    bool have_target();

    struct {
        float p1;
        float p2;
        float p3;
        float p4;
        float p11;
        float p12;
        float p13;
        float p14;
        float p21;
        float p22;
        float p23;
        float p24;
        bool new_data;
        uint16_t count;
        uint16_t count_log;
    } display_info;

    Gimbal_State _state;

    Vector2f bf_info;
    Vector2f ef_info;
    Vector2f ef_rate_info;
    bool _ret_valid;
    float _gimbal_pitch_rate;
    float _gimbal_yaw_rate;
    float _cam_roll;
    float _cam_pitch;
    float _cam_bf_yaw;
    float _cam_yaw;


private:

    AP_Int16        print;
    AP_Int8         use_gimbal_cam;
    AP_Int8         use_gimbal_loc;
    AP_Float        filt_yaw_hz;
    AP_Float        filt_pithc_hz;

    AC_PID          lock_yaw_pid{0.5f, 0.0f, 0.01f, 0.0f, 0.0f, 5.0f, 5.0f, 5.0f, 0.0f};
    AC_PID          lock_pitch_pid{0.5f, 0.0f, 0.01f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

    FD_Gimbal_Base*   _Gimbal_ptr;
    FD_Gimbal_HaoFu*  _Gimbal_ptr_cam;
    FD_Gimbal_Loc*    _Gimbal_ptr_loc;

    uint32_t _last_ms;
    uint32_t _last_switch_ms;
    int8_t current_idx;

    DerivativeFilterFloat_Size7 _pitch_filter;
    DerivativeFilterFloat_Size7 _yaw_filter;
    LowPassFilterFloat _yaw_sample_filter;
    LowPassFilterFloat _pitch_sample_filter;
    float _last_yaw;
    float _last_yaw_sample;


    #define UDELAY_BUFFER 100
    class UDelay {
    public:
        UDelay() {;};
        
        void init();
        void push();
        bool get_idx(uint16_t step, float &roll, float &pitch, float &yaw);

    private:
        struct {
            float roll;
            float pitch;
            float yaw;
            uint32_t time_ms;
        } _buffer[UDELAY_BUFFER];
        uint16_t _idx;
    };

    UDelay udelay;

};
