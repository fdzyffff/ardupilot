#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>
#include <FD_Target_Uart/FD_YOLO.h>
#include <SRV_Channel/SRV_Channel.h>

#define YDROP_CONFIRM_COUNT       10
#define YDROP_LOST_TIMEOUT_MS     3000
#define YDROP_CENTER_THRESHOLD    0.08f
#define YDROP_AREA_THRESHOLD      0.03f
#define YDROP_DESCEND_SPEED       200.0f
#define YDROP_MAX_DESCEND_RATE    100.0f
#define YDROP_MAX_CLIMB_RATE      50.0f
#define YDROP_HOVER_TIME_MS       2000
#define YDROP_RELEASE_TIME_MS     1500

class Copter;

class YoloDrop {
public:
    friend class Copter;

    YoloDrop();

    static const struct AP_Param::GroupInfo var_info[];

    void init();
    void update();
    void print_debug();

    bool is_active() const { return _state != State::IDLE && _state != State::DONE; }
    bool is_drop_completed() const { return _drop_completed; }

private:
    FD_YOLO *_yolo_ptr;

    enum class State : uint8_t {
        IDLE,
        APPROACHING,
        DESCENDING,
        HOVERING,
        RELEASING,
        DONE,
    };
    State _state;

    uint16_t _confirm_counter;
    uint32_t _state_enter_ms;
    uint32_t _last_valid_ms;
    bool     _drop_completed;
    bool     _initialized;
    YoloDetection _best;

    float _cmd_vel_n;
    float _cmd_vel_e;
    float _cmd_vel_d;

    uint32_t _last_parse_count;
    uint32_t _last_fps_ms;
    float    _parse_fps;

    AP_Int8   enable;
    AP_Float  conf_threshold;
    AP_Float  approach_speed;
    AP_Float  target_area;
    AP_Int16  drop_pwm_open;
    AP_Int16  drop_pwm_close;

    void update_idle();
    void update_approaching();
    void update_descending();
    void update_hovering();
    void update_releasing();
    void update_done();

    bool select_best_target();
    void send_guided_velocity(float vn, float ve, float vd);
    void calc_body_to_ned(float bx, float by, float yaw_rad, float &vn, float &ve);
    void compute_horizontal_velocity(float &vn, float &ve);
    void set_servo_open();
    void set_servo_close();
    void switch_to_guided();
    void switch_to_loiter();
    void enter_state(State new_state);
    void handle_target_lost();
};
