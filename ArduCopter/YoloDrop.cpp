#include "Copter.h"

const AP_Param::GroupInfo YoloDrop::var_info[] = {
    // @Param: EN
    // @DisplayName: YoloDrop Enable
    // @Description: Master switch: 0=disabled, 1=enabled, 2=enabled+debug
    // @Values: 0:Disabled,1:Enabled,2:Enabled+Debug
    // @User: Standard
    AP_GROUPINFO_FLAGS("EN", 0, YoloDrop, enable, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: CONF
    // @DisplayName: Confidence threshold
    // @Description: Minimum YOLO confidence to accept detection
    // @Range: 0.1 1.0
    // @User: Standard
    AP_GROUPINFO("CONF", 1, YoloDrop, conf_threshold,  0.5f),

    // @Param: SPD
    // @DisplayName: Approach speed gain
    // @Description: Horizontal velocity gain in cm/s when offset=1.0
    // @Units: cm/s
    // @Range: 100 1000
    // @User: Standard
    AP_GROUPINFO("SPD",  2, YoloDrop, approach_speed,  300.0f),

    // @Param: AREA
    // @DisplayName: Target area
    // @Description: Desired normalised target area (width*height) for descent stop
    // @Range: 0.01 0.5
    // @User: Standard
    AP_GROUPINFO("AREA", 3, YoloDrop, target_area,     0.10f),

    // @Param: PWMO
    // @DisplayName: Servo open PWM
    // @Description: PWM value to open the gripper servo
    // @Units: PWM
    // @Range: 1000 2000
    // @User: Standard
    AP_GROUPINFO("PWMO", 4, YoloDrop, drop_pwm_open,   1900),

    // @Param: PWMC
    // @DisplayName: Servo close PWM
    // @Description: PWM value to close the gripper servo
    // @Units: PWM
    // @Range: 1000 2000
    // @User: Standard
    AP_GROUPINFO("PWMC", 5, YoloDrop, drop_pwm_close,  1100),

    AP_GROUPEND
};

YoloDrop::YoloDrop()
    : _yolo_ptr(nullptr)
    , _state(State::IDLE)
    , _confirm_counter(0)
    , _state_enter_ms(0)
    , _last_valid_ms(0)
    , _drop_completed(false)
    , _initialized(false)
    , _cmd_vel_n(0)
    , _cmd_vel_e(0)
    , _cmd_vel_d(0)
    , _last_parse_count(0)
    , _last_fps_ms(0)
    , _parse_fps(0)
{
    AP_Param::setup_object_defaults(this, var_info);
    memset(&_best, 0, sizeof(_best));
}

void YoloDrop::init()
{
    if (enable.get() == 0) {
        return;
    }

    _yolo_ptr = new FD_YOLO(AP_SerialManager::SerialProtocol_YOLO);
    if (_yolo_ptr == nullptr || !_yolo_ptr->initialized()) {
        gcs().send_text(MAV_SEVERITY_WARNING, "YoloDrop: YOLO serial not found");
        return;
    }

    set_servo_close();
    _state = State::IDLE;
    _drop_completed = false;
    _confirm_counter = 0;
    _initialized = true;

    gcs().send_text(MAV_SEVERITY_INFO, "YoloDrop: init OK (EN=%d)", enable.get());
}

void YoloDrop::update()
{
    if (enable.get() == 0 || !_initialized) {
        return;
    }

    _yolo_ptr->read();

    if (select_best_target()) {
        _last_valid_ms = AP_HAL::millis();
    }

    switch (_state) {
    case State::IDLE:
        update_idle();
        break;
    case State::APPROACHING:
        update_approaching();
        break;
    case State::DESCENDING:
        update_descending();
        break;
    case State::HOVERING:
        update_hovering();
        break;
    case State::RELEASING:
        update_releasing();
        break;
    case State::DONE:
        update_done();
        break;
    }
}

bool YoloDrop::select_best_target()
{
    YoloFrame &frm = _yolo_ptr->get_msg().frame;
    if (!frm.updated) {
        return false;
    }
    frm.updated = false;

    if (frm.num_detections == 0) {
        return false;
    }

    float best_conf = -1.0f;
    uint8_t best_idx = 0;
    for (uint8_t i = 0; i < frm.num_detections; i++) {
        if (frm.detections[i].confidence > best_conf) {
            best_conf = frm.detections[i].confidence;
            best_idx = i;
        }
    }

    if (best_conf < conf_threshold.get()) {
        return false;
    }

    _best = frm.detections[best_idx];
    return true;
}

// ---- state handlers ----

void YoloDrop::update_idle()
{
    if (copter.flightmode->mode_number() != Mode::Number::AUTO) {
        _confirm_counter = 0;
        return;
    }
    if (_drop_completed) {
        return;
    }

    uint32_t now = AP_HAL::millis();
    if (now - _last_valid_ms < 200) {
        _confirm_counter++;
    } else {
        _confirm_counter = 0;
    }

    if (_confirm_counter >= YDROP_CONFIRM_COUNT) {
        switch_to_guided();
        enter_state(State::APPROACHING);
        gcs().send_text(MAV_SEVERITY_INFO, "YoloDrop: target confirmed, GUIDED");
    }
}

void YoloDrop::update_approaching()
{
    if (copter.flightmode->mode_number() != Mode::Number::GUIDED) {
        enter_state(State::IDLE);
        return;
    }

    uint32_t now = AP_HAL::millis();
    if (now - _last_valid_ms > YDROP_LOST_TIMEOUT_MS) {
        handle_target_lost();
        return;
    }

    float vn, ve;
    compute_horizontal_velocity(vn, ve);
    send_guided_velocity(vn, ve, 0);

    if (fabsf(_best.offset_x) < YDROP_CENTER_THRESHOLD &&
        fabsf(_best.offset_y) < YDROP_CENTER_THRESHOLD) {
        enter_state(State::DESCENDING);
        gcs().send_text(MAV_SEVERITY_INFO, "YoloDrop: centered, descending");
    }
}

void YoloDrop::update_descending()
{
    if (copter.flightmode->mode_number() != Mode::Number::GUIDED) {
        enter_state(State::IDLE);
        return;
    }

    uint32_t now = AP_HAL::millis();
    if (now - _last_valid_ms > YDROP_LOST_TIMEOUT_MS) {
        handle_target_lost();
        return;
    }

    float vn, ve;
    compute_horizontal_velocity(vn, ve);

    float area = _best.norm_width * _best.norm_height;
    float area_error = target_area.get() - area;
    float vd = area_error * YDROP_DESCEND_SPEED;
    vd = constrain_float(vd, -YDROP_MAX_CLIMB_RATE, YDROP_MAX_DESCEND_RATE);

    send_guided_velocity(vn, ve, vd);

    if (fabsf(area_error) < YDROP_AREA_THRESHOLD &&
        fabsf(_best.offset_x) < YDROP_CENTER_THRESHOLD &&
        fabsf(_best.offset_y) < YDROP_CENTER_THRESHOLD) {
        enter_state(State::HOVERING);
        gcs().send_text(MAV_SEVERITY_INFO, "YoloDrop: area OK, hovering");
    }
}

void YoloDrop::update_hovering()
{
    if (copter.flightmode->mode_number() != Mode::Number::GUIDED) {
        enter_state(State::IDLE);
        return;
    }

    uint32_t now = AP_HAL::millis();
    if (now - _last_valid_ms > YDROP_LOST_TIMEOUT_MS) {
        handle_target_lost();
        return;
    }

    float area = _best.norm_width * _best.norm_height;
    bool offset_ok = fabsf(_best.offset_x) < (YDROP_CENTER_THRESHOLD + 0.01f) &&
                     fabsf(_best.offset_y) < (YDROP_CENTER_THRESHOLD + 0.01f);
    bool area_ok = fabsf(target_area.get() - area) < (YDROP_AREA_THRESHOLD + 0.01f);

    if (!offset_ok || !area_ok) {
        enter_state(State::DESCENDING);
        return;
    }

    send_guided_velocity(0, 0, 0);

    if (now - _state_enter_ms >= YDROP_HOVER_TIME_MS) {
        enter_state(State::RELEASING);
        gcs().send_text(MAV_SEVERITY_INFO, "YoloDrop: releasing");
    }
}

void YoloDrop::update_releasing()
{
    set_servo_open();

    uint32_t now = AP_HAL::millis();
    send_guided_velocity(0, 0, 0);

    if (now - _state_enter_ms >= YDROP_RELEASE_TIME_MS) {
        enter_state(State::DONE);
        gcs().send_text(MAV_SEVERITY_INFO, "YoloDrop: drop complete, LOITER");
    }
}

void YoloDrop::update_done()
{
    set_servo_close();
    _drop_completed = true;
    switch_to_loiter();
}

// ---- helpers ----

void YoloDrop::compute_horizontal_velocity(float &vn, float &ve)
{
    float k = approach_speed.get();
    float body_vx = -_best.offset_y * k;
    float body_vy =  _best.offset_x * k;

    float yaw = copter.ahrs.get_yaw();
    calc_body_to_ned(body_vx, body_vy, yaw, vn, ve);
}

void YoloDrop::calc_body_to_ned(float bx, float by, float yaw_rad, float &vn, float &ve)
{
    float cy = cosf(yaw_rad);
    float sy = sinf(yaw_rad);
    vn = bx * cy - by * sy;
    ve = bx * sy + by * cy;
}

void YoloDrop::send_guided_velocity(float vn, float ve, float vd)
{
    _cmd_vel_n = vn;
    _cmd_vel_e = ve;
    _cmd_vel_d = vd;

    Vector3f vel_cms(vn, ve, vd);
    Vector3f zero_accel;
    copter.mode_guided.set_velaccel(vel_cms, zero_accel, false, 0, false, 0, false);
}

void YoloDrop::set_servo_open()
{
    SRV_Channels::set_output_pwm(SRV_Channel::k_gripper, (uint16_t)drop_pwm_open.get());
}

void YoloDrop::set_servo_close()
{
    SRV_Channels::set_output_pwm(SRV_Channel::k_gripper, (uint16_t)drop_pwm_close.get());
}

void YoloDrop::switch_to_guided()
{
    copter.set_mode(Mode::Number::GUIDED, ModeReason::SCRIPTING);
}

void YoloDrop::switch_to_loiter()
{
    if (copter.flightmode->mode_number() != Mode::Number::LOITER) {
        copter.set_mode(Mode::Number::LOITER, ModeReason::SCRIPTING);
    }
}

void YoloDrop::enter_state(State new_state)
{
    _state = new_state;
    _state_enter_ms = AP_HAL::millis();
}

void YoloDrop::handle_target_lost()
{
    gcs().send_text(MAV_SEVERITY_WARNING, "YoloDrop: target lost, LOITER");
    switch_to_loiter();
    enter_state(State::IDLE);
    _confirm_counter = 0;
}

void YoloDrop::print_debug()
{
    if (enable.get() < 2 || !_initialized) {
        return;
    }

    uint32_t now = AP_HAL::millis();

    uint32_t cur_count = _yolo_ptr->get_msg().parse_count;
    uint32_t dt_ms = now - _last_fps_ms;
    if (dt_ms > 0) {
        uint32_t delta = cur_count - _last_parse_count;
        _parse_fps = (float)delta * 1000.0f / (float)dt_ms;
    }
    _last_parse_count = cur_count;
    _last_fps_ms = now;

    const char *state_names[] = {"IDLE", "APPR", "DESC", "HOVR", "RELS", "DONE"};
    uint8_t si = (uint8_t)_state;
    if (si > 5) si = 0;

    gcs().send_text(MAV_SEVERITY_INFO, "YDROP: st=%s cfm=%u done=%d fps=%.1f",
                    state_names[si], _confirm_counter, (int)_drop_completed,
                    (double)_parse_fps);

    if (now - _last_valid_ms < 1500) {
        gcs().send_text(MAV_SEVERITY_INFO, "YDROP: id=%u cf=%.2f ox=%.2f oy=%.2f a=%.3f",
                        _best.class_id, (double)_best.confidence,
                        (double)_best.offset_x, (double)_best.offset_y,
                        (double)(_best.norm_width * _best.norm_height));
    }

    if (_state == State::APPROACHING || _state == State::DESCENDING) {
        gcs().send_text(MAV_SEVERITY_INFO, "YDROP: vn=%.0f ve=%.0f vd=%.0f",
                        (double)_cmd_vel_n, (double)_cmd_vel_e, (double)_cmd_vel_d);
    }
}
