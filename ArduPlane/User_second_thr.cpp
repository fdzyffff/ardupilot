#include "Plane.h"

#define USER_SECOND_THR_MIN 0.1f
void User_second_thr::init()
{
    set_state(Thr_State::ZERO);
    return;
}

void User_second_thr::update()
{
    float thr_in = SRV_Channels::get_output_scaled(SRV_Channel::k_throttle)*0.01f;
    uint32_t delta_t = millis() - _last_update_ms;
    switch (_state) {
        case Thr_State::ZERO:
            if (thr_in < USER_SECOND_THR_MIN) {
                break;
            } else {
                set_state(Thr_State::SPINUP);
                break;
            }
            break;
        case Thr_State::SPINUP:
            if (thr_in < USER_SECOND_THR_MIN) {
                set_state(Thr_State::ZERO);
                break;
            }
            if (delta_t > 1000) {
                set_state(Thr_State::NORMAL);
                break;
            }
            break;
        case Thr_State::NORMAL:
            if (thr_in < USER_SECOND_THR_MIN) {
                set_state(Thr_State::ZERO);
                break;
            }
            break;
        default:
            break;
    }

    float thr_out = 0.0f;
    switch (_state) {
        default:
        case Thr_State::ZERO:
        case Thr_State::SPINUP:
            thr_out = 0.0f;
            break;
        case Thr_State::NORMAL:
            thr_out = thr_in*100.f;
            break;
    }
    SRV_Channels::set_output_scaled(SRV_Channel::k_throttle_second, thr_out);
}

void User_second_thr::set_state(Thr_State state_in)
{
    _state = state_in;
    _last_update_ms = millis();
}
