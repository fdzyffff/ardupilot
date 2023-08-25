#pragma once

class User_second_thr{
public:
    enum class Thr_State{
        ZERO = 0,
        SPINUP = 1,
        NORMAL = 2
    };

    void init();
    void update();
    void set_state(Thr_State state_in);

private:
    Thr_State _state;
    uint32_t _last_update_ms;
};
