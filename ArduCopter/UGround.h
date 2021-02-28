#pragma once

class UGround {

public:

    // constructor, destructor
    UGround();

    // initialise
    void init();

    // return true if smart_rtl is usable (it may become unusable if the user took off without GPS lock or the path became too long)
    bool is_active() const { return _active; }

    void handle_info(bool valid, float input_x, float input_y);

    void update();


private:

    Vector2f raw_info;
    Vector2f correct_info;
    bool _active;
    uint32_t _last_update_ms;
};
