#pragma once

class UCam {

public:

    // constructor, destructor
    UCam();

    // initialise
    void init();

    // return true if smart_rtl is usable (it may become unusable if the user took off without GPS lock or the path became too long)
    bool is_active() const { return _active; }

    void handle_info(float input_x, float input_y, bool valid);

    const Vector2f& get_raw_info() const {return raw_info;}
    const Vector2f& get_correct_info() const {return correct_info;}

    void update();

private:

    Vector2f raw_info;
    Vector2f correct_info;
    bool _active;
    uint32_t _last_update_ms;
};
