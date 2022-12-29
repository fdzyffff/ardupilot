#pragma once

class UCam_Port;

class UCam {

public:

    // constructor, destructor
    UCam();

    // initialise
    void init();

    // return true if smart_rtl is usable (it may become unusable if the user took off without GPS lock or the path became too long)
    bool is_active() const { return _active; }
    int16_t cam_state();
    void init_port();
    void port_read();
    void do_cmd(float p1, float p2 = 0.0f, float p3 = 0.0f, float p4 = 0.0f);
    void handle_info(const mavlink_command_long_t* packet);
    void time_out_check();
    void udpate_value(float dt);

    const Vector2f& get_raw_info() ;
    const Vector2f& get_correct_info() ;

    void update();

    float get_target_pitch_rate() {return _target_pitch_rate;}
    float get_target_roll_angle() {return _target_roll_angle;}
    float get_target_yaw_rate() {return _target_yaw_rate_cds;}
    float get_current_angle_deg() {return _current_angle_deg;}
    float get_q_rate_cds() {return _q_rate_cds;}
    AP_HAL::UARTDriver* get_port(void) {return _port;}

    bool display_info_new;
    float display_info_p1;
    float display_info_p2;
    float display_info_p3;
    float display_info_p4;
    int16_t display_info_count;


    Vector2f raw_info;
    Vector2f correct_info;
    bool _active;
    uint32_t _last_update_ms;
    int16_t _n_count;
    int16_t _cam_state;
    bool _new_data;
    AP_HAL::UARTDriver* _port;
    float _target_pitch_rate;
    float _target_roll_angle;
    float _target_yaw_rate_cds;
    float _current_angle_deg;
    float _q_rate_cds;

    UCam_Port* _Ucam_port;

    LowPassFilterVector3f _cam_filter;
    LowPassFilterFloat _q_cds_filter;

private:
    void update_target_pitch_rate();
    void update_target_roll_angle();
    void update_target_yaw_rate();
    void update_target_track_angle();
    void update_q_rate_cds(float dt);
};

class UCam_Port {
public:
    friend class UCam;

    UCam_Port(UCam &frotend_in): _frotend(frotend_in) {};
    virtual void port_read() = 0;
    virtual void do_cmd(float p1, float p2 = 0.0f, float p3 = 0.0f, float p4 = 0.0f) = 0;
    UCam &_frotend;
};

class UCam_Port_Mavlink: public UCam_Port {
public:
    friend class UCam;

    UCam_Port_Mavlink(UCam &frotend_in);
    void port_read() override;
    void do_cmd(float p1, float p2 = 0.0f, float p3 = 0.0f, float p4 = 0.0f) override;

private:
    struct {
        // socket to telem2 on aircraft
        bool connected;
        mavlink_message_t rxmsg;
        mavlink_status_t status;
        uint8_t seq;
    } mavlink;
};

class UCam_Port_ASCII: public UCam_Port {
public:
    friend class UCam;

    UCam_Port_ASCII(UCam &frotend_in);
    void port_read() override;
    void do_cmd(float p1, float p2 = 0.0f, float p3 = 0.0f, float p4 = 0.0f) override;
    bool detect_sentence();
    float get_number();
    void fill_number();
    void handle_info();

private:
    char _term[20];                ///< buffer for the current term within the current sentence
    // uint8_t _sentence_type;        ///< the sentence type currently being processed
    uint8_t _term_number;          ///< term index within the current sentence
    uint8_t _term_offset;          ///< character offset with the term being received
    uint8_t _line_state;           /// 1 for correct line, 0 for invalid line which can not be recongnized correctlly 
    float _left;
    float _right;
    float _top;
    float _bottom;
};
