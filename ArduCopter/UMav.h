class UMav {

public:

    // constructor, destructor
    UMav();


    void handle_msg(const mavlink_message_t &msg);
    void handle_selfcheck(const mavlink_message_t &msg);
    void handle_target(const mavlink_message_t &msg);
    void handle_target_result(const mavlink_message_t &msg);
    void handle_selfcheck_result(const mavlink_message_t &msg);
    void handle_status(const mavlink_message_t &msg);
    void handle_mission(const mavlink_message_t &msg);
    void handle_mission_result(const mavlink_message_t &msg);
    void handle_relay_position(const mavlink_message_t &msg);
    void handle_relay_position_result(const mavlink_message_t &msg);
    void handle_attack_info(const mavlink_message_t &msg);
    void handle_attack_cmd(const mavlink_message_t &msg);
    void handle_nav_cmd(const mavlink_message_t &msg);
    void send_do_selfcheck();
    void send_target();
    void send_status();
    void send_selfcheck_result();
    void send_target_result();
    void send_mission();
    void send_mission_result();
    void send_relay_position();
    void send_relay_position_result();
    void send_apm_status();

    void handle_info_test(int16_t p1);
    void send_all();

    uint8_t _computer_ok;
    uint8_t _target_ok;
    uint8_t _mission_ok;
    uint8_t _all_status;
};
