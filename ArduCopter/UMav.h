class UMav {

public:

    // constructor, destructor
    UMav();

    void handle_msg(const mavlink_message_t &msg);
    void send_status();
    void handle_selfcheck(const mavlink_message_t &msg);
    void send_selfcheck();
    void handle_target(const mavlink_message_t &msg);
    void send_target();
    void handle_mission(const mavlink_message_t &msg);
    void send_mission();
    void handle_info_test(int16_t p1);

    uint8_t _computer_ok;
    uint8_t _target_ok;
    uint8_t _mission_ok;
    uint8_t _all_status;
};
