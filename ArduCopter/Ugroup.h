class my_group_t {
public:
    my_group_t() {;}
    virtual Vector3f get_pos(int16_t id_in, float dist) = 0;
    virtual Vector3f get_offset(int16_t this_id, int16_t sender_id, float dist) = 0;
};

class my_group_1_t : public my_group_t {
public:
    my_group_1_t() {;}
    Vector3f get_pos(int16_t id_in, float dist) override;
    Vector3f get_offset(int16_t this_id, int16_t sender_id, float dist) override;
};

class my_group_2_t : public my_group_t {
public:
    my_group_2_t() {;}
    Vector3f get_pos(int16_t id_in, float dist) override;
    Vector3f get_offset(int16_t this_id, int16_t sender_id, float dist) override;
};
