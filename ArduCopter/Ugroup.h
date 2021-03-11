class my_group_t {
public:
    my_group_t() {_distance = 200.f;}
    void set_distance(float v) {_distance = v;}
    virtual Vector3f get_offset(int16_t this_id, int16_t sender_id) = 0;
    float _distance;
};

class my_group_1_t : public my_group_t {
public:
    Vector3f get_offset(int16_t this_id, int16_t sender_id) override;
};

class my_group_2_t : public my_group_t {
public:
    Vector3f get_offset(int16_t this_id, int16_t sender_id) override;
};
