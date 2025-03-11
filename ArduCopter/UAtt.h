#pragma once

class UAtt {

public:

    // constructor, destructor
    UAtt();

    // initialise
    void init();

    void update();

    const Vector3f& get_ahrs();

    float roll;
    float pitch;
    float yaw;

private:

    LowPassFilterVector3f _ahrs_filter;

};
