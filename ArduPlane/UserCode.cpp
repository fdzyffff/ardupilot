#include "Plane.h"


void Plane::user_50Hz() {
    ubase.update();
    useruartfwd.update();
}

void Plane::user_1Hz() {
    // ubase.print();
}

bool Plane::allow_to_land() {
    if (!rc().has_valid_input()) {
        return true;
    } else {
        const RC_Channel *tchan = rc().channel(plane.rcmap.throttle()-1);
        if (tchan == nullptr) {
            return true;
        }
        float tval = (tchan->norm_input_ignore_trim()+1.0f)*0.5f;
        if (tval >= 0.4f) {
            return false;
        } else {
            return true;
        }
    }
    return true;
}
