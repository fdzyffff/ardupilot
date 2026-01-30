#include "Plane.h"

void Plane::userhook_init()
{
    uart.init();
    uattack.init();
    udelay.init();
}

void Plane::userhook_100Hz()
{
    uattack.update();
    uart.update();
    udelay.push();
}

void Plane::userhook_1Hz()
{
    AP::fd_data().update();

    uattack.do_print();

    // static uint8_t tt = 0;
    // if (uart.get_port() != nullptr) {
    //     // get_port()->write(uart_msg_LS_status._msg_1.content.data, sizeof(uart_msg_LS_status._msg_1.content.data));
    //     uart.get_port()->write(tt++);
    //     gcs().send_text(MAV_SEVERITY_WARNING, "Uart send");
    // }
    
}

// position_ok - returns true if the horizontal absolute position is ok and home position is set
bool Plane::position_ok() const
{
    if (!ahrs.have_inertial_nav()) {
        // do not allow navigation with dcm position
        return false;
    }

    // with EKF use filter status and ekf check
    nav_filter_status filt_status;
    if (AP::ahrs().get_filter_status(filt_status)) {
        return ((filt_status.flags.horiz_pos_abs || filt_status.flags.pred_horiz_pos_abs));
    }
    return false;
}
