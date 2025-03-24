#include "FD_BATT.h"

#include <AP_AHRS/AP_AHRS.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Logger/AP_Logger.h>
#include <GCS_MAVLink/GCS.h>
#include <SRV_Channel/SRV_Channel.h>

extern const AP_HAL::HAL &hal;


FD_BATT::FD_BATT(FD_CAN *frotend) {
    _frotend_ptr = frotend;
}

void FD_BATT::handle_info(AP_HAL::CANFrame &in_frame, bool do_print) {
    switch (in_frame.id) {
        case 0x16:
            if (in_frame.data[0] == 0x1 && in_frame.data[1] == 0xFF) {
<<<<<<< HEAD
                if (do_print) {gcs().send_text(MAV_SEVERITY_INFO, "get %x", in_frame.id);}
=======
                if (do_print) {gcs().send_text(MAV_SEVERITY_INFO, "get 0x16");}
>>>>>>> dcf5ca1547eb96d80843a6f43115fde9fcceee3e
                status.vfc  = float(in_frame.data[2]) + float(in_frame.data[3])*0.01f;
                status.vout = float(in_frame.data[4]) + float(in_frame.data[5])*0.01f;
                status.I    = float(in_frame.data[6]) + float(in_frame.data[7])*0.01f;
            } else {
<<<<<<< HEAD
                if (do_print) {gcs().send_text(MAV_SEVERITY_INFO, "Err %x", in_frame.id);}
=======
                if (do_print) {gcs().send_text(MAV_SEVERITY_INFO, "Err 0x16");}
>>>>>>> dcf5ca1547eb96d80843a6f43115fde9fcceee3e
            }
            break;
        case 0x17:
            if (in_frame.data[0] == 0x1 && in_frame.data[1] == 0xFE) {
<<<<<<< HEAD
                if (do_print) {gcs().send_text(MAV_SEVERITY_INFO, "get %x", in_frame.id);}
=======
                if (do_print) {gcs().send_text(MAV_SEVERITY_INFO, "get 0x16");}
>>>>>>> dcf5ca1547eb96d80843a6f43115fde9fcceee3e
                status.T1   = float(in_frame.data[2]) + float(in_frame.data[3])*0.01f;
                status.T2   = float(in_frame.data[4]) + float(in_frame.data[5])*0.01f;
                status.P    = float(in_frame.data[6]) + float(in_frame.data[7])*0.01f;
            } else {
<<<<<<< HEAD
                if (do_print) {gcs().send_text(MAV_SEVERITY_INFO, "Err %x", in_frame.id);}
=======
                if (do_print) {gcs().send_text(MAV_SEVERITY_INFO, "Err 0x16");}
>>>>>>> dcf5ca1547eb96d80843a6f43115fde9fcceee3e
            }
            break;
        case 0x18:
            if (in_frame.data[0] == 0x1 && in_frame.data[1] == 0xFD) {
<<<<<<< HEAD
                if (do_print) {gcs().send_text(MAV_SEVERITY_INFO, "get %x", in_frame.id);}
                status.PWM1 = in_frame.data[2];
                status.PWM2 = in_frame.data[3];
            } else {
                if (do_print) {gcs().send_text(MAV_SEVERITY_INFO, "Err %x", in_frame.id);}
=======
                if (do_print) {gcs().send_text(MAV_SEVERITY_INFO, "get 0x16");}
                status.PWM1 = in_frame.data[2];
                status.PWM2 = in_frame.data[3];
            } else {
                if (do_print) {gcs().send_text(MAV_SEVERITY_INFO, "Err 0x16");}
>>>>>>> dcf5ca1547eb96d80843a6f43115fde9fcceee3e
            }
            break;
        case 0x19:
            if (in_frame.data[0] == 0x1 && in_frame.data[1] == 0xFC) {
<<<<<<< HEAD
                if (do_print) {gcs().send_text(MAV_SEVERITY_INFO, "get %x", in_frame.id);}
=======
                if (do_print) {gcs().send_text(MAV_SEVERITY_INFO, "get 0x16");}
>>>>>>> dcf5ca1547eb96d80843a6f43115fde9fcceee3e
                status.vli  = float(in_frame.data[2]) + float(in_frame.data[3])*0.01f;
                status.vhy  = float(in_frame.data[4]) + float(in_frame.data[5])*0.01f;
                status.vbus = float(in_frame.data[6]) + float(in_frame.data[7])*0.01f;
            } else {
<<<<<<< HEAD
                if (do_print) {gcs().send_text(MAV_SEVERITY_INFO, "Err %x", in_frame.id);}
=======
                if (do_print) {gcs().send_text(MAV_SEVERITY_INFO, "Err 0x16");}
>>>>>>> dcf5ca1547eb96d80843a6f43115fde9fcceee3e
            }
            break;
        case 0x20:
            if (in_frame.data[0] == 0x1 && in_frame.data[1] == 0xFB) {
<<<<<<< HEAD
                if (do_print) {gcs().send_text(MAV_SEVERITY_INFO, "get %x", in_frame.id);}
=======
                if (do_print) {gcs().send_text(MAV_SEVERITY_INFO, "get 0x16");}
>>>>>>> dcf5ca1547eb96d80843a6f43115fde9fcceee3e
                status.power = float(in_frame.data[2])*100.f + float(in_frame.data[3]) + float(in_frame.data[4])*0.01f;
                status.HPWM1 = in_frame.data[5];
                status.HPWM2 = in_frame.data[6];
            } else {
<<<<<<< HEAD
                if (do_print) {gcs().send_text(MAV_SEVERITY_INFO, "Err %x", in_frame.id);}
=======
                if (do_print) {gcs().send_text(MAV_SEVERITY_INFO, "Err 0x16");}
>>>>>>> dcf5ca1547eb96d80843a6f43115fde9fcceee3e
            }
            break;
        case 0x14:
            if (in_frame.data[0] == 0x1 && in_frame.data[1] == 0xFC) {
<<<<<<< HEAD
                if (do_print) {gcs().send_text(MAV_SEVERITY_INFO, "get %x", in_frame.id);}
                status.error = in_frame.data[2];
                status.run   = in_frame.data[3];
            } else {
                if (do_print) {gcs().send_text(MAV_SEVERITY_INFO, "Err %x", in_frame.id);}
=======
                if (do_print) {gcs().send_text(MAV_SEVERITY_INFO, "get 0x16");}
                status.error = in_frame.data[2];
                status.run   = in_frame.data[3];
            } else {
                if (do_print) {gcs().send_text(MAV_SEVERITY_INFO, "Err 0x16");}
>>>>>>> dcf5ca1547eb96d80843a6f43115fde9fcceee3e
            }
            break;
        case 0x13:
            if (in_frame.data[0] == 0x5 && in_frame.data[1] == 0x5) {
                gcs().send_text(MAV_SEVERITY_INFO, "cmd OK");
            } else if (in_frame.data[0] == 0x6 && in_frame.data[1] == 0x6) {
                gcs().send_text(MAV_SEVERITY_INFO, "cmd Fail");
            } else {
<<<<<<< HEAD
                if (do_print) {gcs().send_text(MAV_SEVERITY_INFO, "Err %x", in_frame.id);}
=======
                if (do_print) {gcs().send_text(MAV_SEVERITY_INFO, "Err 0x16");}
>>>>>>> dcf5ca1547eb96d80843a6f43115fde9fcceee3e
            }
            break;
        default:
            if (do_print) {gcs().send_text(MAV_SEVERITY_INFO, "unknown %ld", (uint32_t)in_frame.id);}
            break;
    }
}

void FD_BATT::send_cmd(uint32_t id, uint8_t *data) {
    if (_frotend_ptr == nullptr) {return;}
    const uint8_t data_length = 8;
    AP_HAL::CANFrame txFrame{};
    memcpy(txFrame.data, data, data_length);
    txFrame.id = id;
    _frotend_ptr->write_frame(txFrame, 0);
}
