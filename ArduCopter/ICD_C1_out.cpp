#include "Copter.h"
#include "ICD_C1_out.h"

void Copter::icd_c1_out_init() {
	ICD_C1_out_info.ICD_C1_out_port = nullptr;
    ICD_C1_out_info.ICD_C1_out_port = serial_manager.find_serial(AP_SerialManager::SerialProtocol_ICD_C1_out, 0);
}

void Copter::icd_c1_out_update() {
	if (ICD_C1_out_info.ICD_C1_out_port == nullptr) {
		return;
	}

    icd_c1_out_write();
}

void Copter::icd_c1_out_write() {

    float terr_alt = 0.0f;
    if (rangefinder_state.enabled) {
        terr_alt = rangefinder_state.alt_cm_filt.get();
    }

    ICD_C1_out_Buf.ICD_C1_out_data.head_1 = ICD_C1_OUT_HEAD_1 ;
    ICD_C1_out_Buf.ICD_C1_out_data.head_2 = ICD_C1_OUT_HEAD_2 ;
    ICD_C1_out_Buf.ICD_C1_out_data.len = ICD_C1_OUT_LEN ;
    ICD_C1_out_Buf.ICD_C1_out_data.command = 0x10 ;
    ICD_C1_out_Buf.ICD_C1_out_data.sub_command = 0x10 ;
    ICD_C1_out_Buf.ICD_C1_out_data.lng = inertial_nav.get_longitude() ;
    ICD_C1_out_Buf.ICD_C1_out_data.lat = inertial_nav.get_latitude() ;
    ICD_C1_out_Buf.ICD_C1_out_data.alt_sealevel = inertial_nav.get_altitude()/100;
    ICD_C1_out_Buf.ICD_C1_out_data.alt_rel = inertial_nav.get_altitude()/10;
    ICD_C1_out_Buf.ICD_C1_out_data.alt_terrain = terr_alt/10 ;
    ICD_C1_out_Buf.ICD_C1_out_data.vel_target = inertial_nav.get_velocity_xy() ;
    ICD_C1_out_Buf.ICD_C1_out_data.roll_cd = ahrs.roll_sensor ;
    ICD_C1_out_Buf.ICD_C1_out_data.pitch_cd = ahrs.pitch_sensor ;
    ICD_C1_out_Buf.ICD_C1_out_data.yaw = ahrs.yaw_sensor ;
    ICD_C1_out_Buf.ICD_C1_out_data.tail = ICD_C1_OUT_TAIL ;

    ICD_C1_out_info.ICD_C1_out_port->write(ICD_C1_out_Buf.bytes, 128);
}
