#include "Copter.h"
#include "ICD_A1_in.h"

void Copter::icd_a1_in_init() {
	ICD_A1_in_info.ICD_A1_in_port = nullptr;
    ICD_A1_in_info.ICD_A1_in_port = serial_manager.find_serial(AP_SerialManager::SerialProtocol_ICD_A1_in, 0);

    if (ICD_A1_in_info.ICD_A1_in_port != nullptr) {
    	memset(&ICD_A1_in_info.ICD_A1_in_data_recv, 0, sizeof(ICD_A1_in_info.ICD_A1_in_data_recv));
        memset(&ICD_A1_in_Buf, 0, sizeof(ICD_A1_in_Buf));

		ICD_A1_in_info.i_counter = 0;
        ICD_A1_in_info.find_head_1 = false;
        ICD_A1_in_info.find_head_2 = false;
    }
}

void Copter::icd_a1_in_update() {
	if (ICD_A1_in_info.ICD_A1_in_port == nullptr) {
		return;
	}

	int16_t numc = ICD_A1_in_info.ICD_A1_in_port->available();
    uint8_t data_recv;
    for (int16_t i = 0; i < numc; i++)
    {
        data_recv = ICD_A1_in_info.ICD_A1_in_port->read();
        icd_a1_in_dealRecv(data_recv);
    }
}

void Copter::icd_a1_in_dealRecv(uint8_t data_in) {
	if (!ICD_A1_in_info.find_head_1 && !ICD_A1_in_info.find_head_2 && data_in == ICD_A1_IN_HEAD_1) {
		ICD_A1_in_info.find_head_1 = true;
	}
	if (ICD_A1_in_info.find_head_1 && !ICD_A1_in_info.find_head_2 && data_in == ICD_A1_IN_HEAD_2) {
		ICD_A1_in_info.find_head_2 = true;
		ICD_A1_in_info.i_counter = 2;
		ICD_A1_in_Buf.bytes[0] = ICD_A1_IN_HEAD_1;
		ICD_A1_in_Buf.bytes[1] = ICD_A1_IN_HEAD_2;
	}
	if (ICD_A1_in_info.find_head_2) {
		ICD_A1_in_Buf.bytes[ICD_A1_in_info.i_counter] = data_in;
		if (data_in == ICD_A1_IN_TAIL && ICD_A1_in_info.i_counter == 23) {
			memcpy(&ICD_A1_in_info.ICD_A1_in_data_recv, &ICD_A1_in_Buf, 24);
			ICD_A1_in_info.last_update_ms = millis();
		}
		ICD_A1_in_info.i_counter++;
	}
	if (ICD_A1_in_info.i_counter > 23) {
		ICD_A1_in_info.find_head_1 = false;
       	ICD_A1_in_info.find_head_2 = false;
       	ICD_A1_in_info.i_counter = 0;
	}
	ICD_A1_in_info.find_head_1 = false;
}
