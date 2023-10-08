void Ucmd_list::start() {
	running = true;
	switch (g.this_sysid) {
		case 1:
		case 2:
			packup_1st_1p1();
			break;
		case 3:
		case 4:
			packup_1st_2p1();
			break;
		case 5:
		case 6:
			packup_1st_3p1();
			break;
		default:
			break;
	}
}

void Ucmd_list::reset() {
	current_cmd_id = 0;
	running = false;
}

void Ucmd_list::update() {
	if (!running) {
		return;
	}

	Ucmd &current_cmd = ucmds[current_cmd_id];
	bool verified = false;
	if (current_cmd.type == Ucmd::Type::Normal) {
		verified = verify_normal(current_cmd);
	}
	if (current_cmd.type == Ucmd::Type::Normal) {
		verified = verify_normal(current_cmd);
	}
	if (verified) {
		if (current_cmd_id < total_cmd-1) {
			current_cmd_id++;
		}
	}
}

void Ucmd_list::verify_normal(Ucmd &cmd) {
	bool yaw_ok = false;
	bool pos_ok = false;
	bool pos_delay_ok = false;

	static last_pos_ok_ms = millis();
	if (yaw_ok && pos_ok) {
		if (millis() - last_pos_ok_ms > 1000) {
			pos_delay_ok = true;
		}
	} else {
		last_pos_ok_ms = millis();
	}
	return (yaw_ok && pos_ok && pos_delay_ok);
}

void Ucmd_list::verify_sync(Ucmd &cmd) {
	bool yaw_ok = false;
	bool pos_ok = false;
	bool pos_delay_ok = false;

	static last_pos_ok_ms = millis();
	if (yaw_ok && pos_ok) {
		if (millis() - last_pos_ok_ms > 1000) {
			pos_delay_ok = true;
		}
	} else {
		last_pos_ok_ms = millis();
	}
	bool others_ok = true;
	for (uint8_t i_uav = 0; i_uav < copter.uconnection.total_uav; iuav++) {
		if (copter.uconnection.uavs[i_uav].valid() && copter.uconnection.uavs[i_uav].current_cmd_id >= current_cmd_id) {
			;
		} else {
			others_ok = false;
			break;
		}
	}
	return (yaw_ok && pos_ok && pos_delay_ok && others_ok);
}

void Ucmd_list::verify_choose(Ucmd &cmd) {
	bool yaw_ok = false;
	bool pos_ok = false;
	bool pos_delay_ok = false;

	static last_pos_ok_ms = millis();
	if (yaw_ok && pos_ok) {
		if (millis() - last_pos_ok_ms > 1000) {
			pos_delay_ok = true;
		}
	} else {
		last_pos_ok_ms = millis();
	}
	if (yaw_ok && pos_ok && pos_delay_ok) {
		if (distance_m > 2.0f) {
			if (cmd.id == 1) {	
				switch (g.this_sysid) {
					case 1:
					case 2:
						packup_1st_1p2();
						break;
					case 3:
					case 4:
						packup_1st_2p2();
						break;
					case 5:
					case 6:
						packup_1st_3p2();
						break;
					default:
						break;
				}
			}
			if (cmd.id == 2) {	
				switch (g.this_sysid) {
					case 1:
					case 2:
						packup_1st_1p3();
						break;
					case 3:
					case 4:
						packup_1st_2p3();
						break;
					case 5:
					case 6:
						packup_1st_3p3();
						break;
					default:
						break;
				}
			}
			if (cmd.id == 3) {	
				switch (g.this_sysid) {
					case 1:
						packup_2nd_1p4();
					case 2:
						packup_2nd_2p4();
						break;
					case 3:
						packup_2nd_3p4();
						break;
					case 4:
						packup_2nd_4p4();
						break;
					case 5:
						packup_2nd_5p4();
						break;
					case 6:
						packup_2nd_6p4();
						break;
					default:
						break;
				}
			}
		}
	}
}

void Ucmd_list::packup_1st_1p1()
{
	uint16_t temp_id = 0
	ucmds[temp_id] = new Ucmd();
	temp_id++;
	total_cmd++;
}
