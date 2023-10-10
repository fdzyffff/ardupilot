
class Ucmd()
{
public:
	enum class Type {
		Normal = 0,
		Choose = 1,
		Sync = 2,
	}
	Ucmd();
	Type type;
	uint16_t id; 
	uint32_t time_out_ms;
	Vector3f target_pos; // neu
	float target_yaw_deg;
};

class Ucmd_list()
{
public:
	void start();
	void reset();
	void update();
	void verify_normal(Ucmd &cmd);
	void verify_sync(Ucmd &cmd);
	Ucmd ucmds[100];
	uint16_t current_cmd_id;
	uint16_t total_cmd;
	bool running;

	void packup_1st_1p1();
	void packup_1st_2p1();
	void packup_1st_3p1();
	void packup_1st_1p2();
	void packup_1st_2p2();
	void packup_1st_3p2();
	void packup_1st_1p3();
	void packup_1st_2p3();
	void packup_1st_3p3();
	void packup_2nd_1p4();
	void packup_2nd_2p4();
	void packup_2nd_3p4();
	void packup_2nd_4p4();
	void packup_2nd_5p4();
	void packup_2nd_6p4();
};
