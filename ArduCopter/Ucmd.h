
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
};
