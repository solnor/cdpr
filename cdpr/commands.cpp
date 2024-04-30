#include "commands.h"
#include <string>
#include <iostream>
#include <sstream>
#include <iomanip>
//#include <vector>


int find_driver_errors(HANDLE handles, odrive_state *state) {
	/* UNFINISHED */
	bool errors_found = false;
	bool disarm_reasons_found = false;
	//bool active_errors = 

	char active_errors_command[]  = "r axis0.active_errors\n";
	char disarm_reason_command[] = "r axis0.disarm_reason\n";

	return 1;
}





std::string to_string(double x, uint8_t prec)
{
	std::ostringstream ss;
	ss << std::setprecision(prec) << x;
	return ss.str();
}

int set_axis_state(HANDLE handle, axis_states ax_state) {
	char c[100];
	uint8_t prec = 1;
	std::string cstr = "w axis0.requested_state " + to_string(ax_state, prec) + "\n";
	strncpy_s(c, sizeof(c), cstr.c_str(), sizeof(cstr));
	std::cout << c << std::endl;
	com_write_ln(handle, c);
	return 1;
}

int set_encoder_position(HANDLE handle, double position) {
	char c[100];
	uint8_t prec = 5;
	std::string cstr = "w axis0.pos_estimate " + to_string(position, prec) + "\n";
	strncpy_s(c, sizeof(c), cstr.c_str(), sizeof(cstr));
	std::cout << c << std::endl;
	com_write_ln(handle, c);
	return 1;
}

int get_motor_state(HANDLE handle, motor_state* state) {
	std::cout << handle << ", pos: " << state->pos << std::endl;
	std::cout << handle << ", vel: " << state->vel << std::endl;
	char c[] = "f 0\n";
	char r[25];
	com_write_ln(handle, c);
	com_read_ln(handle, r);
	std::string rstr = r;
	//std::string r = "1.234567 8.91234";
	int index_delim = rstr.find(" ");

	state->pos = std::stof(rstr.substr(0, index_delim));
	state->vel = std::stof(rstr.substr(index_delim + 1, rstr.length() - index_delim - 1));
	
	//std::cout << handle << ", pos: " << state->pos << std::endl;
	//std::cout << handle << ", vel: " << state->vel << std::endl;

	return 1;
}

int get_all_motor_states(HANDLE handles[], std::vector<motor_state*> motor_states) {
	int r = 0;
	for (uint8_t i = 0; i < 4; i++) {
		r = get_motor_state(handles[i], motor_states[i]);
		if (r = 0) {
			return 0;
		}
	}
	return 1;
}

int set_motor_torque(HANDLE handle, double torque) {
	//std::cout << std::to_string(1) << std::endl;
	constexpr uint8_t prec = 5;
	char c[100];
	std::string cstr = "c 0 " + to_string(torque, prec) + "\n";
	strncpy_s(c, sizeof(c), cstr.c_str(), sizeof(cstr));
	std::cout << handle << ": " << c << std::endl;
	com_write_ln(handle, c);
	return 1;
}