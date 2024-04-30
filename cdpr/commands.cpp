#include "commands.h"
#include <string>
#include <iostream>
#include <sstream>
#include <iomanip>
#include <vector>


int find_driver_errors(HANDLE handles, odrive_state *state) {
	/* UNFINISHED */
	bool errors_found = false;
	bool disarm_reasons_found = false;
	//bool active_errors = 

	char active_errors_command[]  = "r axis0.active_errors\n";
	char disarm_reason_command[] = "r axis0.disarm_reason\n";

	return 1;
}



int set_axis_state(HANDLE handle, axis_states ax_state) {
	return 1;
}

std::string to_string(double x, uint8_t prec)
{
	std::ostringstream ss;
	ss << std::setprecision(prec) << x;
	return ss.str();
}

int get_motor_state(HANDLE handle, motor_state* state) {
	state->pos  = 0.0;
	state->vel = 0.0;
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
	std::cout << std::to_string(1) << std::endl;
	//char t[] = "c";
	char tor[4];
	//std::strncpy(tor, std::to_string(torque).c_str(),4);
	constexpr uint8_t prec = 5;
	char c[prec];
	std::string cstr = "c 0 " + to_string(torque, prec);
	strncpy_s(c, sizeof(cstr), to_string(torque, prec).c_str(), sizeof(cstr));

	//char c[] = std::strcat((char*)"c", *);
	return 1;
}