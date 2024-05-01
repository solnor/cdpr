#include "commands.h"
#include <string>
#include <iostream>
#include <sstream>
#include <iomanip>
//#include <vector>
#include <chrono>


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
	//std::cout << c << std::endl;
	com_write_ln(handle, c);
	return 1;
}

int set_encoder_position(HANDLE handle, double position) {
	char c[100];
	uint8_t prec = 5;
	std::string cstr = "w axis0.pos_estimate " + to_string(position, prec) + "\n";
	strncpy_s(c, sizeof(c), cstr.c_str(), sizeof(cstr));
	//std::cout << c << std::endl;
	com_write_ln(handle, c);
	return 1;
}

int get_motor_state(HANDLE handle, motor_state* state) {
	//std::cout << handle << ", pos: " << state->pos << std::endl;
	//std::cout << handle << ", vel: " << state->vel << std::endl;
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
	uint8_t prec = 5;
	char c[100];
	std::string cstr = "c 0 " + to_string(torque, prec) + "\n";
	strncpy_s(c, sizeof(c), cstr.c_str(), sizeof(cstr));
	com_write_ln(handle, c);
	return 1;
}


int set_all_motor_torques(HANDLE handles[], const Eigen::Ref<const Eigen::Vector4d>& torques) {
	int r = 0;
	for (uint8_t i = 0; i < 4; i++) {
		r = set_motor_torque(handles[i], torques(i));
		if (r = 0) {
			return 0;
		}
	}
	return 1;
}

// https://stackoverflow.com/questions/1070497/c-convert-hex-string-to-signed-integer
int conv_hex_ss_to_int(std::stringstream& ss) {
	int x;
	ss >> x;
	return static_cast<int>(x);
}

int read_driver_error_status(HANDLE handle) {
	bool errors_found = 0;
	bool disarm_reasons_found = 0;

	char c[] = "r axis0.active_errors \n";
	char r[25];
	com_write_ln(handle, c);
	com_read_ln(handle, r);
	//char r[] = "0x00000002"; // Testing

	std::stringstream ss;
	ss << std::hex << r;
	int error = conv_hex_ss_to_int(ss);

	odrive_error oe;
	oe = (odrive_error)error;

	switch (oe) {
	case ODRIVE_ERROR_NONE:
		std::cout << "No errors" << std::endl;
		return 0;
		break;
	case ODRIVE_ERROR_CONTROL_ITERATION_MISSED:
		std::cout << "Odrive error: ODRIVE_ERROR_CONTROL_ITERATION_MISSED" << std::endl;
		return 1;
		break;
	case ODRIVE_ERROR_DC_BUS_UNDER_VOLTAGE:
		std::cout << "Odrive error: ODRIVE_ERROR_DC_BUS_UNDER_VOLTAGE" << std::endl;
		return 2;
		break;
	case ODRIVE_ERROR_DC_BUS_OVER_VOLTAGE:
		std::cout << "Odrive error: ODRIVE_ERROR_DC_BUS_OVER_VOLTAGE" << std::endl;
		return 3;
		break;
	case ODRIVE_ERROR_DC_BUS_OVER_REGEN_CURRENT:
		std::cout << "Odrive error: ODRIVE_ERROR_DC_BUS_OVER_REGEN_CURRENT" << std::endl;
		return 4;
		break;
	case ODRIVE_ERROR_DC_BUS_OVER_CURRENT:
		std::cout << "Odrive error: ODRIVE_ERROR_DC_BUS_OVER_CURRENT" << std::endl;
		return 5;
		break;
	case ODRIVE_ERROR_BRAKE_DEADTIME_VIOLATION:
		std::cout << "Odrive error: ODRIVE_ERROR_BRAKE_DEADTIME_VIOLATION" << std::endl;
		return 6;
		break;
	case ODRIVE_ERROR_BRAKE_DUTY_CYCLE_NAN:
		std::cout << "Odrive error: ODRIVE_ERROR_BRAKE_DUTY_CYCLE_NAN" << std::endl;
		return 7;
		break;
	case ODRIVE_ERROR_INVALID_BRAKE_RESISTANCE:
		std::cout << "Odrive error: ODRIVE_ERROR_INVALID_BRAKE_RESISTANCE" << std::endl;
		return 8;
		break;
	default:
		std::cout << "Unknown error" << std::endl;
		return 9;
		break;
	}

	return 0;
}

int read_all_driver_error_statuses(HANDLE handles[], int *errors) {
	for (uint8_t i = 0; i < 4; i++) {
		errors[i] = read_driver_error_status(handles[i]);
	}
	return 0;
}

int check_if_driver_error(HANDLE handle) {
	bool errors_found = 0;
	bool disarm_reasons_found = 0;

	char c[] = "r axis0.active_errors \n";
	char r[25];
	com_write_ln(handle, c);
	com_read_ln(handle, r);

	std::stringstream ss;
	ss << std::hex << r;
	int error = conv_hex_ss_to_int(ss);

	if (((int)0x11111111 & error)) {
		return 1;
	}

	return 0;
}

int check_if_any_driver_errors(HANDLE handles[]) {
	int error = 0;
	for (uint8_t i = 0; i < 4; i++) {
		error = check_if_driver_error(handles[i]);
		if (error) break;
	}
	return error;
}