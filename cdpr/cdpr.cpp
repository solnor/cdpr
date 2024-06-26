// cdpr.cpp : This file contains the 'main' function. Program execution begins and ends there.
//
#include <iostream>
#include "cdpr_params.h"
#include "algorithms.h"
#include "commands.h"
#include "odrive_definitions.h"
#include "controllers.h"
#include <Dense>
#include <QR>
#include <cstdlib>
#include <string>
#include <chrono>
#include <vector>
#include <fstream>
#include <iterator>


HANDLE handles[n_odrv];
LPCSTR odrv_ports[n_odrv] = { (LPCSTR)"\\\\.\\COM4", (LPCSTR)"\\\\.\\COM5", (LPCSTR)"\\\\.\\COM6", (LPCSTR)"\\\\.\\COM7" };

bool running = 1;
int init_cdpr_params() {
	a << -0.7560, -0.7560, 0.7560, 0.7560,
	     -0.4181, 0.4181, 0.4181, -0.4181;
	a << -0.7490, -0.7490, 0.7530, 0.7530,
		 -0.4041, 0.4321, 0.4321, -0.4041;
	a << -0.7510, -0.7510, 0.7510, 0.7510,
		-0.4181, 0.4181, 0.4181, -0.4181;
	// Trapezoidal b
	b << -0.0250, -0.0750, 0.0750,  0.0250,
	     -0.0100,  0.0100, 0.0100, -0.0100;

	length_drum_x = length_drum_x * convmm2m;
	return 0;
}

int set_standard_tension(HANDLE handles[]) {
	set_all_axis_states(handles, AXIS_STATE_CLOSED_LOOP_CONTROL);
	Sleep(10);
	Eigen::Vector4d torques = -0.15*motor_signs;
	set_all_motor_torques(handles, torques);
	return 1;
}

// https://stackoverflow.com/questions/4654636/how-to-determine-if-a-string-is-a-number-with-c
bool is_number(const std::string& s)
{
	std::string::const_iterator it = s.begin();
	while (it != s.end() && isdigit(*it)) ++it;
	return !s.empty() && it == s.end();
}

// Control loop
// Get pos, vel
// Process pos to get l
// Find l_fk
// Get q0
// Get q from forward kinematics
// Get betar from inverse kinematics
// Calculate structure matrix
// Set qd
// Calculate error
// Get forces from force allocation
// Add on f0 to the forces
// Turn forces into motor torques
// Set motor torques



int testt() {
	auto start = std::chrono::high_resolution_clock::now();
	double T = 0;
	while (1) {
		auto t_now = std::chrono::high_resolution_clock::now();
		auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(t_now - start);
		T = std::chrono::duration<double>(duration).count();
		//std::cout << "T: " << T << std::endl;
		if (T >= 0.015) break;

	}
	auto t_now = std::chrono::high_resolution_clock::now();
	auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(t_now - start);
	std::cout << "T: " << T << std::endl;
	std::cout << duration.count() << std::endl;
	std::cout << "Testt running" << std::endl;
	return 0;
}

int main()
{
	
	int ret = init_cdpr_params();
	
	auto start = std::chrono::high_resolution_clock::now();
	q << 0.0, -0.2, 0.0;
	inv_res inv_kin = inverse_kinematics(a, b, q, r_p);
	std::cout << "l:\n" << inv_kin.l << std::endl;
	std::cout << "betar:\n" << inv_kin.betar << std::endl;
	Eigen::Vector4d betae(0.188115601037228, 0.186058206914978, 0.187081318922497, 0.187081318922497);
	//inv_kin.l << inv_kin.l + (inv_kin.betar - 0.15194*Eigen::Vector4d::Ones())*r_p;
	inv_kin.l << inv_kin.l + (inv_kin.betar - betae)*r_p;
	std::cout << "l:\n" << inv_kin.l << std::endl;
	Eigen::Vector3d q0 = fk_init_estimate(a, b, inv_kin.l);
	std::cout << "q0:\n" << q0 << std::endl;
	Eigen::Vector3d qe = forward_kinematics(a, b, q0, inv_kin.l, r_p);
	std::cout << "qe" << std::endl;
	std::cout << qe << std::endl;
	Eigen::MatrixXd AT = calculate_structure_matrix(a, b, qe, inv_kin.betar, r_p);
	std::cout << AT << std::endl;

	double f_min = 10;
	double f_max = 80;
	double f_ref = (f_max + f_min) / 2;;
	Eigen::Vector4d f_prev = f_ref*Eigen::Vector4d::Ones();
	Eigen::Vector3d w_c(0, 0, 0);
	force_alloc_res fares;
	fares = force_alloc_iterative_slack(AT.transpose(), f_min, f_max, f_ref, f_prev, w_c);
	std::cout << "force allocation: \n" << fares.f << std::endl;
	std::cout << "force allocation flag: \n" << fares.flag << std::endl;


	auto stop = std::chrono::high_resolution_clock::now();
	auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
	std::cout << "Duration: " << duration.count() << " [ms]" << std::endl;

	Eigen::Vector4d f_static(0.0840 * 1 / r_d,
		0.0820 * 1 / r_d,
		0.1040 * 1 / r_d,
		0.0780 * 1 / r_d);

	Eigen::Vector4d f_pinv(0,0.1,-1, -3);
	Eigen::Vector3d er(0.1,0.1,-1);
	Eigen::Vector4d vels(1,-3.1,-0.01, 3.001);
	double precv = 2;
	double precx = 3;
	double precy = 3;
	double prect = 0;
	//std::cout << "fs:\n"<< calculate_fs(vels, er, f_static, precv, precx, precy, prect) << std::endl;
	//std::cout << map(11, 0, 10, 0, 1) << std::endl;
	bool move_on = 0;
	com_init(handles, odrv_ports);

	Eigen::Vector4d p0 = Eigen::Vector4d::Zero();
	motor_state ms0;
	motor_state ms1;
	motor_state ms2;
	motor_state ms3;
	std::vector<motor_state*> motor_states;
	motor_states.push_back(&ms0);
	motor_states.push_back(&ms1);
	motor_states.push_back(&ms2);
	motor_states.push_back(&ms3);

	double encpos[] = { 0,0,0,0 };
	std::cout << std::endl;
	while (!move_on) {
		std::cout << "Select procedure (1-8):" << std::endl;
		std::cout << "1) Move platform with arrow keys" << std::endl;
		std::cout << "2) Set tension" << std::endl;
		std::cout << "3) Set home position" << std::endl;
		std::cout << "4) Clear errors" << std::endl;
		std::cout << "5) Disable anticogging" << std::endl;
		std::cout << "6) Enable enable_dc_bus_voltage_feedback on all ODrives" << std::endl;
		std::cout << "7) Print carth pos" << std::endl;
		std::cout << "8) Run control loop" << std::endl;
		std::cout << "9) Exit" << std::endl;
		std::cout << "10) Zero torque" << std::endl;

		std::string input;
		std::cin >> input;

		// TODO:
		// Make function for moving platform with arrow keys
		// Verify function for setting enable_dc_bus_voltage_feedback
		if (is_number(input)) {
			int inputi = stoi(input);
			switch (inputi) {
			case 1:
				break;
			case 2:
				//com_init(handles, odrv_ports);
				set_standard_tension(handles);
				break;
			case 3:
				//com_init(handles, odrv_ports);

				double positions[4];
				//set_all_encoder_positions(handles, positions);
				
				get_all_motor_states(handles, motor_states);
				p0 << ms0.pos,
					  ms1.pos,
					  ms2.pos,
					  ms3.pos;
				std::cout << "p0: \n" << p0 << std::endl;

				break;
			case 4:
				//com_init(handles, odrv_ports);
				clear_errors(handles);
				break;
			case 5:
				//com_init(handles, odrv_ports);
				
				set_all_encoder_positions(handles, encpos);
				break;
			case 6:
				//com_init(handles, odrv_ports);
				enable_all_brake_resistor_voltage_feedback(handles);
				std::cout << "Brake resistor voltage feedback enabled" << std::endl;
				break;
			case 7:
				//testt();
				pos_test(handles);
				break;
			case 8:
				tension_control_loop(handles, p0);
				break;
			case 9:
				move_on = 1;
				break;
			case 10:
				set_all_motor_torques(handles, Eigen::Vector4d::Zero());
				set_all_axis_states(handles, AXIS_STATE_IDLE);
				break;
			default:
				break;
			}
		}

	}
	system("pause");
}
