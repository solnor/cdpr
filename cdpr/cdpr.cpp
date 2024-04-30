// cdpr.cpp : This file contains the 'main' function. Program execution begins and ends there.
//
#include <iostream>
//#include "communication.h"
#include "cdpr_params.h"
#include "algorithms.h"
#include "commands.h"
#include "odrive_definitions.h"
#include <Dense>
#include <cstdlib>
#include <string>
#include <chrono>
//#include <Windows.h>
#include <vector>


//constexpr int n_odrv = 4;
HANDLE handles[n_odrv];
//LPCSTR odrv0_port = "\\\\.\\COM4";
//LPCSTR odrv1_port = "\\\\.\\COM5";
//LPCSTR odrv2_port = "\\\\.\\COM6";
//LPCSTR odrv3_port = "\\\\.\\COM7";
LPCSTR odrv_ports[n_odrv] = { (LPCSTR)"\\\\.\\COM4", (LPCSTR)"\\\\.\\COM5", (LPCSTR)"\\\\.\\COM6", (LPCSTR)"\\\\.\\COM7" };
bool running = 1;
int init_cdpr_params() {
	a << -0.7560, -0.7560, 0.7560, 0.7560,
	     -0.4181, 0.4181, 0.4181, -0.4181;
	// Trapezoidal b
	b << -0.0250, -0.0750, 0.0750,  0.0250,
	     -0.0100,  0.0100, 0.0100, -0.0100;

	length_drum_x = length_drum_x * convmm2m;
	return 0;
}

int poll_keys() {
	if (GetKeyState('A') & 0x8000)
	{
		std::cout << "Pressing A" << std::endl;
	}
	if (GetKeyState('Q') & 0x8000)
	{
		running = 0;
	}
	return 0;
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


int control_loop() {
	motor_state ms0;
	motor_state ms1;
	motor_state ms2;
	motor_state ms3;
	std::vector<motor_state*> motor_states;
	motor_states.push_back(&ms0);
	motor_states.push_back(&ms1);
	motor_states.push_back(&ms2);
	motor_states.push_back(&ms3);

	Eigen::Vector4d pos;
	Eigen::Vector4d l;
	Eigen::Vector4d l0(1.2260, 
		               1.1833,
		               1.1833, 
		               1.2260);
	Eigen::Vector4d lfk;
	
	inv_res invkin;

	Eigen::MatrixXd AT = Eigen::MatrixXd::Zero(3, 4);

	double f_min = 15;
	double f_max = 80;
	double f_ref = (f_max + f_min) / 2;
	Eigen::Vector4d f_prev = f_ref * Eigen::Vector4d::Ones();
	force_alloc_res fres;
	Eigen::Vector4d f0 = Eigen::Vector4d::Zero();

	Eigen::Vector3d q = Eigen::Vector3d::Zero();
	Eigen::Vector3d qd;
	Eigen::Vector3d e;
	Eigen::Vector3d wd;
	Eigen::Vector4d T = Eigen::Vector4d::Zero();

	Eigen::Matrix3d Kp = Eigen::Matrix3d::Zero();
	Kp(0, 0) = 250; // Proportional gain x
	Kp(1, 1) = 150; // Proportional gain y
	Kp(2, 2) = 5;   // Proportional gain theta

	Eigen::Matrix3d Ki = Eigen::Matrix3d::Zero();
	Ki(0, 0) = 0; // Integral gain x
	Ki(1, 1) = 0; // Integral gain y
	Ki(2, 2) = 0; // Integral gain theta
	Eigen::Vector3d eint = Eigen::Vector3d::Zero();

	double r_d = 20 * convmm2m;
	double pitch_drum = 2.9 * convmm2m;
	double ydiff = 371.4759 * convmm2m;
	double xdiff = 56.40783 * convmm2m;

	Eigen::Vector4d test = Eigen::Vector4d::Zero();
	com_init(handles, odrv_ports);
	for (uint8_t i = 0; i < 4; i++) {
		set_axis_state(handles[i], AXIS_STATE_CLOSED_LOOP_CONTROL);
		Sleep(10);
		test(i) = 0.3*(-1)*motor_signs(i);
		set_motor_torque(handles[i], test(i));
		
	}
	std::cout << "Set home position?" << std::endl;
	std::string input;
	std::cin >> input;
	if (is_number(input)) {
		std::cout << "Setting home position" << std::endl;
		for (uint8_t i = 0; i < 4; i++) {
			set_encoder_position(handles[i], 0.0);
			Sleep(10);
		}
		
	}
	get_all_motor_states(handles, motor_states);
	std::cout << "Motor positions: " << ms0.pos << ", " << ms1.pos << ", " << ms2.pos << ", " << ms3.pos << std::endl;
	std::cout << "Start control loop?" << std::endl;
	std::cin >> input;
	std::cout << "Running" << std::endl;
	auto start_loop = std::chrono::high_resolution_clock::now();
	while (running) {
		//std::cout << "Start loop" << std::endl;
		auto start = std::chrono::high_resolution_clock::now();
		//std::cout << "Getting motor states" << std::endl;
		get_all_motor_states(handles, motor_states);
		auto t_motor_states = std::chrono::high_resolution_clock::now();
		auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(t_motor_states - start);
		//std::cout << "get_all_motor_states duration: " << duration.count() << " [ms]" << std::endl;
		pos << ms0.pos,
			   ms1.pos,	
			   ms2.pos,
			   ms3.pos;
		//std::cout << "Calculating l" << std::endl;
		l << l0 + pos.cwiseProduct(r_p*motor_signs);
		lfk << l(0) - sqrt( sqrt( pow(pos(0)*pitch_drum, 2) + pow(ydiff,2) ) + pow(xdiff,2)), // Subtract length between
			   l(1) - sqrt( sqrt( pow(pos(1)*pitch_drum, 2) + pow(ydiff,2) ) + pow(xdiff,2)), // drum and pulley from
			   l(2) - sqrt( sqrt( pow(pos(2)*pitch_drum, 2) + pow(ydiff,2) ) + pow(xdiff,2)), // total cable length
			   l(3) - sqrt( sqrt( pow(pos(3)*pitch_drum, 2) + pow(ydiff,2) ) + pow(xdiff,2)); // to get length used in FK
		//std::cout << "Calculating forward kinematics" << std::endl;
		q = forward_kinematics(a, b, 
							   fk_init_estimate(a, b, l), 
							   lfk, r_p);
		auto t_fk = std::chrono::high_resolution_clock::now();
		duration = std::chrono::duration_cast<std::chrono::milliseconds>(t_fk - t_motor_states);
		//std::cout << "forward_kinematics duration: " << duration.count() << " [ms]" << std::endl;
		//std::cout << "Calculating inverse kinematics" << std::endl;
		invkin = inverse_kinematics(a, b, q, r_p);
		auto t_ik = std::chrono::high_resolution_clock::now();
		duration = std::chrono::duration_cast<std::chrono::milliseconds>(t_ik - t_fk);
		//std::cout << "inverse_kinematics duration: " << duration.count() << " [ms]" << std::endl;
		auto t_loop = std::chrono::high_resolution_clock::now();
		auto t_since_start = std::chrono::duration_cast<std::chrono::seconds>(t_loop - start_loop);
		qd << 0.15*cos(8*std::chrono::duration<double>(t_since_start).count()), 0, 0;
		e  << qd - q;
		//std::cout << "q: \n" << q << std::endl;
		//std::cout << "e: \n" << e << std::endl;
		wd << Kp * e + Ki * eint;
		//std::cout << "Calculating structure matrix" << std::endl;
		AT = calculate_structure_matrix(a, b, q, invkin.betar, r_p);
		auto t_sm = std::chrono::high_resolution_clock::now();
		duration = std::chrono::duration_cast<std::chrono::milliseconds>(t_sm - t_ik);
		//std::cout << "calculate_structure_matrix duration: " << duration.count() << " [ms]" << std::endl;
		//std::cout << "l: \n" << l << std::endl;
		//std::cout << "wd: \n" << wd << std::endl;
		//std::cout << "AT: \n" << AT << std::endl;
		fres = force_alloc_iterative_slack(AT.transpose(), f_min, f_max, f_ref, f_prev, wd);
		auto t_fa = std::chrono::high_resolution_clock::now();
		duration = std::chrono::duration_cast<std::chrono::milliseconds>(t_fa - t_sm);
		//std::cout << "force_alloc_iterative_slack duration: " << duration.count() << " [ms]" << std::endl;
		// TODO: Add f0

		T = (fres.f + f0).cwiseProduct(r_d*(-1)*motor_signs);
		//std::cout << "T: \n" << T << std::endl;
		// TODO: Write torques to motor drivers
		//std::cout << "Done" << std::endl;
		for (uint8_t i = 0; i < 4; i++) {
			//test(i) = T*(-1)*motor_signs(i);
			//std::cout << test << "\n" << std::endl;
			//set_axis_state(handles[i], AXIS_STATE_CLOSED_LOOP_CONTROL);
			set_motor_torque(handles[i], T(i));
			//Sleep(1);
		}
		auto t_st = std::chrono::high_resolution_clock::now();
		duration = std::chrono::duration_cast<std::chrono::milliseconds>(t_st - t_fa);
		//std::cout << "set_motor_torque duration: " << duration.count() << " [ms]" << std::endl;
		//std::cout << "f: \n" << fres.f << std::endl;
		auto stop = std::chrono::high_resolution_clock::now();
		duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
		std::cout << "Loop duration: " << duration.count() << " [ms]" << std::endl;
		poll_keys();
	}
	return 1;
}

int main()
{
	
	int ret = init_cdpr_params();
	
	auto start = std::chrono::high_resolution_clock::now();
	inv_res inv_kin = inverse_kinematics(a, b, q, r_p);
	std::cout << inv_kin.l << std::endl;
	Eigen::Vector3d q0 = fk_init_estimate(a, b, inv_kin.l);
	Eigen::Vector3d qe = forward_kinematics(a, b, q0, inv_kin.l, r_p);
	std::cout << "qe" << std::endl;
	std::cout << qe*pow(10,3) << std::endl;
	Eigen::MatrixXd AT = calculate_structure_matrix(a, b, qe, inv_kin.betar, r_p);
	std::cout << AT << std::endl;

	double f_min = 5;
	double f_max = 60;
	double f_ref = 25;
	Eigen::Vector4d f_prev = f_ref*Eigen::Vector4d::Ones();
	Eigen::Vector3d w_c(0, 0, 0);
	force_alloc_res fares;
	fares = force_alloc_iterative_slack(AT.transpose(), f_min, f_max, f_ref, f_prev, w_c);
	std::cout << "force allocation: \n" << fares.f << std::endl;
	auto stop = std::chrono::high_resolution_clock::now();
	auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
	std::cout << "Duration: " << duration.count() << " [ms]" << std::endl;

	std::string input;
	std::cin >> input;
	if (is_number(input)) {
		std::cout << "Oke" << std::endl;
	}

	control_loop();

	int i = 0;
	while (i < 1000) {
		poll_keys();
		i++;
		Sleep(1);
	}
	system("pause");
}
