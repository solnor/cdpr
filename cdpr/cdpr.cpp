// cdpr.cpp : This file contains the 'main' function. Program execution begins and ends there.
//
#include <iostream>
#include "cdpr_params.h"
#include "algorithms.h"
#include "commands.h"
#include <Dense>
#include <cstdlib>
#include <string>
#include <chrono>
#include <Windows.h>
#include <vector>


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
	//motor_state
	std::vector<motor_state> motor_states;
	bool running = 1;
	while (running) {
		
	}
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
	int i = 0;
	while (i < 1000) {
		poll_keys();
		i++;
		Sleep(1);
	}
	system("pause");
}
