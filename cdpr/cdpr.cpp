// cdpr.cpp : This file contains the 'main' function. Program execution begins and ends there.
//
#include <iostream>
//#include "communication.h"
#include "cdpr_params.h"
#include "algorithms.h"
#include "commands.h"
#include "odrive_definitions.h"
#include <Dense>
#include <QR>
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

int set_standard_tension(HANDLE handles[]) {
	set_all_axis_states(handles, AXIS_STATE_CLOSED_LOOP_CONTROL);
	Sleep(10);
	Eigen::Vector4d torques = -0.2*motor_signs;
	set_all_motor_torques(handles, torques);
	return 1;
}

int poll_keys() {
	//auto t_now = std::chrono::high_resolution_clock::now();
	if (GetAsyncKeyState('A') & 0x8000) {
		std::cout << "Pressing A" << std::endl;
		return 1;
	}
	if (GetAsyncKeyState('Q') & 0x8000) {
		running = 0;
		return 2;
	}
	if (GetAsyncKeyState('Y') & 0x8000) {
		/*static auto t_press = std::chrono::high_resolution_clock::now();
		auto t_diff = std::chrono::duration_cast<std::chrono::seconds>(t_press - t_now);
		auto t_diff_d = std::chrono::duration<double>(t_diff).count();*/
		//if (t_diff_d > 1.0) {
		return 3;
		//}
	}
	if (GetAsyncKeyState('N') & 0x8000) {
		return 4;
	}
	if (GetAsyncKeyState(0x25) & 0x8000) { // Left arrow key
		return 5;
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

double ceil_abs_w_sign(double d) {
	return d > 0 ? ceil(d) :
		           floor(d);
}
// https://stackoverflow.com/questions/1903954/is-there-a-standard-sign-function-signum-sgn-in-c-c
template <typename T> int sgn(T val) {
	return (T(0) < val) - (val < T(0));
}

double map(double x, double in_min, double in_max, double out_min, double out_max) {
	if (x < in_min) {
		return out_min;
	} else if (x > in_max) {
		return out_max;
	} else {
		return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
	}
}

Eigen::Vector4d calculate_f_loss_dir(const Eigen::Ref<const Eigen::Vector4d>& vels,
									 double precv,
									 double precx,
									 double precy,
									 double prect) {
	Eigen::Vector4d velp;
	double veln = vels.norm();

	/*velp << std::trunc(vels(0)*pow(10, precv)) / pow(10, precv),
			std::trunc(vels(1)*pow(10, precv)) / pow(10, precv),
			std::trunc(vels(2)*pow(10, precv)) / pow(10, precv),
			std::trunc(vels(3)*pow(10, precv)) / pow(10, precv);*/

	/*velp << (bool)(ceil(abs(velp(0)))),
			(bool)(ceil(abs(velp(1)))),
			(bool)(ceil(abs(velp(2)))),
			(bool)(ceil(abs(velp(3))));*/
	double in_min = 0.5;
	double in_max = 1.5;
	velp << map(abs(vels(0)), in_min, in_max, 0, 1),
			map(abs(vels(1)), in_min, in_max, 0, 1),
			map(abs(vels(2)), in_min, in_max, 0, 1),
			map(abs(vels(3)), in_min, in_max, 0, 1);
	double mapped_vel_norm = map(veln, in_min, in_max, 0, 1);
	velp << sgn(vels(0))*mapped_vel_norm,
			sgn(vels(1))*mapped_vel_norm,
			sgn(vels(2))*mapped_vel_norm,
			sgn(vels(3))*mapped_vel_norm;
	std::cout << "veln: " << veln << std::endl;
	std::cout << "velp:\n" << velp << std::endl;

	/*velp << sgn(vels(0))*velp(0),
			sgn(vels(1))*velp(1),
			sgn(vels(2))*velp(2),
			sgn(vels(3))*velp(3);*/
	return velp;
}

Eigen::Vector4d calculate_fs(const Eigen::Ref<const Eigen::Vector4d>& vels,
							 const Eigen::Ref<const Eigen::Vector3d>& e,
							 const Eigen::Ref<const Eigen::Vector4d>& f_static,
							 double precv,
	                         double precx,
	                         double precy,
	                         double prect) {
	Eigen::Vector4d velp;
	
	velp << std::trunc(vels(0)*pow(10, precv)) / pow(10, precv),
			std::trunc(vels(1)*pow(10, precv)) / pow(10, precv),
			std::trunc(vels(2)*pow(10, precv)) / pow(10, precv),
			std::trunc(vels(3)*pow(10, precv)) / pow(10, precv);
	//std::cout << "4&1: " << (1 & (bool)4) << std::endl;
	//std::cout << (bool)(ceil(abs(velp(1)))) << std::endl;
	//std::cout << (1 & (bool)(ceil(abs(velp(1))))) << std::endl;
	//std::cout << !(1 & (bool)(ceil(abs(velp(1))))) << std::endl;
	//std::cout << "velp trunc: \n" << velp << std::endl;
	velp << !(1 & (bool)(ceil(abs(velp(0))))),
			!(1 & (bool)(ceil(abs(velp(1))))),
			!(1 & (bool)(ceil(abs(velp(2))))),
			!(1 & (bool)(ceil(abs(velp(3)))));
	//std::cout << "velp bool: \n" << velp << std::endl;
	
	Eigen::Vector3d ep;
	ep << std::trunc(e(0)*pow(10, precx)) / pow(10, precx),
		  std::trunc(e(1)*pow(10, precy)) / pow(10, precy),
		  std::trunc(e(2)*pow(10, prect)) / pow(10, prect);
	//std::cout << "ep: \n" << ep << std::endl;

	/*Eigen::Vector4d dir(sgn(f_pinv(0)), 
						sgn(f_pinv(1)), 
						sgn(f_pinv(2)), 
						sgn(f_pinv(3)));*/
	Eigen::Vector4d fs;
	//fs << (int)f_pinvp.any()*velp.cwiseProduct(dir.cwiseProduct(f_static));
	fs << (int)ep.any()*velp.cwiseProduct(f_static);
	return fs;
	//Eigen::Vector4d smt;
	///*smt << ((int)(ceil_abs_w_sign(vel_trunc(0)))),
	//	((int)(ceil_abs_w_sign(vel_trunc(1)))),
	//	((int)(ceil_abs_w_sign(vel_trunc(2)))),
	//	((int)(ceil_abs_w_sign(vel_trunc(3))));*/
	//std::cout << "ceiled: \n" << smt << "\n" << std::endl;
	//std::cout << "smt: \n" << smt << "\n" << std::endl;

	////Eigen::Vector4d f_pinv(0, 0.1, 0.03, 3);
	////Eigen::Vector4d f_pinv(0,0,0,0.0001);
	////Eigen::Vector4d t4 = ;
	//std::cout << "any: " << f_pinv.any() << "\n" << std::endl;

	//
	////std::cout << "f_pinv_trunc: \n" << f_pinv_trunc << "\n" << std::endl;
	////std::cout << "any f_pinv_trunc: \n" << f_pinv_trunc.any() << "\n" << std::endl;
	//
	////t6 << t6 + Eigen::Vector4d::Ones();
	//std::cout << "t6: \n" << t6 << std::endl;
	///*smt << !(1&(int)(ceil(vel_trunc(0)))),
	//	   !(1&(int)(ceil(vel_trunc(1)))),
	//	   !(1&(int)(ceil(vel_trunc(2)))),
	//	   !(1&(int)(ceil(vel_trunc(3))));
	//std::cout << smt << "\n" << std::endl;*/
	//std::cout << smt.cwiseProduct(t3) << "\n" << std::endl;
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
	Eigen::Vector4d pos_rad;
	Eigen::Vector4d vel;
	Eigen::Vector4d fvel = Eigen::Vector4d::Zero();
	Eigen::Vector4d vel_m;
	Eigen::Vector4d l;
	Eigen::Vector4d l0(1.2260, 
		               1.1833,
		               1.1833, 
		               1.2260);
	Eigen::Vector4d lfk;
	
	inv_res invkin;

	Eigen::MatrixXd AT      = Eigen::MatrixXd::Zero(3, 4);
	Eigen::MatrixXd AT_pinv = Eigen::MatrixXd::Zero(3, 4);

	double f_min = 15;
	double f_max = 80;
	double f_ref = (f_max + f_min) / 2;
	Eigen::Vector4d f_prev = f_ref * Eigen::Vector4d::Ones();
	force_alloc_res fres;

	Eigen::Vector4d f_static(0.0840 * 1 / r_d,
							 0.0820 * 1 / r_d,
							 0.1040 * 1 / r_d,
							 0.0780 * 1 / r_d);
	Eigen::Vector4d f_loss(2.195,
						   1.995,
						   2.245,
						   1.845);
	//Eigen::Vector4d f_loss(6.195,
	//					   2.295,
	//					   2.245,
	//					   6.245);
	//Eigen::Vector4d f_loss(2.195,
	//					   -1,
	//					   -1,
	//					   2.245);
	/*Eigen::Vector4d f_loss(1.,
						   1.,
						   1.,
						   1.);*/
	Eigen::Vector4d f_pinv   = Eigen::Vector4d::Zero();
	Eigen::Vector3d e_t      = Eigen::Vector3d::Zero();
	Eigen::Vector4d fs       = Eigen::Vector4d::Zero();
	Eigen::Vector4d f0       = Eigen::Vector4d::Zero();
	double precv = 2;
	double precx = 3;
	double precy = 3;
	double prect = 0;

	Eigen::Vector3d q = Eigen::Vector3d::Zero();
	Eigen::Vector3d qd;
	Eigen::Vector3d e;
	Eigen::Vector3d wd;
	Eigen::Vector3d we;
	Eigen::Vector4d T = Eigen::Vector4d::Zero();

	Eigen::Matrix3d Kp = Eigen::Matrix3d::Zero();
	Kp(0, 0) = 200; // Proportional gain x
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

	bool any_error = 0;

	Eigen::Vector4d test = Eigen::Vector4d::Zero();

	std::cout << "Set starting torques? (y/n)" << std::endl;
	int r = 0;
	while (!(r == 3))
	{
		r = poll_keys();
		if (r == 4) return 0;
		//std::cout << r << std::endl;
	}
	Sleep(250);
	
	std::cout << "Setting starting torques" << std::endl;
	for (uint8_t i = 0; i < 4; i++) {
		//set_axis_state(handles[i], AXIS_STATE_CLOSED_LOOP_CONTROL); // TODO: Change to set_all_axis_states()
		Sleep(10);
		test(i) = 0.2*(-1)*motor_signs(i);
		//set_motor_torque(handles[i], test(i)); // TODO: Change to set_all_motor_torques()
		
	}
	std::string input;
	//std::cin >> input;
	std::cout << "Set home position?" << std::endl;
	r = 0;
	while (!(r == 3))
	{
		r = poll_keys();
		if (r == 4) return 0;
		//std::cout << r << std::endl;
	}
	Sleep(250);
	//std::cin >> input; // TODO: Change to key presses
	// while (keypress != y or n)
	//		keypress = poll keys
	//		if keypress == y or n
	//			continue
	//		else
	//          return
	//if (is_number(input)) {
	std::cout << "Setting home position" << std::endl;
	for (uint8_t i = 0; i < 4; i++) {
		set_encoder_position(handles[i], 0.0);
		Sleep(10);
	}
	std::cout << "Set home position?" << std::endl;
	//std::cin >> input;
	r = 0;
	while (!(r == 3))
	{
		r = poll_keys();
		if (r == 4) return 0;
	}
	Sleep(250);
	std::cout << "Setting home position" << std::endl;
	for (uint8_t i = 0; i < 4; i++) {
		set_encoder_position(handles[i], 0.0);
		Sleep(10);
	}
		
	//}
	get_all_motor_states(handles, motor_states);
	std::cout << "Motor positions: " << ms0.pos << ", " << ms1.pos << ", " << ms2.pos << ", " << ms3.pos << std::endl;
	std::cout << "Start control loop?" << std::endl;
	r = 0;
	while (!(r == 3))
	{
		r = poll_keys();
		if (r == 4) return 0;
	}
	Sleep(250);
	std::cout << "Running" << std::endl;
	auto start_loop = std::chrono::high_resolution_clock::now();
	while (running) {

		any_error = check_if_any_driver_errors(handles);
		if (any_error) { 
			running = 0;
			std::cout << "Breaking" << std::endl;
			break;
		};

		auto start = std::chrono::high_resolution_clock::now();
		get_all_motor_states(handles, motor_states);
		pos << ms0.pos,
			   ms1.pos,	
			   ms2.pos,
			   ms3.pos;
		pos_rad << pos * 2*PI;

		vel << ms0.vel,
			   ms1.vel,	
			   ms2.vel,
			   ms3.vel;

		vel_m << vel.cwiseProduct(motor_signs);
		l << l0 + pos_rad.cwiseProduct(r_d*motor_signs);
		//std::cout << "pos: \n" << pos << std::endl;
		//std::cout << "pos_rad: \n" << pos_rad << std::endl;

		lfk << l(0) - sqrt( pow(sqrt( pow(pos_rad(0)*pitch_drum, 2) + pow(ydiff,2) ), 2) + pow(xdiff,2)), // Subtract length between
			   l(1) - sqrt( pow(sqrt( pow(pos_rad(1)*pitch_drum, 2) + pow(ydiff,2) ), 2) + pow(xdiff,2)), // drum and pulley from
			   l(2) - sqrt( pow(sqrt( pow(pos_rad(2)*pitch_drum, 2) + pow(ydiff,2) ), 2) + pow(xdiff,2)), // total cable length
			   l(3) - sqrt( pow(sqrt( pow(pos_rad(3)*pitch_drum, 2) + pow(ydiff,2) ), 2) + pow(xdiff,2)); // to get length used in FK

		q = forward_kinematics(a, b, 
							   fk_init_estimate(a, b, lfk), 
							   lfk, r_p);
		invkin = inverse_kinematics(a, b, q, r_p);
		
		auto t_loop = std::chrono::high_resolution_clock::now();
		auto t_since_start = std::chrono::duration_cast<std::chrono::seconds>(t_loop - start_loop);
		qd << 0.1*cos(8*std::chrono::duration<double>(t_since_start).count()), 0, 0;
		//qd << 0.1, 0, 0;
		e  << qd - q;
		e << 0,0,0;
		wd << Kp * e + Ki * eint;
		AT_pinv = AT.completeOrthogonalDecomposition().pseudoInverse();
		//f_pinv = AT_pinv * we;
		/*f0 << sgn(f_pinv(0))*f_loss(0),
			  sgn(f_pinv(1))*f_loss(1),
			  sgn(f_pinv(2))*f_loss(2),
			  sgn(f_pinv(3))*f_loss(3);*/
		fvel << 0.9*fvel + 0.1*vel;
		Eigen::Vector4d flossdir = calculate_f_loss_dir(fvel, precv, precx, precy, prect);
		f0 << flossdir.cwiseProduct((-1)*motor_signs);
		f0 << f0.cwiseProduct(f_loss);
		we << AT * f0;
		wd << 0, 0, 0;
		wd << wd + we;
		//wd << wd + AT * f_loss;

		AT = calculate_structure_matrix(a, b, q, invkin.betar, r_p);
		
		//std::cout << "we: \n" << wd << std::endl;
		std::cout << "q: \n" << q << std::endl;
		//std::cout << "vel: \n" << vel << std::endl;
		//std::cout << "vel_m: \n" << vel_m << std::endl;
		//std::cout << "fvel: \n" << fvel << std::endl;
		std::cout << "f0: \n" << f0 << std::endl;

		/*std::cout << "l: \n" << l << std::endl;
		std::cout << "lfk: \n" << lfk << std::endl;*/
		//std::cout << "wd: \n" << wd << std::endl;
		//std::cout << "AT: \n" << AT << std::endl;
		
		//f_ref = (f_min + f_max) / 2;
		fres.f = f_ref*Eigen::Vector4d::Ones() - AT_pinv*(wd + AT * f_ref*Eigen::Vector4d::Ones());
		//fres = force_alloc_iterative_slack(AT.transpose(), f_min, f_max, f_ref, f_prev, wd);

		
		fs = calculate_fs(vel_m, e, f_static, precv, precx, precy, prect);
		fs << 0, 0, 0, 0;
		/*e_t << std::trunc(f_pinv(0)*pow(10, precf)) / pow(10, precf),
			   std::trunc(f_pinv(1)*pow(10, precf)) / pow(10, precf),
			   std::trunc(f_pinv(2)*pow(10, precf)) / pow(10, precf),
			   std::trunc(f_pinv(3)*pow(10, precf)) / pow(10, precf);*/
		/*f0 << sgn(f_pinv(0))*(fs(0) + f_loss(0)),
			  sgn(f_pinv(1))*(fs(1) + f_loss(1)),
			  sgn(f_pinv(2))*(fs(2) + f_loss(2)),
			  sgn(f_pinv(3))*(fs(3) + f_loss(3));
		f0 << 0, 0, 0, 0;*/
		T = (fres.f).cwiseProduct(r_d*(-1)*motor_signs);

		//std::cout << "f_pinv: \n" << f_pinv << std::endl;
		
		//std::cout << "f: \n" << fres.f << std::endl;


		//std::cout << "f_pinv_t: \n" << f_pinv_t << std::endl;
		//std::cout << "vel_m: \n" << vel_m << std::endl;
		//std::cout << "f0: \n" << f0 << std::endl;
		//std::cout << "A^T(f+f0): \n" << AT * (fres.f+f0) << std::endl;
		//std::cout << "fs: \n" << fs << std::endl;
		//std::cout << "T: \n" << T << std::endl;
		
		/*for (uint8_t i = 0; i < 4; i++) {
			set_motor_torque(handles[i], T(i));
		}*/

		set_all_motor_torques(handles, T);

		//std::cout << "f: \n" << fres.f << std::endl;
		
		poll_keys();
	}
	if (any_error) {
		set_all_motor_torques(handles, Eigen::Vector4d::Zero());
		std::cout << "Error encountered" << std::endl;
		int errors[4];
		read_all_driver_error_statuses(handles, errors);
		std::cout << "Setting all torques to zero" << std::endl;
	} else {
		std::cout << "Setting standard torque" << std::endl;
		set_all_motor_torques(handles, (0.3*Eigen::Vector4d::Ones()).cwiseProduct((-1)*motor_signs));
	}
	return 1;
}

int testt() {
	std::cout << "Testt running" << std::endl;
	return 0;
}

int main()
{
	
	int ret = init_cdpr_params();
	
	auto start = std::chrono::high_resolution_clock::now();
	q << 0, -0.3, 0;
	inv_res inv_kin = inverse_kinematics(a, b, q, r_p);
	std::cout << "l:\n" << inv_kin.l << std::endl;
	std::cout << "betar:\n" << inv_kin.betar << std::endl;
	inv_kin.l << inv_kin.l + (inv_kin.betar - 0.15194*Eigen::Vector4d::Ones())*r_p;
	std::cout << "l:\n" << inv_kin.l << std::endl;
	Eigen::Vector3d q0 = fk_init_estimate(a, b, inv_kin.l);
	std::cout << "q0:\n" << q0 << std::endl;
	Eigen::Vector3d qe = forward_kinematics(a, b, q0, inv_kin.l, r_p);
	std::cout << "qe" << std::endl;
	std::cout << qe*pow(10,3) << std::endl;
	Eigen::MatrixXd AT = calculate_structure_matrix(a, b, qe, inv_kin.betar, r_p);
	std::cout << AT << std::endl;

	double f_min = 15;
	double f_max = 80;
	double f_ref = (f_max + f_min) / 2;;
	Eigen::Vector4d f_prev = f_ref*Eigen::Vector4d::Ones();
	Eigen::Vector3d w_c(0, 0, 0);
	force_alloc_res fares;
	fares = force_alloc_iterative_slack(AT.transpose(), f_min, f_max, f_ref, f_prev, w_c);
	std::cout << "force allocation: \n" << fares.f << std::endl;
	auto stop = std::chrono::high_resolution_clock::now();
	auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
	std::cout << "Duration: " << duration.count() << " [ms]" << std::endl;

	/*std::string input;
	std::cin >> input;*/
	/*if (is_number(input)) {
		std::cout << "Oke" << std::endl;
	}*/

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
	std::cout << "fs:\n"<< calculate_fs(vels, er, f_static, precv, precx, precy, prect) << std::endl;
	std::cout << map(11, 0, 10, 0, 1) << std::endl;
	bool move_on = 0;
	std::string input;
	std::cin >> input;
	com_init(handles, odrv_ports);

	while (!move_on) {
		std::cout << "Select procedure (1-8):" << std::endl;
		std::cout << "1) Move platform with arrow keys" << std::endl;
		std::cout << "2) Set tension" << std::endl;
		std::cout << "3) Set home position" << std::endl;
		std::cout << "4) Clear errors" << std::endl;
		std::cout << "5) Run control loop" << std::endl;
		std::cout << "6) Enable enable_dc_bus_voltage_feedback on all ODrives" << std::endl;
		std::cout << "7) testt" << std::endl;
		std::cout << "8) Exit" << std::endl;

		std::string input;
		std::cin >> input;
		// TODO:
		// Make function for moving platform with arrow keys
		// Verify function for setting tension
		// Verify function for setting home position
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
				set_all_encoder_positions(handles, positions);
				break;
			case 4:
				//com_init(handles, odrv_ports);
				clear_errors(handles);
				break;
			case 5:
				//com_init(handles, odrv_ports);
				control_loop();
				break;
			case 6:
				//com_init(handles, odrv_ports);
				enable_all_brake_resistor_voltage_feedback(handles);
				break;
			case 7:
				testt();
				break;
			case 8:
				move_on = 1;
				break;
			default:
				break;
			}
		}

	}
	//control_loop();


	/*std::cout << "Error: " << read_driver_error_status(handles[0]) << std::endl;
	int errors[4];
	read_all_driver_error_statuses(handles, errors);
	std::cout << "Errors: " << errors[0] << errors[1] << errors[2] << errors[3] << std::endl;*/
	/*int i = 0;
	while (i < 1000) {
		poll_keys();
		i++;
		Sleep(1);
	}*/
	system("pause");
}
