#include "controllers.h"
#include <iostream>

int set_standard_tension(HANDLE handles[], const Eigen::Ref<const Eigen::Vector4d>& motor_signs) {
	set_all_axis_states(handles, AXIS_STATE_CLOSED_LOOP_CONTROL);
	Sleep(10);
	Eigen::Vector4d torques = -0.2*motor_signs;
	set_all_motor_torques(handles, torques);
	return 1;
}

int poll_keys2() {
	if (GetAsyncKeyState('A') & 0x8000) {
		std::cout << "Pressing A" << std::endl;
		return 1;
	}
	if (GetAsyncKeyState('Q') & 0x8000) {
		return 2;
	}
	if (GetAsyncKeyState('Y') & 0x8000) {
		return 3;
	}
	if (GetAsyncKeyState('N') & 0x8000) {
		return 4;
	}
	if (GetAsyncKeyState(0x25) & 0x8000) { // Left arrow key
		return 5;
	}
	return 0;
}

int run_initial_prompts(HANDLE handles[], const Eigen::Ref<const Eigen::Vector4d>& motor_signs, std::vector<motor_state*> motor_states) {
	std::cout << "Set starting torques? (y/n)" << std::endl;
	
	int r = 0;
	while (!(r == 3)) {
		r = poll_keys2();
		if (r == 4) return 0;
	}
	Sleep(250);

	std::cout << "Setting starting torques" << std::endl;
	set_standard_tension(handles, motor_signs);

	std::cout << "Set home position?" << std::endl;
	r = 0;
	while (!(r == 3)) {
		r = poll_keys2();
		if (r == 4) return 0;
	}
	Sleep(250);
	
	std::cout << "Setting home position" << std::endl;
	double positions[4];
	set_all_encoder_positions(handles, positions);
	std::cout << "Set home position?" << std::endl;
	r = 0;
	while (!(r == 3)) {
		r = poll_keys2();
		if (r == 4) return 0;
	}
	Sleep(250);
	std::cout << "Setting home position" << std::endl;
	set_all_encoder_positions(handles, positions);

	//}
	get_all_motor_states(handles, motor_states);
	std::cout << "Motor positions: " << &motor_states[0]->pos \
							 << ", " << &motor_states[1]->pos \
							 << ", " << &motor_states[2]->pos \
							 << ", " << &motor_states[3]->pos \
							 << std::endl;
	std::cout << "Start control loop?" << std::endl;
	r = 0;
	while (!(r == 3)) {
		r = poll_keys2();
		if (r == 4) return 0;
	}
	Sleep(250);
	return 1;
}

int tension_control_loop(HANDLE handles[]) {

	double mm2m = pow(10, -3);
	double r_d = 20*mm2m;
	double r_p = 12*mm2m;
	double pitch_drum = 2.9 * mm2m;
	double ydiff = 371.4759 * mm2m;
	double xdiff = 56.40783 * mm2m;
	
	Eigen::MatrixXd a(2, 4);
	a << -0.7560, -0.7560, 0.7560, 0.7560,
		 -0.4181, 0.4181, 0.4181, -0.4181;
	// Trapezoidal b
	Eigen::MatrixXd b(2, 4);
	b << -0.0250, -0.0750, 0.0750, 0.0250,
		 -0.0100, 0.0100, 0.0100, -0.0100;

	Eigen::Vector4d motor_signs(-1, 
								 1,	
		                        -1, 
								 1);

	motor_state ms0, ms1, ms2, ms3;
	std::vector<motor_state*> motor_states;
	motor_states.push_back(&ms0);
	motor_states.push_back(&ms1);
	motor_states.push_back(&ms2);
	motor_states.push_back(&ms3);

	Eigen::Vector4d pos, pos_rad;
	//Eigen::Vector4d ;
	Eigen::Vector4d vel, vel_rad;
	Eigen::Vector4d fvel = Eigen::Vector4d::Zero();
	Eigen::Vector4d vel_m, vel_m_rad;
	Eigen::Vector4d l, ldot, lfk;
	Eigen::Vector4d l0(1.2260,
					   1.1833,
					   1.1833,
					   1.2260);

	inv_res invkin;

	Eigen::MatrixXd AT, AT_pinv, A_pinv = Eigen::MatrixXd::Zero(3, 4);
	//Eigen::MatrixXd  = Eigen::MatrixXd::Zero(3, 4);
	//Eigen::MatrixXd A_pinv  = Eigen::MatrixXd::Zero(3, 4);

	double f_min = 15;
	double f_max = 80;
	double f_ref = (f_max + f_min) / 2;
	Eigen::Vector4d f_prev = f_ref * Eigen::Vector4d::Ones();
	force_alloc_res fres;

	Eigen::Vector4d f_static(0.0840 / r_d,
							 0.0820 / r_d,
							 0.1040 / r_d,
							 0.0780 / r_d);
	Eigen::Vector4d f_loss(2.195,
						   1.995,
						   2.245,
						   1.845);
	
	Eigen::Vector4d f_pinv = Eigen::Vector4d::Zero();
	Eigen::Vector3d e_t = Eigen::Vector3d::Zero();
	Eigen::Vector4d fs = Eigen::Vector4d::Zero();
	Eigen::Vector4d f0 = Eigen::Vector4d::Zero();
	double precv = 2;
	double precx = 3;
	double precy = 3;
	double prect = 0;

	Eigen::Vector3d q, qdot = Eigen::Vector3d::Zero();
	Eigen::Vector3d qd;
	Eigen::Vector3d e;
	Eigen::Vector3d wd, we;
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
	int key_pressed = 0;

	if (!(run_initial_prompts(handles, motor_signs, motor_states))) {
		return 0;
	}
	
	bool running = 1;
	bool any_error = 0;
	while (running) {
		any_error = check_if_any_driver_errors(handles);
		if (any_error) {
			running = 0;
			std::cout << "Breaking" << std::endl;
			break;
		};

		// Read motor velocities and positions from motor drivers
		// and update variables
		get_all_motor_states(handles, motor_states);
		pos << ms0.pos,
			   ms1.pos,
			   ms2.pos,
			   ms3.pos;

		vel << ms0.vel,
			   ms1.vel,
			   ms2.vel,
			   ms3.vel;
		pos_rad << pos * 2 * PI;
		vel_rad << vel * 2 * PI;

		vel_m << vel.cwiseProduct(motor_signs);
		vel_m_rad << vel_rad.cwiseProduct(motor_signs);

		l << l0 + pos_rad.cwiseProduct(r_d*motor_signs);
		ldot << r_d * vel_m_rad;

		// Subtract length between drum and pulley from total cable length to get length used in FK
		lfk << l(0) - sqrt( pow(sqrt( pow(pos_rad(0)*pitch_drum, 2) + pow(ydiff,2) ), 2) + pow(xdiff,2)),
			   l(1) - sqrt( pow(sqrt( pow(pos_rad(1)*pitch_drum, 2) + pow(ydiff,2) ), 2) + pow(xdiff,2)),
			   l(2) - sqrt( pow(sqrt( pow(pos_rad(2)*pitch_drum, 2) + pow(ydiff,2) ), 2) + pow(xdiff,2)),
			   l(3) - sqrt( pow(sqrt( pow(pos_rad(3)*pitch_drum, 2) + pow(ydiff,2) ), 2) + pow(xdiff,2));

		q = forward_kinematics(a, b, 
							   fk_init_estimate(a, b, lfk), 
							   lfk, r_p);
		invkin = inverse_kinematics(a, b, q, r_p);

		AT = calculate_structure_matrix(a, b, q, invkin.betar, r_p);
		AT_pinv = AT.completeOrthogonalDecomposition().pseudoInverse();
		A_pinv = (-1*AT).transpose().completeOrthogonalDecomposition().pseudoInverse();

		wd << 0, 0, 0;
		fres = force_alloc_iterative_slack(AT.transpose(), f_min, f_max, f_ref, f_prev, wd);

		// Convert tensions to torque and set the torque on each motor
		T = (fres.f).cwiseProduct(r_d*(-1)*motor_signs);
		set_all_motor_torques(handles, T);

		if (!fres.flag) {
			f_prev = fres.f;
		}

		key_pressed = poll_keys2();
		if (key_pressed == 2) {
			break;
		}

	}
	return 1;
}

int position_control_loop(HANDLE handles[]) {
	return 1;
}