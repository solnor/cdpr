#include "controllers.h"
#include <iostream>
#include <chrono>

typedef struct {
	double p0;
	double p_enc;
	double p;
} enc_positions;

// https://stackoverflow.com/questions/1903954/is-there-a-standard-sign-function-signum-sgn-in-c-c
template <typename T> int sgn(T val) {
	return (T(0) < val) - (val < T(0));
}

Eigen::Vector4d get_positions_with_offset(const Eigen::Ref<const Eigen::Vector4d>& p_enc, 
										  const Eigen::Ref<const Eigen::Vector4d>& p0) {
	Eigen::Vector4d pos = Eigen::Vector4d::Zero();
	/*pos << (p_enc(0) >= 0) ? p_enc(0) - p0(0) : p0(0) - p_enc(0),
		   (p_enc(1) >= 0) ? p_enc(1) - p0(1) : p0(1) - p_enc(1),
		   (p_enc(2) >= 0) ? p_enc(2) - p0(2) : p0(2) - p_enc(2),
		   (p_enc(3) >= 0) ? p_enc(3) - p0(3) : p0(3) - p_enc(3);*/
	/*pos(0) = (p_enc(0) >= 0) ? p_enc(0) - p0(0) : p0(0) - p_enc(0);
	pos(1) = (p_enc(1) >= 0) ? p_enc(1) - p0(1) : p0(1) - p_enc(1);
	pos(2) = (p_enc(2) >= 0) ? p_enc(2) - p0(2) : p0(2) - p_enc(2);
	pos(3) = (p_enc(3) >= 0) ? p_enc(3) - p0(3) : p0(3) - p_enc(3);*/
	/*pos(0) = sgn(p_enc(0))*(abs(p_enc(0)) - abs(p0(0)));
	pos(1) = sgn(p_enc(1))*(abs(p_enc(1)) - abs(p0(1)));
	pos(2) = sgn(p_enc(2))*(abs(p_enc(2)) - abs(p0(2)));
	pos(3) = sgn(p_enc(3))*(abs(p_enc(3)) - abs(p0(3)));*/
	//pos << sgn(p_enc(0))*(abs(p_enc(0)) - abs(p0(0))),
	//	   sgn(p_enc(1))*(abs(p_enc(1)) - abs(p0(1))),
	//	   sgn(p_enc(2))*(abs(p_enc(2)) - abs(p0(2))),
	//	   sgn(p_enc(3))*(abs(p_enc(3)) - abs(p0(3)));
	/*pos(0) = (p_enc(0) >= 0) ? p_enc(0) - p0(0) : p0(0) - p_enc(0);
	pos(1) = (p_enc(1) >= 0) ? p_enc(1) - p0(1) : p0(1) - p_enc(1);
	pos(2) = (p_enc(2) >= 0) ? p_enc(2) - p0(2) : p0(2) - p_enc(2);
	pos(3) = (p_enc(3) >= 0) ? p_enc(3) - p0(3) : p0(3) - p_enc(3);*/
	pos << p_enc(0) - p0(0),
		   p_enc(1) - p0(1),
		   p_enc(2) - p0(2),
		   p_enc(3) - p0(3);
	return pos;
}

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
	//set_all_encoder_positions(handles, positions);
	get_all_motor_states(handles, motor_states);

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
	std::cout << "Motor positions: " << (*motor_states[0]).pos \
							 << ", " << (*motor_states[1]).pos \
							 << ", " << (*motor_states[2]).pos \
							 << ", " << (*motor_states[3]).pos \
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

Eigen::Vector4d calculate_fs2(const Eigen::Ref<const Eigen::Vector4d>& vels,
	const Eigen::Ref<const Eigen::Vector3d>& t,
	const Eigen::Ref<const Eigen::MatrixXd>& AT,
	const Eigen::Ref<const Eigen::Vector3d>& e,
	const Eigen::Ref<const Eigen::Vector4d>& f_static,
	double precv,
	double precx,
	double precy,
	double prect,
	double prectw) {
	/*Eigen::Vector4d velp;

	velp << std::trunc(vels(0) * pow(10, precv)) / pow(10, precv),
		std::trunc(vels(1) * pow(10, precv)) / pow(10, precv),
		std::trunc(vels(2) * pow(10, precv)) / pow(10, precv),
		std::trunc(vels(3) * pow(10, precv)) / pow(10, precv);*/
	Eigen::Vector3d tp;
	//tp << !(1 & (bool)(ceil(abs(velp(0)))))
	tp << std::trunc(t(0) * pow(10, prectw)) / pow(10, prectw),
		  std::trunc(t(1) * pow(10, prectw)) / pow(10, prectw),
		  std::trunc(t(2) * pow(10, prectw)) / pow(10, prectw);
	tp << !(1 & (bool)(ceil(abs(tp(0))))),
		  !(1 & (bool)(ceil(abs(tp(1))))),
		  !(1 & (bool)(ceil(abs(tp(2)))));
	std::cout << "tp: \n" << tp << std::endl;
	/*velp << !(1 & (bool)(ceil(abs(velp(0))))),
		    !(1 & (bool)(ceil(abs(velp(1))))),
		    !(1 & (bool)(ceil(abs(velp(2))))),
		    !(1 & (bool)(ceil(abs(velp(3)))));
	std::cout << "velp bool: \n" << velp << std::endl;*/
	Eigen::Vector4d ldot;
	ldot << AT.transpose()*tp;
	ldot << (bool)abs(ldot(0)), (bool)abs(ldot(1)), (bool)abs(ldot(2)), (bool)abs(ldot(3));
	Eigen::Vector3d ep;
	ep << std::trunc(e(0) * pow(10, precx)) / pow(10, precx),
		  std::trunc(e(1) * pow(10, precy)) / pow(10, precy),
		  std::trunc(e(2) * pow(10, prect)) / pow(10, prect);
	std::cout << "ep: \n" << ep << std::endl;
	Eigen::Vector4d fs;
	fs << (int)ep.any() * ldot.cwiseProduct(f_static);
	return fs;
}

int tension_control_loop(HANDLE handles[], const Eigen::Ref<const Eigen::Vector4d>& p0_in) {

	double mm2m = (double)pow(10, -3);
	double r_d = 20*mm2m;
	double r_p = 12*mm2m;
	double pitch_drum = 2.9 * mm2m;
	double ydiff = 371.4759 * mm2m;
	double xdiff = 56.40783 * mm2m;
	
	Eigen::MatrixXd a(2, 4);
	/*a << -0.7560, -0.7560, 0.7560, 0.7560,
		 -0.4181, 0.4181, 0.4181, -0.4181;*/
	a << -0.7490, -0.7490, 0.7530,  0.7530,
	     -0.4041,  0.4321, 0.4321, -0.4041;
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

	Eigen::Vector4d pos_enc, pos, p0, pos_rad;
	if (p0_in.any()) {
		p0 << p0_in;
	}
	//Eigen::Vector4d ;
	Eigen::Vector4d vel, vel_rad;
	Eigen::Vector4d fvel = Eigen::Vector4d::Zero();
	Eigen::Vector4d vel_m, vel_m_rad;
	Eigen::Vector4d l, ldot, lfk;
	//Eigen::Vector4d l0(1.2260,
	//				   1.1833,
	//				   1.1833,
	//				   1.2260);
	//Eigen::Vector4d l0(1.2060,
	//				   1.1820,
	//				   1.1820,
	//				   1.2060);
	Eigen::Vector4d l0(1.1962, 
					   1.1723, 
					   1.1736, 
					   1.2017);
	Eigen::Vector3d lde2pe(0.3597,
						   0.3636,
						   0.3617,
						   0.3617);
	inv_res invkin;

	Eigen::MatrixXd AT, AT_pinv, A_pinv = Eigen::MatrixXd::Zero(3, 4);
	//Eigen::MatrixXd  = Eigen::MatrixXd::Zero(3, 4);
	//Eigen::MatrixXd A_pinv  = Eigen::MatrixXd::Zero(3, 4);

	double f_min = 10;
	double f_max = 60;
	double f_ref = (f_max + f_min) / 2;
	Eigen::Vector4d f_prev = f_ref*Eigen::Vector4d::Ones();
	force_alloc_res fres;

	//Eigen::Vector4d f_static(0.0840 / r_d,
	//						 0.0820 / r_d,
	//						 0.1040 / r_d,
	//						 0.0780 / r_d);
	//Eigen::Vector4d f_static(0.06040 / r_d,
	//						 0.1520 / r_d,
	//						 0.06040 / r_d,
	//						 0.15080 / r_d);
	//Eigen::Vector4d f_static(0.1 / r_d,
	//						 0.05 / r_d,
	//						 0.1 / r_d,
	//						 0.1 / r_d);
	//Eigen::Vector4d f_static(3.28099,
	//						 0.862874,
	//						 1.71798,
	//						 1.63746);
	//Eigen::Vector4d f_static(3.58099,
	//						 2.0862874,
	//						 6.7821798,
	//						 3.52063746); // Good for wd = 1,0,0
	Eigen::Vector4d f_static(12.58099,
							 3.5862874,
							 6.7821798,
							 3.52063746);
		
		
	Eigen::Vector4d f_loss(2.195,
						   1.995,
						   2.245,
						   1.845);
	f_loss << 1.0, 0, 0, 0.5;
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
	Eigen::Vector3d wd, we, w0;
	Eigen::Vector4d T = Eigen::Vector4d::Zero();

	Eigen::Matrix3d Kp = Eigen::Matrix3d::Zero();
	Kp(0, 0) = 125; // Proportional gain x
	Kp(1, 1) = 125; // Proportional gain y
	Kp(2, 2) = 5;   // Proportional gain theta

	Eigen::Matrix3d Ki = Eigen::Matrix3d::Zero();
	Ki(0, 0) = 0; // Integral gain x
	Ki(1, 1) = 0; // Integral gain y
	Ki(2, 2) = 0; // Integral gain theta
	Eigen::Vector3d eint = Eigen::Vector3d::Zero();
	int key_pressed = 0;

	/*if (!(run_initial_prompts(handles, motor_signs, motor_states))) {
		return 0;
	}*/

	std::string input;
	std::cin >> input;
	get_all_motor_states(handles, motor_states);
	p0 << ms0.pos,
		  ms1.pos,
		  ms2.pos,
		  ms3.pos;
	std::cout << "p0: \n" << p0 << std::endl;
	/*p0 << -1.30595,
		   2.90906,
		   1.20864,
		  -3.02383;*/
	/*p0 << -1.25022,
		   2.91746,
		   1.20932,
		  -3.03257;
	p0 << -1.14896,
		   3.15568,
		   1.27374,
		  -3.09029;
	p0 << -1.18695,
		   3.00702,
		   1.2469,
		  -3.03695;*/
	std::cout << "p0: \n" << p0 << std::endl;

	bool running = 1;
	bool any_error = 0;
	auto start = std::chrono::high_resolution_clock::now();
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
		pos_enc << ms0.pos,
				   ms1.pos,
				   ms2.pos,
			       ms3.pos;
		pos = get_positions_with_offset(pos_enc, p0);
		//std::cout << "pos: \n" << pos << std::endl;
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
		/*lfk << l(0) - sqrt( pow(sqrt( pow(pos(0)*pitch_drum, 2) + pow(ydiff,2) ), 2) + pow(xdiff,2)),
			   l(1) - sqrt( pow(sqrt( pow(pos(1)*pitch_drum, 2) + pow(ydiff,2) ), 2) + pow(xdiff,2)),
			   l(2) - sqrt( pow(sqrt( pow(pos(2)*pitch_drum, 2) + pow(ydiff,2) ), 2) + pow(xdiff,2)),
			   l(3) - sqrt( pow(sqrt( pow(pos(3)*pitch_drum, 2) + pow(ydiff,2) ), 2) + pow(xdiff,2));*/
		lfk << l(0) - sqrt( pow(pos(0) * pitch_drum, 2) + pow(lde2pe(0), 2)),
			   l(0) - sqrt( pow(pos(1) * pitch_drum, 2) + pow(lde2pe(1), 2)),
			   l(0) - sqrt( pow(pos(2) * pitch_drum, 2) + pow(lde2pe(2), 2)),
			   l(0) - sqrt( pow(pos(3) * pitch_drum, 2) + pow(lde2pe(3), 2));
		//std::cout << "lfk: \n" << lfk << std::endl;
		q = forward_kinematics(a, b, 
							   fk_init_estimate(a, b, lfk), 
							   lfk, r_p);
		//q << 0, 0, 0;
		std::cout << "q: \n" << q << std::endl;
		invkin = inverse_kinematics(a, b, q, r_p);

		AT = calculate_structure_matrix(a, b, q, invkin.betar, r_p);
		std::cout << "A^T: \n" << AT << std::endl;
		AT_pinv = AT.completeOrthogonalDecomposition().pseudoInverse();
		A_pinv = (-1*AT).transpose().completeOrthogonalDecomposition().pseudoInverse();
		
		//wd << 0, 0, 0;

		auto t_loop = std::chrono::high_resolution_clock::now();
		auto t_since_start = std::chrono::duration_cast<std::chrono::seconds>(t_loop - start);
		//qd << 0.1*cos(2*std::chrono::duration<double>(t_since_start).count()), -0.1, 0;
		// * cos(8 * std::chrono::duration<double>(t_since_start).count())
		qd << 0.1, 0, 0;
		e << qd - q;
		wd << Kp * e;
		wd << 0.0, 0.0, 0.0;
		e << 1.0, 1.0, 1.0;
		//std::cout << "vel: \n" << vel << std::endl;
		qdot = A_pinv * ldot;
		std::cout << "qdot: \n" << qdot << std::endl;
		fs = calculate_fs2(vel, qdot, AT, e, f_static,precv, precx, precy, prect, 2);
		std::cout << "fs: \n" << fs << std::endl;

		f_pinv << AT_pinv * wd;
		//f0 << sgn(f_pinv(0))*(f_loss(0)+fs(0)),
		//	  sgn(f_pinv(1))*(f_loss(1)+fs(1)),
		//	  sgn(f_pinv(2))*(f_loss(2)+fs(2)),
		//	  sgn(f_pinv(3))*(f_loss(3)+fs(3));
		fs << f_static;
		f0 << sgn(f_pinv(0))*(fs(0)),
			  sgn(f_pinv(1))*(fs(1)),
			  sgn(f_pinv(2))*(fs(2)),
			  sgn(f_pinv(3))*(fs(3));
		//std::cout << "wd: \n" << wd << std::endl;
		//wd << wd + AT * f0;
		//wd << wd + Eigen::Vector3d(5, 2.2, 0);
		//f_pinv << AT_pinv * Eigen::Vector3d(5, 2.2, 0);
		//std::cout << "f_pinv: \n" << f_pinv << std::endl;
		std::cout << "f0: \n" << f0 << std::endl;
		//std::cout << "wd+w0: \n" << wd << std::endl;

		fres = force_alloc_iterative_slack(AT.transpose(), f_min, f_max, f_ref, f_prev, wd);
		//fres.f = f_ref * Eigen::Vector4d::Ones() - AT_pinv * (wd + AT * f_ref * Eigen::Vector4d::Ones());
		std::cout << "f: \n" << fres.f << std::endl;
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

int pos_test(HANDLE handles[]) {
	motor_state ms0, ms1, ms2, ms3;
	std::vector<motor_state*> motor_states;
	motor_states.push_back(&ms0);
	motor_states.push_back(&ms1);
	motor_states.push_back(&ms2);
	motor_states.push_back(&ms3);

	Eigen::Vector4d pos_enc, pos, p0, pos_rad;
	std::string input;
	std::cin >> input;
	get_all_motor_states(handles, motor_states);
	p0 << ms0.pos,
		  ms1.pos,
		  ms2.pos,
		  ms3.pos;

	while (1) {
		get_all_motor_states(handles, motor_states);
		pos_enc << ms0.pos,
				   ms1.pos,
				   ms2.pos,
				   ms3.pos;
		pos = get_positions_with_offset(pos_enc, p0);
		std::cout << "pos: \n" << pos << std::endl;
	}
}