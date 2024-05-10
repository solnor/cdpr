#include "controllers.h"
#include <iostream>
#include <chrono>
#include <fstream>
#include <iterator>

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

Eigen::Vector4d calculate_fs3(const Eigen::Ref<const Eigen::Vector4d>& vels,
							  const Eigen::Ref<const Eigen::Vector3d>& e,
							  const Eigen::Ref<const Eigen::Vector4d>& f_static,
							  double precv,
							  double precx,
							  double precy,
							  double prect) {
	Eigen::Vector4d velp;

	velp << std::trunc(vels(0) * pow(10, precv)) / pow(10, precv),
		std::trunc(vels(1) * pow(10, precv)) / pow(10, precv),
		std::trunc(vels(2) * pow(10, precv)) / pow(10, precv),
		std::trunc(vels(3) * pow(10, precv)) / pow(10, precv);
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
	ep << std::trunc(e(0) * pow(10, precx)) / pow(10, precx),
		std::trunc(e(1) * pow(10, precy)) / pow(10, precy),
		std::trunc(e(2) * pow(10, prect)) / pow(10, prect);
	//std::cout << "ep: \n" << ep << std::endl;

	/*Eigen::Vector4d dir(sgn(f_pinv(0)),
						sgn(f_pinv(1)),
						sgn(f_pinv(2)),
						sgn(f_pinv(3)));*/
	Eigen::Vector4d fs;
	//fs << (int)f_pinvp.any()*velp.cwiseProduct(dir.cwiseProduct(f_static));
	fs << (int)ep.any() * velp.cwiseProduct(f_static);
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


	std::vector<Eigen::Vector3d> p_log;
	std::vector<Eigen::Vector3d> i_log;
	std::vector<Eigen::Vector3d> d_log;
	std::vector<Eigen::Vector3d> q_log;
	std::vector<Eigen::Vector3d> qdot_log;
	std::vector<Eigen::Vector3d> fqdot_log;
	std::vector<Eigen::Vector3d> qd_log;
	std::vector<Eigen::Vector3d> qddot_log;
	std::vector<Eigen::Vector3d> wd_log;
	std::vector<Eigen::Vector3d> wa_log;
	std::vector<Eigen::Vector3d> eint_log;
	std::vector<Eigen::Vector3d> wint_log;
	std::vector<Eigen::Vector4d> vel_log;
	std::vector<Eigen::Vector4d> fvel_log;
	std::vector<Eigen::Vector4d> vel_m_rad_log;
	std::vector<Eigen::Vector4d> fvel_m_rad_log;
	std::vector<Eigen::Vector4d> f_log;
	std::vector<Eigen::Vector4d> t0_log;
	std::vector<double> t_log;

	Eigen::Vector4d pos_enc, pos, p0, pos_rad;
	
	//Eigen::Vector4d ;
	Eigen::Vector4d vel, vel_rad;
	Eigen::Vector4d fvel = Eigen::Vector4d::Zero();
	Eigen::Vector4d vel_m, vel_m_rad, fvel_m_rad = Eigen::Vector4d::Zero();
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
	Eigen::Vector4d lde2pe(0.3597,
						   0.3636,
						   0.3617,
						   0.3617);
	inv_res invkin;

	Eigen::MatrixXd AT, AT_pinv, A_pinv, AT_fa = Eigen::MatrixXd::Zero(3, 4);
	//Eigen::MatrixXd  = Eigen::MatrixXd::Zero(3, 4);
	//Eigen::MatrixXd A_pinv  = Eigen::MatrixXd::Zero(3, 4);

	double f_min = 10;
	double f_max = 80;
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
	//Eigen::Vector4d t_static(0.056, 0.0504, 0.0498, 0.0354);
	//Eigen::Vector4d t_static(0.03, 0.03, 0.03, 0.03);
	Eigen::Vector4d t_static(0.04, 0.04, 0.03, 0.03);
		
	Eigen::Vector4d f_loss(2.195,
						   1.995,
						   2.245,
						   1.845);
	f_loss << 1.0, 0, 0, 0.5;
	Eigen::Vector4d f_pinv = Eigen::Vector4d::Zero();
	Eigen::Vector3d e_t = Eigen::Vector3d::Zero();
	Eigen::Vector4d fs = Eigen::Vector4d::Zero();
	Eigen::Vector4d ts = Eigen::Vector4d::Zero();
	Eigen::Vector4d f0 = Eigen::Vector4d::Zero();
	Eigen::Vector4d t0 = Eigen::Vector4d::Zero();
	double precv = 2;
	double precx = 3;
	double precy = 3;
	double prect = 0;

	Eigen::Vector3d q, qdot, fq = Eigen::Vector3d::Zero();
	Eigen::Vector3d qd, qddot, fqdot = Eigen::Vector3d::Zero();
	double omega = 2.5;
	double Lx = 0.15;
	double Ly = 0.15;
	Eigen::Vector3d e, edot, eint = Eigen::Vector3d::Zero();
	double dt = 15*pow(10,-3);
	Eigen::Vector3d wd, we, wa, w0, wint = Eigen::Vector3d::Zero();
	Eigen::Vector3d intlimit(6, 6, 0.1);
	Eigen::Vector4d T = Eigen::Vector4d::Zero();

	Eigen::Matrix3d Kp = Eigen::Matrix3d::Zero();
	Kp(0, 0) = 225; // Proportional gain x
	Kp(1, 1) = 150; // Proportional gain y
	Kp(2, 2) = 5;   // Proportional gain theta

	Eigen::Matrix3d Ki = Eigen::Matrix3d::Zero();
	Ki(0, 0) = 425; // Integral gain x
	Ki(1, 1) = 125; // Integral gain y
	Ki(2, 2) = 0; // Integral gain theta

	Eigen::Matrix3d Kd = Eigen::Matrix3d::Zero();
	Kd(0, 0) = 15;  // Derivative gain x
	Kd(1, 1) = 21;  // Derivative gain y
	Kd(2, 2) = 0.01;  // Derivative gain theta
	//Eigen::Vector3d eint = Eigen::Vector3d::Zero();
	int key_pressed = 0;

	/*if (!(run_initial_prompts(handles, motor_signs, motor_states))) {
		return 0;
	}*/

	std::string input;
	std::cin >> input;
	get_all_motor_states(handles, motor_states);
	if (p0_in.any()) {
		p0 << p0_in;
		std::cout << "p0_in: \n" << p0_in << std::endl;
	} else {
		p0 << ms0.pos,
			  ms1.pos,
			  ms2.pos,
			  ms3.pos;
	}
	/*p0 << -0.3085,
		  -0.0683,
		   0.2052,
		  -0.0371;*/
	std::cout << "p0: \n" << p0 << std::endl;
	
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

	double t_since_start_c = 0;
	double loop_period = 0;


	bool running = 1;
	bool any_error = 0;
	auto t_start = std::chrono::high_resolution_clock::now();
	while (running) {
		any_error = check_if_any_driver_errors(handles);
		if (any_error) {
			running = 0;
			std::cout << "Breaking" << std::endl;
			break;
		};

		auto t_loop_start = std::chrono::high_resolution_clock::now();
		auto t_since_start = std::chrono::duration_cast<std::chrono::milliseconds>(t_loop_start - t_start);

		// Read motor velocities and positions from motor drivers
		// and update variables
		get_all_motor_states(handles, motor_states);
		pos_enc << ms0.pos,
				   ms1.pos,
				   ms2.pos,
			       ms3.pos;
		pos = get_positions_with_offset(pos_enc, p0);
		pos_rad << pos * 2 * PI;
		//std::cout << "pos: \n" << pos << std::endl;
		vel << ms0.vel,
			   ms1.vel,
			   ms2.vel,
			   ms3.vel;
		fvel = 0.5*fvel + 0.5*vel;
		vel_rad << vel * 2 * PI;
		vel_m << vel.cwiseProduct(motor_signs);
		vel_m_rad << vel_rad.cwiseProduct(motor_signs);
		
		fvel_m_rad << 0.6*fvel_m_rad + 0.4*vel_m_rad;


		l << l0 + pos_rad.cwiseProduct(r_d*motor_signs);
		ldot << r_d * fvel_m_rad;

		// Subtract length between drum and pulley from total cable length to get length used in FK
		lfk << l(0) - sqrt( pow(pos(0)*pitch_drum, 2) + pow(lde2pe(0), 2)),
			   l(1) - sqrt( pow(pos(1)*pitch_drum, 2) + pow(lde2pe(1), 2)),
			   l(2) - sqrt( pow(pos(2)*pitch_drum, 2) + pow(lde2pe(2), 2)),
			   l(3) - sqrt( pow(pos(3)*pitch_drum, 2) + pow(lde2pe(3), 2));
		//std::cout << "lfk: \n" << lfk << std::endl;
		q = forward_kinematics(a, b, 
							   fk_init_estimate(a, b, lfk), 
							   lfk, r_p);
		//q << 0, 0, 0;
		//std::cout << "q: \n" << q << std::endl;
		std::cout << "q: " << q(0)*1000 << ", " << q(1) * 1000 << ", " << q(2)*180/PI << std::endl;
		invkin = inverse_kinematics(a, b, q, r_p);

		AT = calculate_structure_matrix(a, b, q, invkin.betar, r_p);
		//std::cout << "A^T: \n" << AT << std::endl;
		AT_pinv = AT.completeOrthogonalDecomposition().pseudoInverse();
		A_pinv = (-1*AT).transpose().completeOrthogonalDecomposition().pseudoInverse();
		qdot = A_pinv*ldot;
		fqdot << qdot(0), qdot(1), 0.7*fqdot(2) + 0.3*qdot(2);
		
		//wd << 0, 0, 0;

		t_since_start_c = std::chrono::duration<double>(t_since_start).count();
		std::cout << "Time since start: " << t_since_start_c << std::endl;
		qd << Lx*sin(omega*t_since_start_c),
			  Ly*cos(omega*t_since_start_c)-0.15,
			  0;
		qddot <<  omega*Lx*cos(omega*t_since_start_c),
			     -omega*Ly*sin(omega*t_since_start_c),
			      0;

		/*qd << 0.2 * cos(t_since_start_c),
			  0.75 * sin(t_since_start_c) * cos(t_since_start_c) - 0.1,
			  0;
		
		qddot << -0.2 * sin(t_since_start_c),
				 -0.75 * cos(2*t_since_start_c),
				  0;*/

		// * cos(8 * std::chrono::duration<double>(t_since_start).count())
		//qd << 0.1, 0, 0;
		e << qd - q;
		edot << qddot - fqdot;
		eint += e * dt;
		wint = Ki * eint;
		std::cout << "wint: \n" << wint << std::endl;
		// Saturation limits
		for (uint8_t j = 0; j < 3; j++) {
			if (wint(j) > intlimit(j)) {
				wint(j) = intlimit(j);
			} else if (wint(j) < -intlimit(j)) {
				wint(j) = -intlimit(j);
			}
		}
		std::cout << "wint: \n" << wint << std::endl;
		/*wint(0) = wint(0) > intlimit(0) ? intlimit(0) :
			                              abs(wint(0)) > intlimit(0) ? intlimit(0) : wint(0);
		wint(1) = wint(1) > intlimit(1) ? intlimit(1) :
										  abs(wint(1)) > intlimit(1) ? intlimit(1) : wint(1);
		wint(2) = wint(2) > intlimit(2) ? intlimit(2) :
										  abs(wint(2)) > intlimit(2) ? intlimit(2) : wint(2);*/
		wd << Kp * e + Kd*edot + wint;
		
		//wd << 0.0, 0.0, 0.0;
		//e << 1.0, 1.0, 1.0;
		//std::cout << "vel: \n" << vel << std::endl;
		//qdot = A_pinv * ldot;
		//std::cout << "qdot: \n" << qdot << std::endl;
		//fs = calculate_fs2(vel, qdot, AT, e, f_static,precv, precx, precy, prect, 2);
		//std::cout << "fs: \n" << fs << std::endl;

		f_pinv << AT_pinv * wd;
		//f0 << sgn(f_pinv(0))*(f_loss(0)+fs(0)),
		//	  sgn(f_pinv(1))*(f_loss(1)+fs(1)),
		//	  sgn(f_pinv(2))*(f_loss(2)+fs(2)),
		//	  sgn(f_pinv(3))*(f_loss(3)+fs(3));
		ts = calculate_fs3(fvel, e, t_static, precv, precx, precy, prect);
		fs << f_static;
		f0 << sgn(f_pinv(0))*(fs(0)),
			  sgn(f_pinv(1))*(fs(1)),
			  sgn(f_pinv(2))*(fs(2)),
			  sgn(f_pinv(3))*(fs(3));
		t0 << sgn(f_pinv(0))*(ts(0)),
			  sgn(f_pinv(1))*(ts(1)),
			  sgn(f_pinv(2))*(ts(2)),
			  sgn(f_pinv(3))*(ts(3));
		std::cout << "t0: \n" << t0 << std::endl;
		//std::cout << "wd: \n" << wd << std::endl;
		//wd << wd + AT * f0;
		//wd << wd + Eigen::Vector3d(5, 2.2, 0);
		//f_pinv << AT_pinv * Eigen::Vector3d(5, 2.2, 0);
		//std::cout << "f_pinv: \n" << f_pinv << std::endl;
		//std::cout << "f0: \n" << f0 << std::endl;
		//std::cout << "wd: \n" << wd << std::endl;
		//q_fa << q(1), q(2), 0;
		/*AT_fa = calculate_structure_matrix(a, b, q, invkin.betar, r_p);
		AT_fa(2, 0) = 0;
		AT_fa(2, 1) = 0;
		AT_fa(2, 2) = 0;
		wd(2) = 0;*/
		fres = force_alloc_iterative_slack(AT.transpose(), f_min, f_max, f_ref, f_prev, wd);
		wa << AT * fres.f;
		//fres.f = f_ref * Eigen::Vector4d::Ones() - AT_pinv * (wd + AT * f_ref * Eigen::Vector4d::Ones());
		std::cout << "f: \n" << fres.f << std::endl;
		// Convert tensions to torque and set the torque on each motor
		T = (fres.f).cwiseProduct(r_d*(-1)*motor_signs);
		T += t0.cwiseProduct((-1)*motor_signs);
		set_all_motor_torques(handles, T);

		if (!fres.flag) {
			f_prev = fres.f;
		}

		
		p_log.push_back(Kp*e);
		i_log.push_back(wint);
		d_log.push_back(Kd*edot);
		q_log.push_back(q);
		qdot_log.push_back(qdot);
		fqdot_log.push_back(fqdot);
		qddot_log.push_back(qddot);
		qd_log.push_back(qd);
		wd_log.push_back(wd);
		wa_log.push_back(wa);
		vel_log.push_back(vel);
		fvel_log.push_back(fvel);
		fvel_m_rad_log.push_back(fvel_m_rad);
		vel_m_rad_log.push_back(vel_m_rad);
		f_log.push_back(fres.f);
		t0_log.push_back(t0);
		eint_log.push_back(eint);
		wint_log.push_back(wint);
		t_log.push_back(t_since_start_c);

		key_pressed = poll_keys2();
		if (key_pressed == 2) {
			break;
		}

		while (1) {
			auto t_loop_end = std::chrono::high_resolution_clock::now();
			auto t_loop = std::chrono::duration_cast<std::chrono::milliseconds>(t_loop_end - t_loop_start);
			loop_period = std::chrono::duration<double>(t_loop).count();
			if (loop_period >= 0.015) break;
		}
		
	}

	if (any_error) {
		set_all_motor_torques(handles, Eigen::Vector4d::Zero());
		std::cout << "Error encountered" << std::endl;
		int errors[4];
		read_all_driver_error_statuses(handles, errors);
		std::cout << "Setting all torques to zero" << std::endl;
	}
	else {
		std::cout << "Setting standard torque" << std::endl;
		set_all_motor_torques(handles, (0.3 * Eigen::Vector4d::Ones()).cwiseProduct((-1) * motor_signs));
	}
	
	std::ofstream logfile_q("logs/q.txt");
	std::copy(q_log.begin(), q_log.end(), std::ostream_iterator<Eigen::Vector3d>(logfile_q, "\n"));
	logfile_q.close();

	std::ofstream logfile_qd("logs/qd.txt");
	std::copy(qd_log.begin(), qd_log.end(), std::ostream_iterator<Eigen::Vector3d>(logfile_qd, "\n"));
	logfile_qd.close();
	
	std::ofstream logfile_qdot("logs/qdot.txt");
	std::copy(qdot_log.begin(), qdot_log.end(), std::ostream_iterator<Eigen::Vector3d>(logfile_qdot, "\n"));
	logfile_qdot.close();

	std::ofstream logfile_fqdot("logs/fqdot.txt");
	std::copy(fqdot_log.begin(), fqdot_log.end(), std::ostream_iterator<Eigen::Vector3d>(logfile_fqdot, "\n"));
	logfile_fqdot.close();

	std::ofstream logfile_qddot("logs/qddot.txt");
	std::copy(qddot_log.begin(), qddot_log.end(), std::ostream_iterator<Eigen::Vector3d>(logfile_qddot, "\n"));
	logfile_qddot.close();
	
	std::ofstream logfile_t("logs/t.txt");
	std::copy(t_log.begin(), t_log.end(), std::ostream_iterator<double>(logfile_t, "\n"));
	logfile_t.close();

	std::ofstream logfile_eint("logs/eint.txt");
	std::copy(eint_log.begin(), eint_log.end(), std::ostream_iterator<Eigen::Vector3d>(logfile_eint, "\n"));
	logfile_eint.close();
	
	std::ofstream logfile_pw("logs/pw.txt");
	std::copy(p_log.begin(), p_log.end(), std::ostream_iterator<Eigen::Vector3d>(logfile_pw, "\n"));
	logfile_pw.close();

	std::ofstream logfile_dw("logs/dw.txt");
	std::copy(d_log.begin(), d_log.end(), std::ostream_iterator<Eigen::Vector3d>(logfile_dw, "\n"));
	logfile_dw.close();

	std::ofstream logfile_wint("logs/wint.txt");
	std::copy(wint_log.begin(), wint_log.end(), std::ostream_iterator<Eigen::Vector3d>(logfile_wint, "\n"));
	logfile_wint.close();

	std::ofstream logfile_wd("logs/wd.txt");
	std::copy(wd_log.begin(), wd_log.end(), std::ostream_iterator<Eigen::Vector3d>(logfile_wd, "\n"));
	logfile_wd.close();

	std::ofstream logfile_wa("logs/wa.txt");
	std::copy(wa_log.begin(), wa_log.end(), std::ostream_iterator<Eigen::Vector3d>(logfile_wa, "\n"));
	logfile_wa.close();

	std::ofstream logfile_vel("logs/vel.txt");
	std::copy(vel_log.begin(), vel_log.end(), std::ostream_iterator<Eigen::Vector4d>(logfile_vel, "\n"));
	logfile_vel.close();

	std::ofstream logfile_fvel("logs/fvel.txt");
	std::copy(fvel_log.begin(), fvel_log.end(), std::ostream_iterator<Eigen::Vector4d>(logfile_fvel, "\n"));
	logfile_fvel.close();
	
	std::ofstream logfile_vel_m_rad("logs/vel_m_rad.txt");
	std::copy(vel_m_rad_log.begin(), vel_m_rad_log.end(), std::ostream_iterator<Eigen::Vector4d>(logfile_vel_m_rad, "\n"));
	logfile_vel_m_rad.close();

	std::ofstream logfile_fvel_m_rad("logs/fvel_m_rad.txt");
	std::copy(fvel_m_rad_log.begin(), fvel_m_rad_log.end(), std::ostream_iterator<Eigen::Vector4d>(logfile_fvel_m_rad, "\n"));
	logfile_fvel_m_rad.close();

	std::ofstream logfile_f("logs/f.txt");
	std::copy(f_log.begin(), f_log.end(), std::ostream_iterator<Eigen::Vector4d>(logfile_f, "\n"));
	logfile_f.close();

	std::ofstream logfile_t0("logs/t0.txt");
	std::copy(t0_log.begin(), t0_log.end(), std::ostream_iterator<Eigen::Vector4d>(logfile_t0, "\n"));
	logfile_t0.close();

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