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


double friction_compensation(uint8_t odrive_number, double v) {
	double vl = 0.5;
	double f = 0;
	Eigen::Vector3d c0(0.00118181818181995, -0.00205151515152902, 0.0587848484848724);
	Eigen::Vector3d c1(-0.000606060606041070, 0.00579393939384920, 0.0754727272727929);
	Eigen::Vector3d c1in(-0.000242424242411388, 0.00422424242418717, 0.0589757575757854);
	Eigen::Vector3d c1out(-0.00103030303029028, 0.00867636363630303, 0.0939503030303555);
	Eigen::Vector3d c2(3.03030303101526 * pow(10, -5), 0.00346363636360586, 0.0467696969697113);
	Eigen::Vector3d c3(-0.000727272727265027, 0.00773939393935047, 0.0427939393939897);

	Eigen::Vector4d b0(0.175189393939453, -0.00118181818181998, -0.236321212121310, 0.00472727272727985);
	Eigen::Vector4d b1(0.235559090909163, 0.146424848484926, -0.306003636363810, -0.289670303030437); // Asymmetric
	Eigen::Vector4d b2(0.143795454545473, -3.03030303101526 * pow(10, -5), -0.187109090909156, 0.000121212121240610);
	Eigen::Vector4d b3(0.135575757575871, 0.000727272727265027, -0.170448484848694, -0.00290909090906011);

	switch (odrive_number) {
	case 0:
		if (v >= vl) {
			f = c0(0) * v * v + c0(1) * v + c0(2);
		}
		else if (v < -vl) {
			f = -(c0(0) * abs(v) * abs(v) + c0(1) * abs(v) + c0(2));
		}
		else if ((-vl <= v) && (v < vl)) {
			f = b0(0) * v + b0(1) * v * v + b0(2) * v * v * v + b0(3) * v * v * v * v;
		}
		break;
	case 1:
		if (v >= vl) {
			f = c1out(0) * v * v + c1out(1) * v + c1out(2);
		}
		else if (v < -vl) {
			f = -(c1in(0) * abs(v) * abs(v) + c1in(1) * abs(v) + c1in(2));
		}
		else if ((-vl <= v) && (v < vl)) {
			f = b1(0) * v + b1(1) * v * v + b1(2) * v * v * v + b1(3) * v * v * v * v;
		}
		break;
	case 2:
		if (v >= vl) {
			f = c2(0) * v * v + c2(1) * v + c2(2);
		}
		else if (v < -vl) {
			f = -(c2(0) * abs(v) * abs(v) + c2(1) * abs(v) + c2(2));
		}
		else if ((-vl <= v) && (v < vl)) {
			f = b2(0) * v + b2(1) * v * v + b2(2) * v * v * v + b2(3) * v * v * v * v;
		}
		break;
	case 3:
		if (v >= vl) {
			f = c3(0) * v * v + c3(1) * v + c3(2);
		}
		else if (v < -vl) {
			f = -(c3(0) * abs(v) * abs(v) + c3(1) * abs(v) + c3(2));
		}
		else if ((-vl <= v) && (v < vl)) {
			f = b3(0) * v + b3(1) * v * v + b3(2) * v * v * v + b3(3) * v * v * v * v;
		}
		break;
	default:
		f = 0;
		break;
	}
	return f;
}


Eigen::Vector4d get_positions_with_offset(const Eigen::Ref<const Eigen::Vector4d>& p_enc, 
										  const Eigen::Ref<const Eigen::Vector4d>& p0) {
	Eigen::Vector4d pos = Eigen::Vector4d::Zero();
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

int poll_keys() {
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
		r = poll_keys();
		if (r == 4) return 0;
	}
	Sleep(250);

	std::cout << "Setting starting torques" << std::endl;
	set_standard_tension(handles, motor_signs);

	std::cout << "Set home position?" << std::endl;
	r = 0;
	while (!(r == 3)) {
		r = poll_keys();
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
		r = poll_keys();
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
		r = poll_keys();
		if (r == 4) return 0;
	}
	Sleep(250);
	return 1;
}

Eigen::Vector4d calculate_ts(const Eigen::Ref<const Eigen::Vector4d>& vels,
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

	Eigen::Vector4d ts;
	//fs << (int)f_pinvp.any()*velp.cwiseProduct(dir.cwiseProduct(f_static));
	ts << (int)ep.any() * velp.cwiseProduct(f_static);
	return ts;
}

int tension_control_loop(HANDLE handles[], const Eigen::Ref<const Eigen::Vector4d>& p0_in) {

	// Vectors for logging data values
	std::vector<Eigen::Vector3d> p_log, i_log, d_log;			// Proportional, integral, and derivative terms in controller
	std::vector<Eigen::Vector3d> q_log;							// Generalized coordinates q
	std::vector<Eigen::Vector3d> qdot_log, fqdot_log;			// Derivative of q and its filtered version
	std::vector<Eigen::Vector3d> qd_log, qddot_log;				// Derivative of q desired and its filtered version
	std::vector<Eigen::Vector3d> wd_log;						// Desired wrench
	std::vector<Eigen::Vector3d> wa_log;
	std::vector<Eigen::Vector3d> eint_log;						// Accumulated positional error
	std::vector<Eigen::Vector3d> wint_log;						// Integral term in controller (Duplicate)
	std::vector<Eigen::Vector4d> vel_log, fvel_log;             // Motor velocity in rev/s and its filtered version
	std::vector<Eigen::Vector4d> vel_m_rad_log, fvel_m_rad_log; // Motor velocity in rad/s and its filtered version (with correct signs)
	std::vector<Eigen::Vector4d> f_log;                         // Cable forces
	std::vector<Eigen::Vector4d> t0_log;					    // Impulse torque
	std::vector<Eigen::Vector4d> l_log;                         // Cable lengths
	std::vector<Eigen::Vector4d> ldot_log;                      // Derivative of cable lengths
	std::vector<Eigen::Vector4d> lfk_log;                       // Cable lengths sent to forward kinematics algorithm
	std::vector<double> t_log;								    // Time since starting control loop
	std::vector<double> sample_time_log;                        // Period of each control loop
	std::vector<double> real_sample_time_log;




	double mm2m = (double)pow(10, -3);
	double r_d = 20*mm2m;         // Effective radius of drums
	double r_p = 12*mm2m;         // Effective radius of pulleys
	double pitch_drum = 2.9*mm2m; // Pitch of drums
	
	// Proximal anchor points
	Eigen::MatrixXd a(2, 4);
	a << -0.7510,   -0.7510,    0.7510,    0.7510,
         -0.4181,    0.4181,    0.4181,   -0.4181;

	// Distal anchor points for trapezoidal mobile platform
	Eigen::MatrixXd b(2, 4);
	b << -0.0250, -0.0750, 0.0750, 0.0250,
		 -0.0100, 0.0100, 0.0100, -0.0100;

	Eigen::Vector4d lde2pe(0.359809755287430,
						   0.363766765936637,
						   0.361788142425923,
						   0.361788142425923); // Length from drum exit to pulley entry

	// Multipliers for getting correct signs on velocity and torque on motors
	Eigen::Vector4d motor_signs(-1, 
								 1,	
		                        -1, 
								 1);

	Eigen::Vector4d p0;                                         // Starting encoder positions. Assumed to be captured when 
																// the mobile platform is in the center with an angle of zero
	Eigen::Vector4d pos_enc, pos;                               // Raw motor positions from encoder and processed motor positions (pos is offset from p0)
	Eigen::Vector4d pos_rad;                                    // Motor positions in radians
	
	Eigen::Vector4d vel, vel_rad;                               // Motor velocities in rev/s and rad/s respectively
	Eigen::Vector4d fvel = Eigen::Vector4d::Zero();             // Filtered motor velocities
	Eigen::Vector4d vel_m, vel_m_rad = Eigen::Vector4d::Zero(); // Motor velocities in rev/s and rad/s respectively (with correct signs)
	Eigen::Vector4d fvel_m_rad       = Eigen::Vector4d::Zero(); // Filtered motor velocities with correct sign in rad/s

	Eigen::Vector4d l, ldot, lfk;                               // Cable lengths, the derivative of cable lengths, 
																// and cable lengths used by forward kinematics algorithm

	// EMA filter values for states
	Eigen::Vector3d alpha3(0.4, 0.4, 0.3);

	Eigen::Vector4d l0(	1.196549826581494,
						1.172604084137631,
						1.173956813721921,
						1.202012667688769); // Starting cable lengths assuming the mobile platform 
											// is in the center with an angle of zero

	// Timers
	double t_since_start_c = 0;
	double loop_period = 0;
	double tdiffhalf_c = 0.0;
	double tafterstuff_c = 0.0;


	inv_res invkin; // Inverse kinematics result
	
	// Creating vector of states of each motor - i.e. position and velocity
	motor_state ms0, ms1, ms2, ms3;
	std::vector<motor_state*> motor_states;
	motor_states.push_back(&ms0);
	motor_states.push_back(&ms1);
	motor_states.push_back(&ms2);
	motor_states.push_back(&ms3);

	Eigen::MatrixXd AT, AT_pinv, A_pinv = Eigen::MatrixXd::Zero(3, 4); // Structure matrix, pseudo-inverse of structure matrix, 
																	   // and pseudo inverse of the transpose of the structure matrix
	// Force allocation parameters
	double f_min = 10;										// Minimum cable force for force allocation
	double f_max = 80;										// Maximum cable force for force allocation
	double f_ref = (f_max + f_min) / 2;                     // Reference cable force for force allocation
	Eigen::Vector4d f_prev = f_ref*Eigen::Vector4d::Ones(); // Previous cable forces
	force_alloc_res fres;                                   // Force allocation results

	
	Eigen::Vector4d f_static(12.58099,
							 3.5862874,
							 6.7821798,
							 3.52063746);
	Eigen::Vector4d t_static(0.04, 
							 0.04,
							 0.03,
							 0.03);
	double precv = 2;
	double precx = 3;
	double precy = 3;
	double prect = 0;

	Eigen::Vector4d f_pinv = Eigen::Vector4d::Zero(); // Force allocation not taking f_min and f_max into account
	Eigen::Vector4d ts = Eigen::Vector4d::Zero();     // Static torque required to get motors running from a standstill. Used for generating t0
	Eigen::Vector4d t0 = Eigen::Vector4d::Zero();     // Impulse torque
	

	// Controller parameters
	Eigen::Vector3d q, qprev, qdot, fq = Eigen::Vector3d::Zero();
	q     << 0, 0, 0; // Generalized coordinates - q = [x, y, psi]^T
	qprev << 0, 0, 0;
	Eigen::Vector3d qd, qddot, fqdot = Eigen::Vector3d::Zero();
	double omega = 3;
	double Lx = 0.10;
	double Ly = 0.10;
	Eigen::Vector3d e, edot, eint = Eigen::Vector3d::Zero();    // Errors
	Eigen::Vector3d wd, wa = Eigen::Vector3d::Zero();			// Desired wrench, actual wrench
	Eigen::Vector3d wint = Eigen::Vector3d::Zero();             // Wrench generated by integral term of controller
	Eigen::Vector3d intlimit(6, 6, 0.1);                        // Saturation limits on integral term of controller
	Eigen::Vector4d T = Eigen::Vector4d::Zero();				// Torque applied to each motor
	Eigen::Vector4d T_friction_model = Eigen::Vector4d::Zero(); // Torque generated from friction model
	double dt = 35*pow(10,-3);                                  // Control loop period

	Eigen::Matrix3d Kp = Eigen::Matrix3d::Zero();
	Kp(0, 0) = 225; // Proportional gain x
	Kp(1, 1) = 125; // Proportional gain y
	Kp(2, 2) = 5;   // Proportional gain psi

	Eigen::Matrix3d Ki = Eigen::Matrix3d::Zero();
	Ki(0, 0) = 425; // Integral gain x
	Ki(1, 1) = 250; // Integral gain y
	Ki(2, 2) = 0;   // Integral gain psi

	Eigen::Matrix3d Kd = Eigen::Matrix3d::Zero();
	Kd(0, 0) = 15;   // Derivative gain x
	Kd(1, 1) = 21;   // Derivative gain y
	Kd(2, 2) = 0.0;  // Derivative gain psi

	int key_pressed = 0;




	std::cout << "If home position is not set, move MP to centre then enter any letter" << std::endl;
	std::string input;
	std::cin >> input;
	get_all_motor_states(handles, motor_states);
	if (p0_in.any()) {
		p0 << p0_in;
	} else {
		std::cout << "Move mobile platform to center and enter any letter/number." << std::endl;
		std::string input;
		std::cin >> input;
		p0 << ms0.pos,
			  ms1.pos,
			  ms2.pos,
			  ms3.pos;
	}
	std::cout << "p0: \n" << p0 << std::endl;


	bool running = 1;
	bool any_error = 0;
	auto t_start = std::chrono::high_resolution_clock::now();
	while (running) {

		auto t_loop = std::chrono::high_resolution_clock::now();

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

		vel << ms0.vel,
			   ms1.vel,
			   ms2.vel,
			   ms3.vel;
		
		// EMA filter
		double alpha = 1;
		fvel = (1-alpha)* fvel + alpha * vel;

		vel_rad << vel * 2 * PI;
		vel_m << vel.cwiseProduct(motor_signs);
		vel_m_rad << vel_rad.cwiseProduct(motor_signs);
		
		// EMA filter
		double alpha2 = 1;
		fvel_m_rad << (1-alpha2) * fvel_m_rad + alpha2 * vel_m_rad;


		l << l0 + pos_rad.cwiseProduct(r_d*motor_signs);

		ldot << r_d * fvel_m_rad;

		// Subtract length between drum and pulley from total cable length to get length used in FK
		lfk << l(0) - sqrt( pow(pos(0)*pitch_drum, 2) + pow(lde2pe(0), 2)),
			   l(1) - sqrt( pow(pos(1)*pitch_drum, 2) + pow(lde2pe(1), 2)),
			   l(2) - sqrt( pow(pos(2)*pitch_drum, 2) + pow(lde2pe(2), 2)),
			   l(3) - sqrt( pow(pos(3)*pitch_drum, 2) + pow(lde2pe(3), 2));
		//std::cout << "lfk: \n" << lfk << std::endl;

		qprev << q; // Using previous q as initial estimate of pose. Can also use fk_init_estimate().
		q = forward_kinematics(a, b, 
							   qprev,
							   lfk, r_p);
		auto thalf = std::chrono::high_resolution_clock::now();
		auto tdiffhalf = std::chrono::duration_cast<std::chrono::milliseconds>(thalf - t_loop_start);
		tdiffhalf_c = std::chrono::duration<double>(tdiffhalf).count();
		//std::cout << "Time before forward: " << tdiffhalf_c << std::endl;
		
		std::cout << "q: " << q(0)*1000 << ", " << q(1) * 1000 << ", " << q(2)*180/PI << std::endl;
		invkin = inverse_kinematics(a, b, q, r_p);

		AT      = calculate_structure_matrix(a, b, q, invkin.betar, r_p);
		AT_pinv = AT.completeOrthogonalDecomposition().pseudoInverse();                   // Might exist better decomposition alternatives, not explored
		A_pinv  = (-1*AT).transpose().completeOrthogonalDecomposition().pseudoInverse();  // Might exist better decomposition alternatives, not explored
		
		
		qdot = A_pinv*ldot;
		fqdot <<	alpha3(0) * qdot(0) + (1 - alpha3(0)) * fqdot(0), 
					alpha3(1) * qdot(1) + (1 - alpha3(1)) * fqdot(1), 
					alpha3(2) * qdot(2) + (1 - alpha3(2)) * fqdot(2);

		t_since_start_c = std::chrono::duration<double>(t_since_start).count();
		std::cout << "Time since start: " << t_since_start_c << std::endl;
		
		// Desired trajectory generation for a circular path
		qd << Lx*sin(omega*t_since_start_c),
			  Ly*cos(omega*t_since_start_c)-0.2,
			  0;
		qddot <<  omega*Lx*cos(omega*t_since_start_c),
			     -omega*Ly*sin(omega*t_since_start_c),
			      0;
		
		// Step respones
		//qd << 0.0, -0.05, 0.0;
		//qddot << 0, 0, 0;

		// Desired trajectory generation for infinity path
		//qd << 0.15 * sin(omega * t_since_start_c),
		//	  0.1*cos(omega*t_since_start_c)*sin(omega*t_since_start_c) - 0.2,
		//	  0;
		////std::cout << "qd:\n" << qd << std::endl;
		//
		//qddot << -0.15*omega*sin(omega*t_since_start_c),
		//		 -0.1*omega*cos(2*omega*t_since_start_c),
		//		  0;

		// Controller
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
		wd << Kp * e + Kd*edot + wint;


		f_pinv << AT_pinv * wd;
		ts = calculate_ts(fvel, e, t_static, precv, precx, precy, prect);
		t0 << sgn(f_pinv(0))*(ts(0)),
			  sgn(f_pinv(1))*(ts(1)),
			  sgn(f_pinv(2))*(ts(2)),
			  sgn(f_pinv(3))*(ts(3));
		//std::cout << "t0: \n" << t0 << std::endl;
		
		fres = force_alloc_iterative_slack(AT.transpose(), f_min, f_max, f_ref, f_prev, wd);
		wa << AT * fres.f; // Actual wrench resulting from force allocation

		// Convert tensions to torque and set the torque on each motor
		for (int i = 0; i < 4; i++) {
			T_friction_model(i) = friction_compensation(i, vel(i));
		}

		std::cout << "qd: " << qd(0) * 1000 << ", " << qd(1) * 1000 << ", " << qd(2) * 180 / PI << std::endl;
		std::cout << "qddot: " << qddot(0) * 1000 << ", " << qddot(1) * 1000 << ", " << qddot(2) * 180 / PI << std::endl;

		T = (fres.f).cwiseProduct(r_d*(-1)*motor_signs);
		T += t0.cwiseProduct((-1)*motor_signs);
		//T += T_friction_model;
		set_all_motor_torques(handles, T);

		if (!fres.flag) {
			f_prev = fres.f;
		}
		auto tafterstuff = std::chrono::high_resolution_clock::now();
		auto tdiffafterstuff = std::chrono::duration_cast<std::chrono::milliseconds>(tafterstuff - t_loop_start);
		tafterstuff_c = std::chrono::duration<double>(tdiffafterstuff).count();
		//std::cout << "Time after stuff: " << tafterstuff_c << std::endl;
		


		key_pressed = poll_keys();
		if (key_pressed == 2) {
			break;
		}

		// Ensure that each loop is not smaller than dt
		while (1) {
			auto t_loop_end = std::chrono::high_resolution_clock::now();
			auto t_loop = std::chrono::duration_cast<std::chrono::milliseconds>(t_loop_end - t_loop_start);
			loop_period = std::chrono::duration<double>(t_loop).count();
			if (loop_period >= dt) break;
		}

		
		std::cout << "Time half: " << tdiffhalf_c << std::endl;
		std::cout << "Time after stuff: " << tafterstuff_c << std::endl;
		std::cout << "Time loop: " << loop_period << std::endl;
		sample_time_log.push_back(loop_period);
		real_sample_time_log.push_back(tafterstuff_c);

		// Log data in vectors
		p_log.push_back(Kp * e);
		i_log.push_back(wint);
		d_log.push_back(Kd * edot);
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
		ldot_log.push_back(ldot);
		l_log.push_back(l);
		lfk_log.push_back(lfk);
		
	}

	// Check whether the control loop terminated because of an error
	if (any_error) {
		// If an error occured, set motor torques to zero such that the mobile platform remains under control
		set_all_motor_torques(handles, Eigen::Vector4d::Zero());
		std::cout << "Error encountered" << std::endl;
		std::cout << "Setting all torques to zero" << std::endl;
		int errors[4];
		read_all_driver_error_statuses(handles, errors);
	}
	else {
		std::cout << "Setting standard torque" << std::endl;
		set_all_motor_torques(handles, (0.15 * Eigen::Vector4d::Ones()).cwiseProduct((-1) * motor_signs));
	}
	
	// Log all data values to text files
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
	
	std::ofstream logfile_l("logs/l.txt");
	std::copy(l_log.begin(), l_log.end(), std::ostream_iterator<Eigen::Vector4d>(logfile_l, "\n"));
	logfile_l.close();

	std::ofstream logfile_ldot("logs/ldot.txt");
	std::copy(ldot_log.begin(), ldot_log.end(), std::ostream_iterator<Eigen::Vector4d>(logfile_ldot, "\n"));
	logfile_ldot.close();

	std::ofstream logfile_lfk("logs/lfk.txt");
	std::copy(lfk_log.begin(), lfk_log.end(), std::ostream_iterator<Eigen::Vector4d>(logfile_lfk, "\n"));
	logfile_lfk.close();

	std::ofstream logfile_sample_time_log("logs/t_loop.txt");
	std::copy(sample_time_log.begin(), sample_time_log.end(), std::ostream_iterator<double>(logfile_sample_time_log, "\n"));
	logfile_sample_time_log.close();

	std::ofstream logile_real_sample_time_log("logs/real_t_loop.txt");
	std::copy(real_sample_time_log.begin(), real_sample_time_log.end(), std::ostream_iterator<double>(logile_real_sample_time_log, "\n"));
	logile_real_sample_time_log.close();

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