#ifndef CDPR_PARAMS // include guard
#define CDPR_PARAMS
#include <Dense>
//#include <math.h>

#define PI 3.14159265358979323846

// Initializing General Robot Parameters

// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
// %              PARAMETERS THAT CAN BE CHANGED                %
// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
// Initial platform position
double x0 = -0.2f;
double y0_ = -0.2f; // Initial y - y0 is a built in gcc function
double theta0 = 0.0f;
double xd0 = 0.0f;
double yd0 = 0.0f;
double thetad0 = 0.0f;

double convmm2m = (double)pow(10, -3);

Eigen::Vector2d r0(x0,y0_);
Eigen::Vector3d q0(x0, y0_, theta0);
Eigen::Vector3d qd0(xd0, yd0, thetad0);
//r0 = [x0; y0];
//q0 = [r0; theta0];
//qd0 = [xd0;yd0; thetad0];

double h = 0.01f;       // Sampling time(can be overwritten in main_script)

// Spool Parameters
double Rs = 0.02f;                 // Spool radius
double P = 2.9f*convmm2m;          // Pitch of spool
//% d = % Horizontal distance from cable outlet of spool to pulley
//% h0 = % Vertical Height from spool to pulley
//% hs = sqrt(d ^ 2 + h0 ^ 2);    % Length of cable between spool and pulley(at x = 0, home position)

// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
// Physical parameters
double g = 9.81f;            // m / s ^ 2

// Mobile Platform parameters

// Lengths of mobile platform
double MP_len_x = 0.15f;                   // [m]
double MP_len_y = 0.05f;                   // [m]

// Dimension of the frame
double F_len_x = 1.46f;                    // [m]
double F_len_y = 1.f;                       // [m]

double d = 0.01f;                    // [m]
double V = MP_len_x*MP_len_y*d;     // [m^3]
double rho = 2710.f;                  // [kg/m^3]
//double mp = V*rho;                  // [kg]
double mp = 0.07f;                   // [kg] Målt med vekta til Solve
double Izz = (double)(1/12*mp*(pow(MP_len_x,2) + pow(MP_len_y,2)));       // [kgm^2]
double dtx = 0.0f;                   // Translational dampening coefficient in the x - direction
double dty = 0.0f;                   // Translational dampening coefficient in the y - direction
double dr  = 0.0f;                   // Rotational dampening coefficient about the z - axis

// Wrench due to gravity
Eigen::Vector3d tmp(0, -g, 0);
Eigen::Vector3d Wp = mp*tmp;

// Cable attachment points PULLEY(In INERTIA coordinates) CONSTANT
// SØLVE
double h_pb = 30.2f*convmm2m; // [m] - height of pillow block bearing
double a_pb = 127.f*convmm2m;  // [m] - length of pillow block bearing

double x = 0.f;
double y = 0.f;
Eigen::Vector2d r(x, y);
double theta = 0.f;
Eigen::Vector3d q(r(0), r(1), theta); // Generalized coordinates of platform
//double q = [r;theta]; 

double profile_side_length = 30.f*convmm2m; // [m]
double fspace_w = 1400.f*convmm2m;  // [m]
double fspace_h = 1000.f*convmm2m; // [m]

// length_drum_x = fspace_w / 2 + profile_side_length + a_pb / 2 - 1 * 1e-3;
Eigen::Vector4d length_drum_x(sqrt(pow(1.52f, 2) + pow(2.99f, 2)), 
							  sqrt(pow(2.61f, 2) + pow(1.83f, 2)),
	                          sqrt(pow(0.98f, 2) + pow(0.8f,  2)),
							  sqrt(pow(0.65f, 2) + pow(0.55f, 2)));

//
//[sqrt(1.52 ^ 2 + 2.99 ^ 2);
//sqrt(2.61 ^ 2 + 1.83 ^ 2);
//sqrt(0.98 ^ 2 + 0.8 ^ 2);
//sqrt(0.65 ^ 2 + 0.55 ^ 2)] * 10 ^ (-3);

double length_drum_y = profile_side_length / 2 + h_pb;
//length_drum_y = profile_side_length / 2 + h_pb;

Eigen::Vector2d dc1(-length_drum_x(0), -length_drum_y); // Centre of drum 1
Eigen::Vector2d dc2(-length_drum_x(1), -length_drum_y); // Centre of drum 2
Eigen::Vector2d dc3(-length_drum_x(2), -length_drum_y); // Centre of drum 3
Eigen::Vector2d dc4(-length_drum_x(3), -length_drum_y); // Centre of drum 4
//dc1 = [-length_drum_x(1); -length_drum_y];
//dc2 = [-length_drum_x(2);  length_drum_y];
//dc3 = [length_drum_x(3);  length_drum_y];
//dc4 = [length_drum_x(4); -length_drum_y];

double beta_e = 0.15194;
double r_d = 20*convmm2m;
// beta_e * 180 / pi

Eigen::Vector2d de1(r_d*cos(PI + beta_e),   r_d*sin(PI + beta_e));
Eigen::Vector2d de2(r_d*cos(PI - beta_e),   r_d*sin(PI - beta_e));
Eigen::Vector2d de3(r_d*cos(beta_e),        r_d*sin(beta_e));
Eigen::Vector2d de4(r_d*cos(2*PI - beta_e), r_d*sin(2*PI - beta_e));

//de1 = dc1 + [r_d*cos(PI + beta_e);   r_d*sin(PI + beta_e)];
//de2 = dc2 + [r_d*cos(PI - beta_e);   r_d*sin(PI - beta_e)];
//de3 = dc3 + [r_d*cos(beta_e);      r_d*sin(beta_e)];
//de4 = dc4 + [r_d*cos(2 * PI - beta_e); r_d*sin(2 * PI - beta_e)];

double r_p = 12*convmm2m;
double angle_profile_side_length = 40*convmm2m; // [m]
double pulley_mounting_hole_offset = 4*convmm2m;
Eigen::Vector2d pulley_mh_to_wh = Eigen::Vector2d(26.4, 41.9)*convmm2m; // Length between the pulley assembly
																		// mounting hole to the pulley wheel hole
double length_pulley_x = fspace_w/2 + profile_side_length +
                        angle_profile_side_length        -
                        pulley_mounting_hole_offset      -
                        pulley_mh_to_wh(0);
double length_pulley_y = fspace_h / 2 - angle_profile_side_length - pulley_mh_to_wh(1);

Eigen::Vector2d pc1(-length_pulley_x, -length_pulley_y);
Eigen::Vector2d pc2(-length_pulley_x,  length_pulley_y);
Eigen::Vector2d pc3( length_pulley_x,  length_pulley_y);
Eigen::Vector2d pc4( length_pulley_x, -length_pulley_y);

/////////////////////////////////////////////////////////////////////////////////////////////
// Fix later
/////////////////////////////////////////////////////////////////////////////////////////////
//Eigen::MatrixXd b(2, 4);
////b(1) = 1;
//b << 1;
////<< 2, 3,
////1, 4;
//Eigen::VectorXf pc(pc1(0), pc1(1), pc2(0), pc2(0), pc3, pc4);
//pc1 = [-length_pulley_x; -length_pulley_y];
//pc2 = [-length_pulley_x;  length_pulley_y - 30 * 1e-3];
//pc3 = [length_pulley_x;  length_pulley_y - 30 * 1e-3];
//pc4 = [length_pulley_x; -length_pulley_y];
//pc = [pc1 pc2 pc3 pc4];
//
//pe1 = pc1 + [r_p*cos(pi + beta_e);   r_p*sin(pi + beta_e)];
//pe2 = pc2 + [r_p*cos(pi - beta_e);   r_p*sin(pi - beta_e)];
//pe3 = pc3 + [r_p*cos(beta_e);      r_p*sin(beta_e)];
//pe4 = pc4 + [r_p*cos(2 * pi - beta_e); r_p*sin(2 * pi - beta_e)];
//
//a1 = pc1 - [r_p; 0];
//a2 = pc2 - [r_p; 0];
//a3 = pc3 + [r_p; 0];
//a4 = pc4 + [r_p; 0];

/////////////////////////////////////////////////////////////////////////////////////////////
// Fix later
/////////////////////////////////////////////////////////////////////////////////////////////

// a = [a1 a2 a3 a4];
Eigen::MatrixXd a(2,4);
//a = [-0.7560 - 0.7560    0.7560    0.7560;
//     -0.4181    0.4181    0.4181 - 0.4181];
//
//% a = [-0.7516 - 0.7516    0.7516    0.7516;
//%       -0.4181    0.4181    0.4181 - 0.4181];

//% a = [a1 a2 a3 a4];
//
//% a = [0.2 - 0.2; 0 0]; % 1 DoF test rigg
//% b = [0 0; 0 0];

//% Cable attachment point PLATFORM
//
//% % RECTANGLE FOR TESTRIGG4
//% b1 = [-30 + 4.12;-10.24] * 1e-3;
//% b2 = [-30 + 4.36;11.24] * 1e-3;
//% b3 = [30 - 4.52;11.24] * 1e-3;
//% b4 = [30 - 4.54;-10.24] * 1e-3;
//% b_rectangle = [b1 b2 b3 b4];




//
//% RECTANGLE
//b1 = [-MP_len_x / 2;-MP_len_y / 2];
//b2 = [-MP_len_x / 2;MP_len_y / 2];
//b3 = [MP_len_x / 2;MP_len_y / 2];
//b4 = [MP_len_x / 2;-MP_len_y / 2];
//b_rectangle = [b1 b2 b3 b4];
//
//% TRIANGLE
//b1 = [0;-MP_len_y / 2];
//b2 = [-MP_len_x / 2;MP_len_y / 2];
//b3 = [MP_len_x / 2;MP_len_y / 2];
//b4 = b1;
//b_triangle = [b1 b2 b3 b4];

// TRAPEZOIDAL
Eigen::MatrixXd b(2, 4);
//b_trapez = [-0.0250 - 0.0750    0.0750    0.0250;
//-0.0100    0.0100    0.0100 - 0.0100];


//%% Discrete Linear Model
//A_c = [0  0 0  1       0       0; %
//0  0 0  0       1       0;
//0  0 0  0       0       1;
//0  0 0 - dtx / mp  0       0;
//0  0 0  0 - dty / mp  0;
//0  0 0  0       0 - dr / Izz];

//B_c = [0    0    0;
//0    0    0;
//0    0    0;
//1 / mp 0    0;
//0    1 / mp 0;
//0    0    1 / Izz;];
//d_c = [0;
//0;
//0;
//0;
//-g;
//0];

//% Augmented Model for Integral States
//A_c_aug = [A_c    zeros(6, 6);
//-eye(6) zeros(6, 6)];
//B_c_aug = [B_c; zeros(6, 3)];
//d_c_aug = [d_c; zeros(6, 1)];
//zeroI = [zeros(6, 6);eye(6)];
//
//K_d = B_c\eye(6);
//K_f = [180 * diag([1, 1, 1]) 10 * diag([1, 1, 1])];
//K_r = pinv((B_c*K_f - A_c)\B_c);
//K_a = [diag([-10, -10, -1]) zeros(3, 3)];

// Motorsign(CHANGE IF NEEDED)
double motor_sign0 = -1;
double motor_sign1 =  1;
double motor_sign2 = -1;
double motor_sign3 =  1;

Eigen::Vector4d motor_signs(motor_sign0, motor_sign1, motor_sign2, motor_sign3);
//motorsigns = [motorsign0;motorsign1;motorsign2;motorsign3];



// Structs

//% Standard Geometric Model Params
//CDPR_BodyAnchorPoints = struct("RECTANGLE", b_rectangle, ...
//	"TRIANGLE", b_triangle, ...
//	"TRAPEZOID", b_trapez, ...
//	"TEST", b_rectangle);
//CDPR_SGM = struct("FrameAP", a, ...
//	"BodyAP", CDPR_BodyAnchorPoints);
//
//% Spool Params
//CDPR_SpoolParams = struct("SPOOL_RADIUS", Rs, ...
//	"PITCH", P);%, ...
//	%"SPOOL_PULLEY_LENGTH", hs);
//
//	% Control Params
//		CDPR_ControlParams = struct("K_d", K_d, ...
//			"K_f", K_f, ...
//			"K_r", K_r, ...
//			"K_a", K_a, ...
//			"d_c", d_c);
//
//	% System Matrices
//		CDPR_SystemMatrices = struct("A_c_aug", A_c_aug, ...
//			"B_c_aug", B_c_aug, ...
//			"d_c_aug", d_c_aug, ...
//			"zeroI", zeroI);
//
//	% General Parameters
//		CDPR_GenParams = struct("SAMPLING_TIME", h, ...
//			"MASS_PLATFORM", mp, ...
//			"MOTOR_SIGNS", motorsigns, ...
//			"SpoolParams", CDPR_SpoolParams);
//	% More ?
//
//
//
//		% Combine all structs into a master struct
//		CDPR_Params = struct("SGM", CDPR_SGM, ...
//			"ControlParams", CDPR_ControlParams, ...
//			"SystemMatrices", CDPR_SystemMatrices, ...
//			"Gen_Params", CDPR_GenParams);


#endif