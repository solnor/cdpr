#include "algorithms.h"
#include <iostream>

inv_res inverse_kinematics(const Eigen::Ref<const Eigen::MatrixXd>& a_i,
	const Eigen::Ref<const Eigen::MatrixXd>& b_b,
	const Eigen::Ref<const Eigen::Vector3d>& q,
	double r_p
) {
	double x = q(0);
	double y = q(1);
	double theta = q(2);
	Eigen::Vector2d r(x, y);
	Eigen::Matrix2d R;
	R << cos(theta), -sin(theta),
		 sin(theta),  cos(theta);

	Eigen::Matrix2d R1, R2, R3, R4;
	R1 << 1, 0,
		0, cos(PI);
	R2 << 1, 0,
		0, 1;
	R3 << cos(PI), 0,
		0, cos(0);
	R4 << cos(PI), 0,
		0, cos(PI);

	const Eigen::Vector2d b1p = R1 * (R*b_b(Eigen::all, 0) + r - a_i(Eigen::all, 0));
	const Eigen::Vector2d b2p = R2 * (R*b_b(Eigen::all, 1) + r - a_i(Eigen::all, 1));
	const Eigen::Vector2d b3p = R3 * (R*b_b(Eigen::all, 2) + r - a_i(Eigen::all, 2));
	const Eigen::Vector2d b4p = R4 * (R*b_b(Eigen::all, 3) + r - a_i(Eigen::all, 3));

	Eigen::MatrixXd bp(2, 4);
	bp << b1p, b2p, b3p, b4p;

	double lf1, lf2, lf3, lf4;
	lf1 = sqrt(pow(bp(0, 0), 2) - 2 * bp(0, 0)*r_p + pow(bp(1, 0), 2));
	lf2 = sqrt(pow(bp(0, 1), 2) - 2 * bp(0, 1)*r_p + pow(bp(1, 1), 2));
	lf3 = sqrt(pow(bp(0, 2), 2) - 2 * bp(0, 2)*r_p + pow(bp(1, 2), 2));
	lf4 = sqrt(pow(bp(0, 3), 2) - 2 * bp(0, 3)*r_p + pow(bp(1, 3), 2));

	Eigen::Vector4d lf;
	lf << lf1, lf2, lf3, lf4;
	
	double mb1 = sqrt(pow((bp(0, 0) - r_p), 2) + pow(bp(1, 0), 2));
	double mb2 = sqrt(pow((bp(0, 1) - r_p), 2) + pow(bp(1, 1), 2));
	double mb3 = sqrt(pow((bp(0, 2) - r_p), 2) + pow(bp(1, 2), 2));
	double mb4 = sqrt(pow((bp(0, 3) - r_p), 2) + pow(bp(1, 3), 2));

	// beta_1
	double alpha1 = acos(lf1 / mb1);
	double alpha2 = acos(lf2 / mb2);
	double alpha3 = acos(lf3 / mb3);
	double alpha4 = acos(lf4 / mb4);

	// beta_2
	double gamma1 = acos(bp(1, 0) / mb1);
	double gamma2 = acos(bp(1, 1) / mb2);
	double gamma3 = acos(bp(1, 2) / mb3);
	double gamma4 = acos(bp(1, 3) / mb4);

	double beta1r = alpha1 + gamma1;
	double beta2r = alpha2 + gamma2;
	double beta3r = alpha3 + gamma3;
	double beta4r = alpha4 + gamma4;
	
	inv_res res;
	res.l = lf;
	res.betar = Eigen::Vector4d(beta1r, beta2r, beta3r, beta4r);
	return res;
}

Eigen::Vector3d fk_init_estimate(const Eigen::Ref<const Eigen::MatrixXd>& a_i,
	const Eigen::Ref<const Eigen::MatrixXd>& b_b,
	const Eigen::Ref<const Eigen::Vector4d>& ln
) {
	uint8_t m = 4;
	Eigen::MatrixXd rlowi(2, m);
	Eigen::MatrixXd rhighi(2, m);

	for (uint8_t i = 0; i < m; i++) {
		rlowi(Eigen::all, i) = \
			a_i(Eigen::all, i) - (ln(i) + b_b(Eigen::all,i).norm())*Eigen::Vector2d(1,1);
		rhighi(Eigen::all, i) = \
			a_i(Eigen::all, i) + (ln(i) + b_b(Eigen::all, i).norm())*Eigen::Vector2d(1, 1);
	}

	Eigen::Vector2d rlow(2, m);
	Eigen::Vector2d rhigh(2, m);
	rlow  << rlowi.rowwise().maxCoeff();
	rhigh << rhighi.rowwise().minCoeff();

	Eigen::Vector2d r0(0.5*(rlow(0) + rhigh(0)),
		               0.5*(rlow(1) + rhigh(1)));
	
	Eigen::Vector3d q0(0, 0, 0);
	q0(Eigen::seq(0, 1)) = r0;

	return q0;
}


Eigen::MatrixXd fk_jacobian(const Eigen::Ref<const Eigen::MatrixXd>& a_i,
							const Eigen::Ref<const Eigen::MatrixXd>& b_b,
							const Eigen::Ref<const Eigen::Vector3d>& q,
							const Eigen::Ref<const Eigen::Vector4d>& ln,
							double r_p) {
	double x     = q(0);
	double y     = q(1);
	double theta = q(2);

	double c = cos(theta);
	double s = sin(theta);

	double a1x, a1y;
	double a2x, a2y;
	double a3x, a3y;
	double a4x, a4y;
	
	a1x = a_i(0, 0);
	a1y = a_i(1, 0);

	a2x = a_i(0, 1);
	a2y = a_i(1, 1);
	
	a3x = a_i(0, 2);
	a3y = a_i(1, 2);

	a4x = a_i(0, 3);
	a4y = a_i(1, 3);

	double b1x, b1y;
	double b2x, b2y;
	double b3x, b3y;
	double b4x, b4y;

	b1x = b_b(0, 0);
	b1y = b_b(1, 0);

	b2x = b_b(0, 1);
	b2y = b_b(1, 1);

	b3x = b_b(0, 2);
	b3y = b_b(1, 2);

	b4x = b_b(0, 3);
	b4y = b_b(1, 3);

	double l1 = ln(0);
	double l2 = ln(1);
	double l3 = ln(2);
	double l4 = ln(3);

	double J1x, J1y, J1theta;
	double J2x, J2y, J2theta;
	double J3x, J3y, J3theta;
	double J4x, J4y, J4theta;
	
	J1x = (r_p*((1.0/pow(pow(-a1y+y+b1y*c+b1x*s,2.0)+pow(a1x+r_p-x-b1x*c+b1y*s,2.0),3.0/2.0)*1.0/sqrt(-pow(-a1y+y+b1y*c+b1x*s,2.0)/(pow(-a1y+y+b1y*c+b1x*s,2.0)+pow(a1x+r_p-x-b1x*c+b1y*s,2.0))+1.0)*(-a1y+y+b1y*c+b1x*s)*(a1x*2.0+r_p*2.0-x*2.0-b1x*c*2.0+b1y*s*2.0))/2.0+(r_p*r_p)*1.0/sqrt(-(r_p*(a1x*2.0-x*2.0-b1x*c*2.0+b1y*s*2.0)+pow(a1x-x-b1x*c+b1y*s,2.0)+pow(-a1y+y+b1y*c+b1x*s,2.0))/(pow(-a1y+y+b1y*c+b1x*s,2.0)+pow(a1x+r_p-x-b1x*c+b1y*s,2.0))+1.0)*1.0/pow(pow(-a1y+y+b1y*c+b1x*s,2.0)+pow(a1x+r_p-x-b1x*c+b1y*s,2.0),3.0/2.0)*1.0/sqrt(r_p*(a1x*2.0-x*2.0-b1x*c*2.0+b1y*s*2.0)+pow(a1x-x-b1x*c+b1y*s,2.0)+pow(-a1y+y+b1y*c+b1x*s,2.0))*(a1x+r_p-x-b1x*c+b1y*s))-(1.0/sqrt(r_p*(a1x*2.0-x*2.0-b1x*c*2.0+b1y*s*2.0)+pow(a1x-x-b1x*c+b1y*s,2.0)+pow(-a1y+y+b1y*c+b1x*s,2.0))*(a1x*2.0+r_p*2.0-x*2.0-b1x*c*2.0+b1y*s*2.0))/2.0)*(sqrt(r_p*(a1x*2.0-x*2.0-b1x*c*2.0+b1y*s*2.0)+pow(a1x-x-b1x*c+b1y*s,2.0)+pow(-a1y+y+b1y*c+b1x*s,2.0))+r_p*(3.141592653589793-acos(1.0/sqrt(pow(-a1y+y+b1y*c+b1x*s,2.0)+pow(a1x+r_p-x-b1x*c+b1y*s,2.0))*(-a1y+y+b1y*c+b1x*s))+acos(1.0/sqrt(pow(-a1y+y+b1y*c+b1x*s,2.0)+pow(a1x+r_p-x-b1x*c+b1y*s,2.0))*sqrt(r_p*(a1x*2.0-x*2.0-b1x*c*2.0+b1y*s*2.0)+pow(a1x-x-b1x*c+b1y*s,2.0)+pow(-a1y+y+b1y*c+b1x*s,2.0)))-1.7896E-1))*(l1*l1-pow(r_p*-2.237E+3-r_p*acos(1.0/sqrt(pow(-a1y+y+b1y*c+b1x*s,2.0)+pow(a1x+r_p-x-b1x*c+b1y*s,2.0))*(-a1y+y+b1y*c+b1x*s))*1.25E+4+r_p*3.141592653589793*1.25E+4+sqrt(r_p*(a1x*2.0-x*2.0-b1x*c*2.0+b1y*s*2.0)+pow(a1x-x-b1x*c+b1y*s,2.0)+pow(-a1y+y+b1y*c+b1x*s,2.0))*1.25E+4+r_p*acos(1.0/sqrt(pow(-a1y+y+b1y*c+b1x*s,2.0)+pow(a1x+r_p-x-b1x*c+b1y*s,2.0))*sqrt(r_p*(a1x*2.0-x*2.0-b1x*c*2.0+b1y*s*2.0)+pow(a1x-x-b1x*c+b1y*s,2.0)+pow(-a1y+y+b1y*c+b1x*s,2.0)))*1.25E+4,2.0)/1.5625E+8)*-4.0;
	J1y     = (sqrt(r_p*(a1x*2.0-x*2.0-b1x*c*2.0+b1y*s*2.0)+pow(a1x-x-b1x*c+b1y*s,2.0)+pow(-a1y+y+b1y*c+b1x*s,2.0))+r_p*(3.141592653589793-acos(1.0/sqrt(pow(-a1y+y+b1y*c+b1x*s,2.0)+pow(a1x+r_p-x-b1x*c+b1y*s,2.0))*(-a1y+y+b1y*c+b1x*s))+acos(1.0/sqrt(pow(-a1y+y+b1y*c+b1x*s,2.0)+pow(a1x+r_p-x-b1x*c+b1y*s,2.0))*sqrt(r_p*(a1x*2.0-x*2.0-b1x*c*2.0+b1y*s*2.0)+pow(a1x-x-b1x*c+b1y*s,2.0)+pow(-a1y+y+b1y*c+b1x*s,2.0)))-1.7896E-1))*(l1*l1-pow(r_p*-2.237E+3-r_p*acos(1.0/sqrt(pow(-a1y+y+b1y*c+b1x*s,2.0)+pow(a1x+r_p-x-b1x*c+b1y*s,2.0))*(-a1y+y+b1y*c+b1x*s))*1.25E+4+r_p*3.141592653589793*1.25E+4+sqrt(r_p*(a1x*2.0-x*2.0-b1x*c*2.0+b1y*s*2.0)+pow(a1x-x-b1x*c+b1y*s,2.0)+pow(-a1y+y+b1y*c+b1x*s,2.0))*1.25E+4+r_p*acos(1.0/sqrt(pow(-a1y+y+b1y*c+b1x*s,2.0)+pow(a1x+r_p-x-b1x*c+b1y*s,2.0))*sqrt(r_p*(a1x*2.0-x*2.0-b1x*c*2.0+b1y*s*2.0)+pow(a1x-x-b1x*c+b1y*s,2.0)+pow(-a1y+y+b1y*c+b1x*s,2.0)))*1.25E+4,2.0)/1.5625E+8)*(r_p*(1.0/pow(pow(-a1y+y+b1y*c+b1x*s,2.0)+pow(a1x+r_p-x-b1x*c+b1y*s,2.0),3.0/2.0)*1.0/sqrt(-pow(-a1y+y+b1y*c+b1x*s,2.0)/(pow(-a1y+y+b1y*c+b1x*s,2.0)+pow(a1x+r_p-x-b1x*c+b1y*s,2.0))+1.0)*pow(a1x+r_p-x-b1x*c+b1y*s,2.0)-(r_p*r_p)*1.0/sqrt(-(r_p*(a1x*2.0-x*2.0-b1x*c*2.0+b1y*s*2.0)+pow(a1x-x-b1x*c+b1y*s,2.0)+pow(-a1y+y+b1y*c+b1x*s,2.0))/(pow(-a1y+y+b1y*c+b1x*s,2.0)+pow(a1x+r_p-x-b1x*c+b1y*s,2.0))+1.0)*1.0/pow(pow(-a1y+y+b1y*c+b1x*s,2.0)+pow(a1x+r_p-x-b1x*c+b1y*s,2.0),3.0/2.0)*1.0/sqrt(r_p*(a1x*2.0-x*2.0-b1x*c*2.0+b1y*s*2.0)+pow(a1x-x-b1x*c+b1y*s,2.0)+pow(-a1y+y+b1y*c+b1x*s,2.0))*(-a1y+y+b1y*c+b1x*s))+(1.0/sqrt(r_p*(a1x*2.0-x*2.0-b1x*c*2.0+b1y*s*2.0)+pow(a1x-x-b1x*c+b1y*s,2.0)+pow(-a1y+y+b1y*c+b1x*s,2.0))*(a1y*-2.0+y*2.0+b1y*c*2.0+b1x*s*2.0))/2.0)*-4.0;
	J1theta = (sqrt(r_p*(a1x*2.0-x*2.0-b1x*c*2.0+b1y*s*2.0)+pow(a1x-x-b1x*c+b1y*s,2.0)+pow(-a1y+y+b1y*c+b1x*s,2.0))+r_p*(3.141592653589793-acos(1.0/sqrt(pow(-a1y+y+b1y*c+b1x*s,2.0)+pow(a1x+r_p-x-b1x*c+b1y*s,2.0))*(-a1y+y+b1y*c+b1x*s))+acos(1.0/sqrt(pow(-a1y+y+b1y*c+b1x*s,2.0)+pow(a1x+r_p-x-b1x*c+b1y*s,2.0))*sqrt(r_p*(a1x*2.0-x*2.0-b1x*c*2.0+b1y*s*2.0)+pow(a1x-x-b1x*c+b1y*s,2.0)+pow(-a1y+y+b1y*c+b1x*s,2.0)))-1.7896E-1))*(l1*l1-pow(r_p*-2.237E+3-r_p*acos(1.0/sqrt(pow(-a1y+y+b1y*c+b1x*s,2.0)+pow(a1x+r_p-x-b1x*c+b1y*s,2.0))*(-a1y+y+b1y*c+b1x*s))*1.25E+4+r_p*3.141592653589793*1.25E+4+sqrt(r_p*(a1x*2.0-x*2.0-b1x*c*2.0+b1y*s*2.0)+pow(a1x-x-b1x*c+b1y*s,2.0)+pow(-a1y+y+b1y*c+b1x*s,2.0))*1.25E+4+r_p*acos(1.0/sqrt(pow(-a1y+y+b1y*c+b1x*s,2.0)+pow(a1x+r_p-x-b1x*c+b1y*s,2.0))*sqrt(r_p*(a1x*2.0-x*2.0-b1x*c*2.0+b1y*s*2.0)+pow(a1x-x-b1x*c+b1y*s,2.0)+pow(-a1y+y+b1y*c+b1x*s,2.0)))*1.25E+4,2.0)/1.5625E+8)*((1.0/sqrt(r_p*(a1x*2.0-x*2.0-b1x*c*2.0+b1y*s*2.0)+pow(a1x-x-b1x*c+b1y*s,2.0)+pow(-a1y+y+b1y*c+b1x*s,2.0))*(a1x*b1x*s*2.0+a1y*b1y*s*2.0+b1y*r_p*c*2.0-b1y*x*c*2.0+b1x*y*c*2.0+b1x*r_p*s*2.0-b1x*x*s*2.0-b1y*y*s*2.0+a1x*b1y*c*2.0-a1y*b1x*c*2.0))/2.0-r_p*(1.0/pow(pow(-a1y+y+b1y*c+b1x*s,2.0)+pow(a1x+r_p-x-b1x*c+b1y*s,2.0),3.0/2.0)*1.0/sqrt(-pow(-a1y+y+b1y*c+b1x*s,2.0)/(pow(-a1y+y+b1y*c+b1x*s,2.0)+pow(a1x+r_p-x-b1x*c+b1y*s,2.0))+1.0)*(a1x+r_p-x-b1x*c+b1y*s)*(b1x*b1x+b1y*b1y+a1x*b1y*s-a1y*b1x*s-b1x*r_p*c+b1x*x*c+b1y*y*c+b1y*r_p*s-b1y*x*s+b1x*y*s-a1x*b1x*c-a1y*b1y*c)+(r_p*r_p)*1.0/sqrt(-(r_p*(a1x*2.0-x*2.0-b1x*c*2.0+b1y*s*2.0)+pow(a1x-x-b1x*c+b1y*s,2.0)+pow(-a1y+y+b1y*c+b1x*s,2.0))/(pow(-a1y+y+b1y*c+b1x*s,2.0)+pow(a1x+r_p-x-b1x*c+b1y*s,2.0))+1.0)*1.0/pow(pow(-a1y+y+b1y*c+b1x*s,2.0)+pow(a1x+r_p-x-b1x*c+b1y*s,2.0),3.0/2.0)*1.0/sqrt(r_p*(a1x*2.0-x*2.0-b1x*c*2.0+b1y*s*2.0)+pow(a1x-x-b1x*c+b1y*s,2.0)+pow(-a1y+y+b1y*c+b1x*s,2.0))*(a1x*b1x*s+a1y*b1y*s+b1y*r_p*c-b1y*x*c+b1x*y*c+b1x*r_p*s-b1x*x*s-b1y*y*s+a1x*b1y*c-a1y*b1x*c)))*-4.0;

	J2x     = (pow(sqrt(r_p*(a2x*2.0-x*2.0-b2x*c*2.0+b2y*s*2.0)+pow(a2x-x-b2x*c+b2y*s,2.0)+pow(-a2y+y+b2y*c+b2x*s,2.0))*1.25E+4+r_p*(acos(1.0/sqrt(pow(-a2y+y+b2y*c+b2x*s,2.0)+pow(a2x+r_p-x-b2x*c+b2y*s,2.0))*(-a2y+y+b2y*c+b2x*s))*1.25E+4+acos(1.0/sqrt(pow(-a2y+y+b2y*c+b2x*s,2.0)+pow(a2x+r_p-x-b2x*c+b2y*s,2.0))*sqrt(r_p*(a2x*2.0-x*2.0-b2x*c*2.0+b2y*s*2.0)+pow(a2x-x-b2x*c+b2y*s,2.0)+pow(-a2y+y+b2y*c+b2x*s,2.0)))*1.25E+4-2.237E+3),2.0)/1.5625E+8-l2*l2)*(r_p*((1.0/pow(pow(-a2y+y+b2y*c+b2x*s,2.0)+pow(a2x+r_p-x-b2x*c+b2y*s,2.0),3.0/2.0)*1.0/sqrt(-pow(-a2y+y+b2y*c+b2x*s,2.0)/(pow(-a2y+y+b2y*c+b2x*s,2.0)+pow(a2x+r_p-x-b2x*c+b2y*s,2.0))+1.0)*(-a2y+y+b2y*c+b2x*s)*(a2x*2.0+r_p*2.0-x*2.0-b2x*c*2.0+b2y*s*2.0))/2.0-(r_p*r_p)*1.0/sqrt(-(r_p*(a2x*2.0-x*2.0-b2x*c*2.0+b2y*s*2.0)+pow(a2x-x-b2x*c+b2y*s,2.0)+pow(-a2y+y+b2y*c+b2x*s,2.0))/(pow(-a2y+y+b2y*c+b2x*s,2.0)+pow(a2x+r_p-x-b2x*c+b2y*s,2.0))+1.0)*1.0/pow(pow(-a2y+y+b2y*c+b2x*s,2.0)+pow(a2x+r_p-x-b2x*c+b2y*s,2.0),3.0/2.0)*1.0/sqrt(r_p*(a2x*2.0-x*2.0-b2x*c*2.0+b2y*s*2.0)+pow(a2x-x-b2x*c+b2y*s,2.0)+pow(-a2y+y+b2y*c+b2x*s,2.0))*(a2x+r_p-x-b2x*c+b2y*s))+(1.0/sqrt(r_p*(a2x*2.0-x*2.0-b2x*c*2.0+b2y*s*2.0)+pow(a2x-x-b2x*c+b2y*s,2.0)+pow(-a2y+y+b2y*c+b2x*s,2.0))*(a2x*2.0+r_p*2.0-x*2.0-b2x*c*2.0+b2y*s*2.0))/2.0)*(r_p*(acos(1.0/sqrt(pow(-a2y+y+b2y*c+b2x*s,2.0)+pow(a2x+r_p-x-b2x*c+b2y*s,2.0))*(-a2y+y+b2y*c+b2x*s))+acos(1.0/sqrt(pow(-a2y+y+b2y*c+b2x*s,2.0)+pow(a2x+r_p-x-b2x*c+b2y*s,2.0))*sqrt(r_p*(a2x*2.0-x*2.0-b2x*c*2.0+b2y*s*2.0)+pow(a2x-x-b2x*c+b2y*s,2.0)+pow(-a2y+y+b2y*c+b2x*s,2.0)))-1.7896E-1)+sqrt(r_p*(a2x*2.0-x*2.0-b2x*c*2.0+b2y*s*2.0)+pow(a2x-x-b2x*c+b2y*s,2.0)+pow(-a2y+y+b2y*c+b2x*s,2.0)))*-4.0;
	J2y     = (pow(sqrt(r_p*(a2x*2.0-x*2.0-b2x*c*2.0+b2y*s*2.0)+pow(a2x-x-b2x*c+b2y*s,2.0)+pow(-a2y+y+b2y*c+b2x*s,2.0))*1.25E+4+r_p*(acos(1.0/sqrt(pow(-a2y+y+b2y*c+b2x*s,2.0)+pow(a2x+r_p-x-b2x*c+b2y*s,2.0))*(-a2y+y+b2y*c+b2x*s))*1.25E+4+acos(1.0/sqrt(pow(-a2y+y+b2y*c+b2x*s,2.0)+pow(a2x+r_p-x-b2x*c+b2y*s,2.0))*sqrt(r_p*(a2x*2.0-x*2.0-b2x*c*2.0+b2y*s*2.0)+pow(a2x-x-b2x*c+b2y*s,2.0)+pow(-a2y+y+b2y*c+b2x*s,2.0)))*1.25E+4-2.237E+3),2.0)/1.5625E+8-l2*l2)*(r_p*(1.0/pow(pow(-a2y+y+b2y*c+b2x*s,2.0)+pow(a2x+r_p-x-b2x*c+b2y*s,2.0),3.0/2.0)*1.0/sqrt(-pow(-a2y+y+b2y*c+b2x*s,2.0)/(pow(-a2y+y+b2y*c+b2x*s,2.0)+pow(a2x+r_p-x-b2x*c+b2y*s,2.0))+1.0)*pow(a2x+r_p-x-b2x*c+b2y*s,2.0)+(r_p*r_p)*1.0/sqrt(-(r_p*(a2x*2.0-x*2.0-b2x*c*2.0+b2y*s*2.0)+pow(a2x-x-b2x*c+b2y*s,2.0)+pow(-a2y+y+b2y*c+b2x*s,2.0))/(pow(-a2y+y+b2y*c+b2x*s,2.0)+pow(a2x+r_p-x-b2x*c+b2y*s,2.0))+1.0)*1.0/pow(pow(-a2y+y+b2y*c+b2x*s,2.0)+pow(a2x+r_p-x-b2x*c+b2y*s,2.0),3.0/2.0)*1.0/sqrt(r_p*(a2x*2.0-x*2.0-b2x*c*2.0+b2y*s*2.0)+pow(a2x-x-b2x*c+b2y*s,2.0)+pow(-a2y+y+b2y*c+b2x*s,2.0))*(-a2y+y+b2y*c+b2x*s))-(1.0/sqrt(r_p*(a2x*2.0-x*2.0-b2x*c*2.0+b2y*s*2.0)+pow(a2x-x-b2x*c+b2y*s,2.0)+pow(-a2y+y+b2y*c+b2x*s,2.0))*(a2y*-2.0+y*2.0+b2y*c*2.0+b2x*s*2.0))/2.0)*(r_p*(acos(1.0/sqrt(pow(-a2y+y+b2y*c+b2x*s,2.0)+pow(a2x+r_p-x-b2x*c+b2y*s,2.0))*(-a2y+y+b2y*c+b2x*s))+acos(1.0/sqrt(pow(-a2y+y+b2y*c+b2x*s,2.0)+pow(a2x+r_p-x-b2x*c+b2y*s,2.0))*sqrt(r_p*(a2x*2.0-x*2.0-b2x*c*2.0+b2y*s*2.0)+pow(a2x-x-b2x*c+b2y*s,2.0)+pow(-a2y+y+b2y*c+b2x*s,2.0)))-1.7896E-1)+sqrt(r_p*(a2x*2.0-x*2.0-b2x*c*2.0+b2y*s*2.0)+pow(a2x-x-b2x*c+b2y*s,2.0)+pow(-a2y+y+b2y*c+b2x*s,2.0)))*-4.0;
	J2theta = (pow(sqrt(r_p*(a2x*2.0-x*2.0-b2x*c*2.0+b2y*s*2.0)+pow(a2x-x-b2x*c+b2y*s,2.0)+pow(-a2y+y+b2y*c+b2x*s,2.0))*1.25E+4+r_p*(acos(1.0/sqrt(pow(-a2y+y+b2y*c+b2x*s,2.0)+pow(a2x+r_p-x-b2x*c+b2y*s,2.0))*(-a2y+y+b2y*c+b2x*s))*1.25E+4+acos(1.0/sqrt(pow(-a2y+y+b2y*c+b2x*s,2.0)+pow(a2x+r_p-x-b2x*c+b2y*s,2.0))*sqrt(r_p*(a2x*2.0-x*2.0-b2x*c*2.0+b2y*s*2.0)+pow(a2x-x-b2x*c+b2y*s,2.0)+pow(-a2y+y+b2y*c+b2x*s,2.0)))*1.25E+4-2.237E+3),2.0)/1.5625E+8-l2*l2)*((1.0/sqrt(r_p*(a2x*2.0-x*2.0-b2x*c*2.0+b2y*s*2.0)+pow(a2x-x-b2x*c+b2y*s,2.0)+pow(-a2y+y+b2y*c+b2x*s,2.0))*(a2x*b2x*s*2.0+a2y*b2y*s*2.0+b2y*r_p*c*2.0-b2y*x*c*2.0+b2x*y*c*2.0+b2x*r_p*s*2.0-b2x*x*s*2.0-b2y*y*s*2.0+a2x*b2y*c*2.0-a2y*b2x*c*2.0))/2.0+r_p*(1.0/pow(pow(-a2y+y+b2y*c+b2x*s,2.0)+pow(a2x+r_p-x-b2x*c+b2y*s,2.0),3.0/2.0)*1.0/sqrt(-pow(-a2y+y+b2y*c+b2x*s,2.0)/(pow(-a2y+y+b2y*c+b2x*s,2.0)+pow(a2x+r_p-x-b2x*c+b2y*s,2.0))+1.0)*(a2x+r_p-x-b2x*c+b2y*s)*(b2x*b2x+b2y*b2y+a2x*b2y*s-a2y*b2x*s-b2x*r_p*c+b2x*x*c+b2y*y*c+b2y*r_p*s-b2y*x*s+b2x*y*s-a2x*b2x*c-a2y*b2y*c)-(r_p*r_p)*1.0/sqrt(-(r_p*(a2x*2.0-x*2.0-b2x*c*2.0+b2y*s*2.0)+pow(a2x-x-b2x*c+b2y*s,2.0)+pow(-a2y+y+b2y*c+b2x*s,2.0))/(pow(-a2y+y+b2y*c+b2x*s,2.0)+pow(a2x+r_p-x-b2x*c+b2y*s,2.0))+1.0)*1.0/pow(pow(-a2y+y+b2y*c+b2x*s,2.0)+pow(a2x+r_p-x-b2x*c+b2y*s,2.0),3.0/2.0)*1.0/sqrt(r_p*(a2x*2.0-x*2.0-b2x*c*2.0+b2y*s*2.0)+pow(a2x-x-b2x*c+b2y*s,2.0)+pow(-a2y+y+b2y*c+b2x*s,2.0))*(a2x*b2x*s+a2y*b2y*s+b2y*r_p*c-b2y*x*c+b2x*y*c+b2x*r_p*s-b2x*x*s-b2y*y*s+a2x*b2y*c-a2y*b2x*c)))*(r_p*(acos(1.0/sqrt(pow(-a2y+y+b2y*c+b2x*s,2.0)+pow(a2x+r_p-x-b2x*c+b2y*s,2.0))*(-a2y+y+b2y*c+b2x*s))+acos(1.0/sqrt(pow(-a2y+y+b2y*c+b2x*s,2.0)+pow(a2x+r_p-x-b2x*c+b2y*s,2.0))*sqrt(r_p*(a2x*2.0-x*2.0-b2x*c*2.0+b2y*s*2.0)+pow(a2x-x-b2x*c+b2y*s,2.0)+pow(-a2y+y+b2y*c+b2x*s,2.0)))-1.7896E-1)+sqrt(r_p*(a2x*2.0-x*2.0-b2x*c*2.0+b2y*s*2.0)+pow(a2x-x-b2x*c+b2y*s,2.0)+pow(-a2y+y+b2y*c+b2x*s,2.0)))*4.0;

	J3x     = (pow(sqrt(-r_p*(a3x*2.0-x*2.0-b3x*c*2.0+b3y*s*2.0)+pow(a3x-x-b3x*c+b3y*s,2.0)+pow(-a3y+y+b3y*c+b3x*s,2.0))*1.25E+4+r_p*(acos(1.0/sqrt(pow(-a3y+y+b3y*c+b3x*s,2.0)+pow(-a3x+r_p+x+b3x*c-b3y*s,2.0))*(-a3y+y+b3y*c+b3x*s))*1.25E+4+acos(1.0/sqrt(pow(-a3y+y+b3y*c+b3x*s,2.0)+pow(-a3x+r_p+x+b3x*c-b3y*s,2.0))*sqrt(-r_p*(a3x*2.0-x*2.0-b3x*c*2.0+b3y*s*2.0)+pow(a3x-x-b3x*c+b3y*s,2.0)+pow(-a3y+y+b3y*c+b3x*s,2.0)))*1.25E+4-2.237E+3),2.0)/1.5625E+8-l3*l3)*(r_p*((1.0/pow(pow(-a3y+y+b3y*c+b3x*s,2.0)+pow(-a3x+r_p+x+b3x*c-b3y*s,2.0),3.0/2.0)*1.0/sqrt(-pow(-a3y+y+b3y*c+b3x*s,2.0)/(pow(-a3y+y+b3y*c+b3x*s,2.0)+pow(-a3x+r_p+x+b3x*c-b3y*s,2.0))+1.0)*(-a3y+y+b3y*c+b3x*s)*(a3x*-2.0+r_p*2.0+x*2.0+b3x*c*2.0-b3y*s*2.0))/2.0-(r_p*r_p)*1.0/sqrt(-(-r_p*(a3x*2.0-x*2.0-b3x*c*2.0+b3y*s*2.0)+pow(a3x-x-b3x*c+b3y*s,2.0)+pow(-a3y+y+b3y*c+b3x*s,2.0))/(pow(-a3y+y+b3y*c+b3x*s,2.0)+pow(-a3x+r_p+x+b3x*c-b3y*s,2.0))+1.0)*1.0/pow(pow(-a3y+y+b3y*c+b3x*s,2.0)+pow(-a3x+r_p+x+b3x*c-b3y*s,2.0),3.0/2.0)*1.0/sqrt(-r_p*(a3x*2.0-x*2.0-b3x*c*2.0+b3y*s*2.0)+pow(a3x-x-b3x*c+b3y*s,2.0)+pow(-a3y+y+b3y*c+b3x*s,2.0))*(-a3x+r_p+x+b3x*c-b3y*s))+(1.0/sqrt(-r_p*(a3x*2.0-x*2.0-b3x*c*2.0+b3y*s*2.0)+pow(a3x-x-b3x*c+b3y*s,2.0)+pow(-a3y+y+b3y*c+b3x*s,2.0))*(a3x*-2.0+r_p*2.0+x*2.0+b3x*c*2.0-b3y*s*2.0))/2.0)*(r_p*(acos(1.0/sqrt(pow(-a3y+y+b3y*c+b3x*s,2.0)+pow(-a3x+r_p+x+b3x*c-b3y*s,2.0))*(-a3y+y+b3y*c+b3x*s))+acos(1.0/sqrt(pow(-a3y+y+b3y*c+b3x*s,2.0)+pow(-a3x+r_p+x+b3x*c-b3y*s,2.0))*sqrt(-r_p*(a3x*2.0-x*2.0-b3x*c*2.0+b3y*s*2.0)+pow(a3x-x-b3x*c+b3y*s,2.0)+pow(-a3y+y+b3y*c+b3x*s,2.0)))-1.7896E-1)+sqrt(-r_p*(a3x*2.0-x*2.0-b3x*c*2.0+b3y*s*2.0)+pow(a3x-x-b3x*c+b3y*s,2.0)+pow(-a3y+y+b3y*c+b3x*s,2.0)))*4.0;
	J3y     = (pow(sqrt(-r_p*(a3x*2.0-x*2.0-b3x*c*2.0+b3y*s*2.0)+pow(a3x-x-b3x*c+b3y*s,2.0)+pow(-a3y+y+b3y*c+b3x*s,2.0))*1.25E+4+r_p*(acos(1.0/sqrt(pow(-a3y+y+b3y*c+b3x*s,2.0)+pow(-a3x+r_p+x+b3x*c-b3y*s,2.0))*(-a3y+y+b3y*c+b3x*s))*1.25E+4+acos(1.0/sqrt(pow(-a3y+y+b3y*c+b3x*s,2.0)+pow(-a3x+r_p+x+b3x*c-b3y*s,2.0))*sqrt(-r_p*(a3x*2.0-x*2.0-b3x*c*2.0+b3y*s*2.0)+pow(a3x-x-b3x*c+b3y*s,2.0)+pow(-a3y+y+b3y*c+b3x*s,2.0)))*1.25E+4-2.237E+3),2.0)/1.5625E+8-l3*l3)*(r_p*(1.0/pow(pow(-a3y+y+b3y*c+b3x*s,2.0)+pow(-a3x+r_p+x+b3x*c-b3y*s,2.0),3.0/2.0)*1.0/sqrt(-pow(-a3y+y+b3y*c+b3x*s,2.0)/(pow(-a3y+y+b3y*c+b3x*s,2.0)+pow(-a3x+r_p+x+b3x*c-b3y*s,2.0))+1.0)*pow(-a3x+r_p+x+b3x*c-b3y*s,2.0)+(r_p*r_p)*1.0/sqrt(-(-r_p*(a3x*2.0-x*2.0-b3x*c*2.0+b3y*s*2.0)+pow(a3x-x-b3x*c+b3y*s,2.0)+pow(-a3y+y+b3y*c+b3x*s,2.0))/(pow(-a3y+y+b3y*c+b3x*s,2.0)+pow(-a3x+r_p+x+b3x*c-b3y*s,2.0))+1.0)*1.0/pow(pow(-a3y+y+b3y*c+b3x*s,2.0)+pow(-a3x+r_p+x+b3x*c-b3y*s,2.0),3.0/2.0)*1.0/sqrt(-r_p*(a3x*2.0-x*2.0-b3x*c*2.0+b3y*s*2.0)+pow(a3x-x-b3x*c+b3y*s,2.0)+pow(-a3y+y+b3y*c+b3x*s,2.0))*(-a3y+y+b3y*c+b3x*s))-(1.0/sqrt(-r_p*(a3x*2.0-x*2.0-b3x*c*2.0+b3y*s*2.0)+pow(a3x-x-b3x*c+b3y*s,2.0)+pow(-a3y+y+b3y*c+b3x*s,2.0))*(a3y*-2.0+y*2.0+b3y*c*2.0+b3x*s*2.0))/2.0)*(r_p*(acos(1.0/sqrt(pow(-a3y+y+b3y*c+b3x*s,2.0)+pow(-a3x+r_p+x+b3x*c-b3y*s,2.0))*(-a3y+y+b3y*c+b3x*s))+acos(1.0/sqrt(pow(-a3y+y+b3y*c+b3x*s,2.0)+pow(-a3x+r_p+x+b3x*c-b3y*s,2.0))*sqrt(-r_p*(a3x*2.0-x*2.0-b3x*c*2.0+b3y*s*2.0)+pow(a3x-x-b3x*c+b3y*s,2.0)+pow(-a3y+y+b3y*c+b3x*s,2.0)))-1.7896E-1)+sqrt(-r_p*(a3x*2.0-x*2.0-b3x*c*2.0+b3y*s*2.0)+pow(a3x-x-b3x*c+b3y*s,2.0)+pow(-a3y+y+b3y*c+b3x*s,2.0)))*-4.0;
	J3theta = (pow(sqrt(-r_p*(a3x*2.0-x*2.0-b3x*c*2.0+b3y*s*2.0)+pow(a3x-x-b3x*c+b3y*s,2.0)+pow(-a3y+y+b3y*c+b3x*s,2.0))*1.25E+4+r_p*(acos(1.0/sqrt(pow(-a3y+y+b3y*c+b3x*s,2.0)+pow(-a3x+r_p+x+b3x*c-b3y*s,2.0))*(-a3y+y+b3y*c+b3x*s))*1.25E+4+acos(1.0/sqrt(pow(-a3y+y+b3y*c+b3x*s,2.0)+pow(-a3x+r_p+x+b3x*c-b3y*s,2.0))*sqrt(-r_p*(a3x*2.0-x*2.0-b3x*c*2.0+b3y*s*2.0)+pow(a3x-x-b3x*c+b3y*s,2.0)+pow(-a3y+y+b3y*c+b3x*s,2.0)))*1.25E+4-2.237E+3),2.0)/1.5625E+8-l3*l3)*((1.0/sqrt(-r_p*(a3x*2.0-x*2.0-b3x*c*2.0+b3y*s*2.0)+pow(a3x-x-b3x*c+b3y*s,2.0)+pow(-a3y+y+b3y*c+b3x*s,2.0))*(a3x*b3x*s*-2.0-a3y*b3y*s*2.0+b3y*r_p*c*2.0+b3y*x*c*2.0-b3x*y*c*2.0+b3x*r_p*s*2.0+b3x*x*s*2.0+b3y*y*s*2.0-a3x*b3y*c*2.0+a3y*b3x*c*2.0))/2.0+r_p*(1.0/pow(pow(-a3y+y+b3y*c+b3x*s,2.0)+pow(-a3x+r_p+x+b3x*c-b3y*s,2.0),3.0/2.0)*1.0/sqrt(-pow(-a3y+y+b3y*c+b3x*s,2.0)/(pow(-a3y+y+b3y*c+b3x*s,2.0)+pow(-a3x+r_p+x+b3x*c-b3y*s,2.0))+1.0)*(-a3x+r_p+x+b3x*c-b3y*s)*(b3x*b3x+b3y*b3y+a3x*b3y*s-a3y*b3x*s+b3x*r_p*c+b3x*x*c+b3y*y*c-b3y*r_p*s-b3y*x*s+b3x*y*s-a3x*b3x*c-a3y*b3y*c)-(r_p*r_p)*1.0/sqrt(-(-r_p*(a3x*2.0-x*2.0-b3x*c*2.0+b3y*s*2.0)+pow(a3x-x-b3x*c+b3y*s,2.0)+pow(-a3y+y+b3y*c+b3x*s,2.0))/(pow(-a3y+y+b3y*c+b3x*s,2.0)+pow(-a3x+r_p+x+b3x*c-b3y*s,2.0))+1.0)*1.0/pow(pow(-a3y+y+b3y*c+b3x*s,2.0)+pow(-a3x+r_p+x+b3x*c-b3y*s,2.0),3.0/2.0)*1.0/sqrt(-r_p*(a3x*2.0-x*2.0-b3x*c*2.0+b3y*s*2.0)+pow(a3x-x-b3x*c+b3y*s,2.0)+pow(-a3y+y+b3y*c+b3x*s,2.0))*(-a3x*b3x*s-a3y*b3y*s+b3y*r_p*c+b3y*x*c-b3x*y*c+b3x*r_p*s+b3x*x*s+b3y*y*s-a3x*b3y*c+a3y*b3x*c)))*(r_p*(acos(1.0/sqrt(pow(-a3y+y+b3y*c+b3x*s,2.0)+pow(-a3x+r_p+x+b3x*c-b3y*s,2.0))*(-a3y+y+b3y*c+b3x*s))+acos(1.0/sqrt(pow(-a3y+y+b3y*c+b3x*s,2.0)+pow(-a3x+r_p+x+b3x*c-b3y*s,2.0))*sqrt(-r_p*(a3x*2.0-x*2.0-b3x*c*2.0+b3y*s*2.0)+pow(a3x-x-b3x*c+b3y*s,2.0)+pow(-a3y+y+b3y*c+b3x*s,2.0)))-1.7896E-1)+sqrt(-r_p*(a3x*2.0-x*2.0-b3x*c*2.0+b3y*s*2.0)+pow(a3x-x-b3x*c+b3y*s,2.0)+pow(-a3y+y+b3y*c+b3x*s,2.0)))*-4.0;

	J4x     = (r_p*((1.0/pow(pow(-a4y+y+b4y*c+b4x*s,2.0)+pow(-a4x+r_p+x+b4x*c-b4y*s,2.0),3.0/2.0)*1.0/sqrt(-pow(-a4y+y+b4y*c+b4x*s,2.0)/(pow(-a4y+y+b4y*c+b4x*s,2.0)+pow(-a4x+r_p+x+b4x*c-b4y*s,2.0))+1.0)*(-a4y+y+b4y*c+b4x*s)*(a4x*-2.0+r_p*2.0+x*2.0+b4x*c*2.0-b4y*s*2.0))/2.0+(r_p*r_p)*1.0/sqrt(-(-r_p*(a4x*2.0-x*2.0-b4x*c*2.0+b4y*s*2.0)+pow(a4x-x-b4x*c+b4y*s,2.0)+pow(-a4y+y+b4y*c+b4x*s,2.0))/(pow(-a4y+y+b4y*c+b4x*s,2.0)+pow(-a4x+r_p+x+b4x*c-b4y*s,2.0))+1.0)*1.0/pow(pow(-a4y+y+b4y*c+b4x*s,2.0)+pow(-a4x+r_p+x+b4x*c-b4y*s,2.0),3.0/2.0)*1.0/sqrt(-r_p*(a4x*2.0-x*2.0-b4x*c*2.0+b4y*s*2.0)+pow(a4x-x-b4x*c+b4y*s,2.0)+pow(-a4y+y+b4y*c+b4x*s,2.0))*(-a4x+r_p+x+b4x*c-b4y*s))-(1.0/sqrt(-r_p*(a4x*2.0-x*2.0-b4x*c*2.0+b4y*s*2.0)+pow(a4x-x-b4x*c+b4y*s,2.0)+pow(-a4y+y+b4y*c+b4x*s,2.0))*(a4x*-2.0+r_p*2.0+x*2.0+b4x*c*2.0-b4y*s*2.0))/2.0)*(sqrt(-r_p*(a4x*2.0-x*2.0-b4x*c*2.0+b4y*s*2.0)+pow(a4x-x-b4x*c+b4y*s,2.0)+pow(-a4y+y+b4y*c+b4x*s,2.0))+r_p*(3.141592653589793-acos(1.0/sqrt(pow(-a4y+y+b4y*c+b4x*s,2.0)+pow(-a4x+r_p+x+b4x*c-b4y*s,2.0))*(-a4y+y+b4y*c+b4x*s))+acos(1.0/sqrt(pow(-a4y+y+b4y*c+b4x*s,2.0)+pow(-a4x+r_p+x+b4x*c-b4y*s,2.0))*sqrt(-r_p*(a4x*2.0-x*2.0-b4x*c*2.0+b4y*s*2.0)+pow(a4x-x-b4x*c+b4y*s,2.0)+pow(-a4y+y+b4y*c+b4x*s,2.0)))-1.7896E-1))*(l4*l4-pow(r_p*-2.237E+3-r_p*acos(1.0/sqrt(pow(-a4y+y+b4y*c+b4x*s,2.0)+pow(-a4x+r_p+x+b4x*c-b4y*s,2.0))*(-a4y+y+b4y*c+b4x*s))*1.25E+4+r_p*3.141592653589793*1.25E+4+sqrt(-r_p*(a4x*2.0-x*2.0-b4x*c*2.0+b4y*s*2.0)+pow(a4x-x-b4x*c+b4y*s,2.0)+pow(-a4y+y+b4y*c+b4x*s,2.0))*1.25E+4+r_p*acos(1.0/sqrt(pow(-a4y+y+b4y*c+b4x*s,2.0)+pow(-a4x+r_p+x+b4x*c-b4y*s,2.0))*sqrt(-r_p*(a4x*2.0-x*2.0-b4x*c*2.0+b4y*s*2.0)+pow(a4x-x-b4x*c+b4y*s,2.0)+pow(-a4y+y+b4y*c+b4x*s,2.0)))*1.25E+4,2.0)/1.5625E+8)*4.0;
	J4y     = (sqrt(-r_p*(a4x*2.0-x*2.0-b4x*c*2.0+b4y*s*2.0)+pow(a4x-x-b4x*c+b4y*s,2.0)+pow(-a4y+y+b4y*c+b4x*s,2.0))+r_p*(3.141592653589793-acos(1.0/sqrt(pow(-a4y+y+b4y*c+b4x*s,2.0)+pow(-a4x+r_p+x+b4x*c-b4y*s,2.0))*(-a4y+y+b4y*c+b4x*s))+acos(1.0/sqrt(pow(-a4y+y+b4y*c+b4x*s,2.0)+pow(-a4x+r_p+x+b4x*c-b4y*s,2.0))*sqrt(-r_p*(a4x*2.0-x*2.0-b4x*c*2.0+b4y*s*2.0)+pow(a4x-x-b4x*c+b4y*s,2.0)+pow(-a4y+y+b4y*c+b4x*s,2.0)))-1.7896E-1))*(l4*l4-pow(r_p*-2.237E+3-r_p*acos(1.0/sqrt(pow(-a4y+y+b4y*c+b4x*s,2.0)+pow(-a4x+r_p+x+b4x*c-b4y*s,2.0))*(-a4y+y+b4y*c+b4x*s))*1.25E+4+r_p*3.141592653589793*1.25E+4+sqrt(-r_p*(a4x*2.0-x*2.0-b4x*c*2.0+b4y*s*2.0)+pow(a4x-x-b4x*c+b4y*s,2.0)+pow(-a4y+y+b4y*c+b4x*s,2.0))*1.25E+4+r_p*acos(1.0/sqrt(pow(-a4y+y+b4y*c+b4x*s,2.0)+pow(-a4x+r_p+x+b4x*c-b4y*s,2.0))*sqrt(-r_p*(a4x*2.0-x*2.0-b4x*c*2.0+b4y*s*2.0)+pow(a4x-x-b4x*c+b4y*s,2.0)+pow(-a4y+y+b4y*c+b4x*s,2.0)))*1.25E+4,2.0)/1.5625E+8)*(r_p*(1.0/pow(pow(-a4y+y+b4y*c+b4x*s,2.0)+pow(-a4x+r_p+x+b4x*c-b4y*s,2.0),3.0/2.0)*1.0/sqrt(-pow(-a4y+y+b4y*c+b4x*s,2.0)/(pow(-a4y+y+b4y*c+b4x*s,2.0)+pow(-a4x+r_p+x+b4x*c-b4y*s,2.0))+1.0)*pow(-a4x+r_p+x+b4x*c-b4y*s,2.0)-(r_p*r_p)*1.0/sqrt(-(-r_p*(a4x*2.0-x*2.0-b4x*c*2.0+b4y*s*2.0)+pow(a4x-x-b4x*c+b4y*s,2.0)+pow(-a4y+y+b4y*c+b4x*s,2.0))/(pow(-a4y+y+b4y*c+b4x*s,2.0)+pow(-a4x+r_p+x+b4x*c-b4y*s,2.0))+1.0)*1.0/pow(pow(-a4y+y+b4y*c+b4x*s,2.0)+pow(-a4x+r_p+x+b4x*c-b4y*s,2.0),3.0/2.0)*1.0/sqrt(-r_p*(a4x*2.0-x*2.0-b4x*c*2.0+b4y*s*2.0)+pow(a4x-x-b4x*c+b4y*s,2.0)+pow(-a4y+y+b4y*c+b4x*s,2.0))*(-a4y+y+b4y*c+b4x*s))+(1.0/sqrt(-r_p*(a4x*2.0-x*2.0-b4x*c*2.0+b4y*s*2.0)+pow(a4x-x-b4x*c+b4y*s,2.0)+pow(-a4y+y+b4y*c+b4x*s,2.0))*(a4y*-2.0+y*2.0+b4y*c*2.0+b4x*s*2.0))/2.0)*-4.0;
	J4theta = (sqrt(-r_p*(a4x*2.0-x*2.0-b4x*c*2.0+b4y*s*2.0)+pow(a4x-x-b4x*c+b4y*s,2.0)+pow(-a4y+y+b4y*c+b4x*s,2.0))+r_p*(3.141592653589793-acos(1.0/sqrt(pow(-a4y+y+b4y*c+b4x*s,2.0)+pow(-a4x+r_p+x+b4x*c-b4y*s,2.0))*(-a4y+y+b4y*c+b4x*s))+acos(1.0/sqrt(pow(-a4y+y+b4y*c+b4x*s,2.0)+pow(-a4x+r_p+x+b4x*c-b4y*s,2.0))*sqrt(-r_p*(a4x*2.0-x*2.0-b4x*c*2.0+b4y*s*2.0)+pow(a4x-x-b4x*c+b4y*s,2.0)+pow(-a4y+y+b4y*c+b4x*s,2.0)))-1.7896E-1))*(l4*l4-pow(r_p*-2.237E+3-r_p*acos(1.0/sqrt(pow(-a4y+y+b4y*c+b4x*s,2.0)+pow(-a4x+r_p+x+b4x*c-b4y*s,2.0))*(-a4y+y+b4y*c+b4x*s))*1.25E+4+r_p*3.141592653589793*1.25E+4+sqrt(-r_p*(a4x*2.0-x*2.0-b4x*c*2.0+b4y*s*2.0)+pow(a4x-x-b4x*c+b4y*s,2.0)+pow(-a4y+y+b4y*c+b4x*s,2.0))*1.25E+4+r_p*acos(1.0/sqrt(pow(-a4y+y+b4y*c+b4x*s,2.0)+pow(-a4x+r_p+x+b4x*c-b4y*s,2.0))*sqrt(-r_p*(a4x*2.0-x*2.0-b4x*c*2.0+b4y*s*2.0)+pow(a4x-x-b4x*c+b4y*s,2.0)+pow(-a4y+y+b4y*c+b4x*s,2.0)))*1.25E+4,2.0)/1.5625E+8)*((1.0/sqrt(-r_p*(a4x*2.0-x*2.0-b4x*c*2.0+b4y*s*2.0)+pow(a4x-x-b4x*c+b4y*s,2.0)+pow(-a4y+y+b4y*c+b4x*s,2.0))*(a4x*b4x*s*-2.0-a4y*b4y*s*2.0+b4y*r_p*c*2.0+b4y*x*c*2.0-b4x*y*c*2.0+b4x*r_p*s*2.0+b4x*x*s*2.0+b4y*y*s*2.0-a4x*b4y*c*2.0+a4y*b4x*c*2.0))/2.0-r_p*(1.0/pow(pow(-a4y+y+b4y*c+b4x*s,2.0)+pow(-a4x+r_p+x+b4x*c-b4y*s,2.0),3.0/2.0)*1.0/sqrt(-pow(-a4y+y+b4y*c+b4x*s,2.0)/(pow(-a4y+y+b4y*c+b4x*s,2.0)+pow(-a4x+r_p+x+b4x*c-b4y*s,2.0))+1.0)*(-a4x+r_p+x+b4x*c-b4y*s)*(b4x*b4x+b4y*b4y+a4x*b4y*s-a4y*b4x*s+b4x*r_p*c+b4x*x*c+b4y*y*c-b4y*r_p*s-b4y*x*s+b4x*y*s-a4x*b4x*c-a4y*b4y*c)+(r_p*r_p)*1.0/sqrt(-(-r_p*(a4x*2.0-x*2.0-b4x*c*2.0+b4y*s*2.0)+pow(a4x-x-b4x*c+b4y*s,2.0)+pow(-a4y+y+b4y*c+b4x*s,2.0))/(pow(-a4y+y+b4y*c+b4x*s,2.0)+pow(-a4x+r_p+x+b4x*c-b4y*s,2.0))+1.0)*1.0/pow(pow(-a4y+y+b4y*c+b4x*s,2.0)+pow(-a4x+r_p+x+b4x*c-b4y*s,2.0),3.0/2.0)*1.0/sqrt(-r_p*(a4x*2.0-x*2.0-b4x*c*2.0+b4y*s*2.0)+pow(a4x-x-b4x*c+b4y*s,2.0)+pow(-a4y+y+b4y*c+b4x*s,2.0))*(-a4x*b4x*s-a4y*b4y*s+b4y*r_p*c+b4y*x*c-b4x*y*c+b4x*r_p*s+b4x*x*s+b4y*y*s-a4x*b4y*c+a4y*b4x*c)))*4.0;
	
	Eigen::MatrixXd J(4, 3);
	J << J1x, J1y, J1theta,
	     J2x, J2y, J2theta,
		 J3x, J3y, J3theta,
		 J4x, J4y, J4theta;
	/*std::cout << "\n" << std::endl;
	std::cout << J << std::endl;*/
	return J;
}

Eigen::Vector4d fk_nusq(const Eigen::Ref<const Eigen::MatrixXd>& a_i,
	const Eigen::Ref<const Eigen::MatrixXd>& b_b,
	const Eigen::Ref<const Eigen::Vector3d>& q,
	const Eigen::Ref<const Eigen::Vector4d>& ln,
	double r_p
) {
	double x     = q(0);
	double y     = q(1);
	double theta = q(2);

	double c = cos(theta);
	double s = sin(theta);

	double a1x, a1y;
	double a2x, a2y;
	double a3x, a3y;
	double a4x, a4y;

	a1x = a_i(0, 0);
	a1y = a_i(1, 0);

	a2x = a_i(0, 1);
	a2y = a_i(1, 1);

	a3x = a_i(0, 2);
	a3y = a_i(1, 2);

	a4x = a_i(0, 3);
	a4y = a_i(1, 3);

	double b1x, b1y;
	double b2x, b2y;
	double b3x, b3y;
	double b4x, b4y;

	b1x = b_b(0, 0);
	b1y = b_b(1, 0);

	b2x = b_b(0, 1);
	b2y = b_b(1, 1);

	b3x = b_b(0, 2);
	b3y = b_b(1, 2);

	b4x = b_b(0, 3);
	b4y = b_b(1, 3);

	double l1 = ln(0);
	double l2 = ln(1);
	double l3 = ln(2);
	double l4 = ln(3);

	double nu1sq = pow(pow(sqrt(r_p*(a1x*2.0-x*2.0-b1x*c*2.0+b1y*s*2.0)+pow(a1x-x-b1x*c+b1y*s,2.0)+pow(-a1y+y+b1y*c+b1x*s,2.0))+r_p*(3.141592653589793-acos(1.0/sqrt(pow(-a1y+y+b1y*c+b1x*s,2.0)+pow(a1x+r_p-x-b1x*c+b1y*s,2.0))*(-a1y+y+b1y*c+b1x*s))+acos(1.0/sqrt(pow(-a1y+y+b1y*c+b1x*s,2.0)+pow(a1x+r_p-x-b1x*c+b1y*s,2.0))*sqrt(r_p*(a1x*2.0-x*2.0-b1x*c*2.0+b1y*s*2.0)+pow(a1x-x-b1x*c+b1y*s,2.0)+pow(-a1y+y+b1y*c+b1x*s,2.0)))-1.7896E-1),2.0)-l1*l1,2.0);
	double nu3sq = pow(pow(r_p*(acos(1.0/sqrt(pow(-a3y+y+b3y*cos(theta)+b3x*sin(theta),2.0)+pow(-a3x+r_p+x+b3x*cos(theta)-b3y*sin(theta),2.0))*(-a3y+y+b3y*cos(theta)+b3x*sin(theta)))+acos(1.0/sqrt(pow(-a3y+y+b3y*cos(theta)+b3x*sin(theta),2.0)+pow(-a3x+r_p+x+b3x*cos(theta)-b3y*sin(theta),2.0))*sqrt(-r_p*(a3x*2.0-x*2.0-b3x*cos(theta)*2.0+b3y*sin(theta)*2.0)+pow(a3x-x-b3x*cos(theta)+b3y*sin(theta),2.0)+pow(-a3y+y+b3y*cos(theta)+b3x*sin(theta),2.0)))-1.7896E-1)+sqrt(-r_p*(a3x*2.0-x*2.0-b3x*cos(theta)*2.0+b3y*sin(theta)*2.0)+pow(a3x-x-b3x*cos(theta)+b3y*sin(theta),2.0)+pow(-a3y+y+b3y*cos(theta)+b3x*sin(theta),2.0)),2.0)-l3*l3,2.0);
	double nu2sq = pow(pow(r_p*(acos(1.0/sqrt(pow(-a2y+y+b2y*c+b2x*s,2.0)+pow(a2x+r_p-x-b2x*c+b2y*s,2.0))*(-a2y+y+b2y*c+b2x*s))+acos(1.0/sqrt(pow(-a2y+y+b2y*c+b2x*s,2.0)+pow(a2x+r_p-x-b2x*c+b2y*s,2.0))*sqrt(r_p*(a2x*2.0-x*2.0-b2x*c*2.0+b2y*s*2.0)+pow(a2x-x-b2x*c+b2y*s,2.0)+pow(-a2y+y+b2y*c+b2x*s,2.0)))-1.7896E-1)+sqrt(r_p*(a2x*2.0-x*2.0-b2x*c*2.0+b2y*s*2.0)+pow(a2x-x-b2x*c+b2y*s,2.0)+pow(-a2y+y+b2y*c+b2x*s,2.0)),2.0)-l2*l2,2.0);
	double nu4sq = pow(pow(sqrt(-r_p*(a4x*2.0-x*2.0-b4x*c*2.0+b4y*s*2.0)+pow(a4x-x-b4x*c+b4y*s,2.0)+pow(-a4y+y+b4y*c+b4x*s,2.0))+r_p*(3.141592653589793-acos(1.0/sqrt(pow(-a4y+y+b4y*c+b4x*s,2.0)+pow(-a4x+r_p+x+b4x*c-b4y*s,2.0))*(-a4y+y+b4y*c+b4x*s))+acos(1.0/sqrt(pow(-a4y+y+b4y*c+b4x*s,2.0)+pow(-a4x+r_p+x+b4x*c-b4y*s,2.0))*sqrt(-r_p*(a4x*2.0-x*2.0-b4x*c*2.0+b4y*s*2.0)+pow(a4x-x-b4x*c+b4y*s,2.0)+pow(-a4y+y+b4y*c+b4x*s,2.0)))-1.7896E-1),2.0)-l4*l4,2.0);
	Eigen::Vector4d nusqv(nu1sq, nu2sq, nu3sq, nu4sq);
	return nusqv;
}

Eigen::Vector3d forward_kinematics(const Eigen::Ref<const Eigen::MatrixXd>& a_i,
								   const Eigen::Ref<const Eigen::MatrixXd>& b_b,
								   const Eigen::Ref<const Eigen::Vector3d>& q0,
								   const Eigen::Ref<const Eigen::Vector4d>& ln,
								   double r_p) {


	Eigen::Vector3d qi = q0;
	Eigen::MatrixXd J = fk_jacobian(a_i, b_b, qi, ln, r_p);
	Eigen::MatrixXd A = J.transpose()*J;
	//std::cout << A << std::endl;
	Eigen::Vector4d phi = fk_nusq(a_i, b_b, qi, ln, r_p);
	//std::cout << phi << std::endl;

	Eigen::Vector3d g = J.transpose()*phi;

	// MarLev position estimation constants
	double tau      = 1*pow(10, -6);
	double epsilon1 = 1*pow(10, -17);
	double epsilon2 = 1*pow(10, -17);
	double xi       = 2;
	double mu       = tau*A.diagonal().maxCoeff();
	//std::cout << mu << std::endl;

	int iter = 0;
	int maxiter = 100;

	Eigen::Matrix3d eye3 = Eigen::Matrix3d::Identity();
	Eigen::Matrix3d tmp;
	Eigen::Vector2d t; 
	Eigen::Vector3d h;
	Eigen::Vector3d qnew;
	double Phi = 0;
	double Phi_x_plus_h = 0;
	Eigen::Vector4d phi_x_plus_h;

	double rho = 0;

	bool stop = false;

	while (!stop && iter < maxiter) {
		iter++;
		tmp = A + mu * eye3;
		h = tmp.colPivHouseholderQr().solve(-g);
		//std::cout << "h:" << std::endl;
		//std::cout << h << std::endl;

		if (h.norm() <= epsilon2 * (qi.norm() + epsilon2)) {
			stop = true;
		} else {
			qnew = qi + h;

			// Gain ratio
			Phi = 0.5*phi.transpose()*phi;
			phi_x_plus_h = fk_nusq(a_i, b_b, qnew, ln, r_p);

			Phi_x_plus_h = 0.5*phi_x_plus_h.transpose()*phi_x_plus_h;
			rho = (Phi - Phi_x_plus_h) / (0.5*h.transpose()*(mu*h - g));
			if (rho > 0) {
				qi = qnew;
				J = fk_jacobian(a_i, b_b, qi, ln, r_p);
				A = J.transpose()*J;
				phi = fk_nusq(a_i, b_b, qi, ln, r_p);
				g = J.transpose()*phi;
				if (g.norm() < epsilon1) {
					stop = true;
				}
				t << 1 / 3, pow(1 - (2 * rho - 1), 3);
				mu = mu * tmp.maxCoeff();
				xi = 2;
			} else {
				mu = mu * xi;
				xi *= 2;
			}
		}
	}

	return qi;
}


Eigen::Matrix3d skewvec2(const Eigen::Ref<const Eigen::Vector2d>& u) {
	Eigen::Matrix3d uskew;
	uskew <<  0,    0,  u(1),
		      0, 0,    -u(0),
		     -u(1), u(0),  0;
	return uskew;
}

Eigen::MatrixXd calculate_structure_matrix(const Eigen::Ref<const Eigen::MatrixXd>& a_i,
										   const Eigen::Ref<const Eigen::MatrixXd>& b_b,
										   const Eigen::Ref<const Eigen::Vector3d>& q,
										   const Eigen::Ref<const Eigen::Vector4d>& betar,
										   double r_p) {
	double x     = q(0);
	double y     = q(1);
	double theta = q(2);

	double c = cos(theta);
	double s = sin(theta);

	Eigen::Vector2d r(x, y);

	double beta1p = PI   + betar(0);
	double beta2p = PI   - betar(1);
	double beta3p = 0    + betar(2);
	double beta4p = 2*PI - betar(3);

	Eigen::Vector4d betap;
	betap << PI   + betar(0),
		     PI   - betar(1),
		     0    + betar(2),
		     2*PI - betar(3);


	Eigen::Vector2d c1, c2, c3, c4;
	c1 << a_i(0,0) + r_p*(1 + cos(betap(0))),
		  a_i(1,0) + r_p*sin(betap(0));
	c2 << a_i(0,1) + r_p*(1 + cos(betap(1))),
		  a_i(1,1) + r_p*sin(betap(1));
	c3 << a_i(0,2) + r_p*(cos(betap(2)) - 1),
		  a_i(1,2) + r_p*sin(betap(2));
	c4 << a_i(0,3) + r_p*(cos(betap(3)) - 1),
		  a_i(1,3) + r_p*sin(betap(3));
	Eigen::MatrixXd c_i(2, 4);
	c_i << c1, c2, c3, c4;

	Eigen::Matrix2d R;
	R << c, -s,
		 s,  c;

	Eigen::MatrixXd b_i(2, 4);
	b_i << R*b_b(Eigen::all, 0),
		   R*b_b(Eigen::all, 1),
		   R*b_b(Eigen::all, 2),
		   R*b_b(Eigen::all, 3);

	Eigen::Vector2d l1, l2, l3, l4;
	l1 = c_i(Eigen::all, 0) - r - b_i(Eigen::all, 0);
	l2 = c_i(Eigen::all, 1) - r - b_i(Eigen::all, 1);
	l3 = c_i(Eigen::all, 2) - r - b_i(Eigen::all, 2);
	l4 = c_i(Eigen::all, 3) - r - b_i(Eigen::all, 3);

	Eigen::Vector2d u1, u2, u3, u4;
	u1 = l1 / l1.norm();
	u2 = l2 / l2.norm();
	u3 = l3 / l3.norm();
	u4 = l4 / l4.norm();

	Eigen::MatrixXd u(2, 4);
	u << u1(0), u2(0), u3(0), u4(0),
		 u1(1), u2(1), u3(1), u4(1);
	//u << u1, u2, u3, u4;

	Eigen::Vector4d bcrossu;
	//Eigen::Matrix3d biskew;
	
	for (uint8_t i = 0; i < 4; i++) {
		//biskew << skewvec2(b_i(Eigen::all, i));
		bcrossu(i) = b_i(0, i)*u(1, i) - b_i(1, i)*u(0, i);
	}
	Eigen::MatrixXd AT(3, 4);
	AT << u(0, 0), u(0, 1), u(0, 2), u(0, 3),
		  u(1, 0), u(1, 1), u(1, 2), u(1, 3),
		  bcrossu(0), bcrossu(1), bcrossu(2), bcrossu(3);
	//AT << u, bcrossu;
	return AT;
}

Eigen::VectorXd fa_gradient_x(const Eigen::Ref<const Eigen::VectorXd>& x,
							  double f_min,
							  double f_max, 
							  double f_ref, 
							  int p,
							  double c,
							  double b,
							  double epsilon) {

	double f1 = x(0, 0);
	double f2 = x(1, 0);
	double f3 = x(2, 0);
	double f4 = x(3, 0);

	double s1 = x(4, 0);
	double s2 = x(5, 0);
	double s3 = x(6, 0);

	double t2 = -f_min;
	double t3 = -f_max;
	double t4 = -f_ref;
	double t5 = -p;
	double t6 = p - 1.0;
	double t7 = f_min / 2.0;
	double t8 = f_max / 2.0;
	double t9 = -t7;
	double t10 = t8 + t9;
	double t11 = pow(t10, t5);
	
	Eigen::VectorXd gradient_x(7);
	gradient_x << -c/(f1 + t2) - c/(f1 + t3) + p*t11*pow(f1 + t4, t6),
			   	  -c/(f2 + t2) - c/(f2 + t3) + p*t11*pow(f2 + t4, t6),
			   	  -c/(f3 + t2) - c/(f3 + t3) + p*t11*pow(f3 + t4, t6),
			   	  -c/(f4 + t2) - c/(f4 + t3) + p*t11*pow(f4 + t4, t6),
			   	   s1*2.0 + b * s1*1.0/sqrt(epsilon + s1*s1),
			       s2*2.0 + b * s2*1.0/sqrt(epsilon + s2*s2),
				   s3*2.0 + b * s3*1.0/sqrt(epsilon + s3*s3);
	return gradient_x;
}

Eigen::MatrixXd fa_hessian_x(const Eigen::Ref<const Eigen::VectorXd>& x,
							 double f_min,
							 double f_max,
							 double f_ref,
							 int p,
							 double c,
							 double b,
							 double epsilon) {
	double f1 = x(0, 0);
	double f2 = x(1, 0);
	double f3 = x(2, 0);
	double f4 = x(3, 0);

	double s1 = x(4, 0);
	double s2 = x(5, 0);
	double s3 = x(6, 0);

	double t2 = pow(s1, 2);
	double t3 = pow(s2, 2);
	double t4 = pow(s3, 2);

	double t5 = -f_min;
	double t6 = -f_max;
	double t7 = -f_ref;

	double t8 = -p;
	double t9 = p - 1.0;

	double t10 = t9 - 1.0;
	double t11 = epsilon + t2;
	double t12 = epsilon + t3;
	double t13 = epsilon + t4;

	double t14 = f_min / 2.0;
	double t15 = f_max / 2.0;

	double t16 = -t14;
	double t17 = t15 + t16;

	double t18 = pow(t17, t8);

	Eigen::MatrixXd hessian_x = Eigen::MatrixXd::Zero(7,7);
	hessian_x(0,0) = c*1.0/pow(f1+t5,2.0)+c*1.0/pow(f1+t6,2.0)+p*t9*t18*pow(f1+t7,t10);
	hessian_x(1,1) = c*1.0/pow(f2 + t5, 2.0) + c*1.0/pow(f2 + t6, 2.0) + p*t9*t18*pow(f2 + t7, t10);
	hessian_x(2,2) = c*1.0/pow(f3 + t5, 2.0) + c*1.0/pow(f3 + t6, 2.0) + p*t9*t18*pow(f3 + t7, t10);
	hessian_x(3,3) = c*1.0/pow(f4 + t5, 2.0) + c*1.0/pow(f4 + t6, 2.0) + p*t9*t18*pow(f4 + t7, t10);
	hessian_x(4,4) = b*1.0/sqrt(t11) - b*t2*1.0/pow(t11, 3.0/2.0) + 2.0;
	hessian_x(5,5) = b*1.0/sqrt(t12) - b*t3*1.0/pow(t12, 3.0/2.0) + 2.0;
	hessian_x(6,6) = b*1.0/sqrt(t13) - b*t4*1.0/pow(t13, 3.0/2.0) + 2.0;

	return hessian_x;
}

double fa_merit_function(const Eigen::Ref<const Eigen::VectorXd>& z, 
						 const Eigen::Ref<const Eigen::MatrixXd>& G_x, 
						 const Eigen::Ref<const Eigen::MatrixXd>& A_,
						 const Eigen::Ref<const Eigen::Vector3d>& w_ref) {

	Eigen::VectorXd x_      = z(Eigen::seq(0,6));
	Eigen::Vector3d lambda_ = z(Eigen::seq(7, 9));
	Eigen::VectorXd phivec(G_x.rows() + A_.rows());
	phivec << G_x + A_.transpose()*lambda_,
			  A_*x_ - w_ref;

	double phi_merit = phivec.lpNorm<Eigen::Infinity>();
	return phi_merit;
}
// Function for calculating the "Directional Derivative" of merit function
// (Markus Grasmair, Department of Mathematics,
// Norwegian University of Science and Technology,
// Trondheim, Norway)
//
// (OBS: Tror ikke denne er helt riktig)
double fa_d_merit(const Eigen::Ref<const Eigen::VectorXd>& z,
				  const Eigen::Ref<const Eigen::MatrixXd>& G_x,
				  const Eigen::Ref<const Eigen::MatrixXd>& A_,
				  const Eigen::Ref<const Eigen::Vector3d>& w_ref,
				  const Eigen::Ref<const Eigen::VectorXd>& d_k) {
	Eigen::VectorXd x_ = z(Eigen::seq(0, 6));
	Eigen::Vector3d lambda_ = z(Eigen::seq(7, 9));
	
	Eigen::VectorXd p_k = d_k(Eigen::seq(0, 6));
	Eigen::VectorXd dphivec(A_.cols() + A_.rows());

	dphivec << (G_x.transpose()*p_k)(0)*Eigen::VectorXd::Ones(A_.cols()) + A_.transpose()*lambda_,  // VEEELDIG USIKKER PÅ DENNE
			   A_*x_ - w_ref;
	double dphi_merit = dphivec.lpNorm<Eigen::Infinity>();
	return dphi_merit;
}

force_alloc_res force_alloc_iterative_slack(const Eigen::Ref<const Eigen::MatrixXd>& A,
										    double f_min,
										    double f_max,
										    double f_ref,
										    const Eigen::Ref<const Eigen::Vector4d>& f_prev,
										    const Eigen::Ref<const Eigen::Vector3d>& w_ref) {
	force_alloc_res res;
	res.flag = 0;
	constexpr int m = 3; // Number of controllable DOFs
	constexpr int n = 4; // Number of actuating cables
	int p = 2; // P-norm value

	// Defining the optimization matrices
	Eigen::MatrixXd W = A.transpose();
	Eigen::Matrix3d Q = Eigen::Matrix3d::Identity();

	Eigen::MatrixXd A_(W.rows(), W.cols() + Q.cols());
	A_ << W, Q;


	// Parameters
	double c       = 1.5;         // Parameter adjusting how fast the cost function for the
								  // standard formulation increases
	double epsilon = pow(10, -3); // Parameter adjusting the curvature of the cost function
								  // for the slacked formulation
	double b = 200;				  // Parameter steering the gradient of the cost term for 
								  // the slacked formulation
	double c_phi = 1;			  // Parameter for checking merit function value

	// Newtons method on the KKT conditions
	// Initialization
	int iter    = 0;    // Initializing iteration counter
	int itermax = 100;  // Maximum iterations

	Eigen::Vector4d f = f_prev;                  // Initial force
	double f0 = f_ref;                           // Reference force
	Eigen::Vector3d s = Eigen::Vector3d::Zero(); // Initial slack variables
	Eigen::VectorXd x(f.rows() + s.rows());      // Optimization variables
	x << f, s;

	Eigen::Vector3d lambda      = Eigen::Vector3d::Zero(); // Initial Lagrangian multipliers
	Eigen::Vector3d lambda_prev = Eigen::Vector3d::Zero(); // Previous Lagrangian multipliers
	
	Eigen::VectorXd z(x.rows() + lambda.rows()); // Initial full state
	z << x, lambda;
	double tol = 5 * pow(10, -5);

	Eigen::Matrix3d zero_block = Eigen::Matrix3d::Zero();

	double kappa = 1; // Initial Step Size for Newton Step
	
	int iter_merit = 0;
	int iter_merit_max = 100;


	while (iter <= itermax) {
		// 1) Calculate Newton step

		Eigen::VectorXd G_x = fa_gradient_x(x, f_min, f_max, f_ref, p, c, b, epsilon); // Calculate gradient of objective function
		Eigen::MatrixXd H_x = fa_hessian_x(x, f_min, f_max, f_ref, p, c, b, epsilon);  // Calculate hessian of objective function

		Eigen::MatrixXd A_KKT(H_x.rows() + A_.rows(), A_.cols() + zero_block.cols()); // KKT Matrix
		A_KKT << H_x, A_.transpose(),
			     A_, zero_block;
		
		Eigen::VectorXd B_KKT(G_x.rows() + (A_*x - w_ref).rows()); 
		B_KKT << G_x, 
			     A_*x - w_ref;
		
		Eigen::VectorXd d_k = A_KKT.colPivHouseholderQr().solve(-B_KKT); // Calculate Newton step
		d_k(Eigen::seq(7, 9)) -= lambda_prev;

		// 2) Calculate step length
		kappa = 1;

		// Linesearch with Merit function
		double phi_merit = fa_merit_function(z, G_x, A_, w_ref);
		double phi_kappa = fa_merit_function(z + kappa * d_k, G_x, A_, w_ref);
		double dphi = fa_d_merit(z, G_x, A_, w_ref, d_k);
		
		// Check if merit function is below the predetermined tolerance threshold
		if (phi_merit < tol) {
			break;
		}
		// Else, update
		iter_merit = 0;
		while ((phi_kappa > phi_merit + c_phi * kappa*dphi) 
			   && (iter_merit <= iter_merit_max)) {
			kappa -= 0.01;
			if (kappa <= 0) {
				kappa = 0;
				break;
			}
			
			phi_kappa = fa_merit_function(z + kappa * d_k, G_x, A_, w_ref);
			// Update iteration for merit function
			iter_merit++;
			if (iter_merit > iter_merit_max) {
				std::cout << "FA: Too many iterations (Merit function)" << std::endl;
			}
		}
		
		// 3) Perform Newton step
		z += kappa * d_k;
		// Update previous lambda
		lambda_prev = z(Eigen::seq(7, 9));

		// Extract state
		x = z(Eigen::seq(0,6));

		// 4) Update main iterations
		iter++;
		if (iter > itermax) {
			std::cout << "FA: Max iterations reached. Setting forces equal to previous ones." << std::endl;
			res.f = f_prev;
			res.w = Eigen::Vector3d(0, 0, 0);
			res.flag = 1;
			return res;
		}
	}

	res.f = z(Eigen::seq(0, 3));
	//if ((res.f).array() <= 0.0).Eigen::any();
	//std::cout << "FA: z: \n" << z << std::endl;
	//std::cout << "FA: res.f: \n" << res.f << std::endl;
	for (uint8_t i = 0; i < 4; i++) {
		if (res.f(i) < 0) {
			res.f = f_prev;
			res.flag = 1;
			std::cout << "FA: Forces below zero. Setting forces equal to previous ones." << std::endl;
			break;
		}
	}

	res.w = W * f;
	return res;
}