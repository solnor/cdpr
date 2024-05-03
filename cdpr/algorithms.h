#pragma once
#include <Dense>

#define PI 3.14159265358979323846

typedef struct {
	Eigen::Vector4d l; //= Eigen::MatrixXd(2,4);
	Eigen::Vector4d betar;
} inv_res;

inv_res inverse_kinematics(const Eigen::Ref<const Eigen::MatrixXd>& a_i,
	const Eigen::Ref<const Eigen::MatrixXd>& b_b,
	const Eigen::Ref<const Eigen::Vector3d>& q,
	double r_p
);

Eigen::Vector3d fk_init_estimate(const Eigen::Ref<const Eigen::MatrixXd>& a_i,
							     const Eigen::Ref<const Eigen::MatrixXd>& b_b,
								 const Eigen::Ref<const Eigen::Vector4d>& ln
);

Eigen::MatrixXd fk_jacobian(const Eigen::Ref<const Eigen::MatrixXd>& a_i,
							const Eigen::Ref<const Eigen::MatrixXd>& b_b,
							const Eigen::Ref<const Eigen::Vector3d>& q,
	                        const Eigen::Ref<const Eigen::Vector4d>& ln,
							double r_p
);

Eigen::Vector4d fk_nusq(const Eigen::Ref<const Eigen::MatrixXd>& a_i,
						const Eigen::Ref<const Eigen::MatrixXd>& b_b,
					    const Eigen::Ref<const Eigen::Vector3d>& q,
						const Eigen::Ref<const Eigen::Vector4d>& ln,
						double r_p
);

Eigen::Vector3d forward_kinematics(const Eigen::Ref<const Eigen::MatrixXd>& a_i,
								   const Eigen::Ref<const Eigen::MatrixXd>& b_b,
								   const Eigen::Ref<const Eigen::Vector3d>& q0,
							       const Eigen::Ref<const Eigen::Vector4d>& ln,
								   double r_p);

Eigen::MatrixXd calculate_structure_matrix(const Eigen::Ref<const Eigen::MatrixXd>& a_i,
										   const Eigen::Ref<const Eigen::MatrixXd>& b_b,
										   const Eigen::Ref<const Eigen::Vector3d>& q,
										   const Eigen::Ref<const Eigen::Vector4d>& betar,
										   double r_p);

typedef struct {
	Eigen::Vector4d f;
	Eigen::Vector3d w;
	int flag = 0;
} force_alloc_res;

force_alloc_res force_alloc_iterative_slack(const Eigen::Ref<const Eigen::MatrixXd>& A,
										    double f_min,
										    double f_max,
										    double f_ref,
										    const Eigen::Ref<const Eigen::Vector4d>& f_prev,
										    const Eigen::Ref<const Eigen::Vector3d>& w_ref);