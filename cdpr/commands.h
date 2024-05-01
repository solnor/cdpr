#pragma once
#include "odrive_definitions.h"
#include "communication.h"
#include <vector>
#include <Dense>

typedef struct odrive_state {
	double k = 0.0;
};

typedef struct motor_state {
	double pos;
	double vel;
};

int read_state(HANDLE handle, odrive_state *state);
int find_driver_errors(HANDLE handle, odrive_state *state);
int read_encoder_position(HANDLE handle, double *pos);
int read_encoder_velocity(HANDLE handle, double *vel);
int read_encoder_pos_vel(HANDLE handle);

int get_motor_state(HANDLE handle, motor_state* state);
int get_all_motor_states(HANDLE handles[], std::vector<motor_state*> motor_states);

int set_abs_position(HANDLE handle, double pos);
int set_axis_state(HANDLE handle, axis_states ax_state);
int set_encoder_position(HANDLE handle, double position);

int set_motor_torque(HANDLE handle, double torque);
int set_all_motor_torques(HANDLE handles[], const Eigen::Ref<const Eigen::Vector4d>& torques);

int read_driver_error_status(HANDLE handle);
int read_all_driver_error_statuses(HANDLE handles[], int *errors);

int check_if_driver_error(HANDLE handle);
int check_if_any_driver_errors(HANDLE handles[]);

