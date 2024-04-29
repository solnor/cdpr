#pragma once
#include <Windows.h>

int buf_size = 0xa;

typedef struct odrive_state {

};
#include "odrive_definitions.h"

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
int get_all_motor_states(HANDLE handle, motor_state *states);

int set_abs_position(HANDLE handle, double pos);
int set_axis_state(HANDLE handle, axis_states ax_state);

int set_motor_torque(HANDLE handle, double torque);
