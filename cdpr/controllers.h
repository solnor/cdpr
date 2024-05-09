#pragma once
#include <Dense>
#include "commands.h"
#include "algorithms.h"

//int tension_control_loop(HANDLE handles[]);
int tension_control_loop(HANDLE handles[], const Eigen::Ref<const Eigen::Vector4d>& p0_in);
int position_control_loop(HANDLE handles[]);
int pos_test(HANDLE handles[]);