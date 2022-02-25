#pragma once
#ifndef EVALC_H
#define EVALC_H
#include "ros_acin_robot_support/models/robot_model.h"
extern "C" void py_acin_evalC(double *q, double *qd,double *C, const RobotParams *param);
extern void evalC (const Eigen::Ref<JointVector_t> q, const Eigen::Ref<JointVector_t> qd, Eigen::Ref<JointMatrix_t> C, const RobotParams& param);
void py_acin_evalC(double* q, double* qd, double* C, const RobotParams* param);
#endif // EVALC_H
