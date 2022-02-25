#ifndef EVALJ_H
#define EVALJ_H
#include "ros_acin_robot_support/models/robot_model.h"
extern "C" void py_acin_evalJ(double *q, double *J, const RobotParams *param);
extern void evalJ (const Eigen::Ref<JointVector_t> q, Eigen::Ref<JacobianMatrix_t> J, const RobotParams& param);
void py_acin_evalJ(double* q, double* J, const RobotParams* param);

#endif // EVALJ_H
