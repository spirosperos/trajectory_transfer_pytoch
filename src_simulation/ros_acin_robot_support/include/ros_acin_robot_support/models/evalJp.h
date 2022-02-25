#ifndef EVALJP_H
#define EVALJP_H
#include "ros_acin_robot_support/models/robot_model.h"
extern "C" void py_evalJp(double *q, double *qp, double *Jp, const RobotParams *param);
extern void evalJp (const Eigen::Ref<JointVector_t> q, const Eigen::Ref<JointVector_t> qd, Eigen::Ref<JacobianMatrix_t> Jp, const RobotParams& param);
void py_evalJp(double* q, double* qp, double* Jp, const RobotParams* param);
#endif // EVALJP_H
