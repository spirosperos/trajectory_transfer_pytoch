#ifndef EVALG_H
#define EVALG_H
#include "ros_acin_robot_support/models/robot_model.h"
extern "C" void py_acin_evalG(double *q, double *G, const RobotParams *param);
extern void evalG(const Eigen::Ref<JointVector_t> q, Eigen::Ref<JointVector_t> G, const RobotParams& param);
void py_acin_evalG(double* q, double* G, const RobotParams* param);
#endif // EVALG_H
