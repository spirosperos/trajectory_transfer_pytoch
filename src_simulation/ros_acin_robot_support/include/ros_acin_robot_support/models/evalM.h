#ifndef EVALM_H
#define EVALM_H
#include "ros_acin_robot_support/models/robot_model.h"
extern "C" void py_acin_evalM(double *q, double *M, const RobotParams *param);
extern void evalM (const Eigen::Ref<JointVector_t> q, Eigen::Ref<JointMatrix_t> M, const RobotParams& param);
void py_acin_evalM(double* q, double* M, const RobotParams* param);
#endif // EVALM_H
