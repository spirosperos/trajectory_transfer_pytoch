#ifndef EVALT0E_H
#define EVALT0E_H
#include "ros_acin_robot_support/models/robot_model.h"
extern "C" void py_acin_evalT0E(double *q, double *T0E, const RobotParams *param);
extern void evalT0E (const Eigen::Ref<JointVector_t> q, Eigen::Ref<HomogeneousTransformation_t> T0E, const RobotParams& param);
extern void _evalT0E_ (const Eigen::Ref<JointVector_t> q, Eigen::Ref<HomogeneousTransformation_t> T0E, const RobotParams& param);
void py_acin_evalT0E(double* q, double* T0E, const RobotParams* param);


#endif // EVALT0E_H
