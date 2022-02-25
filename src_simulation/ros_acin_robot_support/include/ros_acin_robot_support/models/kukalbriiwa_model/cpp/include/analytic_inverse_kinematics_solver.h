#ifndef ANALYTIC_INVERSE_KINEMATICS_SOLVER_H
#define ANALYTIC_INVERSE_KINEMATICS_SOLVER_H

#include <cmath>
#include "../parameters/withoutLinAxes/robot_params.h"
#include <Eigen/Dense>

// typedef Eigen::Matrix<double, 7, 1> JointVector;
// typedef Eigen::Matrix<double, 7, 7> JointMatrix;
// typedef Eigen::Matrix<double, 6, 7> JacobianMatrix;
// typedef Eigen::Matrix<double, 4, 4> HomogeneousTransformation;
// typedef Eigen::Matrix<double, 3, 3> RotationMatrix;
// typedef Eigen::Matrix<double, 6, 1> CartesianVector;
typedef Eigen::Matrix<double, 3, 1> Vector3;

class AnalyticInverseKinematicsSolver
{
public:
  static double wrap2pi(double lambda);
  static JointVector_t wrap(JointVector_t lambda);
  static RotationMatrix_t skew(Vector3 v);
  static void calcAnalyticInverseKinematics(HomogeneousTransformation_t x_d, bool find_minimum_norm, const RobotParams& param, bool& success, JointVector_t& q);

};

#endif // ANALYTIC_INVERSE_KINEMATICS_SOLVER_H
