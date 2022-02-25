#include <iostream>
#include <Eigen/Dense>
#include "kukalbriiwa_robot_model.h"
#include "analytic_inverse_kinematics_solver.h"
#include "robot_params.h"
#include <chrono>

typedef Eigen::Matrix<double, 7, 1> JointVector;
typedef Eigen::Matrix<double, 7, 7> JointMatrix;
typedef Eigen::Matrix<double, 6, 7> JacobianMatrix;
typedef Eigen::Matrix<double, 4, 4> HomogeneousTransformation;

typedef KukaLbrIiwaRobotModel<double, JointMatrix, JacobianMatrix, JointVector, HomogeneousTransformation> EigenRobotModel;

typedef AnalyticInverseKinematicsSolver IKSolver;

int main(int argc, char** argv)
{
  const RobotParams param;

  JointVector q = JointVector::Zero();
  JointVector qd = JointVector::Ones();

  JointMatrix M;
  JointMatrix C;
  JointVector G;
  JacobianMatrix J;
  JacobianMatrix Jp;
  HomogeneousTransformation T0E;

  EigenRobotModel::evalM(q, M, param);
  std::cout << std::endl << "M = " << M << std::endl;

  EigenRobotModel::evalC(q, qd, C, param);
  std::cout << "C = " << std::endl << C << std::endl;

  EigenRobotModel::evalG(q, G, param);
  std::cout << "G = " << std::endl << G << std::endl;

  EigenRobotModel::evalJ(q, J, param);
  std::cout << "J = " << std::endl << J << std::endl;

  EigenRobotModel::forwardKinematics(q, T0E, param);
  std::cout << "T0E = " << std::endl << T0E << std::endl;

  JointVector q_wrap1 = M_PI * JointVector::Ones();
  std::cout << "wrap(M_PI): " << std::endl << IKSolver::wrap(q_wrap1) << std::endl;

  JointVector q_wrap2 = 2 * M_PI * JointVector::Ones();
  std::cout << "wrap(2 * M_PI): " << std::endl << IKSolver::wrap(q_wrap2) << std::endl;

  JointVector q_wrap3 = -M_PI * JointVector::Ones();
  std::cout << "wrap(-M_PI): " << std::endl << IKSolver::wrap(q_wrap3) << std::endl;

  JointVector q_wrap4 = -2 * M_PI * JointVector::Ones();
  std::cout << "wrap(-2 * M_PI): " << std::endl << IKSolver::wrap(q_wrap4) << std::endl;

  JointVector q_wrap;
  q_wrap << -5.5795, 5.3706, 5.1675, 4.9750, 4.7974, 4.6403, 4.5186;
  std::cout << "wrap: " << std::endl << IKSolver::wrap(q_wrap) << std::endl;

  HomogeneousTransformation x_d;
  /*
  x_d << -1, 0, 0, 0.5,
         0, 1, 0, 0.2,
         0, 0, -1, 0.32,
         0, 0, 0, 1;
  */
  x_d << 1, 0, 0, -0.2828,
         0, 1, 0, -0.24,
         0, 0, 1, 1.15,
         0, 0, 0, 1;
  JointVector q_ik_sol;

  auto t1 = std::chrono::high_resolution_clock::now();
  bool success = false;
  IKSolver::calcAnalyticInverseKinematics(x_d, false, param, success, q_ik_sol);
  auto t2 = std::chrono::high_resolution_clock::now();

  std::cout << "q_ik_sol = " << std::endl << q_ik_sol << std::endl;

  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count();
  std::cout << "IK solution time in milliseconds: " << duration << std::endl;

}
