#ifndef CONTROLLER_PARAMS_H
#define CONTROLLER_PARAMS_H

#include <Eigen/Dense>

typedef Eigen::Matrix<double, 7, 1> JointVector_t;
typedef Eigen::Matrix<double, 6, 1> CartesianVector_t;
typedef Eigen::Matrix<double, 6, 6> CartesianMatrix_t;
typedef Eigen::Matrix<double, 7, 7> JointMatrix_t;

struct ControllerParams
{
  // filter paramters
  JointVector_t T1_tau = (JointVector_t::Ones() * 1); //tau filter parameter
  JointVector_t T1_tau_p = (JointVector_t::Ones() * 1); //differentiation filter tau  parameter
  JointVector_t T1_q_dot = (JointVector_t::Ones() * 1); //differentiation filter for the robot coordinates

  //singular perturbation controller paramterts
  JointMatrix_t K_tau_sp = (JointVector_t::Ones() * 0).asDiagonal(); //tau singular perturbation contoller K value
  JointMatrix_t D_tau_sp = (JointVector_t::Ones() * 0).asDiagonal(); //tau singular perturbation contoller D value
  //friction compensation
  double L_array[7] = {200,200,300,300,500,1000,1000};
  JointMatrix_t L = JointVector_t(L_array).asDiagonal();
  JointVector_t sensor_torque_offset = JointVector_t::Zero();
  // cartesian CT controller
  CartesianMatrix_t K0_cart = (CartesianVector_t::Ones() * 120.0).asDiagonal();

  CartesianMatrix_t K1_cart = (CartesianVector_t::Ones() * 60.0).asDiagonal();

  JointMatrix_t K0_N_cart = (JointVector_t::Ones() * 0.008).asDiagonal();
  JointMatrix_t K1_N_cart = (JointVector_t::Ones() * 0.008).asDiagonal();

  // cartesian compliance controller
  //CartesianMatrix K_d = (CartesianVector::Ones() * 300.0).asDiagonal();
  //CartesianMatrix D_d = (CartesianVector::Ones() * 40.0).asDiagonal();

  // joint CT controller
  JointMatrix_t K0_joint = (JointVector_t::Ones() * 600).asDiagonal();
  JointMatrix_t K1_joint = (JointVector_t::Ones() * 60).asDiagonal();
  JointMatrix_t KD_grav = (JointVector_t::Ones() * 1).asDiagonal();
  JointMatrix_t KI_joint = (JointVector_t::Ones() * 80).asDiagonal();
  ControllerParams(bool simFlag=  true){

    if (simFlag){
      T1_tau << 0.3979e-3,0.3979e-3,0.3979e-3,0.3979e-3,0.3183e-3,0.1592e-3,0.1592e-3;
      /**/
      T1_tau_p << 0.3979e-3,0.3979e-3,0.3979e-3,0.3979e-3,0.3183e-3,0.1592e-3,0.1592e-3;
      T1_q_dot <<  0.3979e-3,0.3979e-3,0.3979e-3,0.3979e-3,0.3183e-3,0.1592e-3,0.1592e-3;
      K_tau_sp(0,0) = 4;
      K_tau_sp(1,1) = 4;
      K_tau_sp(2,2) = 4;
      K_tau_sp(3,3) = 5;
      K_tau_sp(4,4) = 3;
      K_tau_sp(5,5) = 2.5;
      K_tau_sp(6,6) = 2.5;

      D_tau_sp(0,0) = 0.015;
      D_tau_sp(1,1) = 0.015;
      D_tau_sp(2,2) = 0.015;
      D_tau_sp(3,3) = 0.02;
      D_tau_sp(4,4) = 0.01;
      D_tau_sp(5,5) = 0.01;
      D_tau_sp(6,6) = 0.01;
      /**/
      K0_joint = (JointVector_t::Ones() * 1200).asDiagonal();
      K1_joint = (JointVector_t::Ones() * 60).asDiagonal();
      KI_joint = (JointVector_t::Ones() * 4000).asDiagonal();
      L(0) = 80;
      L(1) = 80;
      L(2) = 150;
      L(3) = 150;
      L(4) = 250;
      L(5) = 800;
      L(6) = 800;	  
    }else{
      T1_tau << 0.3979e-3,0.3979e-3,0.3979e-3,0.3979e-3,0.3183e-3,0.1592e-3,0.1592e-3;
      /**/
      T1_tau_p << 0.3979e-3,0.3979e-3,0.3979e-3,0.3979e-3,0.3183e-3,0.1592e-3,0.1592e-3;
      T1_q_dot <<  0.3979e-3,0.3979e-3,0.3979e-3,0.3979e-3,0.3183e-3,0.1592e-3,0.1592e-3;
      /*parameter from matlab rigid body controller*/

      /*singular pertubation part*/
      K_tau_sp(0,0) = 4;
      K_tau_sp(1,1) = 4;
      K_tau_sp(2,2) = 4;
      K_tau_sp(3,3) = 5;
      K_tau_sp(4,4) = 3;
      K_tau_sp(5,5) = 2.5;
      K_tau_sp(6,6) = 2.5;

      D_tau_sp(0,0) = 0.015;
      D_tau_sp(1,1) = 0.015;
      D_tau_sp(2,2) = 0.015;
      D_tau_sp(3,3) = 0.02;
      D_tau_sp(4,4) = 0.01;
      D_tau_sp(5,5) = 0.01;
      D_tau_sp(6,6) = 0.01;
      /**/
      K0_joint = (JointVector_t::Ones() * 1200).asDiagonal();
      K1_joint = (JointVector_t::Ones() * 60).asDiagonal();
      KI_joint = (JointVector_t::Ones() * 8000).asDiagonal();

      //%Torque sensor offsets, measured and identified by Bischof on 25.06.2018
      sensor_torque_offset<< 0.53,-0.095,-0.189,0.07,-0.058,0.0025,-0.12;
    }
  }
};

#endif // CONTROLLER_PARAMS_H
