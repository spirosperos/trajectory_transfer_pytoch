#ifndef CONTROLLER_PARAMS_H
#define CONTROLLER_PARAMS_H

#include <Eigen/Dense>

//typedef Eigen::Matrix<double, 7, 1> JointVector_t;
//typedef Eigen::Matrix<double, 6, 1> CartesianVector_t;
//typedef Eigen::Matrix<double, 6, 6> CartesianMatrix_t;
//typedef Eigen::Matrix<double, 7, 7> JointMatrix_t;

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
  double L_array[9] = {200,200,200,200,300,300,500,1000,1000};
  JointMatrix_t L = JointVector_t(L_array).asDiagonal();
  JointVector_t sensor_torque_offset = JointVector_t::Zero();
  // cartesian CT controller
  CartesianMatrix_t K0_cart = (CartesianVector_t::Ones() * 20.0).asDiagonal();

  CartesianMatrix_t K1_cart = (CartesianVector_t::Ones() * 10.0).asDiagonal();

  JointMatrix_t K0_N_cart = (JointVector_t::Ones() * 0.008).asDiagonal();
  JointMatrix_t K1_N_cart = (JointVector_t::Ones() * 0.008).asDiagonal();

  // cartesian compliance controller
  //CartesianMatrix K_d = (CartesianVector::Ones() * 300.0).asDiagonal();
  //CartesianMatrix D_d = (CartesianVector::Ones() * 40.0).asDiagonal();

  // joint CT controller
  JointMatrix_t K0_joint = (JointVector_t::Ones() * 1200).asDiagonal();
  JointMatrix_t K1_joint = (JointVector_t::Ones() * 80).asDiagonal();
  JointMatrix_t KD_grav = (JointVector_t::Ones() * 0.08).asDiagonal();
  JointMatrix_t KI_joint = (JointVector_t::Ones() * 200).asDiagonal();
  ControllerParams(){
    /**/
    T1_tau << 0.3979e-3,0.3979e-3,0.3979e-3,0.3979e-3,0.3979e-3,0.3979e-3,0.3183e-3,0.1592e-3,0.1592e-3;
    /**/
    T1_tau_p << 0.3979e-3,0.3979e-3,0.3979e-3,0.3979e-3,0.3979e-3,0.3979e-3,0.3183e-3,0.1592e-3,0.1592e-3;
    T1_q_dot << 0.3979e-3,0.3979e-3,0.3979e-3,0.3979e-3,0.3979e-3,0.3979e-3,0.3183e-3,0.1592e-3,0.1592e-3;
    /**/
    K0_cart(3,3) = 100;
    K0_cart(4,4) = 100;
    K0_cart(5,5) = 100;
    /**/
    K1_cart(3,3) = 30;
    K1_cart(4,4) = 30;
    K1_cart(5,5) = 30;
    /**/
    K_tau_sp(0,0) = 0.04;
    K_tau_sp(1,1) = 0.04;
    K_tau_sp(2,2) = 0.04;
    K_tau_sp(3,3) = 0.04;
    K_tau_sp(4,4) = 0.04;
    K_tau_sp(5,5) = 0.05;
    K_tau_sp(6,6) = 0.03;
    K_tau_sp(7,7) = 0.025;
    K_tau_sp(8,8) = 0.025;
    /**/
    D_tau_sp(0,0) = 0.0015;
    D_tau_sp(1,1) = 0.0015;
    D_tau_sp(2,2) = 0.0015;
    D_tau_sp(3,3) = 0.0015;
    D_tau_sp(4,4) = 0.0015;
    D_tau_sp(5,5) = 0.002;
    D_tau_sp(6,6) = 0.001;
    D_tau_sp(7,7) = 0.001;
    D_tau_sp(8,8) = 0.001;


    //%Torque sensor offsets, measured and identified by Bischof on 25.06.2018
    sensor_torque_offset << 0,0,0.53,-0.095,-0.189,0.07,-0.058,0.0025,-0.12;
  }
};

#endif // CONTROLLER_PARAMS_H
