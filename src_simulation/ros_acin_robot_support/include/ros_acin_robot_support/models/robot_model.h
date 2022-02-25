#pragma once
#ifndef ROBOT_MODEL_H
#define ROBOT_MODEL_H
#ifdef _ROBOT_q_DIM_
  #error "_ROBOT_q_DIM_: Should not be defined, include file defines it"
#endif
// #ifndef CMAKE_ROBOT_MODEL_FOLDER
//   #error "CMAKE_ROBOT_MODEL_FOLDER"
// #endif
#define CMAKE_ROBOT_MODEL_FOLDER kukalbriiwa_linaxes_model
#define _ROBOT_q_DIM_ 9


#define kukalbriiwa_model 1
#define kukalbriiwa 2
#define kukalbriiwa_linaxes_model 3


#include <ros_acin_robot_support/import_Eigen.h>

typedef Eigen::Matrix<double, _ROBOT_q_DIM_, 1> JointVector_t;
typedef Eigen::Matrix<double, 6, 1> CartesianVector_t; //(x,y,z, (Euler Angles (Yaw, Pitch, Roll)||  omega || omega_p))
typedef Eigen::Matrix<double, 6, 6> CartesianMatrix_t;
typedef Eigen::Matrix<double, 7, 1> PoseVector_t;  //(x, y, z, quotation w, quotation x,quotation y,quotation z)
typedef Eigen::Matrix<double, 3, 1> CartesianPositionVector_t;
typedef Eigen::Matrix<double, 3, 1> angular_t;

typedef Eigen::Matrix<double, _ROBOT_q_DIM_, _ROBOT_q_DIM_> JointMatrix_t;
typedef Eigen::Matrix<double, 4, 4> HomogeneousTransformation_t;
typedef Eigen::Matrix<double, 3, 3> RotationMatrix_t;
typedef Eigen::Matrix<double, 6, _ROBOT_q_DIM_> JacobianMatrix_t;
typedef Eigen::Matrix<double, 6, _ROBOT_q_DIM_,Eigen::RowMajor> JacobianMatrixRowMajor_t;
typedef Eigen::Matrix<double, _ROBOT_q_DIM_, 6> JacobianInverseMatrix_t;
typedef Eigen::Matrix<double, 3, 1,Eigen::RowMajor> Vector3_t;

#ifndef M_PI
    #define M_PI (3.1415926535897931)
#endif
#include "ros_acin_robot_support/import_Eigen.h"
// model headers
#if defined(TWINCAT_COMPILE)
  #define _JointSpaceInterpolator_deg_ 3
  #define _TaskSpaceInterpolator_deg_ 3
  #include <controller_params_TwinCat.h>
#else
  #define _JointSpaceInterpolator_deg_ 3
  #define _TaskSpaceInterpolator_deg_ 3
#endif


#if CMAKE_ROBOT_MODEL_FOLDER == 1
    #ifndef TWINCAT_COMPILE
      #include "kukalbriiwa_model/cpp//parameters/withoutLinAxes/controller_params_simulation.h"
    #endif
   #include "kukalbriiwa_model/cpp/include/kukalbriiwa_robot_model.h"
   #include "kukalbriiwa_model/cpp/parameters/withoutLinAxes/robot_params.h"

   #include "kukalbriiwa_model/cpp/include/analytic_inverse_kinematics_solver.h"
   typedef RobotModel_C<double, JointMatrix_t, JacobianMatrix_t, JointVector_t, HomogeneousTransformation_t> RobotModel;
  //typedef AnalyticInverseKinematicsSolver IKSolver;
#else
  #if CMAKE_ROBOT_MODEL_FOLDER == 2
    #ifndef TWINCAT_COMPILE
      #include "kukalbriiwa_model/cpp/parameters/withoutLinAxes/controller_params_simulation.h"
    #endif
    #include "kukalbriiwa/robot_params.h"
    extern void _evalJ_(const Eigen::Ref<JointVector> q, Eigen::Ref<JacobianMatrix> J, const RobotParams& param);
    extern void _evalC_ (const Eigen::Ref<JointVector> q, const Eigen::Ref<JointVector> qd, Eigen::Ref<JointMatrix> C,const RobotParams& param);
    extern void _evalG_ (const Eigen::Ref<JointVector> q, Eigen::Ref<JointVector> G, const RobotParams& param);
    extern void _evalJp_ (const Eigen::Ref<JointVector> q, const Eigen::Ref<JointVector> qd, Eigen::Ref<JacobianMatrix> Jp, const RobotParams& param);
    extern void _evalM_ (const Eigen::Ref<JointVector> q, Eigen::Ref<JointMatrix> M, const RobotParams& param);
    extern void _evalT0E_ (const Eigen::Ref<JointVector> q, Eigen::Ref<HomogeneousTransformation> T0E, const RobotParams& param);
    #include"<kukalbriiwa/kukalbriiwa_robot_model.h"

    #include "kukalbriiwa_model/cpp/include/analytic_inverse_kinematics_solver.h"
    typedef KukaLbrIiwaRobotModel<double, JointMatrix_t, JacobianMatrix_t, JointVector_t, HomogeneousTransformation_t> RobotModel;
  #else
    #if CMAKE_ROBOT_MODEL_FOLDER == 3
      #ifndef TWINCAT_COMPILE
        #include "kukalbriiwa_model/cpp/parameters/withLinAxes/controller_params_simulation.h"
      #endif
      #include "kukalbriiwa_model/cpp/include/kukalbriiwaLinAxes_robot_model.h"
      #include "kukalbriiwa_model/cpp/parameters/withLinAxes/robot_params.h"
      #include "kukalbriiwa_model/cpp/include/analytic_inverse_kinematics_solver.h"
      typedef RobotModel_C<double, JointMatrix_t, JacobianMatrix_t, JointVector_t, HomogeneousTransformation_t> RobotModel;
    #else
      #error "No Robot is specified, necessary to have the functionalities"
    #endif
  #endif
#endif

#undef kukalbriiwa_model
#undef kukalbriiwa
#undef kukalbriiwa_linaxes_model

#endif
