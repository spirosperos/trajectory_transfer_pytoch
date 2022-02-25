#pragma hdrstop
#pragma once
#ifndef UTIL_CONTROL_H
#define UTIL_CONTROL_H
#include <ros_acin_robot_support/ros_acin_robot_support.h>



//@desc:
// calculate omega(angulr veclocity) from euler angular velocity
//@args:
//euler = [Yaw (Phi), Pitch(Theta), Roll(Psi)] --> euler angles in [rad]!
//eulerDot = d ([Yaw(t)(Phi), Pitch(t)(Theta), Roll(t)(Psi)])/dt euler angles velocity in [rad/s]
//@return:
//omega angular veclocity vector calculated from the euler parametrization
inline void eulerDot2omega (const Eigen::Ref<angular_t> euler, const Eigen::Ref<angular_t> eulerDot, Eigen::Ref<angular_t> omega)
{
  //euler angles = [Yaw (Phi), Pitch(Theta), Roll(Psi)]
  //from script "automateirungs und regelungstechnik WS 2019/2020"
  double cos_Yaw = cos(euler[0]);
  double sin_Yaw = sin(euler[0]);
  double cos_Pitch = cos(euler[1]);
  double sin_Pitch = sin(euler[1]);
  Eigen::Matrix3d T_euler = Eigen::Matrix3d::Zero(); //transformation matrix
  T_euler(0,1) =-sin_Yaw;
  T_euler(0,2) = cos_Yaw*sin_Pitch;
  T_euler(1,1) =cos_Yaw;
  T_euler(1,2) =sin_Yaw*sin_Pitch;
  T_euler(2,0) = 1;
  T_euler(2,2) = cos_Pitch;
  angular_t omega_ = T_euler*eulerDot;
  for(uint16_t ii =0; ii<omega.rows(); ii++){
    omega[ii]=omega_[ii];
  }
}

inline void get_euler_curve_manipulability(  const JointVector_t &q_0,
                                      const spline_Interpolator_sptr_t &point_interpolator,
                                      const spline_Interpolator_sptr_t &euler_interpolator){
  joint_curve_t q_curve;
  point_curve_t point_curve;
  point_curve_t euler_curve;
  point_interpolator->get_curve(point_curve);
  euler_interpolator->get_curve(euler_curve);

}

//@desc:
// calculate omega(angulr veclocity) from euler angular velocity
//@args:
//euler = [Yaw (Phi), Pitch(Theta), Roll(Psi)] --> euler angles in [rad]!
//eulerDot = d ([Yaw(t)(Phi), Pitch(t)(Theta), Roll(t)(Psi)])/dt euler angles velocity in [rad/s]
//@return:
//omega angular veclocity vector calculated from the euler parametrization
inline  void eulerDotDot2omegaDot (const Eigen::Ref<angular_t> euler, const Eigen::Ref<angular_t> eulerDot, const Eigen::Ref<angular_t> eulerDotDot, Eigen::Ref<angular_t> omegaDot)
{
  //euler angles = [Yaw (Phi), Pitch(Theta), Roll(Psi)]
  //from script "automateirungs und regelungstechnik WS 2019/2020"
  double cos_Yaw = cos(euler[0]);
  double sin_Yaw = sin(euler[0]);
  double cos_Pitch = cos(euler[1]);
  double sin_Pitch = sin(euler[1]);
  Eigen::Matrix3d T_euler = Eigen::Matrix3d::Zero(); //transformation matrix
  T_euler(0,1) =-sin_Yaw;
  T_euler(0,2) = cos_Yaw*sin_Pitch;
  T_euler(1,1) =cos_Yaw;
  T_euler(1,2) =sin_Yaw*sin_Pitch;
  T_euler(2,0) = 1;
  T_euler(2,2) = cos_Pitch;


  Eigen::Matrix3d T_dYaw = Eigen::Matrix3d::Zero();
  T_dYaw(0,1) =-cos_Yaw;
  T_dYaw(0,2) = -sin_Yaw*sin_Pitch;
  T_dYaw(1,1) =-sin_Yaw;
  T_dYaw(1,2) =cos_Yaw*sin_Pitch;
  Eigen::Matrix3d T_dPitch = Eigen::Matrix3d::Zero();
  T_dPitch(0,2) = cos_Yaw*cos_Pitch;
  T_dPitch(1,2) =sin_Yaw*cos_Pitch;
  T_dPitch(2,2) =-sin_Pitch;

  angular_t omegaDot_ = (T_dYaw*eulerDot[0]+T_dPitch*eulerDot[1])*eulerDot + T_euler*eulerDotDot;
  for(uint16_t ii =0; ii<omegaDot_.rows(); ii++){
    omegaDot[ii]=omegaDot_[ii];
  }
}


inline void set_quat(const Eigen::Ref<qa_vec_t> q, qa_t &quat){
  Eigen::Matrix<double,4,1> tmp = Eigen::Matrix<double,4,1>::Zero();
  for(uint8_t ii = 0; ii < 4; ii++){
    tmp(ii,0) = q(ii,0);
  }
  quat = tmp;
}
inline void qautDot2omega (const Eigen::Ref<qa_vec_t> qaut, const Eigen::Ref<qa_vec_t> qautDot, Eigen::Ref<angular_t> omega)
{
  #if defined(UTIL_EXCEPTIONS)
    if(qaut.rows()!=4){
      throw std::range_error("qaut number of rows must be 4");
    }
    if(qautDot.rows()!=4){
        throw std::range_error("qautDot number of rows must be 4");
    }
  #endif
  //get quaternion
  qa_t q_qa;
  set_quat(qaut, q_qa);
  //get quaternion velocity, must be scaled, uniform bpline are used for interpilation in the interval [0.,1.0]

  qa_t qp_qa;
  set_quat(qautDot, qp_qa);

  //calculation of omega from the spline velocity
  qa_t q_tmp = q_qa.inverse()*qp_qa;
  qa_t omega_qa_spline =qa_t( 2.0*(q_tmp).coeffs());

  //get quaternion accelerations, must be scaled, uniform bpline are used for interpilation in the interval [0.,1.0]
  // ctrlpt_t qpp_spline = scale*scale*eval_spline_.col(2);
  // qa_t qpp_qa_spline;
  // set_quat(qpp_spline, qpp_qa_spline);
  // //calculation of omega_p from the spline velocity and accelerations
  // ctrlpt_qa_t q_tmp_1 = ctrlpt_qa_t(2.0*(q_qa.inverse()*qpp_qa_spline).coeffs());
  // ctrlpt_qa_t omega_qa_p_spline = ctrlpt_qa_t(q_tmp_1.coeffs()- 0.5*(omega_qa_spline*omega_qa_spline).coeffs());

  // std::cout << "omega spline:= "<<omega_qa_spline.vec().transpose()<<std::endl;
  // std::cout << "omega_w spline (should be 0):= " << omega_qa_spline.w()<<std::endl;
  omega = omega_qa_spline.vec();
  // std::cout << "omega_p spline:= "<<omega_qa_p_spline.vec().transpose()<<std::endl;
  // std::cout << "omega_w_p spline (should be 0):= " << omega_qa_p_spline.w()<<std::endl<<std::endl;
}

inline void qautDotDot2omegaDot(const Eigen::Ref<qa_vec_t> qaut,const Eigen::Ref<qa_vec_t> qautDot, const Eigen::Ref<qa_vec_t> qautDotDot, Eigen::Ref<angular_t> omegaDot)
{
  #if defined(UTIL_EXCEPTIONS)
    if(qaut.rows()!=4){
        throw std::range_error("qaut number of rows must be 4");
    }
    if(qautDot.rows()!=4){
        throw std::range_error("qautDot number of rows must be 4");
      }
  #endif
  //get quaternion
  qa_t q_qa;
  set_quat(qaut, q_qa);
  //get quaternion velocity, must be scaled, uniform bpline are used for interpilation in the interval [0.,1.0]

  qa_t qp_qa;
  set_quat(qautDot, qp_qa);

  //calculation of omega from the spline velocity
  qa_t q_tmp = q_qa.inverse()*qp_qa;
  qa_t omega_qa =qa_t( 2.0*(q_tmp).coeffs());

  //get quaternion accelerations, must be scaled, uniform bpline are used for interpilation in the interval [0.,1.0]
  // ctrlpt_t qpp_spline = scale*scale*eval_spline_.col(2);
  qa_t qpp_qa;
  set_quat(qautDotDot, qpp_qa);
  // //calculation of omega_p from the spline velocity and accelerations
  qa_t q_tmp_1 = qa_t(2.0*(q_qa.inverse()*qpp_qa).coeffs());
  qa_t omega_qa_p = qa_t(q_tmp_1.coeffs()- 0.5*(omega_qa*omega_qa).coeffs());

  // std::cout << "omega spline:= "<<omega_qa_spline.vec().transpose()<<std::endl;
  // std::cout << "omega_w spline (should be 0):= " << omega_qa_spline.w()<<std::endl;
  // omega = omega_qa_p.vec();
  omegaDot = omega_qa_p.vec();
  // std::cout << "omega_p spline:= "<<omega_qa_p_spline.vec().transpose()<<std::endl;
  // std::cout << "omega_w_p spline (should be 0):= " << omega_qa_p_spline.w()<<std::endl<<std::endl;
}

inline void log_barrier(  Eigen::Ref<JointVector_t> b,
                          const Eigen::Ref<JointVector_t> q,
                          const std::array<double, _ROBOT_q_DIM_> &q_lb,
                          const std::array<double, _ROBOT_q_DIM_> &q_ub){
  Eigen::Index N = b.rows();
  std::vector<double> b_ub(b.rows(),0.0);
  std::vector<double> b_lb(b.rows(),0.0);
  for(uint16_t ii = 0; ii < N; ii++){
      double eval_ub = ((q_ub[ii] - q(ii))/q_ub[ii]);
      double eval_lb = ((q_lb[ii] - q(ii))/q_lb[ii]);
//**************************************************
      if(eval_ub > 0){
        if(eval_ub >= 1){
          b_ub[ii] = 0.0;
        }else{
          b_ub[ii] = std::log(eval_ub);
        }
      }else{
        b_ub[ii] = -10;
      }
      if(b_ub[ii] < -10){
        b_ub[ii] = -10;
      }
//**************************************************

//**************************************************
      if( eval_lb > 0){
        if(eval_lb >= 1){
          b_lb[ii] = 0.0;
        }else{
          b_lb[ii] = std::log(eval_lb);
        }
      }else{
        b_lb[ii] = -10;
      }
      if(b_lb[ii]< -10){
        b_lb[ii] = -10;
      }
//**************************************************
      if(q_ub[ii] < 0){
        b_ub[ii] = -1*b_ub[ii];
      }
      if(q_lb[ii] < 0){
        b_lb[ii] = -1*b_lb[ii];
      }
//**************************************************
      b(ii) = b_ub[ii] + b_lb[ii];
  }
  // std::cout<<"b:="<< b<<std::endl;
  // std::cout<<"b_lb:="<< b_lb << std::endl;
  // std::cout<<"b_ub:= "<< b_ub << std::endl;
}

inline void log_barrier_v2(  Eigen::Ref<JointVector_t> b,
                      const Eigen::Ref<JointVector_t> q,
                      const std::array<double, _ROBOT_q_DIM_> &q_lb,
                      const std::array<double, _ROBOT_q_DIM_> &q_ub,
                      const std::vector<double> &q_m){
  Eigen::Index N = b.rows();
  std::vector<double> b_ub(b.rows(),0.0);
  std::vector<double> b_lb(b.rows(),0.0);
  for(uint16_t ii = 0; ii < N; ii++){
      double q_m_ii = q_m[ii];
      //assert(q_m_ii < q_ub[ii]);
      //assert(q_lb[ii] < q_m_ii);
      double eval_ub = (q_ub[ii] - (q(ii) - q_m_ii))/std::abs(q_ub[ii]);
      double eval_lb = ((q(ii) - q_m_ii) - q_lb[ii])/std::abs(q_lb[ii]);
//**************************************************
      if(eval_ub > 0){
        b_ub[ii] = std::log(eval_ub);
      }else{
        b_ub[ii] = -10;
      }
      if(b_ub[ii] < -10){
        b_ub[ii] = -10;
      }
//**************************************************

//**************************************************
      if( eval_lb > 0){
        b_lb[ii] = std::log(eval_lb);
      }else{
        b_lb[ii] = -10;
      }
      if(b_lb[ii]< -10){
        b_lb[ii] = -10;
      }
//**************************************************
      b(ii) = b_ub[ii] + b_lb[ii];
  }
  // std::cout<<"b:="<< b<<std::endl;
  // std::cout<<"b_lb:="<< b_lb << std::endl;
  // std::cout<<"b_ub:= "<< b_ub << std::endl;
}
//grad_mFoM -- gradiant of w = 1/(2*N)*sum(((q[ii]-q_m[ii])/q_ub[ii]-q_lb[ii])²,ii), maximal freedom of movement
inline void grad_mFoM( Eigen::Ref<JointVector_t> gradW_q,
                const Eigen::Ref<JointVector_t> q,
                const double *q_lb,
                const double *q_ub,
                std::vector<double> &q_m){
  Eigen::Index N = gradW_q.rows();
  for(uint16_t ii = 0; ii < static_cast<uint16_t>(N); ii++){
    double q_m_i = q_m[ii];
    double delta_q_i = q_ub[ii]-q_lb[ii];
    if (delta_q_i == 0) {
        return;
    }
    gradW_q(ii) = -1/(2*(double)N)*(1/delta_q_i)*2*(q(ii)-q_m_i)/delta_q_i;
  }
}
inline void grad_log_barrier(  Eigen::Ref<JointVector_t> gradW_q,
                        const Eigen::Ref<JointVector_t> q,
                        const double *q_lb,
                        const double *q_ub,
                        std::vector<double> &q_m){
  Eigen::Index N = gradW_q.rows();
  for(uint16_t ii = 0; ii < static_cast<uint16_t> (N); ii++){
    //double q_m_i = q_m[ii];
    gradW_q(ii) = 1/(2*N)*(1/(q_ub[ii]-q(ii)));
    if( q(ii)  > q_ub[ii] || gradW_q(ii) > 10){
      gradW_q(ii) = 10;
    }
    if(q_lb[ii] < 0 && 1/(2*N)*(1/(q(ii)-q_lb[ii]))  <-10){
      gradW_q(ii) = gradW_q(ii) - 10;
    }else{
      gradW_q(ii) = gradW_q(ii) - 1/(2*N)*(1/(q(ii)-q_lb[ii]));
    }
  }
}

#endif // UTIL_CONTROL_H
