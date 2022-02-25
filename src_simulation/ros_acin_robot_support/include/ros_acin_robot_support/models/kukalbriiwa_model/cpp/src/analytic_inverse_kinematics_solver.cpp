#include "analytic_inverse_kinematics_solver.h"
#include <iostream>
typedef JointVector JointVector_t
typedef JointVector JointVector_t
typedef RotationMatrix RotationMatrix_t
double AnalyticInverseKinematicsSolver::wrap2pi(double lambda)
{
  double q;
  q = fmod(lambda, 2 * M_PI);
  q = q >= 0 ? q : q + 2 * M_PI;

  if(q == 0 && lambda > 0)
  {
    q = 2 * M_PI;
  }
  return q;
}

/**
 * wraps angles in lamdba, in ardians to the interval [-pi, pi]
 * such that pi maps to pi and -pi maps to -pi.
 * In general, odd, positive multiples of pi map to pi and odd
 * negative multiples of pi map to -pi.
 */
JointVector_t AnalyticInverseKinematicsSolver::wrap(JointVector_t lambda)
{
  for(int i = 0; i < lambda.size(); ++i)
  {
    if(lambda(i) < -M_PI || M_PI < lambda(i))
    {
      lambda(i) = wrap2pi(lambda(i) + M_PI) - M_PI;
    }
  }
  return lambda;
}

/**
 *
 */
RotationMatrix_t AnalyticInverseKinematicsSolver::skew(Vector3_t v)
{
  RotationMatrix_t S = RotationMatrix::Zero();
  S(0, 1) = -v(2);
  S(0, 2) = v(1);
  S(1, 2) = -v(0);

  S(1, 0) = v(2);
  S(2, 0) = -v(1);
  S(2, 1) = v(0);

  return S;
}

void AnalyticInverseKinematicsSolver::calcAnalyticInverseKinematics(HomogeneousTransformation_t x_d, bool find_minimum_norm, const RobotParams& param, bool& success, JointVector_t& q)
{
  find_minimum_norm = true;
  bool elbow_up = true;
  success = false;

  const double d_bs = param.d1 + param.d2;
  const double d_se = param.d3 + param.d4;
  const double d_ew = param.d5 + param.d6;
  const double d_wt = param.d7 + param.d8;

  Vector3 l_bs;
  l_bs(0) = 0;
  l_bs(1) = 0;
  l_bs(2) = d_bs;

  Vector3 l_se;
  l_se(0) = 0;
  l_se(1) = -d_se;
  l_se(2) = 0;

  Vector3 l_ew;
  l_ew(0) = 0;
  l_ew(1) = 0;
  l_ew(2) = d_ew;

  Vector3 l_wt;
  l_wt(0) = 0;
  l_wt(1) = 0;
  l_wt(2) = d_wt;

  // we assume the desired point is in robot base coordinates
  Vector3 x_7 = x_d.block<3, 1>(0, 3);

  RotationMatrix R_0_7 = x_d.block<3, 3>(0, 0);

  Vector3 x_sw = x_7 - l_bs - R_0_7 * l_wt;
  Vector3 u_sw = x_sw / x_sw.norm();

  // The angle for joint 4 is solely defined by the distance between the shoulder
  // point and wrist point. If this angle is out of the feasible range, no solution
  // can be found

  double tmp = (x_sw.squaredNorm() - d_se * d_se - d_ew * d_ew) / (2 * d_se * d_ew);
  std::cout << "tmp = " << tmp << std::endl;
  if(abs(tmp) > 1.0 + std::numeric_limits<double>::epsilon())
  {
    std::cout << "target point too far away" << std::endl;
    return;
  }

  q(3) = acos(tmp);

  if(q(3) >= param.q_limit_upper[3] || q(3) <= param.q_limit_lower[3])
  {
    std::cout << "target too close" << std::endl;
  }

  RotationMatrix R_3_4;
  R_3_4 << cos(q(3)), 0, sin(q(3)),
            sin(q(3)), 0, -cos(q(3)),
            0, 1, 0;

  q(0) = atan2(x_sw(1), x_sw(0));

  if(elbow_up)
  {
    double t1 = cos(q(3));
    double t2 = d_ew * t1;
    double t4 = sin(q(3));
    double t5 = t4 * t4;
    double t6 = d_ew * d_ew;
    double t9 = 0.2e1 * d_se * t2;
    double t10 = d_se * d_se;
    double t11 = x_sw(2) * x_sw(2);
    tmp = (t9 + t6 + t10 - t11) * t6 * t5;

    if(tmp >= 0)
    {
      double t14 = sqrt(tmp);
      double t25 = 0.1e1 / (t9 + t6 + t10);
      q(1) = atan2(0.1e1 / t4 * t25 / d_ew * (t14 * (t2 + d_se) + t6 * x_sw(2) * (t1 + 0.1e1) * (t1 - 0.1e1)), t25 * (d_se * x_sw(2) + x_sw(2) * t2 + t14));
    }
    else
    {
      q = JointVector::Zero();
      return;
    }
  }
  else
  {
    if(q(0) > 0)
    {
      q(0) = q(0) - M_PI;
    }
    else
    {
      q(0) = q(0) + M_PI;
    }
    double t1 = cos(q(3));
    double t2 = d_ew * t1;
    double t4 = sin(q(3));
    double t5 = t4 * t4;
    double t6 = d_ew * d_ew;
    double t9 = 0.2e1 * d_se * t2;
    double t10 = d_se * d_se;
    double t11 = x_sw(2) * x_sw(2);
    tmp = (t9 + t6 + t10 - t11) * t6 * t5;

    if(tmp >= 0)
    {
      double t14 = sqrt(tmp);
      double t25 = 0.1e1 / (t9 + t6 + t10);
      q(1) = atan2(0.1e1 / t4 * t25 / d_ew * (t14 * (-d_se - t2) + t6 * x_sw(2) * (t1 + 0.1e1) * (t1 - 0.1e1)), t25 * (d_se * x_sw(2) + x_sw(2) * t2 - t14));
    }
    else
    {
      q = JointVector::Zero();
      return;
    }
  }

  RotationMatrix R_0_3_o;
  R_0_3_o << cos(q(0)) * cos(q(1)), -cos(q(0)) * sin(q(1)), -sin(q(0)),
              sin(q(0)) * cos(q(1)), -sin(q(0)) * sin(q(1)), cos(q(0)),
              -sin(q(1)), -cos(q(1)), 0;

  RotationMatrix U = skew(u_sw);

  RotationMatrix A_s = skew(u_sw) * R_0_3_o;
  RotationMatrix B_s = -(U * U) * R_0_3_o;
  RotationMatrix C_s = (u_sw * u_sw.transpose()) * R_0_3_o;

  RotationMatrix A_w = R_3_4.transpose() * A_s.transpose() * R_0_7;
  RotationMatrix B_w = R_3_4.transpose() * B_s.transpose() * R_0_7;
  RotationMatrix C_w = R_3_4.transpose() * C_s.transpose() * R_0_7;

  // Post process solution: A4 has to be inverted to match the implemented
  // robot kinematic model. After this point, q(3) is not used in the
  // calculations anymore.
  q(3) = -q(3);
  double q3 = q(3);

  Eigen::VectorXd psi_list = Eigen::VectorXd(17);
  psi_list.head<9>() = Eigen::VectorXd::LinSpaced(9, 0, M_PI);
  psi_list.tail<8>() = Eigen::VectorXd::LinSpaced(8, -M_PI / 8, -M_PI);

  Eigen::MatrixXd all_solutions(7, 0);
  JointVector q_tmp;
  if(find_minimum_norm)
  {
    std::cout << "find_minimum_norm" << std::endl;
    // calculate all solutions for the list
    for(int i = 0; i < psi_list.size(); ++i)
    {
      double psi = psi_list(i);
      q_tmp(0) = atan2(-A_s(1, 1) * sin(psi) - B_s(1, 1) * cos(psi) - C_s(1, 1), -A_s(0, 1) * sin(psi) - B_s(0, 1) * cos(psi) - C_s(0, 1));
      q_tmp(1) = -A_s(2, 1) * sin(psi) - B_s(2, 1) * cos(psi) - C_s(2, 1);
      q_tmp(2) = atan2(A_s(2, 2) * sin(psi) + B_s(2, 2) * cos(psi) + C_s(2, 2), -A_s(2, 0) * sin(psi) - B_s(2, 0) * cos(psi) - C_s(2, 0));
      q_tmp(3) = q3;
      q_tmp(4) = atan2(A_w(1, 2) * sin(psi) + B_w(1, 2) * cos(psi) + C_w(1, 2), A_w(0, 2) * sin(psi) + B_w(0, 2) * cos(psi) + C_w(0, 2));
      q_tmp(5) = A_w(2, 2) * sin(psi) + B_w(2, 2) * cos(psi) + C_w(2, 2);
      q_tmp(6) = atan2(A_w(2, 1) * sin(psi) + B_w(2, 1) * cos(psi) + C_w(2, 1), -A_w(2, 0) * sin(psi) - B_w(2, 0) * cos(psi) - C_w(2, 0));

      // add if valid
      if((abs(q_tmp(1)) <= 1) && (abs(q_tmp(5)) <= 1))
      {
        all_solutions.conservativeResize(Eigen::NoChange, all_solutions.cols() + 1);
        all_solutions.block<7, 1>(0, all_solutions.cols() - 1) = q_tmp;
      }
    }
    //std::cout << "all_solutions: " << std::endl << all_solutions << std::endl;
    all_solutions.block(1, 0, 1, all_solutions.cols()) = all_solutions.block(1, 0, 1, all_solutions.cols()).array().acos();
    all_solutions.block(5, 0, 1, all_solutions.cols()) = all_solutions.block(5, 0, 1, all_solutions.cols()).array().acos();
    // wrap
    for(int i = 0; i < all_solutions.cols(); ++i)
    {
      all_solutions.block<7, 1>(0, i) = wrap(all_solutions.block<7, 1>(0, i));
    }

    // calculate all solutions, which can be generated using simple shifting and inverting operations
    Eigen::MatrixXd all_solutions_2(7, 0);

    int N = 2;
    for(int i = -N; i <= N; ++i)
    {
      for(int j = -N; j <= N; ++j)
      {
        for(int k = -N; k <= N; ++k)
        {
          auto tmp = all_solutions;
          if(i != 0)
          {
            //std::cout << "tmp = " << std::endl << tmp << std::endl;
            //std::cout << "block1 = " << std::endl << tmp.block(0, 0, 1, tmp.cols()) << std::endl;
            tmp.block(0, 0, 1, tmp.cols()) = tmp.block(0, 0, 1, tmp.cols()).array() + i * M_PI;
            //std::cout << "block1 = " << std::endl << tmp.block(0, 0, 1, tmp.cols()) << std::endl;
            tmp.block(1, 0, 1, tmp.cols()) = tmp.block(1, 0, 1, tmp.cols()).array() * pow(-1, i);
            tmp.block(2, 0, 1, tmp.cols()) = tmp.block(2, 0, 1, tmp.cols()).array() + i * M_PI;
            //std::cout << "tmp = " << std::endl << tmp << std::endl;
          }
          if(j != 0)
          {
            tmp.block(2, 0, 1, tmp.cols()) = tmp.block(2, 0, 1, tmp.cols()).array() + j * M_PI;
            tmp.block(3, 0, 1, tmp.cols()) = tmp.block(3, 0, 1, tmp.cols()).array() * pow(-1, j);
            tmp.block(4, 0, 1, tmp.cols()) = tmp.block(4, 0, 1, tmp.cols()).array() + j * M_PI;
          }
          if(k != 0)
          {
            tmp.block(4, 0, 1, tmp.cols()) = tmp.block(4, 0, 1, tmp.cols()).array() + k * M_PI;
            tmp.block(5, 0, 1, tmp.cols()) = tmp.block(5, 0, 1, tmp.cols()).array() * pow(-1, k);
            tmp.block(6, 0, 1, tmp.cols()) = tmp.block(6, 0, 1, tmp.cols()).array() + k * M_PI;
          }

          // check if valid before adding it
          JointVector upper_limit_eigen(param.q_limit_upper.data());
          JointVector lower_limit_eigen(param.q_limit_lower.data());
          bool all = true;
          for(int l = 0; l < tmp.cols(); ++l)
          {
            // wrap
            tmp.block<7, 1>(0, l) = wrap(tmp.block<7, 1>(0, l));

            all = (tmp.block<7, 1>(0, l).array() <= upper_limit_eigen.array()).all()
                  && (tmp.block<7, 1>(0, l).array() >= lower_limit_eigen.array()).all();
          }
          if(all)
          {
            all_solutions_2.conservativeResize(Eigen::NoChange, all_solutions_2.cols() + tmp.cols());
            all_solutions_2.block(0, all_solutions_2.cols() - tmp.cols(), 7, tmp.cols()) = tmp;
          }
        }
      }
    }
    //std::cout << "all_solutions_2: " << std::endl << all_solutions_2 << std::endl;
    //std::cout << "all_solutions_2.cols: " << all_solutions_2.cols() << std::endl;
    //std::cout << "all_solutions_2.rows: " << all_solutions_2.rows() << std::endl;

    // calculate minimum norm
    success = all_solutions_2.cols() > 0;
    if(success)
    {
      Eigen::VectorXd norms = all_solutions_2.colwise().norm();
      //std::cout << "norms = " << norms.transpose() << std::endl;
      Eigen::VectorXd::Index min_idx;
      double min = norms.minCoeff(&min_idx);
      std::cout << "q = " << q << std::endl;
      q = all_solutions_2.block<7, 1>(0, min_idx);
      std::cout << "min idx = " << min_idx << std::endl;
      std::cout << "min = " << min << std::endl;
      std::cout << "q_all_sol = " << q << std::endl;
    }
    else
    {
      q = JointVector::Zero();
    }

  }
  else
  {
    for(int i = 0; i < psi_list.size(); ++i)
    {
      double psi = psi_list(i);

      q(0) = atan2(-A_s(1, 1) * sin(psi) - B_s(1, 1) * cos(psi) - C_s(1, 1), -A_s(0, 1) * sin(psi) - B_s(0, 1) * cos(psi) - C_s(0, 1));
      tmp = -A_s(2, 1) * sin(psi) - B_s(2, 1) * cos(psi) - C_s(2, 1);

      std::cout << "A_s = " << A_s(2, 1) << " psi = " << psi << " B_s = " << B_s(2, 1) << " C_s = " << C_s(2, 1) << std::endl;

      if(abs(tmp) > 1)
      {
        continue;
      }

      q(1) = acos(tmp);
      q(2) = atan2(A_s(2, 2) * sin(psi) + B_s(2, 2) * cos(psi) + C_s(2, 2), -A_s(2, 0) * sin(psi) - B_s(2, 0) * cos(psi) - C_s(2, 0));

      // Restore original q(3)
      q(3) = q3;
      q(4) = atan2(A_w(1, 2) * sin(psi) + B_w(1, 2) * cos(psi) + C_w(1, 2), A_w(0, 2) * sin(psi) + B_w(0, 2) * cos(psi) + C_w(0, 2));

      tmp = A_w(2, 2) * sin(psi) + B_w(2, 2) * cos(psi) + C_w(2, 2);

      if(abs(tmp) > 1)
      {
        continue;
      }

      q(5) = acos(tmp);
      q(6) = atan2(A_w(2, 1) * sin(psi) + B_w(2, 1) * cos(psi) + C_w(2, 1), -A_w(2, 0) * sin(psi) - B_w(2, 0) * cos(psi) - C_w(2, 0));
      q = wrap(q);

      if(q(0) < param.q_limit_lower[0]) //Get the "shoulder left" solution
      {
        q(0) = q(0) + M_PI;
        q(1) = -q(1);
        q(2) = q(2) + M_PI;
        q = wrap(q);
      }
      else if(q(0) > param.q_limit_upper[0]) // Get the "shoulder right" solution
      {
        q(0) = q(0) - M_PI;
        q(1) = -q(1);
        q(2) = q(2) - M_PI;
        q = wrap(q);
      }

      if(q(2) < param.q_limit_lower[2]) // Get the "elbow left" solution
      {
        q(2) = q(2) + M_PI;
        q(3) = -q(3);
        q(4) = q(4) + M_PI;
        q = wrap(q);
      }
      else if(q(2) > param.q_limit_upper[2]) // Get the "elbow right" solution
      {
        q(2) = q(2) - M_PI;
        q(3) = -q(3);
        q(4) = q(4) - M_PI;
        q = wrap(q);
      }

      if(q(4) < param.q_limit_lower[4] || q(6) < param.q_limit_lower[6]) // Get the "wrist left" solution
      {
        q(4) = q(4) + M_PI;
        q(5) = -q(5);
        q(6) = q(6) + M_PI;
        q = wrap(q);
      }
      else if(q(4) > param.q_limit_upper[4] || q(6) > param.q_limit_upper[6]) //Get the "wrist right" solution
      {
        q(4) = q(4) - M_PI;
        q(5) = -q(5);
        q(6) = q(6) - M_PI;
        q = wrap(q);
      }

      // check if in limits
      bool all = true;
      for(int j = 0; j < q.size(); ++j)
      {
        if(q(j) > param.q_limit_upper[j] || q(j) < param.q_limit_lower[j])
        {
          all = false;
        }
      }

      if(all)
      {
        std::cout << "i = " << i << std::endl;
        success = true;
        break;
      }
    }
    if(!success)
    {
      q = JointVector::Zero();
    }
  }
}
