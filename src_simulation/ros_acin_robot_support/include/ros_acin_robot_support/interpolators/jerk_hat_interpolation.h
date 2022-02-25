#ifndef JERK_HAT_INTERPOLATION_H

#define JERK_HAT_INTERPOLATION_H

#include <array>
#include <cmath>
#include <eigen3/Eigen/Dense>

template <int steps, int df> class JerkHatInterpolator
{
public:
  typedef Eigen::Matrix<double, steps, df> TrajMatrix;
  typedef Eigen::Matrix<double, df, 1> JointVector;

  JerkHatInterpolator()
  {
    h_ = 1.0;
  }

  JerkHatInterpolator(double h) : h_(h)
  {
    
  }

  //void setTrajectory(std::array<std::array<double, df>, steps>& traj)
  void setTrajectory(TrajMatrix& traj)
  {
    traj_ = traj;
  }

  void calcJerk(double t, JointVector& j_full) const
  {
    for(int i = 0; i < df; ++i)
    {
      for(int j = 0; j < steps; ++j)
      {
        if(j == 0)
        {
          j_full(i) = calcJ0(j * h_, traj_(j, i), h_, t);
        }
        else if(j == (steps - 1))  
        {
          j_full(i) += calcJn((j - 1) * h_, traj_(j, i), h_, t);
        }
        else
        {
          j_full(i) += calcJk((j - 1) * h_, traj_(j, i), h_, t);
        }
      }
    } 
  }

  void calcAcceleration(double t, JointVector& a_full, JointVector& a_init) const
  {
    a_full = a_init;
    for(int i = 0; i < df; ++i)
    {
      for(int j = 0; j < steps; ++j)
      {
        if(j == 0)
        {
          a_full(i) += calcA0(j * h_, traj_(j, i), h_, t);
        }
        else if(j == (steps - 1))  
        {
          a_full(i) += calcAn((j - 1) * h_, traj_(j, i), h_, t);
        }
        else
        {
          a_full(i) += calcAk((j - 1) * h_, traj_(j, i), h_, t);
        }
      }
    } 
  }

  void calcVelocity(double t, JointVector& v_full, JointVector& a_init, JointVector& v_init) const
  {
    v_full = a_init * t + v_init;

    for(int i = 0; i < df; ++i)
    {
      for(int j = 0; j < steps; ++j)
      {
        if(j == 0)
        {
          v_full(i) += calcV0(j * h_, traj_(j, i), h_, t);
        }
        else if(j == (steps - 1))  
        {
          v_full(i) += calcVn((j - 1) * h_, traj_(j, i), h_, t);
        }
        else
        {
          v_full(i) += calcVk((j - 1) * h_, traj_(j, i), h_, t);
        }
      }
    } 
  }

  void calcAngle(double t, JointVector& q_full, JointVector& a_init, JointVector& v_init, JointVector& q_init) const
  {
    q_full = a_init * pow(t, 2) / 2.0 + v_init * t + q_init;

    for(int i = 0; i < df; ++i)
    {
      for(int j = 0; j < steps; ++j)
      {
        if(j == 0)
        {
          q_full(i) += calcQ0(j * h_, traj_(j, i), h_, t);
        }
        else if(j == steps - 1)  
        {
          q_full(i) += calcQn((j - 1) * h_, traj_(j, i), h_, t);
        }
        else
        {
          q_full(i) += calcQk((j - 1) * h_, traj_(j, i), h_, t);
        }
      }
    } 
  }

private:

  TrajMatrix traj_ = TrajMatrix::Zero();

  /*
  std::array<double, df> j_full_;
  std::array<double, df> a_full_;
  std::array<double, df> v_full_;
  std::array<double, df> q_full_;
  */

  double h_;

  double calcJ0(double c1, double p, double h, double t) const
  {
    double j0 = 0;
    if(c1 <= t && t <= c1 + h)
    {
      j0 = p * (c1 + h - t) / h;
    }
    else
    {
      j0 = 0;
    }

    return j0;
  }

  double calcJn(double c1, double p, double h, double t) const
  {
    double jn = 0;

    if(c1 <= t && t <= c1 + h)
    {
      jn = p * (t - c1) / h;
    }
    else
    {
      jn = 0;
    }

    return jn;
  }
  double calcJk(double c1, double p, double h, double t) const
  {
    double jk = 0;
    if(c1 <= t && t <= c1 + h)
    {
      jk = p * (t - c1) / h;
    }
    else if (c1 + h < t && t <= c1 + 2 * h)
    {
      jk = p * (c1 + 2.0 * h - t) / h;
    }
    else
    {
      jk = 0;
    }

    return jk;
  }

  double calcA0(double c1, double p, double h, double t) const
  {
    double a0 = 0;
    if(c1 < t && t <= c1 + h)
    {
      a0 = -p * (t - c1) * (t - 2.0 * h - c1) / h / 2.0;
    }
    else if(t > c1 + h)
    {
      a0 = p * h / 2.0;
    }
    else
    {
      a0 = 0;
    }

    return a0;
  }

  double calcAn(double c1, double p, double h, double t) const
  {
    double an = 0;

    if(c1 < t && t <= c1 + h)
    {
      an = p * pow((t - c1), 2) / h / 2.0;
    }
    else if(t > c1 + h)
    {
      an = p * h / 2.0;
    }
    else
    {
      an = 0;
    }

    return an;
  }
  double calcAk(double c1, double p, double h, double t) const
  {
    double ak = 0;
    if(c1 < t && t <= c1 + h)
    {
      ak = p * pow((t - c1), 2) / h / 2.0;
    }
    else if (c1 + h < t && t <= c1 + 2 * h)
    {
      ak = -(h * h + (-2.0 * t + 2.0 * c1) * h + pow((t - c1), 2) / 2.0) * p / h;
    }
    else if(t > c1 + 2 * h)
    {
      ak = p * h;
    }
    else
    {
      ak = 0;
    }

    return ak;
  }

  double calcV0(double c1, double p, double h, double t) const
  {
    double v0 = 0;
    if(c1 < t && t <= c1 + h)
    {
      v0 = -p * pow((t - c1), 2) * (t - 3.0 * h - c1) / h / 6.0;
    }
    else if(t > c1 + h)
    {
      v0 = p * h * (3.0 * t - h - 3.0 * c1) / 6.0;
    }
    else
    {
      v0 = 0;
    }

    return v0;
  }

  double calcVn(double c1, double p, double h, double t) const
  {
    double vn = 0;

    if(c1 < t && t <= c1 + h)
    {
      vn = -p * pow((-t + c1), 3) / h / 6.0;
    }
    else if(t > c1 + h)
    {
      vn = p * h * (3.0 * t - 2.0 * h - 3.0 * c1) / 6.0;
    }
    else
    {
      vn = 0;
    }

    return vn;
  }
  double calcVk(double c1, double p, double h, double t) const
  {
    double vk = 0;
    if(c1 < t && t <= c1 + h)
    {
      vk = -p * pow((-t + c1), 3) / h / 6.0;
    }
    else if (c1 + h < t && t <= c1 + 2 * h)
    {
      vk = p * (pow(h, 3) + (-3.0 * t + 3.0 * c1) * h * h + 3.0 * pow((t - c1), 2) * h - pow((t - c1), 3) / 2.0) / h / 3.0;
    }
    else if(t > c1 + 2 * h)
    {
      vk = -h * p * (c1 + h - t);
    }
    else
    {
      vk = 0;
    }

    return vk;
  }

  double calcQ0(double c1, double p, double h, double t) const
  {
    double q0 = 0;
    if(c1 < t && t <= c1 + h)
    {
      q0 = -p * pow((t - c1), 3) * (t - 4.0 * h - c1) / h / 24.0;
    }
    else if(t > c1 + h)
    {
      q0 = p * (h * h / 6.0 + (-2.0 / 3.0 * t + 2.0 / 3.0 * c1) * h + pow((t - c1), 2)) * h / 4.0;
    }
    else
    {
      q0 = 0;
    }

    return q0;
  }

  double calcQn(double c1, double p, double h, double t) const
  {
    double qn = 0;

    if(c1 < t && t <= c1 + h)
    {
      qn = p * pow((-t + c1), 4) / h / 24.0;
    }
    else if(t > c1 + h)
    {
      qn = p * h * (h * h / 2 + (-4.0 / 3.0 * t + 4.0 / 3.0 * c1) * h + pow((t - c1), 2)) / 4.0;
    }
    else
    {
      qn = 0;
    }

    return qn;
  }
  double calcQk(double c1, double p, double h, double t) const
  {
    double qk = 0;
    if(c1 < t && t <= c1 + h)
    {
      qk = p * pow((-t + c1), 4) / h / 24;
    }
    else if (c1 + h < t && t <= c1 + 2 * h)
    {
      qk = -(pow(h, 4) + (-4.0 * t + 4.0 * c1) * pow(h, 3) + 6.0 * pow((t - c1), 2) * h * h - 4.0 * pow((t - c1), 3) * h + pow((t - c1), 4) / 2.0) * p / h / 12.0;
    }
    else if(t > c1 + 2 * h)
    {
      qk = 7.0 / 12.0 * h * (h * h + (-12.0 / 7.0 * t + 12.0 / 7.0 * c1) * h + 6.0 / 7.0 * pow((t - c1), 2)) * p;
    }
    else
    {
      qk = 0;
    }

    return qk;
  }

};

#endif // JERK_HAT_INTERPOLATION_H