#include <evalG.h>
void _evalG_ (const Eigen::Ref<JointVector> q, Eigen::Ref<JointVector> G, const RobotParams& param)
{
  double q1 = q(0);
  double q2 = q(1);
  double q3 = q(2);
  double q4 = q(3);
  double q5 = q(4);
  double q6 = q(5);
  double q7 = q(6);

  double m[1][7];
  double t1;
  double t10;
  double t11;
  double t127;
  double t13;
  double t14;
  double t145;
  double t146;
  double t148;
  double t15;
  double t152;
  double t156;
  double t16;
  double t17;
  double t18;
  double t19;
  double t2;
  double t21;
  double t22;
  double t25;
  double t26;
  double t28;
  double t29;
  double t3;
  double t30;
  double t34;
  double t35;
  double t36;
  double t37;
  double t4;
  double t43;
  double t44;
  double t45;
  double t47;
  double t48;
  double t5;
  double t50;
  double t51;
  double t52;
  double t54;
  double t56;
  double t57;
  double t6;
  double t61;
  double t65;
  double t67;
  double t68;
  double t70;
  double t71;
  double t74;
  double t76;
  double t8;
  double t85;
  double t9;
  m[0][0] = 0;
  m[0][1] = 0;
  m[0][2] = 0;
  m[0][3] = 0;
  m[0][4] = 0;
  m[0][5] = 0;
  m[0][6] = 0;
  m[0][0] = 0;
  t1 = param.sp7x * param.m7;
  t2 = cos(q7);
  t3 = t2 * t1;
  t4 = param.m7 * param.sp7y;
  t5 = sin(q7);
  t6 = t5 * t4;
  t8 = param.m6 * param.sp6x - t3 + t6;
  t9 = cos(q6);
  t10 = t9 * t8;
  t11 = -param.sp7z - param.d7;
  t13 = param.m6 * param.sp6y;
  t14 = param.m7 * t11 - t13;
  t15 = sin(q6);
  t16 = t15 * t14;
  t17 = param.m5 * param.sp5x;
  t18 = t10 + t16 + t17;
  t19 = cos(q5);
  t21 = t5 * t1;
  t22 = t2 * t4;
  t25 = -param.m5 * param.sp5y + param.m6 * param.sp6z + t21 + t22;
  t26 = sin(q5);
  t28 = param.m4 * param.sp4x;
  t29 = t18 * t19 + t25 * t26 - t28;
  t30 = cos(q4);
  t34 = -param.m7 * t11 + t13;
  t35 = t9 * t34;
  t36 = t15 * t8;
  t37 = param.d5 + param.d6;
  t43 = t35 + t36 + param.m7 * t37 + param.m6 * t37 + (param.d5 + param.sp5z) * param.m5 + param.m4 * param.sp4y;
  t44 = sin(q4);
  t45 = t44 * t43;
  t47 = -param.m3 * param.sp3x + t29 * t30 + t45;
  t48 = cos(q3);
  t50 = sin(q3);
  t51 = -t25;
  t52 = t51 * t50;
  t54 = t26 * t50;
  t56 = t16 + t17;
  t57 = t56 * t50;
  t61 = param.m3 * param.sp3y - param.m4 * param.sp4z;
  t65 = cos(q2);
  t67 = t30 * t43;
  t68 = -t8;
  t70 = t15 * t34;
  t71 = t68 * t9 - t17 + t70;
  t74 = t19 * t71 + t26 * t51 + t28;
  t76 = param.d3 + param.d4;
  t85 = sin(q2);
  m[0][1] = (param.g * (t65 * (param.sp2x * param.m2 - t10 * t54 - t19 * t52 - t26 * t57 + t47 * t48 + t50 * t61) - t85 * (t67 + t44 * t74 + param.m7 * t76 + param.m6 * t76 + param.m5 * t76 + param.m4 * t76 + (param.d3 + param.sp3z) * param.m3 + param.sp2y * param.m2)));
  m[0][2] = (param.g * t85 * (-t10 * t26 * t48 - t19 * t48 * t51 - t26 * t48 * t56 - t47 * t50 + t48 * t61));
  m[0][3] = (param.g * (t85 * t48 * (-t44 * t29 + t67) + t65 * (t30 * t74 - t45)));
  m[0][4] = (param.g * (t85 * (t48 * t30 * (-t18 * t26 + t19 * t25) + t26 * t52 - t10 * t19 * t50 - t19 * t57) + t65 * t44 * (t19 * t51 - t26 * t71)));
  t127 = -t70 + t10;
  m[0][5] = (param.g * (t85 * (t48 * (t30 * t19 * (t14 * t9 - t36) + t44 * t127) + t36 * t54 - t26 * t9 * t14 * t50) + t65 * (t30 * t127 + t44 * t19 * (-t15 * t68 + t35))));
  t145 = t21 + t22;
  t146 = t9 * t145;
  t148 = t3 - t6;
  t152 = t15 * t145;
  t156 = -t148;
  m[0][6] = (param.g * (t85 * (t48 * (t30 * (t146 * t19 + t26 * t148) + t44 * t152) - t19 * t156 * t50 - t146 * t54) + t65 * (t30 * t152 + t44 * (-t145 * t19 * t9 + t156 * t26))));

  for(int i = 0; i < G.rows(); ++i)
  {
    for(int j = 0; j < G.cols(); ++j)
    {
      G(i, j) = m[j][i];
    }
  }
  //G = JointVector(&(m[0][0]));
}