#include <math.h>

template<typename Scalar> void geo_jacobian_endeffector (Scalar *q,const RobotParams& param, Scalar cgret[6][7])
{
  Scalar m[6][7];
  Scalar t1;
  Scalar t103;
  Scalar t108;
  Scalar t11;
  Scalar t113;
  Scalar t12;
  Scalar t13;
  Scalar t143;
  Scalar t15;
  Scalar t156;
  Scalar t160;
  Scalar t163;
  Scalar t17;
  Scalar t170;
  Scalar t177;
  Scalar t184;
  Scalar t186;
  Scalar t19;
  Scalar t191;
  Scalar t194;
  Scalar t199;
  Scalar t2;
  Scalar t20;
  Scalar t201;
  Scalar t205;
  Scalar t207;
  Scalar t21;
  Scalar t24;
  Scalar t27;
  Scalar t28;
  Scalar t29;
  Scalar t3;
  Scalar t30;
  Scalar t32;
  Scalar t33;
  Scalar t35;
  Scalar t37;
  Scalar t38;
  Scalar t4;
  Scalar t40;
  Scalar t43;
  Scalar t45;
  Scalar t46;
  Scalar t48;
  Scalar t5;
  Scalar t50;
  Scalar t52;
  Scalar t53;
  Scalar t57;
  Scalar t58;
  Scalar t6;
  Scalar t60;
  Scalar t63;
  Scalar t64;
  Scalar t67;
  Scalar t7;
  Scalar t71;
  Scalar t73;
  Scalar t75;
  Scalar t8;
  Scalar t81;
  Scalar t83;
  Scalar t88;
  Scalar t9;
  Scalar t92;
  Scalar t96;
  m[0][0] = 0;
  m[0][1] = 0;
  m[0][2] = 0;
  m[0][3] = 0;
  m[0][4] = 0;
  m[0][5] = 0;
  m[0][6] = 0;
  m[1][0] = 0;
  m[1][1] = 0;
  m[1][2] = 0;
  m[1][3] = 0;
  m[1][4] = 0;
  m[1][5] = 0;
  m[1][6] = 0;
  m[2][0] = 0;
  m[2][1] = 0;
  m[2][2] = 0;
  m[2][3] = 0;
  m[2][4] = 0;
  m[2][5] = 0;
  m[2][6] = 0;
  m[3][0] = 0;
  m[3][1] = 0;
  m[3][2] = 0;
  m[3][3] = 0;
  m[3][4] = 0;
  m[3][5] = 0;
  m[3][6] = 0;
  m[4][0] = 0;
  m[4][1] = 0;
  m[4][2] = 0;
  m[4][3] = 0;
  m[4][4] = 0;
  m[4][5] = 0;
  m[4][6] = 0;
  m[5][0] = 0;
  m[5][1] = 0;
  m[5][2] = 0;
  m[5][3] = 0;
  m[5][4] = 0;
  m[5][5] = 0;
  m[5][6] = 0;
  t1 = q[4];
  t2 = cos(t1);
  t3 = q[1];
  t4 = sin(t3);
  t5 = t4 * t2;
  t6 = param.d7 + param.d8;
  t7 = q[5];
  t8 = sin(t7);
  t9 = t8 * t6;
  t11 = q[2];
  t12 = cos(t11);
  t13 = cos(t7);
  t15 = t13 * t6 + param.d5 + param.d6;
  t17 = cos(t3);
  t19 = t12 * t15 * t17 - t5 * t9;
  t20 = q[3];
  t21 = sin(t20);
  t24 = cos(t20);
  t27 = sin(t1);
  t28 = sin(t11);
  t29 = t28 * t27;
  t30 = t12 * t2 * t24 - t29;
  t32 = t8 * t30 * t6 * t17;
  t33 = t15 * t24;
  t35 = (t33 + param.d3 + param.d4) * t4;
  t37 = q[0];
  t38 = sin(t37);
  t40 = cos(t37);
  t43 = t28 * t2;
  t45 = t12 * t27;
  t46 = t24 * t43 + t45;
  t48 = -t15 * t21 * t28 + t46 * t9;
  m[0][0] = t38 * (t19 * t21 - t32 - t35) - t48 * t40;
  t50 = t8 * t2;
  t52 = t21 * t6 * t50;
  t53 = t33 + t52 + param.d3 + param.d4;
  t57 = t24 * t6 * t50;
  t58 = -t15 * t21 + t57;
  t60 = t9 * t29;
  t63 = t17 * t53 - t4 * (t12 * t58 - t60);
  m[0][1] = t40 * t63;
  t64 = -t58;
  t67 = t28 * t64 - t45 * t9;
  t71 = t12 * t64 + t60;
  m[0][2] = t17 * t40 * t67 + t38 * t71;
  t73 = -t19;
  t75 = t12 * t2;
  t81 = t24 * t73 - (t17 * t75 * t9 + t15 * t4) * t21;
  t83 = t52 + t33;
  m[0][3] = t28 * t38 * t83 + t40 * t81;
  t88 = t21 * t4;
  t92 = t27 * (t12 * t17 * t24 + t88) + t17 * t43;
  t96 = -t24 * t27 * t28 + t75;
  m[0][4] = -t8 * (t38 * t96 + t40 * t92) * t6;
  t103 = t17 * t30 + t21 * t5;
  t108 = t12 * t17 * t21 - t24 * t4;
  t113 = t21 * t28;
  m[0][5] = (t40 * (t103 * t13 + t108 * t8) - t38 * (t113 * t8 + t13 * t46)) * t6;
  m[0][6] = 0.0e0;
  m[1][0] = t40 * (t21 * t73 + t32 + t35) - t48 * t38;
  m[1][1] = t63 * t38;
  m[1][2] = t40 * (t12 * (t21 * (-t13 * t6 - param.d5 - param.d6) + t57) - t60) + t17 * t67 * t38;
  m[1][3] = -t28 * t40 * t83 + t38 * t81;
  m[1][4] = -(t38 * t92 - t40 * t96) * t9;
  t143 = t40 * t113;
  m[1][5] = (t13 * (t103 * t38 + t40 * t46) + (t108 * t38 + t143) * t8) * t6;
  m[1][6] = 0.0e0;
  m[2][0] = 0.0e0;
  m[2][1] = t17 * t71 - t4 * t53;
  m[2][2] = -t4 * t67;
  m[2][3] = t12 * t4 * t83 + t17 * t58;
  t156 = t12 * t24 * t27 + t43;
  t160 = -t17 * t21 * t27 + t156 * t4;
  m[2][4] = t8 * t160 * t6;
  t163 = t21 * t2;
  t170 = t12 * t21 * t4 + t17 * t24;
  m[2][5] = -(t13 * (-t163 * t17 + t30 * t4) + t8 * t170) * t6;
  m[2][6] = 0.0e0;
  m[3][0] = 0.0e0;
  m[3][1] = -t38;
  m[3][2] = t4 * t40;
  m[3][3] = t17 * t28 * t40 + t12 * t38;
  t177 = -t108;
  m[3][4] = t21 * t28 * t38 + t177 * t40;
  t184 = -t156 * t17 - t27 * t88;
  t186 = -t96;
  m[3][5] = t184 * t40 + t186 * t38;
  t191 = t2 * t24 * t8 - t13 * t21;
  t194 = t27 * t28 * t8;
  t199 = t13 * t24 + t163 * t8;
  t201 = t17 * (t12 * t191 - t194) + t199 * t4;
  t205 = -t191;
  t207 = -t12 * t27 * t8 + t205 * t28;
  m[3][6] = t201 * t40 + t207 * t38;
  m[4][0] = 0.0e0;
  m[4][1] = t40;
  m[4][2] = t4 * t38;
  m[4][3] = t17 * t28 * t38 - t12 * m[4][1];
  m[4][4] = t177 * t38 - t143;
  m[4][5] = t184 * t38 - t186 * m[4][1];
  m[4][6] = t201 * t38 - t207 * m[4][1];
  m[5][0] = 0.1e1;
  m[5][1] = 0.0e0;
  m[5][2] = t17;
  m[5][3] = -t28 * t4;
  m[5][4] = t170;
  m[5][5] = t160;
  m[5][6] = t4 * (t12 * t205 + t194) + t199 * m[5][2];
  cgret[0][0] = m[0][0];
  cgret[0][1] = m[0][1];
  cgret[0][2] = m[0][2];
  cgret[0][3] = m[0][3];
  cgret[0][4] = m[0][4];
  cgret[0][5] = m[0][5];
  cgret[0][6] = m[0][6];
  cgret[1][0] = m[1][0];
  cgret[1][1] = m[1][1];
  cgret[1][2] = m[1][2];
  cgret[1][3] = m[1][3];
  cgret[1][4] = m[1][4];
  cgret[1][5] = m[1][5];
  cgret[1][6] = m[1][6];
  cgret[2][0] = m[2][0];
  cgret[2][1] = m[2][1];
  cgret[2][2] = m[2][2];
  cgret[2][3] = m[2][3];
  cgret[2][4] = m[2][4];
  cgret[2][5] = m[2][5];
  cgret[2][6] = m[2][6];
  cgret[3][0] = m[3][0];
  cgret[3][1] = m[3][1];
  cgret[3][2] = m[3][2];
  cgret[3][3] = m[3][3];
  cgret[3][4] = m[3][4];
  cgret[3][5] = m[3][5];
  cgret[3][6] = m[3][6];
  cgret[4][0] = m[4][0];
  cgret[4][1] = m[4][1];
  cgret[4][2] = m[4][2];
  cgret[4][3] = m[4][3];
  cgret[4][4] = m[4][4];
  cgret[4][5] = m[4][5];
  cgret[4][6] = m[4][6];
  cgret[5][0] = m[5][0];
  cgret[5][1] = m[5][1];
  cgret[5][2] = m[5][2];
  cgret[5][3] = m[5][3];
  cgret[5][4] = m[5][4];
  cgret[5][5] = m[5][5];
  cgret[5][6] = m[5][6];
}


template<typename Scalar>  void geo_jacobian_endeffector_tmp(const JointVector_t& q, const RobotParams& param, JacobianMatrix_t& J)
{
  JacobianMatrix_t &m = J;
  Scalar t1;
  Scalar t103;
  Scalar t108;
  Scalar t11;
  Scalar t113;
  Scalar t12;
  Scalar t13;
  Scalar t143;
  Scalar t15;
  Scalar t156;
  Scalar t160;
  Scalar t163;
  Scalar t17;
  Scalar t170;
  Scalar t177;
  Scalar t184;
  Scalar t186;
  Scalar t19;
  Scalar t191;
  Scalar t194;
  Scalar t199;
  Scalar t2;
  Scalar t20;
  Scalar t201;
  Scalar t205;
  Scalar t207;
  Scalar t21;
  Scalar t24;
  Scalar t27;
  Scalar t28;
  Scalar t29;
  Scalar t3;
  Scalar t30;
  Scalar t32;
  Scalar t33;
  Scalar t35;
  Scalar t37;
  Scalar t38;
  Scalar t4;
  Scalar t40;
  Scalar t43;
  Scalar t45;
  Scalar t46;
  Scalar t48;
  Scalar t5;
  Scalar t50;
  Scalar t52;
  Scalar t53;
  Scalar t57;
  Scalar t58;
  Scalar t6;
  Scalar t60;
  Scalar t63;
  Scalar t64;
  Scalar t67;
  Scalar t7;
  Scalar t71;
  Scalar t73;
  Scalar t75;
  Scalar t8;
  Scalar t81;
  Scalar t83;
  Scalar t88;
  Scalar t9;
  Scalar t92;
  Scalar t96;
  m(0,0) = 0;
  m(0,1) = 0;
  m(0,2) = 0;
  m(0,3) = 0;
  m(0,4) = 0;
  m(0,5) = 0;
  m(0,6) = 0;
  m(1,0) = 0;
  m(1,1) = 0;
  m(1,2) = 0;
  m(1,3) = 0;
  m(1,4) = 0;
  m(1,5) = 0;
  m(1,6) = 0;
  m(2,0) = 0;
  m(2,1) = 0;
  m(2,2) = 0;
  m(2,3) = 0;
  m(2,4) = 0;
  m(2,5) = 0;
  m(2,6) = 0;
  m(3,0) = 0;
  m(3,1) = 0;
  m(3,2) = 0;
  m(3,3) = 0;
  m(3,4) = 0;
  m(3,5) = 0;
  m(3,6) = 0;
  m(4,0) = 0;
  m(4,1) = 0;
  m(4,2) = 0;
  m(4,3) = 0;
  m(4,4) = 0;
  m(4,5) = 0;
  m(4,6) = 0;
  m(5,0) = 0;
  m(5,1) = 0;
  m(5,2) = 0;
  m(5,3) = 0;
  m(5,4) = 0;
  m(5,5) = 0;
  m(5,6) = 0;
  t1 = q(4);
  t2 = cos(t1);
  t3 = q(1);
  t4 = sin(t3);
  t5 = t4 * t2;
  t6 = param.d7 + param.d8;
  t7 = q(5);
  t8 = sin(t7);
  t9 = t8 * t6;
  t11 = q(2);
  t12 = cos(t11);
  t13 = cos(t7);
  t15 = t13 * t6 + param.d5 + param.d6;
  t17 = cos(t3);
  t19 = t12 * t15 * t17 - t5 * t9;
  t20 = q(3);
  t21 = sin(t20);
  t24 = cos(t20);
  t27 = sin(t1);
  t28 = sin(t11);
  t29 = t28 * t27;
  t30 = t12 * t2 * t24 - t29;
  t32 = t8 * t30 * t6 * t17;
  t33 = t15 * t24;
  t35 = (t33 + param.d3 + param.d4) * t4;
  t37 = q(0);
  t38 = sin(t37);
  t40 = cos(t37);
  t43 = t28 * t2;
  t45 = t12 * t27;
  t46 = t24 * t43 + t45;
  t48 = -t15 * t21 * t28 + t46 * t9;
  m(0,0) = t38 * (t19 * t21 - t32 - t35) - t48 * t40;
  t50 = t8 * t2;
  t52 = t21 * t6 * t50;
  t53 = t33 + t52 + param.d3 + param.d4;
  t57 = t24 * t6 * t50;
  t58 = -t15 * t21 + t57;
  t60 = t9 * t29;
  t63 = t17 * t53 - t4 * (t12 * t58 - t60);
  m(0,1) = t40 * t63;
  t64 = -t58;
  t67 = t28 * t64 - t45 * t9;
  t71 = t12 * t64 + t60;
  m(0,2) = t17 * t40 * t67 + t38 * t71;
  t73 = -t19;
  t75 = t12 * t2;
  t81 = t24 * t73 - (t17 * t75 * t9 + t15 * t4) * t21;
  t83 = t52 + t33;
  m(0,3) = t28 * t38 * t83 + t40 * t81;
  t88 = t21 * t4;
  t92 = t27 * (t12 * t17 * t24 + t88) + t17 * t43;
  t96 = -t24 * t27 * t28 + t75;
  m(0,4) = -t8 * (t38 * t96 + t40 * t92) * t6;
  t103 = t17 * t30 + t21 * t5;
  t108 = t12 * t17 * t21 - t24 * t4;
  t113 = t21 * t28;
  m(0,5) = (t40 * (t103 * t13 + t108 * t8) - t38 * (t113 * t8 + t13 * t46)) * t6;
  m(0,6) = 0.0e0;
  m(1,0) = t40 * (t21 * t73 + t32 + t35) - t48 * t38;
  m(1,1) = t63 * t38;
  m(1,2) = t40 * (t12 * (t21 * (-t13 * t6 - param.d5 - param.d6) + t57) - t60) + t17 * t67 * t38;
  m(1,3) = -t28 * t40 * t83 + t38 * t81;
  m(1,4) = -(t38 * t92 - t40 * t96) * t9;
  t143 = t40 * t113;
  m(1,5) = (t13 * (t103 * t38 + t40 * t46) + (t108 * t38 + t143) * t8) * t6;
  m(1,6) = 0.0e0;
  m(2,0) = 0.0e0;
  m(2,1) = t17 * t71 - t4 * t53;
  m(2,2) = -t4 * t67;
  m(2,3) = t12 * t4 * t83 + t17 * t58;
  t156 = t12 * t24 * t27 + t43;
  t160 = -t17 * t21 * t27 + t156 * t4;
  m(2,4) = t8 * t160 * t6;
  t163 = t21 * t2;
  t170 = t12 * t21 * t4 + t17 * t24;
  m(2,5) = -(t13 * (-t163 * t17 + t30 * t4) + t8 * t170) * t6;
  m(2,6) = 0.0e0;
  m(3,0) = 0.0e0;
  m(3,1) = -t38;
  m(3,2) = t4 * t40;
  m(3,3) = t17 * t28 * t40 + t12 * t38;
  t177 = -t108;
  m(3,4) = t21 * t28 * t38 + t177 * t40;
  t184 = -t156 * t17 - t27 * t88;
  t186 = -t96;
  m(3,5) = t184 * t40 + t186 * t38;
  t191 = t2 * t24 * t8 - t13 * t21;
  t194 = t27 * t28 * t8;
  t199 = t13 * t24 + t163 * t8;
  t201 = t17 * (t12 * t191 - t194) + t199 * t4;
  t205 = -t191;
  t207 = -t12 * t27 * t8 + t205 * t28;
  m(3,6) = t201 * t40 + t207 * t38;
  m(4,0) = 0.0e0;
  m(4,1) = t40;
  m(4,2) = t4 * t38;
  m(4,3) = t17 * t28 * t38 - t12 * m(4,1);
  m(4,4) = t177 * t38 - t143;
  m(4,5) = t184 * t38 - t186 * m(4,1);
  m(4,6) = t201 * t38 - t207 * m(4,1);
  m(5,0) = 0.1e1;
  m(5,1) = 0.0e0;
  m(5,2) = t17;
  m(5,3) = -t28 * t4;
  m(5,4) = t170;
  m(5,5) = t160;
  m(5,6) = t4 * (t12 * t205 + t194) + t199 * m(5,2);
}
