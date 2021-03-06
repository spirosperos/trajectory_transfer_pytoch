#include <math.h>

template<typename Scalar> void conservative_forces (
  Scalar *q,
  Scalar *q_p,
  Scalar *theta,
 const RobotParams& param,
  Scalar cgret[1][14])
{
  Scalar m[1][14];
  Scalar t1;
  Scalar t10;
  Scalar t103;
  Scalar t105;
  Scalar t106;
  Scalar t107;
  Scalar t11;
  Scalar t110;
  Scalar t111;
  Scalar t114;
  Scalar t116;
  Scalar t118;
  Scalar t12;
  Scalar t129;
  Scalar t13;
  Scalar t132;
  Scalar t136;
  Scalar t137;
  Scalar t14;
  Scalar t147;
  Scalar t15;
  Scalar t159;
  Scalar t166;
  Scalar t169;
  Scalar t17;
  Scalar t170;
  Scalar t18;
  Scalar t188;
  Scalar t19;
  Scalar t191;
  Scalar t194;
  Scalar t199;
  Scalar t2;
  Scalar t20;
  Scalar t21;
  Scalar t221;
  Scalar t222;
  Scalar t225;
  Scalar t228;
  Scalar t23;
  Scalar t231;
  Scalar t235;
  Scalar t239;
  Scalar t24;
  Scalar t25;
  Scalar t26;
  Scalar t260;
  Scalar t261;
  Scalar t262;
  Scalar t264;
  Scalar t27;
  Scalar t270;
  Scalar t274;
  Scalar t279;
  Scalar t28;
  Scalar t286;
  Scalar t29;
  Scalar t3;
  Scalar t30;
  Scalar t310;
  Scalar t311;
  Scalar t313;
  Scalar t314;
  Scalar t316;
  Scalar t318;
  Scalar t319;
  Scalar t325;
  Scalar t328;
  Scalar t33;
  Scalar t334;
  Scalar t339;
  Scalar t34;
  Scalar t346;
  Scalar t35;
  Scalar t36;
  Scalar t369;
  Scalar t37;
  Scalar t38;
  Scalar t39;
  Scalar t4;
  Scalar t40;
  Scalar t43;
  Scalar t45;
  Scalar t46;
  Scalar t47;
  Scalar t48;
  Scalar t49;
  Scalar t5;
  Scalar t50;
  Scalar t52;
  Scalar t53;
  Scalar t55;
  Scalar t56;
  Scalar t57;
  Scalar t58;
  Scalar t6;
  Scalar t64;
  Scalar t65;
  Scalar t66;
  Scalar t68;
  Scalar t69;
  Scalar t7;
  Scalar t72;
  Scalar t73;
  Scalar t74;
  Scalar t76;
  Scalar t77;
  Scalar t78;
  Scalar t80;
  Scalar t82;
  Scalar t84;
  Scalar t85;
  Scalar t86;
  Scalar t87;
  Scalar t88;
  Scalar t89;
  Scalar t90;
  Scalar t91;
  Scalar t96;
  Scalar t98;
  m[0][0] = 0;
  m[0][1] = 0;
  m[0][2] = 0;
  m[0][3] = 0;
  m[0][4] = 0;
  m[0][5] = 0;
  m[0][6] = 0;
  m[0][7] = 0;
  m[0][8] = 0;
  m[0][9] = 0;
  m[0][10] = 0;
  m[0][11] = 0;
  m[0][12] = 0;
  m[0][13] = 0;
  t1 = param.m7 * param.sp7x;
  t2 = q[6];
  t3 = sin(t2);
  t4 = t3 * t1;
  t5 = param.m7 * param.sp7y;
  t6 = cos(t2);
  t7 = t6 * t5;
  t10 = -param.sp5y * param.m5 + param.sp6z * param.m6 + t4 + t7;
  t11 = q[4];
  t12 = cos(t11);
  t13 = t12 * t10;
  t14 = t6 * t1;
  t15 = t3 * t5;
  t17 = -param.m6 * param.sp6x + t14 - t15;
  t18 = q[5];
  t19 = cos(t18);
  t20 = t19 * t17;
  t21 = param.d7 + param.sp7z;
  t23 = param.m6 * param.sp6y;
  t24 = param.m7 * t21 + t23;
  t25 = sin(t18);
  t26 = t25 * t24;
  t27 = param.sp5x * param.m5;
  t28 = t20 + t26 - t27;
  t29 = sin(t11);
  t30 = t29 * t28;
  t33 = param.sp3y * param.m3 - param.sp4z * param.m4 + t13 + t30;
  t34 = q[2];
  t35 = cos(t34);
  t36 = t35 * t33;
  t37 = t19 * t24;
  t38 = -t17;
  t39 = t25 * t38;
  t40 = param.d5 + param.d6;
  t43 = param.d5 + param.sp5z;
  t45 = param.sp4y * param.m4;
  t46 = param.m5 * t43 + param.m6 * t40 + param.m7 * t40 + t37 + t39 + t45;
  t47 = sin(t34);
  t48 = t47 * t46;
  t49 = q[3];
  t50 = sin(t49);
  t52 = t12 * t28;
  t53 = -t10;
  t55 = param.sp4x * param.m4;
  t56 = t29 * t53 + t52 + t55;
  t57 = t56 * t47;
  t58 = cos(t49);
  t64 = param.m3 * param.sp3x * t47 + param.sp1y * param.m1 + param.sp2z * param.m2 - t48 * t50 + t57 * t58 + t36;
  t65 = q[0];
  t66 = sin(t65);
  t68 = t50 * t46;
  t69 = t19 * t38;
  t72 = -param.m7 * t21 - t23;
  t73 = t25 * t72;
  t74 = t69 + t73 + t27;
  t76 = t29 * t10;
  t77 = t12 * t74 - t55 + t76;
  t78 = t58 * t77;
  t80 = -param.sp3x * param.m3 + t68 + t78;
  t82 = t47 * t33;
  t84 = param.sp2x * param.m2 + t35 * t80 + t82;
  t85 = q[1];
  t86 = cos(t85);
  t87 = t86 * t84;
  t88 = t50 * t77;
  t89 = t19 * t72;
  t90 = t25 * t17;
  t91 = -t40;
  t96 = -param.m5 * t43 + param.m6 * t91 + param.m7 * t91 - t45 + t89 + t90;
  t98 = -param.d3 - param.d4;
  t103 = -param.sp3z - param.d3;
  t105 = param.sp2y * param.m2;
  t106 = param.m3 * t103 + param.m4 * t98 + param.m5 * t98 + param.m6 * t98 + param.m7 * t98 + t58 * t96 - t105 + t88;
  t107 = sin(t85);
  t110 = -param.sp1x * param.m1 + t107 * t106 + t87;
  t111 = cos(t65);
  t114 = sin(param.gamma_base);
  t116 = cos(param.beta_base);
  t118 = sin(param.beta_base);
  t129 = (theta[0] - t65) * param.K__1;
  m[0][0] = (t116 * t114 * (-t110 * t111 - t64 * t66) + (-t110 * t66 + t111 * t64) * t118) * param.g + param.beta * q_p[0] - t129;
  t132 = t106 * t86 - t107 * t84;
  t136 = t58 * t46;
  t137 = -t98;
  t147 = cos(param.gamma_base);
  t159 = (theta[1] - t85) * param.K__2;
  m[0][1] = (t116 * (-t114 * t66 * t132 + t147 * (-t107 * (-param.m3 * t103 + param.m4 * t137 + param.m5 * t137 + param.m6 * t137 + param.m7 * t137 + t50 * t56 + t105 + t136) + t87)) + t111 * t132 * t118) * param.g + param.beta * q_p[1] - t159;
  t166 = param.m3 * param.sp3x * t35 - t35 * t46 * t50 + t35 * t56 * t58 - t82;
  t169 = -t47 * t80 + t36;
  t170 = t86 * t169;
  t188 = (theta[2] - t34) * param.K__3;
  m[0][2] = (t116 * (t114 * (t111 * t166 - t170 * t66) + t147 * t107 * t169) + (t111 * t170 + t166 * t66) * t118) * param.g + param.beta * q_p[2] - t188;
  t191 = -t48 * t58 - t50 * t57;
  t194 = t35 * (t136 - t88);
  t199 = t86 * t194 + t107 * (-t50 * t96 + t78);
  t221 = (theta[3] - t49) * param.K__4;
  m[0][3] = (t116 * (t114 * (t111 * t191 - t199 * t66) + t147 * (t86 * (t56 * t58 - t68) + t107 * t194)) + (t111 * t199 + t191 * t66) * t118) * param.g + param.beta * q_p[3] - t221;
  t222 = -t76 + t52;
  t225 = t12 * t53 - t30;
  t228 = t225 * t47 * t58 + t222 * t35;
  t231 = -t29 * t74 + t13;
  t235 = t231 * t35 * t58 + t222 * t47;
  t239 = t107 * t231 * t50 + t235 * t86;
  t260 = (theta[4] - t11) * param.K__5;
  m[0][4] = (t116 * (t114 * (t111 * t228 - t239 * t66) + t147 * (t225 * t50 * t86 + t107 * t235)) + (t111 * t239 + t228 * t66) * t118) * param.g + param.beta * q_p[4] - t260;
  t261 = -t90 + t37;
  t262 = t29 * t261;
  t264 = -t26 + t69;
  t270 = t12 * t261 * t47 * t58 - t264 * t47 * t50 + t262 * t35;
  t274 = t12 * (-t39 + t89);
  t279 = t35 * (t264 * t50 + t274 * t58) + t47 * t262;
  t286 = t86 * t279 + t107 * (t50 * t274 + t58 * (-t73 + t20));
  t310 = (theta[5] - t18) * param.K__6;
  m[0][5] = (t116 * (t114 * (t111 * t270 - t286 * t66) + t147 * (t86 * (t12 * t261 * t50 + t264 * t58) + t107 * t279)) + (t111 * t286 + t270 * t66) * t118) * param.g + param.beta * q_p[5] - t310;
  t311 = t14 - t15;
  t313 = -t4 - t7;
  t314 = t19 * t313;
  t316 = t12 * t311 + t29 * t314;
  t318 = -t313;
  t319 = t25 * t318;
  t325 = t12 * t314 - t29 * t311;
  t328 = -t319 * t47 * t50 + t325 * t47 * t58 + t316 * t35;
  t334 = t12 * t19 * t318 + t29 * t311;
  t339 = t35 * (t319 * t50 + t334 * t58) + t47 * t316;
  t346 = t86 * t339 + t107 * (t25 * t313 * t58 + t334 * t50);
  t369 = (theta[6] - t2) * param.K__7;
  m[0][6] = (t116 * (t114 * (t111 * t328 - t346 * t66) + t147 * (t86 * (t319 * t58 + t325 * t50) + t107 * t339)) + (t111 * t346 + t328 * t66) * t118) * param.g + param.beta * q_p[6] - t369;
  m[0][7] = t129;
  m[0][8] = t159;
  m[0][9] = t188;
  m[0][10] = t221;
  m[0][11] = t260;
  m[0][12] = t310;
  m[0][13] = t369;
  cgret[0][0] = m[0][0];
  cgret[0][1] = m[0][1];
  cgret[0][2] = m[0][2];
  cgret[0][3] = m[0][3];
  cgret[0][4] = m[0][4];
  cgret[0][5] = m[0][5];
  cgret[0][6] = m[0][6];
  cgret[0][7] = m[0][7];
  cgret[0][8] = m[0][8];
  cgret[0][9] = m[0][9];
  cgret[0][10] = m[0][10];
  cgret[0][11] = m[0][11];
  cgret[0][12] = m[0][12];
  cgret[0][13] = m[0][13];
}
