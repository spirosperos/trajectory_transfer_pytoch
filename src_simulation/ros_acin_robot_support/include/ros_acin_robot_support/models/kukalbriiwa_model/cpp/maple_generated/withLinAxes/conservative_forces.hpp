#include <math.h>

template<typename Scalar> void conservative_forces (
  Scalar *q,
  Scalar *q_p,
  Scalar *theta,
 const RobotParams& param,
  Scalar cgret[1][18])
{
  Scalar m[1][18];
  Scalar t100;
  Scalar t102;
  Scalar t104;
  Scalar t105;
  Scalar t106;
  Scalar t107;
  Scalar t109;
  Scalar t110;
  Scalar t111;
  Scalar t112;
  Scalar t113;
  Scalar t114;
  Scalar t115;
  Scalar t116;
  Scalar t12;
  Scalar t121;
  Scalar t123;
  Scalar t128;
  Scalar t130;
  Scalar t131;
  Scalar t132;
  Scalar t135;
  Scalar t136;
  Scalar t142;
  Scalar t15;
  Scalar t153;
  Scalar t156;
  Scalar t159;
  Scalar t161;
  Scalar t162;
  Scalar t183;
  Scalar t186;
  Scalar t188;
  Scalar t190;
  Scalar t194;
  Scalar t195;
  Scalar t214;
  Scalar t217;
  Scalar t22;
  Scalar t220;
  Scalar t225;
  Scalar t23;
  Scalar t24;
  Scalar t248;
  Scalar t25;
  Scalar t251;
  Scalar t253;
  Scalar t256;
  Scalar t259;
  Scalar t26;
  Scalar t261;
  Scalar t265;
  Scalar t27;
  Scalar t28;
  Scalar t288;
  Scalar t289;
  Scalar t29;
  Scalar t292;
  Scalar t294;
  Scalar t295;
  Scalar t298;
  Scalar t3;
  Scalar t302;
  Scalar t306;
  Scalar t308;
  Scalar t315;
  Scalar t32;
  Scalar t33;
  Scalar t34;
  Scalar t341;
  Scalar t342;
  Scalar t344;
  Scalar t345;
  Scalar t349;
  Scalar t35;
  Scalar t351;
  Scalar t352;
  Scalar t353;
  Scalar t356;
  Scalar t358;
  Scalar t36;
  Scalar t363;
  Scalar t365;
  Scalar t37;
  Scalar t371;
  Scalar t373;
  Scalar t380;
  Scalar t39;
  Scalar t4;
  Scalar t40;
  Scalar t405;
  Scalar t41;
  Scalar t42;
  Scalar t43;
  Scalar t45;
  Scalar t46;
  Scalar t47;
  Scalar t48;
  Scalar t49;
  Scalar t50;
  Scalar t51;
  Scalar t52;
  Scalar t53;
  Scalar t54;
  Scalar t55;
  Scalar t56;
  Scalar t57;
  Scalar t61;
  Scalar t62;
  Scalar t63;
  Scalar t64;
  Scalar t67;
  Scalar t69;
  Scalar t70;
  Scalar t71;
  Scalar t72;
  Scalar t73;
  Scalar t74;
  Scalar t75;
  Scalar t76;
  Scalar t77;
  Scalar t78;
  Scalar t79;
  Scalar t80;
  Scalar t81;
  Scalar t82;
  Scalar t83;
  Scalar t84;
  Scalar t85;
  Scalar t86;
  Scalar t88;
  Scalar t89;
  Scalar t90;
  Scalar t92;
  Scalar t93;
  Scalar t95;
  Scalar t96;
  Scalar t97;
  Scalar t98;
  Scalar t99;
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
  m[0][14] = 0;
  m[0][15] = 0;
  m[0][16] = 0;
  m[0][17] = 0;
  t3 = sin(param.gamma_base);
  t4 = cos(param.beta_base);
  t12 = param.K__a * (theta[0] - q[0]);
  m[0][0] = t4 * t3 * param.g * (-param.m7 - param.m6 - param.m5 - param.m4 - param.m3 - param.m2 - param.m1 - param.ma - param.mb) + param.beta * q_p[0] - t12;
  t15 = sin(param.beta_base);
  t22 = param.K__b * (theta[1] - q[1]);
  m[0][1] = t15 * (param.m7 + param.m6 + param.m5 + param.m4 + param.m3 + param.m2 + param.m1 + param.mb) * param.g + param.beta * q_p[1] - t22;
  t23 = param.sp7x * param.m7;
  t24 = q[8];
  t25 = sin(t24);
  t26 = t25 * t23;
  t27 = param.sp7y * param.m7;
  t28 = cos(t24);
  t29 = t28 * t27;
  t32 = param.m5 * param.sp5y - param.sp6z * param.m6 - t26 - t29;
  t33 = q[6];
  t34 = cos(t33);
  t35 = t34 * t32;
  t36 = t28 * t23;
  t37 = t27 * t25;
  t39 = param.m6 * param.sp6x - t36 + t37;
  t40 = q[7];
  t41 = cos(t40);
  t42 = t41 * t39;
  t43 = -param.sp7z - param.d7;
  t45 = param.m6 * param.sp6y;
  t46 = param.m7 * t43 - t45;
  t47 = sin(t40);
  t48 = t47 * t46;
  t49 = param.m5 * param.sp5x;
  t50 = t42 + t48 + t49;
  t51 = sin(t33);
  t52 = t51 * t50;
  t53 = param.m3 * param.sp3y;
  t54 = param.m4 * param.sp4z;
  t55 = t35 + t52 - t53 + t54;
  t56 = q[4];
  t57 = cos(t56);
  t61 = -param.m7 * t43 + t45;
  t62 = t41 * t61;
  t63 = t47 * t39;
  t64 = param.d5 + param.d6;
  t67 = param.d5 + param.sp5z;
  t69 = param.m4 * param.sp4y;
  t70 = param.m5 * t67 + param.m6 * t64 + param.m7 * t64 + t62 + t63 + t69;
  t71 = sin(t56);
  t72 = t71 * t70;
  t73 = q[5];
  t74 = sin(t73);
  t75 = t74 * t72;
  t76 = -t39;
  t77 = t41 * t76;
  t78 = t47 * t61;
  t79 = t77 + t78 - t49;
  t80 = t34 * t79;
  t81 = t51 * t32;
  t82 = param.m4 * param.sp4x;
  t83 = t80 + t81 + t82;
  t84 = t83 * t71;
  t85 = cos(t73);
  t86 = t85 * t84;
  t88 = param.sp3x * param.m3 * t71;
  t89 = param.m2 * param.sp2z;
  t90 = param.m1 * param.sp1y;
  t92 = q[2];
  t93 = sin(t92);
  t95 = t74 * t70;
  t96 = t34 * t50;
  t97 = -t32;
  t98 = t51 * t97;
  t99 = t96 + t98 - t82;
  t100 = t85 * t99;
  t102 = -param.m3 * param.sp3x + t100 + t95;
  t104 = t34 * t97;
  t105 = t51 * t79;
  t106 = t104 + t105 + t53 - t54;
  t107 = t71 * t106;
  t109 = param.m2 * param.sp2x + t102 * t57 + t107;
  t110 = q[3];
  t111 = cos(t110);
  t112 = t111 * t109;
  t113 = t74 * t99;
  t114 = t46 * t41;
  t115 = t47 * t76;
  t116 = -t64;
  t121 = -param.m5 * t67 + param.m6 * t116 + param.m7 * t116 + t114 + t115 - t69;
  t123 = -param.d3 - param.d4;
  t128 = -param.sp3z - param.d3;
  t130 = param.m2 * param.sp2y;
  t131 = param.m3 * t128 + param.m4 * t123 + param.m5 * t123 + param.m6 * t123 + param.m7 * t123 + t121 * t85 + t113 - t130;
  t132 = sin(t110);
  t135 = -param.sp1x * param.m1 + t131 * t132 + t112;
  t136 = cos(t92);
  t142 = t106 * t57;
  t153 = (theta[2] - t92) * param.K__1;
  m[0][2] = (t4 * t3 * (-t93 * (t55 * t57 + t75 - t86 - t88 - t89 - t90) + t136 * t135) + t15 * (-t93 * t135 + t136 * (t142 - t75 + t86 + t88 + t89 + t90))) * param.g + param.beta * q_p[2] - t153;
  t156 = -t109 * t132 + t111 * t131;
  t159 = cos(param.gamma_base);
  t161 = t85 * t70;
  t162 = -t123;
  t183 = (theta[3] - t110) * param.K__2;
  m[0][3] = (t4 * (t3 * t93 * t156 + (-t132 * (-param.m3 * t128 + param.m4 * t162 + param.m5 * t162 + param.m6 * t162 + param.m7 * t162 + t74 * t83 + t130 + t161) + t112) * t159) + t15 * t136 * t156) * param.g + param.beta * q_p[3] - t183;
  t186 = t74 * t57 * t70;
  t188 = t85 * t83 * t57;
  t190 = param.sp3x * param.m3 * t57;
  t194 = -t102 * t71 + t142;
  t195 = t111 * t194;
  t214 = (theta[4] - t56) * param.K__3;
  m[0][4] = (t4 * (t3 * (t136 * (-t55 * t71 + t186 - t188 - t190) + t93 * t195) + t132 * t194 * t159) + t15 * (t136 * t195 + t93 * (-t107 - t186 + t188 + t190))) * param.g + param.beta * q_p[4] - t214;
  t217 = t72 * t85 + t74 * t84;
  t220 = t57 * (t161 - t113);
  t225 = t111 * t220 + t132 * (-t121 * t74 + t100);
  t248 = (theta[5] - t73) * param.K__4;
  m[0][5] = (t4 * (t3 * (t136 * t217 + t225 * t93) + (t111 * (t85 * t83 - t95) + t220 * t132) * t159) + t15 * (t225 * t136 - t217 * t93)) * param.g + param.beta * q_p[5] - t248;
  t251 = -t105 + t35;
  t253 = t85 * t251 * t71;
  t256 = -t52 + t104;
  t259 = -t98 + t80;
  t261 = t256 * t57 * t85 + t259 * t71;
  t265 = t132 * t256 * t74 + t111 * t261;
  t288 = (theta[6] - t33) * param.K__5;
  m[0][6] = (t4 * (t3 * (t136 * (t57 * (-t81 + t96) - t253) + t93 * t265) + (t111 * t251 * t74 + t132 * t261) * t159) + t15 * (t136 * t265 + t93 * (t259 * t57 + t253))) * param.g + param.beta * q_p[6] - t288;
  t289 = -t63 + t114;
  t292 = -t78 + t42;
  t294 = t74 * t71 * t292;
  t295 = -t115 + t62;
  t298 = t85 * t34 * t295 * t71;
  t302 = t34 * t289;
  t306 = t51 * t295;
  t308 = t57 * (t292 * t74 + t302 * t85) + t71 * t306;
  t315 = t111 * t308 + t132 * (t74 * t302 + t85 * (-t48 + t77));
  t341 = (theta[7] - t40) * param.K__6;
  m[0][7] = (t4 * (t3 * (t136 * (t289 * t51 * t57 + t294 - t298) + t93 * t315) + (t111 * (t295 * t34 * t74 + t292 * t85) + t132 * t308) * t159) + t15 * (t136 * t315 + t93 * (t306 * t57 - t294 + t298))) * param.g + param.beta * q_p[7] - t341;
  t342 = -t36 + t37;
  t344 = t26 + t29;
  t345 = t41 * t344;
  t349 = t47 * t344;
  t351 = t74 * t71 * t349;
  t352 = -t344;
  t353 = t41 * t352;
  t356 = t34 * t353 + t342 * t51;
  t358 = t85 * t356 * t71;
  t363 = -t342;
  t365 = t34 * t345 + t363 * t51;
  t371 = t34 * t363 + t353 * t51;
  t373 = t57 * (t349 * t74 + t365 * t85) + t71 * t371;
  t380 = t111 * t373 + t132 * (t352 * t47 * t85 + t365 * t74);
  t405 = (theta[8] - t24) * param.K__7;
  m[0][8] = (t4 * (t3 * (t136 * (t57 * (t34 * t342 + t345 * t51) + t351 - t358) + t93 * t380) + (t111 * (t349 * t85 + t356 * t74) + t132 * t373) * t159) + t15 * (t136 * t380 + t93 * (t371 * t57 - t351 + t358))) * param.g + param.beta * q_p[8] - t405;
  m[0][9] = t12;
  m[0][10] = t22;
  m[0][11] = t153;
  m[0][12] = t183;
  m[0][13] = t214;
  m[0][14] = t248;
  m[0][15] = t288;
  m[0][16] = t341;
  m[0][17] = t405;
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
  cgret[0][14] = m[0][14];
  cgret[0][15] = m[0][15];
  cgret[0][16] = m[0][16];
  cgret[0][17] = m[0][17];
}
