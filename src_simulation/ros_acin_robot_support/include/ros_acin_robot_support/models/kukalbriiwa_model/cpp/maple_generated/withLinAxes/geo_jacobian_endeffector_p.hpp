#include <math.h>

template<typename Scalar> void geo_jacobian_endeffector_p (
  Scalar *q,
  Scalar *q_p,
 const RobotParams& param,
  Scalar cgret[6][9])
{
  Scalar m[6][9];
  Scalar t1;
  Scalar t10;
  Scalar t101;
  Scalar t102;
  Scalar t107;
  Scalar t11;
  Scalar t114;
  Scalar t119;
  Scalar t121;
  Scalar t122;
  Scalar t125;
  Scalar t13;
  Scalar t130;
  Scalar t133;
  Scalar t134;
  Scalar t135;
  Scalar t14;
  Scalar t140;
  Scalar t143;
  Scalar t144;
  Scalar t147;
  Scalar t154;
  Scalar t16;
  Scalar t160;
  Scalar t162;
  Scalar t169;
  Scalar t17;
  Scalar t172;
  Scalar t175;
  Scalar t18;
  Scalar t181;
  Scalar t182;
  Scalar t183;
  Scalar t186;
  Scalar t189;
  Scalar t19;
  Scalar t192;
  Scalar t2;
  Scalar t20;
  Scalar t207;
  Scalar t21;
  Scalar t215;
  Scalar t22;
  Scalar t220;
  Scalar t221;
  Scalar t222;
  Scalar t224;
  Scalar t226;
  Scalar t23;
  Scalar t231;
  Scalar t234;
  Scalar t235;
  Scalar t24;
  Scalar t243;
  Scalar t246;
  Scalar t247;
  Scalar t25;
  Scalar t252;
  Scalar t253;
  Scalar t257;
  Scalar t26;
  Scalar t264;
  Scalar t27;
  Scalar t270;
  Scalar t272;
  Scalar t274;
  Scalar t275;
  Scalar t276;
  Scalar t277;
  Scalar t279;
  Scalar t28;
  Scalar t281;
  Scalar t283;
  Scalar t285;
  Scalar t287;
  Scalar t291;
  Scalar t293;
  Scalar t294;
  Scalar t295;
  Scalar t297;
  Scalar t3;
  Scalar t30;
  Scalar t301;
  Scalar t302;
  Scalar t307;
  Scalar t309;
  Scalar t31;
  Scalar t313;
  Scalar t320;
  Scalar t321;
  Scalar t322;
  Scalar t323;
  Scalar t324;
  Scalar t325;
  Scalar t326;
  Scalar t33;
  Scalar t331;
  Scalar t332;
  Scalar t334;
  Scalar t336;
  Scalar t337;
  Scalar t339;
  Scalar t34;
  Scalar t340;
  Scalar t343;
  Scalar t348;
  Scalar t35;
  Scalar t350;
  Scalar t354;
  Scalar t355;
  Scalar t356;
  Scalar t358;
  Scalar t365;
  Scalar t38;
  Scalar t39;
  Scalar t4;
  Scalar t40;
  Scalar t41;
  Scalar t42;
  Scalar t423;
  Scalar t43;
  Scalar t437;
  Scalar t44;
  Scalar t455;
  Scalar t459;
  Scalar t460;
  Scalar t463;
  Scalar t479;
  Scalar t480;
  Scalar t489;
  Scalar t49;
  Scalar t490;
  Scalar t492;
  Scalar t493;
  Scalar t499;
  Scalar t5;
  Scalar t508;
  Scalar t513;
  Scalar t517;
  Scalar t518;
  Scalar t52;
  Scalar t523;
  Scalar t527;
  Scalar t533;
  Scalar t536;
  Scalar t54;
  Scalar t544;
  Scalar t55;
  Scalar t552;
  Scalar t575;
  Scalar t58;
  Scalar t597;
  Scalar t6;
  Scalar t60;
  Scalar t61;
  Scalar t62;
  Scalar t63;
  Scalar t64;
  Scalar t65;
  Scalar t66;
  Scalar t67;
  Scalar t68;
  Scalar t69;
  Scalar t7;
  Scalar t70;
  Scalar t71;
  Scalar t72;
  Scalar t73;
  Scalar t75;
  Scalar t76;
  Scalar t77;
  Scalar t8;
  Scalar t81;
  Scalar t82;
  Scalar t87;
  Scalar t88;
  Scalar t89;
  Scalar t9;
  Scalar t91;
  Scalar t93;
  Scalar t95;
  Scalar t96;
  m[0][0] = 0;
  m[0][1] = 0;
  m[0][2] = 0;
  m[0][3] = 0;
  m[0][4] = 0;
  m[0][5] = 0;
  m[0][6] = 0;
  m[0][7] = 0;
  m[0][8] = 0;
  m[1][0] = 0;
  m[1][1] = 0;
  m[1][2] = 0;
  m[1][3] = 0;
  m[1][4] = 0;
  m[1][5] = 0;
  m[1][6] = 0;
  m[1][7] = 0;
  m[1][8] = 0;
  m[2][0] = 0;
  m[2][1] = 0;
  m[2][2] = 0;
  m[2][3] = 0;
  m[2][4] = 0;
  m[2][5] = 0;
  m[2][6] = 0;
  m[2][7] = 0;
  m[2][8] = 0;
  m[3][0] = 0;
  m[3][1] = 0;
  m[3][2] = 0;
  m[3][3] = 0;
  m[3][4] = 0;
  m[3][5] = 0;
  m[3][6] = 0;
  m[3][7] = 0;
  m[3][8] = 0;
  m[4][0] = 0;
  m[4][1] = 0;
  m[4][2] = 0;
  m[4][3] = 0;
  m[4][4] = 0;
  m[4][5] = 0;
  m[4][6] = 0;
  m[4][7] = 0;
  m[4][8] = 0;
  m[5][0] = 0;
  m[5][1] = 0;
  m[5][2] = 0;
  m[5][3] = 0;
  m[5][4] = 0;
  m[5][5] = 0;
  m[5][6] = 0;
  m[5][7] = 0;
  m[5][8] = 0;
  m[0][0] = 0.0e0;
  m[0][1] = 0.0e0;
  t1 = q[6];
  t2 = cos(t1);
  t3 = q_p[5];
  t4 = t3 * t2;
  t5 = q_p[7];
  t6 = t4 - t5;
  t7 = q[4];
  t8 = cos(t7);
  t9 = t8 * t6;
  t10 = q_p[3];
  t11 = t10 * t2;
  t13 = q[5];
  t14 = sin(t13);
  t16 = sin(t1);
  t17 = cos(t13);
  t18 = q_p[6];
  t19 = t18 * t17;
  t20 = q_p[4];
  t21 = t19 + t20;
  t22 = t21 * t16;
  t23 = t8 * t22;
  t24 = sin(t7);
  t25 = t2 * t24;
  t26 = t17 * t20;
  t27 = t26 + t18;
  t28 = t27 * t25;
  t30 = q[3];
  t31 = cos(t30);
  t33 = t16 * t18;
  t34 = sin(t30);
  t35 = t14 * t34;
  t38 = t34 * t10;
  t39 = t38 * t2 * t17;
  t40 = q_p[2];
  t41 = t40 * t16;
  t42 = t39 + t41;
  t43 = t8 * t42;
  t44 = t2 * t40;
  t49 = t24 * t16;
  t52 = param.d7 + param.d8;
  t54 = q[7];
  t55 = sin(t54);
  t58 = cos(t54);
  t60 = t52 * t58 + param.d5 + param.d6;
  t61 = t60 * t14;
  t62 = t61 * t20 * t24;
  t63 = t58 * t5;
  t64 = t2 * t52;
  t65 = t64 * t63;
  t66 = t3 * t60;
  t67 = -t65 + t66;
  t68 = t17 * t67;
  t69 = t8 * t68;
  t70 = t60 * t10;
  t71 = t17 * t70;
  t72 = t24 * t5;
  t73 = t52 * t16;
  t75 = t58 * t73 * t72;
  t76 = param.d3 + param.d4;
  t77 = t76 * t10;
  t81 = t8 * t34 * t70;
  t82 = t60 * t40;
  t87 = -t55 * t52 * (t31 * (t14 * (t9 - t11) + t23 + t28) + t35 * t33 + t43 + t17 * (t24 * t44 - t34 * t6) - t38 * t49) + t31 * (t62 - t69 + t71 - t75 + t77) + t14 * (t24 * t82 - t34 * t67 + t81);
  t88 = q[2];
  t89 = sin(t88);
  t91 = t17 * t8;
  t93 = t2 * t91 - t49;
  t95 = t31 * t93 * t40;
  t96 = -t6;
  t101 = t27 * t2;
  t102 = t8 * t101;
  t107 = t8 * t40;
  t114 = -t67;
  t119 = t34 * t40;
  t121 = t55 * (t95 + t14 * (t24 * t96 + t34 * t44) + t102 - t21 * t49) * t52 - t31 * t61 * t107 - t61 * t8 * t20 + t8 * t73 * t63 + t17 * (t114 * t24 + t34 * t82) + t76 * t119;
  t122 = cos(t88);
  m[0][2] = t121 * t122 + t87 * t89;
  t125 = t8 * t96 + t11;
  t130 = t55 * (t125 * t14 - t23 - t28) * t52 + t62 - t69 + t71 - t75 + t77;
  t133 = t11 * t91;
  t134 = t17 * t96;
  t135 = t10 * t24;
  t140 = t8 * t70;
  t143 = t55 * t52 * (-t135 * t16 + t14 * t33 + t133 + t134) - t14 * (t140 + t65 - t66);
  t144 = t143 * t31;
  t147 = t52 * t55;
  t154 = t55 * t2;
  t160 = t40 * (t34 * (t14 * t60 * t8 - t147 * t93) + t31 * (t14 * t154 * t52 + t17 * t60 + param.d3 + param.d4));
  m[0][3] = t122 * (t130 * t34 + t144) + t89 * t160;
  t162 = t14 * t6;
  t169 = t14 * t10;
  t172 = -t55 * (t31 * (t162 + t22) + t39 + t41) * t52 - t31 * t68 + t34 * t60 * t169;
  t175 = t17 * t40;
  t181 = t60 * t20;
  t182 = t14 * t181;
  t183 = t73 * t63;
  t186 = t14 * t40;
  t189 = t8 * (-t55 * t52 * (t101 * t31 - t16 * t38 + t175 * t2) + t31 * (t182 - t183) + t60 * t186);
  t192 = t31 * t40;
  t207 = t24 * (-t55 * (t17 * t192 + t18 + t26) * t64 + t31 * t60 * t186 + t182 - t183) - t8 * (t55 * t52 * (t16 * t192 + t162 + t22) + t68);
  m[0][4] = t122 * (t172 * t24 - t189) + t207 * t89;
  t215 = t31 * t24;
  t220 = t34 * t3;
  t221 = t24 * t40;
  t222 = -t220 + t221;
  t224 = -t55 * (-t16 * t18 * t34 + t125 * t31) * t52 - t215 * t181 - t81 - t2 * t52 * t34 * t63 - t60 * t222;
  t226 = t20 * t2;
  t231 = t8 * t10;
  t234 = t2 * t222;
  t235 = t34 * t5;
  t243 = t14 * (-t55 * t52 * (t31 * (t226 * t24 + t33 * t8) + t34 * t2 * t231 + t234 + t235) + t31 * (t114 * t8 + t70));
  t246 = t3 * t24;
  t247 = t119 - t246;
  t252 = t192 + t20;
  t253 = t8 * t252;
  t257 = t8 * t2;
  t264 = t52 * t24;
  t270 = t17 * (-t55 * (t2 * t247 + t72) * t52 + t60 * t253) + t14 * (t55 * (t192 * t257 + t226 * t8 - t24 * t33) * t52 + t2 * t264 * t63 + t60 * t247);
  m[0][5] = t122 * (t17 * t224 + t243) - t270 * t89;
  t272 = t14 * t8;
  t274 = t24 * t17;
  t275 = t20 * t274;
  t276 = t18 * t24;
  t277 = -t272 * t3 + t169 - t275 - t276;
  t279 = t8 * t38;
  t281 = (t279 - t220 + t221) * t17;
  t283 = t16 * (t277 * t31 - t281);
  t285 = t31 * t21 * t8;
  t287 = -t14 * t18 + t135;
  t291 = t283 - t2 * (t287 * t34 - t107 - t285);
  t293 = t8 * t31;
  t294 = t175 * t293;
  t295 = t8 * t27;
  t297 = -t247 * t14;
  t301 = (t19 + t192 + t20) * t25;
  t302 = t16 * (t294 + t295 - t297) + t301;
  t307 = t17 * t293 + t35;
  t309 = t31 * t25;
  t313 = -t16 * t274 + t257;
  m[0][6] = (t55 * (t122 * t291 - t302 * t89) + (t122 * (t16 * t307 + t309) + t313 * t89) * t63) * t52;
  t320 = t16 * t17;
  t321 = t18 * t320;
  t322 = t14 * t5;
  t323 = t16 * t20;
  t324 = t14 * t2 * t3 + t321 - t322 + t323;
  t325 = t8 * t324;
  t326 = -t275 + t169 - t276;
  t331 = t17 * (t234 + t235);
  t332 = t16 * t34;
  t334 = t31 * (-t2 * t326 + t325) + t43 + t331 - t287 * t332;
  t336 = t2 * t5;
  t337 = -t336 + t3;
  t339 = t8 * t337 * t17;
  t340 = t17 * t10;
  t343 = t14 * t20 - t16 * t5;
  t348 = t336 * t34 - t220 + t221 + t279;
  t350 = t31 * (t24 * t343 - t339 + t340) + t348 * t14;
  t354 = t274 * t33;
  t355 = t2 * t297;
  t356 = t323 - t322;
  t358 = -t24 * t356 + t102 - t354 - t355 + t95;
  t365 = t192 * t272 + t8 * t343 + (-t24 * t336 - t119 + t246) * t17;
  m[0][7] = (t122 * (t334 * t58 + t350 * t55) + (t358 * t58 + t365 * t55) * t89) * t52;
  m[0][8] = 0.0e0;
  m[1][0] = 0.0e0;
  m[1][1] = 0.0e0;
  m[1][2] = -t121 * t89 + t122 * t87;
  m[1][3] = t89 * (-t130 * t34 - t144) + t122 * t160;
  m[1][4] = t89 * (-t172 * t24 + t189) + t122 * t207;
  m[1][5] = t89 * (-t17 * t224 - t243) - t270 * t122;
  m[1][6] = -(t55 * (t122 * t302 + t291 * t89) - (t89 * (-t16 * t307 - t309) + t313 * t122) * t63) * t52;
  m[1][7] = -(t58 * (-t122 * t358 + t334 * t89) + t55 * (-t122 * t365 + t350 * t89)) * t52;
  m[1][8] = 0.0e0;
  m[2][0] = 0.0e0;
  m[2][1] = 0.0e0;
  m[2][2] = 0.0e0;
  m[2][3] = t130 * t31 - t143 * t34;
  t423 = t31 * t10;
  m[2][4] = t34 * (t24 * (t55 * t52 * (t321 + t162 + t323) + t68) + (-t27 * t55 * t64 + t182 - t183) * t8) + (t24 * (-t154 * t17 * t52 + t61) - t147 * t16 * t8) * t423;
  t437 = t147 * t33;
  m[2][5] = t34 * (t17 * (t11 * t147 - t147 * t9 + t181 * t24) - t14 * (t8 * (-t437 + t65 - t66) - t55 * t264 * t226 + t70)) - (t17 * (t140 - t437 + t65 - t66) + (t2 * t231 - t4 + t5) * t52 * t55 * t14) * t31;
  t455 = -t277;
  t459 = t34 * (t16 * t455 - t21 * t257);
  t460 = -t231 + t3;
  t463 = -t287;
  m[2][6] = t52 * (t55 * (t459 + t31 * (t16 * t17 * t460 + t2 * t463)) + t5 * t58 * (t34 * (-t320 * t8 - t25) + t31 * t16 * t14));
  t479 = -t2 * t326;
  t480 = t325 + t479;
  t489 = -t24 * t343;
  t490 = -t339 + t340 - t489;
  t492 = t14 * t31;
  t493 = -t336 - t231 + t3;
  m[2][7] = -(t58 * (t34 * t480 + t31 * (-t16 * t463 + t17 * t6 - t133)) + t55 * (t34 * t490 + t492 * t493)) * t52;
  m[2][8] = 0.0e0;
  m[3][0] = 0.0e0;
  m[3][1] = 0.0e0;
  m[3][2] = 0.0e0;
  m[3][3] = t122 * t40;
  t499 = t89 * t40;
  m[3][4] = -t10 * t122 * t31 + t34 * t499;
  t508 = t10 * t24 * t34 - (t20 * t31 + t40) * t8;
  m[3][5] = t24 * t252 * t89 + t122 * t508;
  t513 = -t20 * t215 + t220 - t221 - t279;
  t517 = t3 * t8 - t10;
  t518 = t517 * t17 * t31;
  t523 = t14 * t253 - t17 * t247;
  m[3][6] = t122 * (t14 * t513 + t518) - t523 * t89;
  t527 = (t34 * t463 + t107 + t285) * t2;
  t533 = t16 * (t247 * t14 + t294 + t295) + t301;
  m[3][7] = t122 * (t283 + t527) - t89 * t533;
  t536 = t463 * t332;
  t544 = t58 * (-t348 * t14 - t31 * t490);
  t552 = t55 * (t24 * t356 - t102 + t354 + t355 - t95) + t58 * t365;
  m[3][8] = t122 * (t55 * (t31 * t480 + t331 + t43 + t536) + t544) - t89 * t552;
  m[4][0] = 0.0e0;
  m[4][1] = 0.0e0;
  m[4][2] = 0.0e0;
  m[4][3] = -t499;
  m[4][4] = t10 * t31 * t89 + t122 * t34 * t40;
  m[4][5] = t122 * t24 * t252 - t508 * t89;
  m[4][6] = t89 * (-t14 * t513 - t518) - t523 * t122;
  m[4][7] = t89 * (t16 * (t31 * t455 + t281) - t527) - t533 * t122;
  t575 = -t8 * t324 - t479;
  m[4][8] = t89 * (t55 * (t31 * t575 - t8 * t42 + t17 * (-t2 * t222 - t235) - t536) - t544) - t122 * t552;
  m[5][0] = 0.0e0;
  m[5][1] = 0.0e0;
  m[5][2] = 0.0e0;
  m[5][3] = 0.0e0;
  m[5][4] = t38;
  m[5][5] = t20 * t34 * t8 + t24 * t423;
  t597 = -t460;
  m[5][6] = t34 * (t14 * t20 * t24 - t17 * t517) - t597 * t492;
  m[5][7] = t459 - (t16 * t17 * t597 + t2 * t287) * t31;
  m[5][8] = t34 * (t55 * t575 + t58 * (-t8 * t337 * t17 + t340 - t489)) + (t55 * (-t16 * t287 + t133 + t134) + t493 * t14 * t58) * t31;
  cgret[0][0] = m[0][0];
  cgret[0][1] = m[0][1];
  cgret[0][2] = m[0][2];
  cgret[0][3] = m[0][3];
  cgret[0][4] = m[0][4];
  cgret[0][5] = m[0][5];
  cgret[0][6] = m[0][6];
  cgret[0][7] = m[0][7];
  cgret[0][8] = m[0][8];
  cgret[1][0] = m[1][0];
  cgret[1][1] = m[1][1];
  cgret[1][2] = m[1][2];
  cgret[1][3] = m[1][3];
  cgret[1][4] = m[1][4];
  cgret[1][5] = m[1][5];
  cgret[1][6] = m[1][6];
  cgret[1][7] = m[1][7];
  cgret[1][8] = m[1][8];
  cgret[2][0] = m[2][0];
  cgret[2][1] = m[2][1];
  cgret[2][2] = m[2][2];
  cgret[2][3] = m[2][3];
  cgret[2][4] = m[2][4];
  cgret[2][5] = m[2][5];
  cgret[2][6] = m[2][6];
  cgret[2][7] = m[2][7];
  cgret[2][8] = m[2][8];
  cgret[3][0] = m[3][0];
  cgret[3][1] = m[3][1];
  cgret[3][2] = m[3][2];
  cgret[3][3] = m[3][3];
  cgret[3][4] = m[3][4];
  cgret[3][5] = m[3][5];
  cgret[3][6] = m[3][6];
  cgret[3][7] = m[3][7];
  cgret[3][8] = m[3][8];
  cgret[4][0] = m[4][0];
  cgret[4][1] = m[4][1];
  cgret[4][2] = m[4][2];
  cgret[4][3] = m[4][3];
  cgret[4][4] = m[4][4];
  cgret[4][5] = m[4][5];
  cgret[4][6] = m[4][6];
  cgret[4][7] = m[4][7];
  cgret[4][8] = m[4][8];
  cgret[5][0] = m[5][0];
  cgret[5][1] = m[5][1];
  cgret[5][2] = m[5][2];
  cgret[5][3] = m[5][3];
  cgret[5][4] = m[5][4];
  cgret[5][5] = m[5][5];
  cgret[5][6] = m[5][6];
  cgret[5][7] = m[5][7];
  cgret[5][8] = m[5][8];
}
