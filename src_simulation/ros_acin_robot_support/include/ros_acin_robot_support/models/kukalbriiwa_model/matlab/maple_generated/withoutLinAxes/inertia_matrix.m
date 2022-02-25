function tmpreturn = inertia_matrix(q, param)
  m = [0 0 0 0 0 0 0; 0 0 0 0 0 0 0; 0 0 0 0 0 0 0; 0 0 0 0 0 0 0; 0 0 0 0 0 0 0; 0 0 0 0 0 0 0; 0 0 0 0 0 0 0;];
  t1 = (param.d3 ^ 2);
  t3 = 2 * param.d3 * param.d4;
  t4 = param.d4 ^ 2;
  t5 = (param.d5 ^ 2);
  t7 = 2 * param.d5 * param.d6;
  t8 = param.d6 ^ 2;
  t9 = (param.sp6x ^ 2);
  t10 = (param.sp6z ^ 2);
  t13 = param.sp3x ^ 2;
  t14 = param.sp3z ^ 2;
  t17 = param.sp2y ^ 2;
  t18 = param.sp2z ^ 2;
  t21 = (param.sp5y ^ 2);
  t22 = (param.sp5z ^ 2);
  t25 = q(4);
  t26 = sin(t25);
  t27 = param.sp7x * param.m7;
  t28 = q(7);
  t29 = cos(t28);
  t31 = param.sp7y * param.m7;
  t32 = sin(t28);
  t33 = t32 * t31;
  t34 = param.m6 * param.sp6x;
  t35 = -t27 * t29 + t33 + t34;
  t37 = param.d3 + param.d4;
  t38 = t37 * t35 * t26;
  t39 = param.sp7y * t27;
  t40 = -t39 + param.I7xy;
  t41 = 0.2e1 * t40;
  t42 = t29 ^ 2;
  t43 = t42 * t41;
  t44 = param.sp7x ^ 2;
  t45 = param.sp7y ^ 2;
  t46 = t44 - t45;
  t48 = param.m7 * t46 - param.I7xx + param.I7yy;
  t49 = t32 * t48;
  t50 = t29 * t49;
  t52 = param.sp6x * param.sp6z * param.m6;
  t53 = t43 - t50 + t39 + t52 - param.I6xz - param.I7xy;
  t54 = q(5);
  t55 = sin(t54);
  t56 = t55 * t53;
  t59 = q(6);
  t60 = cos(t59);
  t62 = -param.sp7z - param.d7;
  t64 = param.m6 * param.sp6y;
  t66 = sin(t59);
  t67 = t66 * (param.m7 * t62 - t64);
  t68 = param.sp5x * param.m5;
  t69 = t67 + t68;
  t70 = t37 * t69;
  t71 = t26 * t70;
  t72 = 0.2e1 * t71;
  t73 = -t62;
  t75 = param.m7 * t73 * param.sp7y;
  t76 = t75 - param.I7yz;
  t78 = t29 * t66 * t76;
  t80 = param.m7 * t73 * param.sp7x;
  t81 = t80 - param.I7xz;
  t82 = t32 * t81;
  t84 = param.sp6y * param.sp6z * param.m6;
  t85 = t82 + t84 - param.I6yz;
  t86 = t66 * t85;
  t88 = param.sp5x * param.sp5y * param.m5;
  t90 = t55 * (t78 + t86 + t88 - param.I5xy);
  t93 = cos(t54);
  t95 = -t81;
  t96 = 0.2e1 * t95;
  t98 = 0.2e1 * t76;
  t99 = t32 * t98;
  t101 = (param.sp6x * param.sp6y * param.m6);
  t102 = 2 * t101;
  t103 = 2 * param.I6xy;
  t104 = t29 * t96 + t102 - t103 + t99;
  t105 = t60 ^ 2;
  t108 = t42 * t48 * t66;
  t109 = -t40;
  t110 = t109 * t32;
  t111 = t66 * t110;
  t113 = param.d5 + param.d6;
  t114 = t113 * t27;
  t115 = -0.2e1 * t111 - t114;
  t117 = param.d7 + param.sp7y + param.sp7z;
  t118 = param.d7 - param.sp7y + param.sp7z;
  t120 = param.m7 * t118 * t117;
  t121 = (param.sp6y ^ 2);
  t122 = (t9 - t121);
  t123 = (param.m6 * t122);
  t125 = t66 * (-t120 + t123 + param.I6yy - param.I7yy + param.I7zz - param.I6xx);
  t126 = t33 + t34;
  t127 = t113 * t126;
  t128 = t115 * t29 + t108 + t125 + t127;
  t129 = t60 * t128;
  t130 = t29 * t81;
  t132 = param.m7 * t73 + t64;
  t133 = t132 * t113;
  t134 = t66 * t133;
  t135 = -t76;
  t136 = t135 * t32;
  t138 = param.d5 * param.sp5x * param.m5;
  t139 = param.sp5z * t68;
  t140 = t104 * t105 - param.I5xz + param.I6xy - t101 + t129 + t130 - t134 + t136 + t138 + t139;
  t141 = t93 * t140;
  t142 = t29 * t76;
  t143 = t142 + t82 + t84 - param.I6yz;
  t144 = t143 * t55;
  t145 = t60 * t144;
  t146 = t66 * t55;
  t148 = t42 * t109 * t146;
  t149 = 0.2e1 * t148;
  t151 = t113 * t31;
  t152 = -t49 * t66 + t151;
  t153 = t55 * t152;
  t154 = t29 * t153;
  t155 = t52 + t39 - param.I6xz - param.I7xy;
  t156 = t66 * t155;
  t157 = t32 * t113;
  t158 = t157 * t27;
  t160 = param.m6 * t113 * param.sp6z;
  t161 = param.d5 * param.m5;
  t162 = param.sp5y * t161;
  t164 = param.sp5y * param.sp5z * param.m5;
  t166 = t55 * (t156 + t158 + t160 - t162 - t164 + param.I5yz);
  t167 = param.sp4x * param.m4;
  t168 = param.sp4y * t167;
  t171 = cos(t25);
  t173 = 0.2e1 * t171 * (t141 + t145 - t149 + t154 + t166 - t168 + param.I4xy) * t26;
  t176 = -param.m7 * t46 + param.I7xx - param.I7yy;
  t177 = t42 * t176;
  t178 = t29 * t110;
  t179 = 0.2e1 * t178;
  t181 = -param.m6 * t122;
  t182 = t177 + t179 + t120 + t181 + param.I6xx - param.I6yy + param.I7yy - param.I7zz;
  t183 = t105 * t182;
  t184 = t29 * t95;
  t185 = t32 * t76;
  t186 = t184 + t185 + t101 - param.I6xy;
  t188 = t60 * t186 * t66;
  t189 = 0.2e1 * t188;
  t190 = param.sp7z + param.d7 + param.sp7x;
  t191 = param.sp7z + param.d7 - param.sp7x;
  t193 = param.m7 * t191 * t190;
  t194 = (-t121 + t10);
  t195 = param.m6 * t194;
  t196 = param.sp5x ^ 2;
  t197 = -t196 + t21;
  t198 = param.m5 * t197;
  t199 = t183 + t189 + t177 + t179 - t193 + t195 + t198 + param.I5xx - param.I5yy + param.I6yy - param.I6zz - param.I7xx + param.I7zz;
  t200 = t93 ^ 2;
  t201 = t200 * t199;
  t203 = 0.2e1 * param.sp5z * t161;
  t205 = -t42 * t41;
  t206 = t205 + t50 - t52 - t39 + param.I6xz + param.I7xy;
  t207 = t60 * t206;
  t208 = t207 + t78 + t86 + t88 - param.I5xy;
  t211 = 0.2e1 * t93 * t55 * t208;
  t212 = param.m5 * t5;
  t213 = t66 * t127;
  t214 = 0.2e1 * t213;
  t215 = 0.2e1 * t46;
  t217 = 0.2e1 * param.I7xx;
  t218 = 0.2e1 * param.I7yy;
  t220 = t42 * (param.m7 * t215 - t217 + t218);
  t221 = 0.4e1 * t178;
  t222 = 0.2e1 * t120;
  t223 = 2 * t122;
  t224 = param.m6 * t223;
  t225 = 0.2e1 * param.I6xx;
  t226 = 0.2e1 * param.I6yy;
  t227 = 0.2e1 * param.I7zz;
  t228 = t220 - t221 - t222 + t224 - t225 + t226 - t218 + t227;
  t231 = t29 * t81 * t66;
  t232 = 0.4e1 * t231;
  t233 = 0.4e1 * t135;
  t235 = 4 * t101;
  t236 = 4 * param.I6xy;
  t239 = 0.2e1 * t133;
  t242 = param.I6xx - param.I6yy - param.I7zz - param.I5xx - t173 + t201 - t203 + t211 - t212 - t214 - param.I4zz + t105 * t228 + t60 * (t232 + t66 * (t233 * t32 - t235 + t236) - t239);
  t243 = t196 - t22;
  t245 = param.sp4y ^ 2;
  t246 = param.sp4z ^ 2;
  t247 = -t245 + t246;
  t249 = param.sp3y ^ 2;
  t250 = -t13 + t249;
  t255 = -t45 - (param.sp7z + param.d5 + param.d6 + param.d7) * (-param.sp7z + param.d5 + param.d6 - param.d7);
  t257 = -t5 - t7 - t8 - t9 + t121;
  t260 = t66 * t113 * t27;
  t262 = 0.2e1 * t260 + 0.2e1 * t110;
  t263 = t29 * t262;
  t264 = 0.2e1 * t231;
  t266 = t66 * (t99 + t102 - t103);
  t268 = t60 * (-t264 + t266 + t239);
  t269 = 0.2e1 * t260;
  t271 = -t269 - 0.4e1 * t110;
  t272 = t29 * t271;
  t276 = t45 + (param.d5 + param.d6 + param.sp7x) * (param.d5 + param.d6 - param.sp7x);
  t277 = param.m7 * t276;
  t278 = t5 + t7 + t8 + t9 - t10;
  t279 = param.m6 * t278;
  t281 = -t21 + t22;
  t282 = param.m5 * t281;
  t283 = param.sp4x ^ 2;
  t284 = -t283 + t245;
  t285 = param.m4 * t284;
  t286 = t203 + t282 - param.I7yy + param.I7xx + t285 + param.I4xx - param.I4yy + param.I5yy - param.I5zz - param.I6xx + param.I6zz;
  t288 = t171 ^ 2;
  t290 = (param.m5 * t243) + param.m4 * t247 + param.m3 * t250 + param.m7 * t255 + (param.m6 * t257) + t263 + t288 * (t201 + t211 + t183 + t268 + t220 + t272 + t214 + t277 + t279 + t212 + t286) + param.I3xx + param.I4yy + param.I5zz - param.I3yy + param.I7yy + t177;
  t292 = q(3);
  t293 = cos(t292);
  t294 = t293 ^ 2;
  t296 = param.sp1x ^ 2;
  t297 = param.sp1y ^ 2;
  t300 = sin(t292);
  t301 = 0.4e1 * t109;
  t302 = t42 * t301;
  t303 = 0.2e1 * t50;
  t304 = 0.2e1 * t52;
  t305 = 0.2e1 * t39;
  t306 = 0.2e1 * param.I6xz;
  t307 = 0.2e1 * param.I7xy;
  t311 = -t96;
  t316 = t66 * (t311 * t32 - 0.2e1 * param.I6yz + 0.2e1 * t84);
  t317 = 0.2e1 * t88;
  t318 = 0.2e1 * param.I5xy;
  t321 = t42 * t48;
  t322 = t321 - t179 - t120 + t123 - param.I6xx + param.I6yy - param.I7yy + param.I7zz;
  t323 = t105 * t322;
  t325 = -param.m6 * t194;
  t327 = -param.m5 * t197;
  t328 = t323 - t189 + t321 - t179 + t193 + t325 + t327 - param.I5xx + param.I5yy - param.I6yy + param.I6zz + param.I7xx - param.I7zz;
  t331 = t60 * t53;
  t332 = t32 * t95;
  t333 = t332 - t84 + param.I6yz;
  t334 = t333 * t66;
  t335 = param.sp4z * t167;
  t336 = t200 * (t60 * (t302 + t303 - t304 - t305 + t306 + t307) + 0.2e1 * t78 + t316 + t317 - t318) + t93 * t328 * t55 + t331 - t78 + t334 - t335 - t88 + param.I4xz + param.I5xy;
  t337 = t171 * t336;
  t338 = t29 * t135;
  t339 = t338 + t332 - t84 + param.I6yz;
  t340 = t60 * t339;
  t342 = t42 * t109 * t66;
  t343 = 0.2e1 * t342;
  t345 = -t29 * t152;
  t346 = -t155;
  t347 = t346 * t66;
  t348 = t340 + t343 + t345 + t347 - t158 - t160 + t162 + t164 - param.I5yz;
  t351 = t26 * t55;
  t354 = 0.2e1 * t105 * t186 * t351;
  t355 = t129 * t351;
  t356 = t55 * t81;
  t357 = t29 * t356;
  t359 = t55 * (-t134 + t136 + t138 + t139 - t101 - param.I5xz + param.I6xy);
  t360 = param.sp4y * param.m4;
  t361 = param.sp4z * t360;
  t365 = param.sp3x * param.sp3y * param.m3;
  t369 = 0.2e1 * t293 * (t337 + t93 * t348 * t26 + t354 + t355 + t26 * (t357 + t359 + t361 - param.I4yz) + t365 - param.I3xy) * t300;
  t373 = -t233 * t32;
  t375 = t105 * (0.4e1 * t29 * t95 + t235 - t236 + t373);
  t386 = t29 * t311;
  t387 = 0.2e1 * t134;
  t389 = -t32 * t98;
  t390 = 2 * t138;
  t391 = 0.2e1 * t139;
  t392 = 0.2e1 * param.I5xz;
  t395 = 0.2e1 * t145;
  t396 = 0.4e1 * t148;
  t397 = 0.2e1 * t154;
  t400 = 0.2e1 * t158;
  t401 = 2 * t160;
  t402 = 2 * t162;
  t403 = 0.2e1 * t164;
  t404 = 0.2e1 * param.I5yz;
  t406 = t55 * (0.2e1 * t66 * t155 + t400 + t401 - t402 - t403 + t404);
  t407 = 0.2e1 * t168;
  t408 = 0.2e1 * param.I4xy;
  t412 = t200 * t328 * t26;
  t415 = t37 * t35;
  t417 = t60 * (-0.2e1 * t26 * t56 + t415);
  t419 = 0.2e1 * t26 * t90;
  t422 = t322 * t26;
  t423 = t105 * t422;
  t427 = (-t231 + t66 * (t185 + t101 - param.I6xy) + t133) * t26;
  t429 = 0.2e1 * t60 * t427;
  t430 = t220 + t272 + t214 + t277 + t279 + t212 + t203 + t282 - param.I7yy + param.I7xx + t285 + param.I4xx - param.I4yy + param.I5yy - param.I5zz - param.I6xx + param.I6zz;
  t431 = t26 * t430;
  t433 = t29 * t55 * t31;
  t434 = t32 * t27;
  t435 = param.sp5y * param.m5;
  t436 = param.sp6z * param.m6;
  t437 = -t434 + t435 - t436;
  t440 = t37 * (t437 * t55 + t167 - t433);
  t444 = t105 * (t386 + t389 - t102 + t103);
  t448 = t66 * (t120 + t181 + param.I6xx - param.I6yy + param.I7yy - param.I7zz);
  t449 = -t115 * t29 - t108 - t127 + t448;
  t452 = t93 * (t449 * t60 + param.I5xz - param.I6xy + t101 + t134 - t138 - t139 + t184 + t185 + t444);
  t453 = t37 * t132;
  t454 = t26 * t453;
  t455 = t454 - t144;
  t456 = t60 * t455;
  t457 = t66 * param.sp7x;
  t459 = t29 * param.m7 * t457;
  t460 = t66 * t126;
  t461 = param.m7 * t113;
  t462 = param.m6 * t113;
  t463 = param.sp5z * param.m5;
  t465 = t37 * (-t459 + t460 + t461 + t462 + t161 + t360 + t463);
  t466 = t26 * t465;
  t468 = t55 * (t347 - t158 - t160 + t162 + t164 - param.I5yz);
  t470 = param.d3 * param.sp3x * param.m3;
  t472 = param.m3 * param.sp3x * param.sp3z;
  t473 = t288 * (t93 * (t375 + t60 * (0.2e1 * t108 + t29 * (-0.4e1 * t111 - 0.2e1 * t114) + t66 * (-t222 + t224 - t225 + t226 - t218 + t227) + 0.2e1 * t127) + t386 - t387 + t389 + t390 + t391 - t102 - t392 + t103) + t395 - t396 + t397 + t406 - t407 + t408) + t171 * (-t412 + t93 * (t417 + t419 + t70) - t423 + t429 + t431 - t440) + t452 + t456 + t466 + t149 - t154 + t468 - t470 - t472 + t168 + param.I3xz - param.I4xy;
  t475 = t93 * t348;
  t478 = 0.2e1 * t105 * t55 * t186;
  t480 = t60 * t55 * t128;
  t484 = t26 * t300;
  t487 = 0.2e1 * t200 * t208 * t484;
  t489 = t105 * t55 * t422;
  t493 = 0.2e1 * t60 * t146 * t186 * t26;
  t494 = t321 - t179 + t193 + t325 + t327 - param.I5xx + param.I5yy - param.I6yy + param.I6zz + param.I7xx - param.I7zz;
  t496 = t26 * t55 * t494;
  t498 = -t29 * t31 - t434 + t435 - t436;
  t499 = t37 * t498;
  t504 = t55 * t415;
  t505 = t206 * t26 + t504;
  t508 = t78 + t86 + t335 + t88 - param.I4xz - param.I5xy;
  t511 = t37 * t300;
  t512 = t55 * t69;
  t514 = param.sp3y * param.m3;
  t516 = -param.sp4z * param.m4 + t514;
  t517 = param.d3 * t516;
  t518 = param.d4 * param.m4;
  t519 = param.sp4z * t518;
  t520 = param.sp3z * t514;
  t526 = q(2);
  t527 = sin(t526);
  t529 = cos(t526);
  t532 = param.I6yy + param.I7zz + (param.m6 * (t1 + t3 + t4 + t5 + t7 + t8 + t9 + t10)) + param.m3 * (t13 + t14) + param.m2 * (t17 + t18) + (param.m5 * (t1 + t3 + t4 + t21 + t22)) + param.I5xx + t93 * (t60 * (-0.2e1 * t38 + 0.2e1 * t56) - t72 - 0.2e1 * t90) + t294 * (t242 + t290) + (t296 + t297) * param.m1 + t369 + t203 + t212 + t214 - 0.2e1 * t529 * t527 * (t293 * t473 - t171 * (t475 + t478 + t480 + t357 + t359 + t361 - param.I4yz) * t300 + t487 - t93 * (-t489 + t493 - t496 + t499) * t300 - t60 * t505 * t300 - t26 * t508 * t300 - t512 * t511 + t300 * (t517 - t519 + t520 - param.I3yz) + param.sp2y * param.sp2x * param.m2 - param.I2xy);
  t533 = t60 * t132;
  t540 = 0.2e1 * t29 * t461 * t457;
  t541 = param.m4 * t4;
  t543 = 0.2e1 * t26 * t440;
  t544 = t37 ^ 2;
  t547 = param.m3 + param.m4;
  t548 = t1 * t547;
  t551 = 0.2e1 * param.m3 * param.sp3z + 0.2e1 * t518;
  t552 = param.d3 * t551;
  t553 = t283 + t245;
  t554 = t553 * param.m4;
  t564 = 0.2e1 * t93 * t140 * t26;
  t578 = param.sp2x ^ 2;
  t584 = -param.m7 * t255;
  t586 = -param.m6 * t257;
  t588 = -param.m5 * t243;
  t590 = -param.m4 * t247;
  t592 = -param.m3 * t250;
  t596 = t42 * (-param.m7 * t215 + t217 - t218);
  t600 = t105 * (-(param.m6 * t223) + t218 + t221 + t222 + t225 - t226 - t227 + t596);
  t601 = -param.I6xx + param.I6yy + param.I7zz + t584 + t586 + t588 + t590 + t592 + t600 + param.I5xx + t173 + t321 + t203;
  t605 = t60 * (-t232 + t66 * (t373 + t235 - t236) + t239);
  t607 = -t29 * t262;
  t608 = t200 * t328;
  t610 = t66 * (t389 - t102 + t103);
  t612 = t60 * (t264 + t610 - t239);
  t614 = -t29 * t271;
  t616 = -param.m7 * t276;
  t618 = -param.m6 * t278;
  t621 = -param.m5 * t281;
  t623 = -param.m4 * t284;
  t624 = -t203 + t621 + param.I7yy - param.I7xx + t623 - param.I4xx + param.I4yy - param.I5yy + param.I5zz + param.I6xx - param.I6zz;
  t626 = t288 * (t608 - t211 + t323 + t612 + t596 + t614 - t214 + t616 + t618 - t212 + t624);
  t627 = -t211 + t212 + t214 + param.I4zz - param.I3xx + t605 + t607 - param.I4yy - param.I5zz + param.I3yy - param.I7yy + t626 + t608;
  t630 = -param.I6yy - param.I7zz - param.I5xx + param.I5yy + param.I6zz + t93 * (t60 * (0.2e1 * t38 - 0.4e1 * t56) + t72 + 0.4e1 * t90) + t171 * (t564 + t60 * (0.2e1 * t144 * t26 - 0.2e1 * t453) + t26 * (-t396 + t397 + t406 - t407 + t408) - 0.2e1 * t465) + param.m4 * (-t283 + t246) + param.m3 * (t249 - t14) + param.m2 * (t578 - t17) + (param.m5 * (-t1 - t3 - t4 + t196 - t21)) + t323 + t321 - t369 + t294 * (t601 + t627) - param.I4zz;
  t640 = (2 * param.m5 * t197) + (2 * param.m6 * t194) + 0.2e1 * param.I5xx - 0.2e1 * param.I5yy - 0.2e1 * param.I6zz + 0.4e1 * t188 - 0.2e1 * t193 - t217 + t221 + t226 + t227 + t596 + t600;
  t653 = -t179 - t189 - t541 - t543 + t200 * t640 + param.m7 * (-t44 - (param.d3 + param.sp7z + param.d4 + param.d7) * (param.d3 - param.sp7z + param.d4 - param.d7)) + (param.m6 * (-t1 - t3 - t4 + t121 - t10)) - t1 * t547 - param.d3 * t551 + param.I2yy + param.I3zz + param.I4xx - param.I2xx - param.I3yy + param.I7xx + t626;
  t655 = t529 ^ 2;
  t657 = 0.2e1 * t171 * t37 * (t533 - t459 + t460 + t461 + t462 + t161 + t463 + t360) + param.I4zz + t268 - t540 + t541 + t543 + param.I1zz + param.I2xx + param.I3yy + param.m7 * (t45 + t44 + t5 + t7 + t8 + t544) + t548 + t552 + t554 + t183 + t655 * (t630 + t653) + t608;
  m(1,1) = t532 + t657;
  t658 = t143 * t60;
  t659 = t29 * t152;
  t662 = t93 * (t658 - t343 + t659 + t156 + t158 + t160 - t162 - t164 + param.I5yz) * t26;
  t663 = t130 - t134 + t136 + t138 + t139 - t101 - param.I5xz + param.I6xy;
  t664 = t55 * t663;
  t671 = t331 - t78 + t334 - t88 + param.I5xy;
  t674 = 0.2e1 * t93 * t671 * t55;
  t676 = t201 - t674 + t183 + t268 + t220 + t272 + t214 + t277 + t279 + t212 + t286;
  t680 = -t343 + t659 + t156 + t158 + t160 - t162 - t164 + param.I5yz;
  t682 = t55 * t680 + param.I4xy + t141 + t145 - t168;
  t691 = t608 + t674 + t600 + t605 + t321 + t607 + t214 + t584 + t586 + t212 + t203 + t588;
  t692 = -param.I7yy + t590 + param.I6yy + param.I7zz - param.I6xx + t592 - param.I3xx + param.I3yy - param.I4yy + param.I4zz + param.I5xx - param.I5zz;
  t693 = t691 + t692;
  t699 = -t498;
  t700 = t37 * t699;
  t705 = t66 * t132;
  t707 = (t705 - t68) * t37;
  t708 = t55 * t707;
  t723 = (t55 * t699 - t167) * t37;
  t735 = t134 + t184 + t185 - t138 - t139 + t101 + param.I5xz - param.I6xy;
  t736 = t55 * t735;
  m(1,2) = 0.2e1 * t294 * (t337 - t662 + t354 + t355 + t26 * (t664 + t361 - param.I4yz) + t365 - param.I3xy) * t527 + t293 * (-t288 * t300 * t527 * t676 + t171 * (0.2e1 * t300 * t527 * t682 * t26 + (t475 + t478 + t480 + t664 + t361 - param.I4yz) * t529) + t300 * t693 * t527 + t529 * (0.2e1 * t200 * t26 * t671 + t93 * (-t489 + t493 - t496 - t700) + t60 * t505 + t26 * t508 - t708 - param.d3 * t516 + t519 - t520 + param.I3yz)) + 0.2e1 * t288 * t529 * t300 * t682 + t171 * (t300 * t529 * (-t412 + t93 * (t417 + t419 - t707) - t423 + t429 + t431 + t723) - t336 * t527) + t300 * t529 * (t452 + t456 + t466 + t55 * (t343 + t345 + t347 - t158 - t160 + t162 + t164 - param.I5yz) - t470 - t472 + t168 + param.I3xz - param.I4xy) + t527 * (t662 - t354 - t355 + t26 * (t736 - t361 + param.I4yz) - param.m2 * param.sp2x * param.sp2z - t365 + param.I2xz + param.I3xy) + (-param.m2 * param.sp2y * param.sp2z + param.I2yz) * t529;
  t747 = t452 - t145 + t149 - t154 + t468 + t168 - param.I4xy;
  t751 = t26 * t293;
  t752 = t130 + t136 - t101 + param.I6xy;
  t753 = t66 * t752;
  t755 = 0.2e1 * t60 * t753;
  t756 = t183 - t755 + t177 + t179 - t193 + t195 + t198 + param.I5xx - param.I5yy + param.I6yy - param.I6zz - param.I7xx + param.I7zz;
  t757 = t200 * t756;
  t762 = -t35;
  t776 = t752 * t300;
  t778 = 0.2e1 * t55 * t776;
  t787 = t596 + t614 - t214 + t616 + t618 - t212 - t203 + t621 + param.I7yy - param.I7xx + t623 - param.I4xx + param.I4yy - param.I5yy + param.I5zz + param.I6xx - param.I6zz;
  t790 = -t437 * t55;
  t807 = t66 * t26;
  t821 = t762 * t55;
  t841 = t323 + t755 + t321 - t179 + t193 + t325 + t327 - param.I5xx + param.I5yy - param.I6yy + param.I6zz + param.I7xx - param.I7zz;
  t843 = t60 * t133;
  t844 = 0.2e1 * t843;
  t845 = param.sp7z ^ 2;
  t847 = 0.2e1 * param.d7 * param.sp7z;
  t848 = param.d7 ^ 2;
  t849 = t113 ^ 2;
  t850 = -t45 - t845 - t847 - t848 - t849;
  t852 = -t5 - t7 - t8 - t9 - t121;
  t854 = -t196 - t22;
  t856 = -t245 - t246;
  t858 = -t13 - t249;
  t860 = t288 * (t757 + t211 + t183 + t268 + t220 + t272 + t214 + t277 + t279 + t212 + t286) + 0.2e1 * t171 * t747 * t26 + t200 * t841 - t211 - t844 + t177 + t263 - t214 + param.m7 * t850 + (param.m6 * t852) - t212 - t203 + (param.m5 * t854) - param.I7xx + param.m4 * t856 - param.I5yy - param.I6zz + param.m3 * t858 - param.I3zz - param.I4xx;
  m(1,3) = t527 * (0.2e1 * t288 * t747 * t293 + t171 * (-t757 * t751 + t93 * (t60 * (t293 * (-0.2e1 * t206 * t26 * t55 + t37 * t762) - t143 * t300) + t293 * (-t419 + t707) - t300 * t680) + t105 * (-t182 * t26 * t293 - t778) + t60 * (-t300 * t449 * t55 - 0.2e1 * t293 * t427) + t293 * (t26 * t787 - t37 * (t433 + t790 - t167)) - (-t357 + t55 * (t134 + t185 - t138 - t139 + t101 + param.I5xz - param.I6xy) - t361 + param.I4yz) * t300) - t487 + t93 * (t105 * (t182 * t484 * t55 + t104 * t293) + t60 * (-0.2e1 * t55 * t776 * t807 + t128 * t293) + t293 * t663 - (t496 + t700) * t300) + t60 * (-t293 * t455 - t300 * (t26 * t53 + t37 * t821)) + t293 * (-t466 - t149 + t154 + t166 + t470 + t472 - t168 - param.I3xz + param.I4xy) - (t26 * (-t78 + t334 - t335 - t88 + param.I4xz + param.I5xy) + t708 + t517 - t519 + t520 - param.I3yz) * t300) - t529 * t860;
  t862 = t40 * t32;
  t864 = 0.2e1 * t29 * t862;
  t867 = -param.m7 * t118 * t117;
  t868 = t177 - t864 - t867 + t181 + param.I6xx - param.I6yy + param.I7yy - param.I7zz;
  t869 = t300 * t868;
  t871 = t32 * t176;
  t872 = t29 * t871;
  t873 = t205 - t872 - t52 - t39 + param.I6xz + param.I7xy;
  t874 = t293 * t873;
  t880 = t66 * t135;
  t881 = t29 * t880;
  t882 = t881 + t334 - t88 + param.I5xy;
  t888 = -param.m7 * t191 * t190;
  t889 = t177 - t864 + t888 + t195 + t198 + param.I5xx - param.I5yy + param.I6yy - param.I6zz - param.I7xx + param.I7zz;
  t893 = t171 * t55;
  t919 = 0.2e1 * t42 * t40 * t66;
  t920 = t66 * t871;
  t921 = t920 + t151;
  t923 = t29 * t921 + param.I5yz + t156 + t158 + t160 - t162 - t164 + t919;
  t929 = t752 * t26;
  t930 = t55 * t293;
  t936 = t42 * t176 * t66;
  t939 = -0.2e1 * t66 * t862 + t114;
  t940 = t29 * t939;
  t943 = t936 + t940 + t66 * (-t867 + t181 + param.I6xx - param.I6yy + param.I7yy - param.I7zz) - t127;
  t944 = t26 * t943;
  t949 = t66 * t95;
  t950 = t29 * t949;
  t964 = t881 + t334 - t335 - t88 + param.I4xz + param.I5xy;
  t969 = -t361 + param.I4yz;
  t973 = t45 + t44 + t849;
  t974 = param.m7 * t973;
  t975 = t5 + t7 + t8 + t9 + t10;
  t976 = param.m6 * t975;
  t977 = t21 + t22;
  t978 = param.m5 * t977;
  t979 = -t540 + t214 + t974 + t976 + t212 + t203 + t978 + param.I6yy + param.I7zz + t554 + param.I4zz + param.I5xx;
  t983 = t43 + t872 + t52 + t39 - param.I6xz - param.I7xy;
  t984 = t60 * t983;
  t985 = t984 + t881 + t334 - t88 + param.I5xy;
  t988 = 0.2e1 * t200 * t26 * t985;
  t989 = t105 * t868;
  t1001 = t752 * t55;
  t1007 = t26 * t873;
  t1010 = t171 * t735;
  t1012 = t969 * t171;
  m(1,4) = t527 * (t200 * (t105 * t869 + t60 * (-0.2e1 * t171 * t874 - 0.2e1 * t300 * t753) + 0.2e1 * t171 * t293 * t882 + t300 * t889) + t93 * (t105 * t868 * t293 * t893 + t60 * (t55 * (-0.2e1 * t171 * t293 * t66 * t752 + 0.2e1 * t300 * t873) + (-t293 * t339 + t300 * t415) * t26) + t55 * (t171 * t293 * t889 - 0.2e1 * t300 * t882) + (t293 * t923 + t300 * t70) * t26) + t105 * (0.2e1 * t929 * t930 - t869) + t60 * (t930 * t944 + t171 * (-t300 * t453 + t874) + 0.2e1 * t300 * (-t950 + t66 * (t136 - t101 + param.I6xy) - t133)) - t55 * t26 * (-t293 * t735 + t300 * t499) + t171 * (-t293 * t964 - t300 * t465) + t26 * (-t167 * t511 + t293 * t969) - t300 * t979) + (-t988 + t93 * (-t989 * t351 + t60 * (0.2e1 * t146 * t929 - t171 * t339) - t55 * t26 * t889 + t171 * t923) + 0.2e1 * t105 * t171 * t1001 + t60 * (t171 * t55 * t943 - t1007) + t55 * t1010 + t1012 + t964 * t26) * t529;
  t1016 = t293 * t171;
  t1017 = t93 * t752;
  t1020 = t868 * t26;
  t1025 = t113 * t35;
  t1026 = t66 * t868 - t1025;
  t1033 = t55 * t339;
  t1036 = 0.2e1 * t26 * t753;
  t1039 = t171 * t415;
  t1040 = t321 + t864 + t867 + t123 + param.I6yy - param.I7yy + param.I7zz - param.I6xx;
  t1041 = t66 * t1040;
  t1050 = t66 * t873;
  t1051 = t29 * t113;
  t1052 = t1051 * t31;
  t1058 = t66 * t983 + param.I5yz + t1052 + t158 + t160 - t162 - t164;
  t1059 = t55 * t1058;
  t1061 = t55 * t499;
  t1062 = t73 ^ 2;
  t1063 = t44 + t1062;
  t1064 = param.m7 * t1063;
  t1065 = t121 + t10;
  t1066 = param.m6 * t1065;
  t1067 = t196 + t21;
  t1068 = t1067 * param.m5;
  t1070 = (t177 - t864 + t1064 + t1066 + t1068 + param.I7yy + param.I5zz + param.I6xx) * t26;
  t1088 = t26 * t339;
  t1092 = t735 * t26;
  m(1,5) = t527 * (t105 * (0.2e1 * t1016 * t1017 - t1020 * t293 - t778) + t60 * (t93 * (t293 * (t1026 * t171 - t415) + t339 * t300) + t293 * (t1033 * t171 + t1036) + t55 * (t1039 + t1041 + t1025) * t300) + t93 * (t293 * (t1010 - t70) + (t171 * t499 - param.I5yz + t1050 - t1052 - t158 - t160 + t162 + t164) * t300) + t293 * (-t1059 * t171 + t1061 + t1070) + t55 * (t171 * t70 - param.I5xz + param.I6xy - t101 + t130 - t134 + t136 + t138 + t139) * t300) - t529 * (t105 * (t171 * t868 + 0.2e1 * t929 * t93) + t60 * (t1026 * t26 * t93 + t1088 * t55 - 0.2e1 * t171 * t753) + t93 * t1092 + t171 * (-(t1067 * param.m5) - (param.m6 * t1065) - param.m7 * t1063 - param.I5zz - param.I6xx - param.I7yy + t321 + t864) - t55 * t26 * t1058);
  t1112 = t113 * t171 + param.d3 + param.d4;
  t1122 = t176 * t93;
  t1125 = t26 * t135;
  t1128 = t300 * param.sp7x;
  t1140 = t171 * t37;
  t1141 = t1140 * t132 * t93;
  t1142 = t93 * t133;
  t1150 = t293 * param.m7;
  t1151 = t1150 * param.sp7x * t1112;
  t1152 = t135 * t300;
  t1160 = t45 + t1062;
  t1161 = param.m7 * t1160;
  t1162 = t9 + t121;
  t1163 = t1162 * param.m6;
  t1164 = t321 + t864 + t1161 + t1163 + param.I6zz + param.I7xx;
  t1169 = t42 * t40;
  t1172 = t135 * t93;
  t1178 = t1140 + param.d5 + param.d6;
  t1183 = t333 * t93;
  t1202 = t26 * t93;
  t1205 = t171 * t135;
  t1215 = t1025 * t66 + param.I6zz + param.I7xx + t1161 + t1163 + t321 + t864;
  m(1,6) = t527 * (t60 * (t55 * (t1112 * t132 * t293 + t29 * t300 * t871 + 0.2e1 * t300 * t40 * t42 - t300 * t346) - 0.2e1 * t42 * t40 * t93 * t1016 + t29 * (t293 * (-t1122 * t171 * t32 + t1125) + t37 * param.m7 * t26 * t1128) + t293 * (t171 * t346 * t93 + t26 * t333) - t300 * (t126 * t26 * t37 - t1141 - t1142)) + t55 * (t66 * (t29 * (-t1151 + t1152) + t293 * t126 * t1112 + t333 * t300) + t171 * t1164 * t293) + t66 * (-0.2e1 * t1169 * t751 + t29 * (t293 * (-t1172 * t171 - t26 * t871) - t1178 * t1128 * param.m7 * t93) + t293 * (-t1183 * t171 + t26 * t346) + t300 * (t1140 * t126 * t93 + t127 * t93 + t454)) + t300 * t1164 * t93) + t529 * (t60 * (-t55 * t132 * t26 * t113 + 0.2e1 * t1169 * t1202 + t29 * (t1122 * t26 * t32 + t1205) + t171 * t333 - t346 * t1202) - t55 * t26 * t1215 + t66 * (-0.2e1 * t42 * t40 * t171 + t29 * (t1172 * t26 - t171 * t871) + t171 * t346 + t26 * t1183));
  t1233 = t300 * param.sp7y;
  t1236 = -param.m7 * t1178 * t1233 - t171 * t293 * t95;
  t1241 = t1150 * t1112 * param.sp7y;
  t1242 = t95 * t300;
  t1249 = -t1178 * t27 * t300 + t1205 * t293;
  t1252 = t37 * param.m7;
  t1274 = (t44 + t45) * param.m7 + param.I7zz;
  t1275 = t66 * t93;
  t1277 = t26 * t60;
  t1286 = t60 * t95 - t114;
  t1289 = t26 * param.sp7y;
  t1291 = t60 * t461;
  t1297 = t60 * t135;
  t1301 = t26 * param.sp7x;
  m(1,7) = t527 * (t29 * (t93 * (t1236 * t60 + t1151 + t1152) - t60 * t55 * (t1241 - t1242) + t55 * t1249 - (t1233 * t1252 + t293 * t95) * t807) + t32 * (t93 * (t1249 * t60 - t1241 + t1242) - t60 * t55 * (t1151 + t1152) - t55 * t1236 - (t1128 * t1252 + t293 * t76) * t807) - (-t300 * t55 * t66 + t1016 * t1275 - t1277 * t293) * t1274) - (t29 * (-t1286 * t26 * t93 - t1289 * t1291 * t55 + t1125 * t55 + t171 * t949) + t32 * (t93 * (t1297 - t151) * t26 - t1291 * t55 * t1301 + t55 * t95 * t26 - t171 * t880) - t1274 * (t1202 * t66 + t171 * t60)) * t529;
  m(2,1) = m(1,2);
  t1319 = t37 * t55;
  t1337 = param.I6xx + (t578 + t17) * param.m2 + (param.m5 * (t1 + t3 + t4 + t196 + t21)) + t323 + param.I2zz + t29 * (-0.2e1 * t1319 * t26 * t31 + 0.2e1 * t110) + (param.m6 * (t1 + t3 + t4 + t121 + t10)) - t189 - 0.2e1 * t1202 * t37 * (t35 * t60 + t67 + t68) - t288 * (t293 - 0.1e1) * t676 * (0.1e1 + t293) + param.I3xx + t541;
  t1338 = t29 * t26;
  t1357 = t66 * t155 * t55;
  t1358 = t158 + t160 - t162 - t164 + param.I5yz;
  t1377 = t37 * t66;
  t1383 = -t1358;
  t1398 = -0.2e1 * t293 * t300 * (-t662 + t354 + t355 + t1338 * t356 + t26 * (-t705 * t113 * t55 + t55 * (t136 - t101 + t138 + t139 - param.I5xz + param.I6xy) + t361 - param.I4yz) + t365 - param.I3xy) - 0.2e1 * t26 * t37 * (t790 - t167) + param.I4yy + param.I5zz + param.m7 * (t44 + t845 + t847 + t848 + t544) + t171 * (0.2e1 * t294 * (t1358 * t55 + param.I4xy + t1357 + t141 + t145 - t149 + t154 - t168) * t26 - 0.2e1 * t293 * t336 * t300 - t564 + t60 * (-0.2e1 * t1338 * t55 * t76 - 0.2e1 * t26 * t55 * t85 + 0.2e1 * t453) + 0.4e1 * t342 * t351 + t29 * (-0.2e1 * t1377 * t27 - 0.2e1 * t153 * t26) + t26 * (0.2e1 * t1383 * t55 - 0.2e1 * t1357 + t407 - t408) + 0.2e1 * t37 * (t460 + t461 + t462 + t161 + t360 + t463)) + param.m4 * (t283 + t246) + param.m3 * (t249 + t14) + t294 * t693 + param.I7yy + t548 + t552 + t177;
  m(2,2) = t1337 + t1398;
  t1400 = t762 * t113;
  t1418 = t143 * t66 - param.I5xy + t207 + t88;
  t1432 = t183 + t60 * (t104 * t66 + t239) - 0.2e1 * t66 * t1400 + t220 - t221 + t277 + t279 + t212 + t203 + t282 - param.I7yy + param.I7xx + t285 + param.I4xx - param.I4yy + param.I5yy - param.I5zz - param.I6xx + param.I6zz;
  t1437 = t182 * t66 + t1400;
  t1445 = t66 * t206;
  t1448 = t288 * (t93 * (t375 + t60 * (t228 * t66 - 0.2e1 * t1400) - t387 + t386 + t389 + t390 + t391 - t102 - t392 + t103) + t395 + t55 * (t66 * (-t42 * t301 - t303 + t304 + t305 - t306 - t307) + 0.2e1 * t1052 + t400 + t401 - t402 - t403 + t404) - t407 + t408) + t171 * (t200 * t756 * t26 + t93 * (0.2e1 * t26 * t55 * t1418 - (t60 * t762 - t68 + t705) * t37) + t26 * t1432 + t723) + t93 * (t1437 * t60 + param.I5xz - param.I6xy + t101 + t134 - t138 - t139 + t184 + t185 + t444) + t26 * t37 * (t35 * t66 + t161 + t360 + t461 + t462 + t463 + t533) - t145 + t55 * (t1445 - t1052 - t158 - t160 + t162 + t164 - param.I5yz) - t470 - t472 + t168 + param.I3xz - param.I4xy;
  t1454 = 0.2e1 * t105 * t1001;
  t1466 = t66 * t339;
  m(2,3) = t300 * t1448 - (t171 * (t93 * (t53 * t66 + param.I5yz + t1052 + t158 + t160 - t162 - t164 + t658) + t1454 + t60 * t1437 * t55 + t736 - t361 + param.I4yz) + 0.2e1 * t200 * t26 * t1418 + t93 * (-t26 * t55 * t756 + t700) + t26 * (t331 + t1466 - t335 - t88 + param.I4xz + param.I5xy) + t60 * t37 * t821 + t708 + t517 - t519 + t520 - param.I3yz) * t293;
  t1473 = t989 - t755 + t177 - t864 + t888 + t195 + t198 + param.I5xx - param.I5yy + param.I6yy - param.I6zz - param.I7xx + param.I7zz;
  t1487 = 0.2e1 * t950;
  t1503 = t200 * t1473 + t93 * (t60 * (t55 * (t302 - 0.2e1 * t872 - t304 - t305 + t306 + t307) + t38) + t55 * (-0.2e1 * t881 + t316 + t317 - t318) + t71) + t105 * t1040 + t60 * (-t171 * t453 - t1487 - t239 + t610) - t1319 * t498 * t26 - t171 * t465 - t26 * t37 * t167 + t540 - t214 - param.m7 * t973 - (param.m6 * t975) - t212 - t203 - (param.m5 * t977) - t553 * param.m4 - param.I4zz - param.I5xx - param.I6yy - param.I7zz;
  m(2,4) = t293 * t1503 + (-0.2e1 * t200 * t171 * t985 + t93 * (-t989 * t893 + t60 * (0.2e1 * t146 * t171 * t752 + t1088) - t55 * t889 * t171 - t26 * t923) - 0.2e1 * t105 * t26 * t1001 + t60 * (-t171 * t873 - t55 * t944) - t55 * t1092 + t171 * t964 - t969 * t26) * t300;
  t1542 = t93 * t663;
  m(2,5) = t300 * (t105 * (-0.2e1 * t1017 * t171 + t1020) + t60 * (t171 * (t93 * (t1041 + t1025) - t1033) + t93 * t415 - t1036) + t171 * (t1542 + t1059) + t93 * t70 - t1061 - t1070) + t293 * (-t1454 + t60 * (t1140 * t35 * t55 - t1026 * t55 + t339 * t93) + t171 * t37 * (t498 * t93 + t512) + t93 * (t1050 - t1052 - t158 - t160 + t162 + t164 - param.I5yz) - t736);
  m(2,6) = t300 * (t60 * (t171 * (-t133 * t55 + t93 * t983) - t55 * t453 - t1088) + t171 * (-t1215 * t55 + t1466 * t93) - t66 * (t504 + t1007)) + (t60 * (t55 * t983 + t1141 + t1142 - t38) + t1039 * t1275 + t93 * t1215 + t66 * (t1033 + t454)) * t293;
  t1587 = t60 * t113;
  t1589 = -t1587 * t31 + param.I7yz - t75;
  t1593 = t93 * t37;
  t1595 = param.sp7y * t55;
  t1596 = t60 * t1252;
  t1605 = -t1587 * t27 + param.I7xz - t80;
  t1610 = param.sp7x * t55;
  t1621 = t60 * t93;
  t1652 = t1274 * t66;
  m(2,7) = t300 * (t29 * (t171 * (t1286 * t93 - t1589 * t55) - t1593 * t27 + t1596 * t1595 + t26 * t949) + t32 * (t171 * (t93 * (t60 * t76 + t151) - t1605 * t55) + t1593 * t31 + t1596 * t1610 - t26 * t880) + (t171 * t66 * t93 - t1277) * t1274) - (t29 * (t1140 * (param.sp7y * t1621 + t1610) * param.m7 - t93 * t1589 - t60 * t55 * t95 + param.m7 * (param.sp7x * t113 * t55 + t1289 * t1377)) + t32 * (t1140 * (param.sp7x * t1621 - t1595) * param.m7 - t93 * t1605 + t60 * t55 * t135 + param.m7 * (-param.sp7y * t113 * t55 + t1301 * t1377)) - t55 * t1652) * t293;
  m(3,1) = m(1,3);
  m(3,2) = m(2,3);
  t1657 = t105 * (t321 + t864 - t120 + t123 + param.I6yy - param.I7yy + param.I7zz - param.I6xx);
  t1658 = t1657 + t755 + t321 + t864 + t193 + t325 + t327 - param.I5xx + param.I5yy - param.I6yy + param.I6zz + param.I7xx - param.I7zz;
  t1659 = t200 * t1658;
  t1663 = 0.2e1 * t93 * (t984 - t78 + t334 - t88 + param.I5xy) * t55;
  t1672 = t444 + t60 * (t936 + t940 + t448 - t127) + t184 + t134 + t185 - t138 - t139 + t101 + param.I5xz - param.I6xy;
  t1686 = t177 - t864 + t120 + t181 + param.I6xx - param.I6yy + param.I7yy - param.I7zz;
  t1687 = t105 * t1686;
  t1688 = t1687 - t755 + t177 - t864 - t193 + t195 + t198 + param.I5xx - param.I5yy + param.I6yy - param.I6zz - param.I7xx + param.I7zz;
  m(3,3) = t288 * (t1659 + t1663 + t1657 + t612 + t596 + t29 * (t269 - 0.4e1 * t862) - t214 + t616 + t618 - t212 + t624) - 0.2e1 * t171 * t26 * (-t29 * t55 * t921 + t346 * t55 * t66 + t1033 * t60 - 0.2e1 * t1169 * t146 + t1383 * t55 + t1672 * t93 - param.I4xy + t168) + t200 * t1688 - t1663 + t844 + t321 + t29 * (-0.2e1 * t260 + 0.2e1 * t862) + t214 - param.m7 * t850 - (param.m6 * t852) + t212 + t203 - (param.m5 * t854) + param.I7xx - param.m4 * t856 + param.I5yy + param.I6zz + param.I4xx - param.m3 * t858 + param.I3zz;
  t1706 = -t29 * t921;
  m(3,4) = -t988 + t93 * (-t55 * t26 * t1688 - (t340 - t919 + t1706 + t347 - t158 - t160 + t162 + t164 - param.I5yz) * t171) + t55 * t1672 * t171 + t26 * (t984 + t881 + t334 - t335 - t88 + param.I4xz + param.I5xy) + t1012;
  t1727 = t1657 + t755 + t177 - t864 + t1064 + t1066 + param.I7yy + param.I6xx + t1068 + param.I5zz;
  m(3,5) = t26 * (-0.2e1 * t105 * t1017 + t60 * (t93 * (-t29 * t939 + t125 + t127 - t936) - t1033) + t1542 - (-t919 + t1706 + t347 - t158 - t160 + t162 + t164 - param.I5yz) * t55) + t1727 * t171;
  t1735 = t260 - 0.2e1 * t862;
  t1738 = t60 * t871 + t880;
  t1743 = -param.m7 * t1160;
  t1745 = -t1162 * param.m6;
  m(3,6) = t26 * (t42 * (0.2e1 * t1621 * t40 + t176 * t55) + t29 * (t1735 * t55 + t1738 * t93) + t55 * (-t843 - t213 + t1743 + t1745 - param.I6zz - param.I7xx) - t93 * (t346 * t60 - t334)) + (-t919 + t29 * (t1297 - t920) + t60 * t333 + t347) * t171;
  t1760 = t1051 * t27;
  t1761 = t157 * t31;
  t1769 = t1587 * (param.sp7x * t32 + param.sp7y * t29) * param.m7 + t142 + t82;
  t1773 = t1274 * t60;
  m(3,7) = t26 * (t93 * (t60 * (t184 - t136) - t1760 + t1761 + t1652) + t1769 * t55) + t171 * (t1773 + t66 * (t130 + t136));
  m(4,1) = m(1,4);
  m(4,2) = m(2,4);
  m(4,3) = m(3,4);
  m(4,4) = t1659 + 0.2e1 * t93 * t985 * t55 + t1687 + t60 * (t1487 + t266 + t239) - t540 + t214 + t974 + t976 + t212 + t203 + t978 + param.I6yy + param.I7zz + param.I5xx + t554 + param.I4zz;
  m(4,5) = t55 * (t444 + t60 * (t1686 * t66 - t1025) + t134 + t184 + t185 - t138 - t139 + t101 + param.I5xz - param.I6xy) - t93 * (t340 + t1050 - t1052 - t158 - t160 + t162 + t164 - param.I5yz);
  m(4,6) = t93 * (t1735 * t29 - param.I6zz - param.I7xx + t1743 + t1745 + t177 - t213 - t843) - (0.2e1 * t40 * t42 * t60 + t155 * t60 + t1738 * t29 + t334) * t55;
  t1800 = t130 - t185;
  m(4,7) = t55 * (t1800 * t60 - t1652 + t1760 - t1761) + t93 * t1769;
  m(5,1) = m(1,5);
  m(5,2) = m(2,5);
  m(5,3) = m(3,5);
  m(5,4) = m(4,5);
  m(5,5) = t1727;
  m(5,6) = t1445 - t658;
  m(5,7) = t1800 * t66 + t1773;
  m(6,1) = m(1,6);
  m(6,2) = m(2,6);
  m(6,3) = m(3,6);
  m(6,4) = m(4,6);
  m(6,5) = m(5,6);
  m(6,6) = t321 - t179 + t1161 + param.I7xx + t1163 + param.I6zz;
  m(6,7) = t338 - t82;
  m(7,1) = m(1,7);
  m(7,2) = m(2,7);
  m(7,3) = m(3,7);
  m(7,4) = m(4,7);
  m(7,5) = m(5,7);
  m(7,6) = m(6,7);
  m(7,7) = t1274;
  tmpreturn = m;