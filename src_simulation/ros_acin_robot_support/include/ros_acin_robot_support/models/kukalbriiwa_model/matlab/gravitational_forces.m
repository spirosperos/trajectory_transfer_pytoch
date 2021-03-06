function tmpreturn = gravitational_forces(q, param)
  m = [0 0 0 0 0 0 0;];
  m(1,1) = 0;
  t1 = param.sp7x * param.m7;
  t2 = q(7);
  t3 = cos(t2);
  t4 = t3 * t1;
  t5 = param.m7 * param.sp7y;
  t6 = sin(t2);
  t7 = t6 * t5;
  t9 = param.m6 * param.sp6x - t4 + t7;
  t10 = q(6);
  t11 = cos(t10);
  t12 = t11 * t9;
  t13 = -param.sp7z - param.d7;
  t15 = param.m6 * param.sp6y;
  t16 = param.m7 * t13 - t15;
  t17 = sin(t10);
  t18 = t17 * t16;
  t19 = param.m5 * param.sp5x;
  t20 = t12 + t18 + t19;
  t21 = q(5);
  t22 = cos(t21);
  t24 = t6 * t1;
  t25 = t3 * t5;
  t28 = -param.m5 * param.sp5y + param.m6 * param.sp6z + t24 + t25;
  t29 = sin(t21);
  t31 = param.m4 * param.sp4x;
  t32 = t20 * t22 + t28 * t29 - t31;
  t33 = q(4);
  t34 = cos(t33);
  t38 = -param.m7 * t13 + t15;
  t39 = t11 * t38;
  t40 = t17 * t9;
  t41 = param.d5 + param.d6;
  t47 = t39 + t40 + param.m7 * t41 + param.m6 * t41 + (param.d5 + param.sp5z) * param.m5 + param.m4 * param.sp4y;
  t48 = sin(t33);
  t49 = t48 * t47;
  t51 = -param.m3 * param.sp3x + t32 * t34 + t49;
  t52 = q(3);
  t53 = cos(t52);
  t55 = sin(t52);
  t56 = -t28;
  t57 = t56 * t55;
  t59 = t29 * t55;
  t61 = t18 + t19;
  t62 = t61 * t55;
  t66 = param.m3 * param.sp3y - param.m4 * param.sp4z;
  t70 = q(2);
  t71 = cos(t70);
  t73 = t34 * t47;
  t74 = -t9;
  t76 = t17 * t38;
  t77 = t11 * t74 - t19 + t76;
  t80 = t22 * t77 + t29 * t56 + t31;
  t82 = param.d3 + param.d4;
  t91 = sin(t70);
  m(1,2) = (param.g * (t71 * (param.m2 * param.sp2x - t12 * t59 - t22 * t57 - t29 * t62 + t51 * t53 + t55 * t66) - t91 * (t73 + t48 * t80 + param.m7 * t82 + param.m6 * t82 + param.m5 * t82 + param.m4 * t82 + (param.d3 + param.sp3z) * param.m3 + param.m2 * param.sp2y)));
  m(1,3) = (param.g * t91 * (-t12 * t29 * t53 - t22 * t53 * t56 - t29 * t53 * t61 - t51 * t55 + t53 * t66));
  m(1,4) = (param.g * (t91 * t53 * (-t32 * t48 + t73) + t71 * (t34 * t80 - t49)));
  m(1,5) = (param.g * (t91 * (t53 * t34 * (-t20 * t29 + t22 * t28) + t29 * t57 - t12 * t22 * t55 - t22 * t62) + t71 * t48 * (t22 * t56 - t29 * t77)));
  t133 = -t76 + t12;
  m(1,6) = (param.g * (t91 * (t53 * (t34 * t22 * (t11 * t16 - t40) + t48 * t133) + t40 * t59 - t29 * t11 * t16 * t55) + t71 * (t34 * t133 + t48 * t22 * (-t17 * t74 + t39))));
  t151 = t24 + t25;
  t152 = t11 * t151;
  t154 = t4 - t7;
  t158 = t17 * t151;
  t162 = -t154;
  m(1,7) = (param.g * (t91 * (t53 * (t34 * (t152 * t22 + t154 * t29) + t48 * t158) - t22 * t162 * t55 - t152 * t59) + t71 * (t34 * t158 + t48 * (-t11 * t151 * t22 + t162 * t29))));
  tmpreturn = m';
