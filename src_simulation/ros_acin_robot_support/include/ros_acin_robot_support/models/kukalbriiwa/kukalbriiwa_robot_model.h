#ifndef KUKALBRIIWA_ROBOT_MODEL_H
#define KUKALBRIIWA_ROBOT_MODEL_H

#include <cmath>
#include "robot_params.h"

/**
 * This class contains the Kuka LBR iiwa R820 robot model.
 * This model is generic in the sense that Scalar and Matrix
 * types can be provided as template parameters.
 * The only requirement is that, the Matrix Type provides the
 * operator() function for element access.
 *
 * Example with Eigen:
 *  typedef Eigen::Matrix<double, 7, 7> JointMatrix;
 *  [...]
 *
 *  typedef KukaLbrIiwaRobotModel<double, JointMatrix, JacobianMatrix,
 *                  JointVector, HomogeneousTransformation> RobotModel;
 *  RobotParams param;
 *  JointVector q = JointVector::Zero();
 *  JacobianMatrix J = JacobianMatrix::Zero();
 *  RobotModel::evalJ(q, J, param);
 */

template<typename Scalar, typename JointMatrix, typename JacobianMatrix, typename JointVector, typename HomogeneousTransformation> class KukaLbrIiwaRobotModel
{
public:

  /**
   * Evaluates the geometric Jacobian matrix
   */
  static void evalJ(const JointVector& q, JacobianMatrix& J, const RobotParams& param)
  {
    _evalJ_(q, J, param);
  }

  /**
   * Evaluates the geometric Jacobian matrix squared, i.e. JJ^T
   */
  static void evalJSquare(const JointVector& q, JointVector& J, const RobotParams& param)
  {
    Scalar q1 = q(0);
    Scalar q2 = q(1);
    Scalar q3 = q(2);
    Scalar q4 = q(3);
    Scalar q5 = q(4);
    Scalar q6 = q(5);
    Scalar q7 = q(6);

    Scalar t1;
    Scalar t10;
    Scalar t100;
    Scalar t102;
    Scalar t103;
    Scalar t105;
    Scalar t109;
    Scalar t11;
    Scalar t114;
    Scalar t119;
    Scalar t123;
    Scalar t124;
    Scalar t128;
    Scalar t129;
    Scalar t13;
    Scalar t130;
    Scalar t143;
    Scalar t148;
    Scalar t15;
    Scalar t150;
    Scalar t153;
    Scalar t156;
    Scalar t16;
    Scalar t162;
    Scalar t165;
    Scalar t167;
    Scalar t170;
    Scalar t177;
    Scalar t181;
    Scalar t185;
    Scalar t186;
    Scalar t189;
    Scalar t19;
    Scalar t196;
    Scalar t197;
    Scalar t198;
    Scalar t2;
    Scalar t200;
    Scalar t206;
    Scalar t208;
    Scalar t209;
    Scalar t212;
    Scalar t213;
    Scalar t215;
    Scalar t219;
    Scalar t22;
    Scalar t221;
    Scalar t223;
    Scalar t23;
    Scalar t231;
    Scalar t234;
    Scalar t235;
    Scalar t239;
    Scalar t24;
    Scalar t246;
    Scalar t247;
    Scalar t249;
    Scalar t25;
    Scalar t250;
    Scalar t251;
    Scalar t254;
    Scalar t269;
    Scalar t27;
    Scalar t271;
    Scalar t28;
    Scalar t283;
    Scalar t284;
    Scalar t285;
    Scalar t287;
    Scalar t288;
    Scalar t291;
    Scalar t294;
    Scalar t297;
    Scalar t299;
    Scalar t3;
    Scalar t30;
    Scalar t313;
    Scalar t314;
    Scalar t315;
    Scalar t319;
    Scalar t32;
    Scalar t322;
    Scalar t327;
    Scalar t329;
    Scalar t33;
    Scalar t333;
    Scalar t335;
    Scalar t337;
    Scalar t338;
    Scalar t34;
    Scalar t347;
    Scalar t359;
    Scalar t362;
    Scalar t363;
    Scalar t364;
    Scalar t365;
    Scalar t37;
    Scalar t373;
    Scalar t374;
    Scalar t376;
    Scalar t377;
    Scalar t39;
    Scalar t4;
    Scalar t40;
    Scalar t42;
    Scalar t43;
    Scalar t44;
    Scalar t45;
    Scalar t46;
    Scalar t47;
    Scalar t49;
    Scalar t5;
    Scalar t50;
    Scalar t54;
    Scalar t55;
    Scalar t57;
    Scalar t6;
    Scalar t60;
    Scalar t61;
    Scalar t63;
    Scalar t66;
    Scalar t67;
    Scalar t70;
    Scalar t72;
    Scalar t73;
    Scalar t74;
    Scalar t76;
    Scalar t8;
    Scalar t82;
    Scalar t84;
    Scalar t87;
    Scalar t88;
    Scalar t89;
    Scalar t92;
    Scalar t96;
    t1 = cos(q5);
    t2 = sin(q2);
    t3 = t2 * t1;
    t4 = param.d7 + param.d8;
    t5 = sin(q6);
    t6 = t5 * t4;
    t8 = cos(q6);
    t10 = t4 * t8 + param.d5 + param.d6;
    t11 = cos(q2);
    t13 = cos(q3);
    t15 = t10 * t11 * t13 - t3 * t6;
    t16 = sin(q4);
    t19 = cos(q4);
    t22 = sin(q5);
    t23 = sin(q3);
    t24 = t23 * t22;
    t25 = t1 * t13 * t19 - t24;
    t27 = t5 * t25 * t4 * t11;
    t28 = t19 * t10;
    t30 = t2 * (t28 + param.d3 + param.d4);
    t32 = sin(q1);
    t33 = t32 * (t15 * t16 - t27 - t30);
    t34 = cos(q1);
    t37 = t23 * t1;
    t39 = t13 * t22;
    t40 = t19 * t37 + t39;
    t42 = -t10 * t16 * t23 + t40 * t6;
    t43 = t42 * t34;
    t44 = t33 - t43;
    t45 = t44 * t44;
    t46 = t34 * t34;
    t47 = t5 * t1;
    t49 = t16 * t4 * t47;
    t50 = t28 + t49 + param.d3 + param.d4;
    t54 = t19 * t4 * t47;
    t55 = -t10 * t16 + t54;
    t57 = t6 * t24;
    t60 = t11 * t50 - t2 * (t13 * t55 - t57);
    t61 = t60 * t60;
    t63 = -t55;
    t66 = t23 * t63 - t39 * t6;
    t67 = t11 * t66;
    t70 = t13 * t63 + t57;
    t72 = t32 * t70 + t34 * t67;
    t73 = t72 * t72;
    t74 = -t15;
    t76 = t13 * t1;
    t82 = t19 * t74 - t16 * (t11 * t6 * t76 + t10 * t2);
    t84 = t49 + t28;
    t87 = t23 * t32 * t84 + t34 * t82;
    t88 = t87 * t87;
    t89 = t4 * t4;
    t92 = t16 * t2;
    t96 = t22 * (t11 * t13 * t19 + t92) + t11 * t37;
    t100 = -t19 * t22 * t23 + t76;
    t102 = t100 * t32 + t34 * t96;
    t103 = t102 * t102;
    t105 = t5 * t5;
    t109 = t11 * t25 + t16 * t3;
    t114 = t11 * t13 * t16 - t19 * t2;
    t119 = t16 * t23;
    t123 = t34 * (t109 * t8 + t114 * t5) - t32 * (t119 * t5 + t40 * t8);
    t124 = t123 * t123;
    J(0, 0) = t103 * t105 * t89 + t124 * t89 + t46 * t61 + t45 + t73 + t88;
    t128 = t34 * (t16 * t74 + t27 + t30);
    t129 = t32 * t42;
    t130 = t128 - t129;
    t143 = t34 * (t13 * (t16 * (-t4 * t8 - param.d5 - param.d6) + t54) - t57) + t32 * t67;
    t148 = -t23 * t34 * t84 + t32 * t82;
    t150 = t102 * t89;
    t153 = -t100 * t34 + t32 * t96;
    t156 = t123 * t89;
    t162 = t34 * t119;
    t165 = t8 * (t109 * t32 + t34 * t40) + t5 * (t114 * t32 + t162);
    J(0, 1) = t105 * t150 * t153 + t32 * t34 * t61 + t130 * t44 + t143 * t72 + t148 * t87 + t156 * t165;
    t167 = t60 * t34;
    t170 = t11 * t70 - t2 * t50;
    t177 = t13 * t2 * t84 + t11 * t55;
    t181 = t13 * t19 * t22 + t37;
    t185 = -t11 * t16 * t22 + t181 * t2;
    t186 = t185 * t105;
    t189 = t16 * t1;
    t196 = t13 * t16 * t2 + t11 * t19;
    t197 = t5 * t196;
    t198 = t8 * (-t11 * t189 + t2 * t25) + t197;
    J(0, 2) = -t2 * t66 * t72 - t150 * t186 - t156 * t198 + t167 * t170 + t177 * t87;
    t200 = t32 * t167;
    t206 = t11 * t23 * t34 + t13 * t32;
    t208 = t102 * t4;
    t209 = -t114;
    t212 = t119 * t32 + t209 * t34;
    t213 = t212 * t5;
    t215 = t123 * t4;
    t219 = -t11 * t181 - t22 * t92;
    t221 = -t100;
    t223 = t219 * t34 + t221 * t32;
    J(0, 3) = t2 * t34 * t72 + t206 * t87 - t208 * t213 + t215 * t223 - t200;
    t231 = t11 * t23 * t32 - t13 * t34;
    t234 = t209 * t32 - t162;
    t235 = t234 * t5;
    t239 = t219 * t32 - t221 * t34;
    J(0, 4) = t2 * t32 * t72 - t208 * t235 + t215 * t239 + t231 * t87 + t46 * t60;
    J(0, 5) = -t2 * t23 * t87 + t11 * t72 + t185 * t215 - t197 * t208 + t33 - t43;
    J(1, 0) = J(0, 1);
    t246 = t130 * t130;
    t247 = t32 * t32;
    t249 = t143 * t143;
    t250 = t148 * t148;
    t251 = t153 * t153;
    t254 = t165 * t165;
    J(1, 1) = t105 * t251 * t89 + t247 * t61 + t254 * t89 + t246 + t249 + t250;
    J(1, 2) = -t143 * t2 * t66 - t153 * t186 * t89 - t165 * t198 * t89 + t170 * t32 * t60 + t148 * t177;
    t269 = t153 * t4;
    t271 = t165 * t4;
    J(1, 3) = t143 * t2 * t34 + t148 * t206 - t213 * t269 + t223 * t271 - t247 * t60;
    J(1, 4) = t143 * t2 * t32 + t148 * t231 - t235 * t269 + t239 * t271 + t200;
    J(1, 5) = -t148 * t2 * t23 + t11 * t143 + t185 * t271 - t197 * t269 + t128 - t129;
    J(2, 0) = J(0, 2);
    J(2, 1) = J(1, 2);
    t283 = t170 * t170;
    t284 = t2 * t2;
    t285 = t66 * t66;
    t287 = t177 * t177;
    t288 = t185 * t185;
    t291 = t198 * t198;
    J(2, 2) = t105 * t288 * t89 + t284 * t285 + t291 * t89 + t283 + t287;
    t294 = t66 * t284;
    t297 = t4 * t185;
    t299 = t198 * t4;
    J(2, 3) = -t170 * t32 + t177 * t206 + t213 * t297 - t223 * t299 - t294 * t34;
    J(2, 4) = t170 * t34 + t177 * t231 + t235 * t297 - t239 * t299 - t294 * t32;
    J(2, 5) = -t11 * t2 * t66 - t177 * t2 * t23 - t185 * t299 + t197 * t297;
    J(3, 0) = J(0, 3);
    J(3, 1) = J(1, 3);
    J(3, 2) = J(2, 3);
    t313 = t206 * t206;
    t314 = t212 * t212;
    t315 = t223 * t223;
    t319 = t1 * t19 * t5 - t16 * t8;
    t322 = t22 * t23 * t5;
    t327 = t189 * t5 + t19 * t8;
    t329 = t11 * (t13 * t319 - t322) + t327 * t2;
    t333 = -t319;
    t335 = -t13 * t22 * t5 + t23 * t333;
    t337 = t32 * t335 + t329 * t34;
    t338 = t337 * t337;
    J(3, 3) = t284 * t46 + t247 + t313 + t314 + t315 + t338;
    t347 = t32 * t329 - t335 * t34;
    J(3, 4) = t284 * t32 * t34 + t206 * t231 + t212 * t234 + t223 * t239 - t32 * t34 + t337 * t347;
    t359 = t2 * (t13 * t333 + t322) + t327 * t11;
    J(3, 5) = t11 * t2 * t34 - t2 * t206 * t23 + t185 * t223 + t196 * t212 + t337 * t359;
    J(4, 0) = J(0, 4);
    J(4, 1) = J(1, 4);
    J(4, 2) = J(2, 4);
    J(4, 3) = J(3, 4);
    t362 = t231 * t231;
    t363 = t234 * t234;
    t364 = t239 * t239;
    t365 = t347 * t347;
    J(4, 4) = t247 * t284 + t362 + t363 + t364 + t365 + t46;
    J(4, 5) = t11 * t2 * t32 - t2 * t23 * t231 + t185 * t239 + t196 * t234 + t347 * t359;
    J(5, 0) = J(0, 5);
    J(5, 1) = J(1, 5);
    J(5, 2) = J(2, 5);
    J(5, 3) = J(3, 5);
    J(5, 4) = J(4, 5);
    t373 = t11 * t11;
    t374 = t23 * t23;
    t376 = t196 * t196;
    t377 = t359 * t359;
    J(5, 5) = t284 * t374 + t288 + t373 + t376 + t377 + 0.1e1;
  }

  /**
   * Evaluates the derivative of the geometric Jacobian matrix
   */
  static void evalJp(const JointVector& q, const JointVector& qd, JacobianMatrix& Jp, const RobotParams& param)
  {
    JointVector q_=q;
    JointVector qd_=qd;
    _evalJp_(q_, qd_, Jp, param);
  }

  /**
   * Evaluates the Mass Matrix
   */
  static void evalM(const JointVector& q, JointMatrix& M, const RobotParams& param)
  {
    JointVector q_=q;
    _evalM_(q_,M,param);
  }

  /**
   * Evaluates the Coriolis Matrix
   */
  static void evalC(const JointVector& q, const JointVector& qd, JointMatrix& C, const RobotParams& param)
  {
    JointVector q_=q;
    JointVector qd_=qd;
    _evalC_(q_,qd_,C,param);
  }
  /**
   * Evaluates the Gravity Vector
   */
  static void evalG(const JointVector& q, JointVector& G, const RobotParams& param)
  {
    JointVector q_=q;
    _evalG_(q_,G,param);
  }
  /**
   * Evaluates the forward Kinematics / T0E
   */
  static void forwardKinematics(const JointVector& q, HomogeneousTransformation& T0E, const RobotParams& param)
  {
    JointVector q_=q;
    _evalT0E_(q_,T0E,param);
  }
};


#endif // KUKALBRIIWA_ROBOT_MODEL_H
