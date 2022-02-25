#ifndef ROBOTMODELCLASS_H
#define ROBOTMODELCLASS_H
template<typename Scalar, typename JointMatrix, typename JacobianMatrix, typename JointVector, typename HomogeneousTransformation> class RobotModel_C
{
public:

  /**
   * Evaluates the geometric Jacobian matrix
   */
  static void evalJ(const JointVector_t& q, JacobianMatrix_t& J, const RobotParams& param)
  {

    if (!J.IsRowMajor){
       JacobianMatrixRowMajor_t J_rowMajor;
       geo_jacobian_endeffector<Scalar>((Scalar*)q.data(), param, (Scalar (*) [_ROBOT_q_DIM_]) J_rowMajor.data());
       J = J_rowMajor;
    }else{
       geo_jacobian_endeffector<Scalar>((Scalar*)q.data(), param, (Scalar (*) [_ROBOT_q_DIM_]) J.data());
    }
  }

  /**
   * Evaluates the geometric Jacobian matrix squared, i.e. JJ^T
   */
  static void evalJSquare(const JointVector_t& q, JacobianMatrix_t& J_square, const RobotParams& param)
  {
    JacobianMatrix J;
    evalJ(q,J_square,param);
    J_square = J*J.transpose();
  }

  /**
   * Evaluates the derivative of the geometric Jacobian matrix
   */
  static void evalJp(const JointVector& q, const JointVector& qd, JacobianMatrix& Jp, const RobotParams& param)
  {
    if (!Jp.IsRowMajor){
      JacobianMatrixRowMajor_t Jp_rowMajor;
      geo_jacobian_endeffector_p((Scalar *)q.data(),(Scalar *)qd.data(),param, (Scalar(*)[_ROBOT_q_DIM_])Jp_rowMajor.data());
      Jp = Jp_rowMajor;
    }else{
      geo_jacobian_endeffector_p((Scalar *)q.data(),(Scalar *)qd.data(),param, (Scalar(*)[_ROBOT_q_DIM_])Jp.data());
    }
  }

  /**
   * Evaluates the Mass Matrix
   */
  static void evalM(const JointVector& q, JointMatrix& M, const RobotParams& param)
  {
    inertia_matrix((Scalar *)q.data(), param,(Scalar(*)[_ROBOT_q_DIM_])M.data());
    /*
    if (!M.IsRowMajor){
      M.transposeInPlace();//should be symetric and therefore not necessary
    }
    */
  }

  /**
   * Evaluates the Coriolis Matrix
   */
  static void evalC(const JointVector& q, const JointVector& qd, JointMatrix& C, const RobotParams& param)
  {
    coriolis_matrix<Scalar>((Scalar *)q.data() ,(Scalar *)qd.data() ,param, (Scalar(*)[_ROBOT_q_DIM_])C.data());
    if (!C.IsRowMajor){
      C.transposeInPlace();
    }
  }

  /**
   * Evaluates the Gravity Vector
   */
  static void evalG(const JointVector_t& q, JointVector_t& G, const RobotParams& param)
  {
      gravitational_forces<Scalar>((Scalar*)q.data(), param, (Scalar (*) [_ROBOT_q_DIM_] )G.data());
  }
  /**
   * Evaluates the forward Kinematics / T0E
   */
  static void forwardKinematics(const JointVector& q, HomogeneousTransformation& T0E, const RobotParams& param)
  {
    //hom_transform_endeffector(  (Scalar (*) [_ROBOT_q_DIM_]) (Scalar * )q.data(), param,  (Scalar (*) [_ROBOT_q_DIM_])tmp_T0E);
    hom_transform_endeffector<Scalar>((Scalar * )q.data(), param,  (Scalar (*) [4])T0E.data());
    if (!T0E.IsRowMajor){
      T0E.transposeInPlace();
    }
  }
};
#endif // ROBOTMODELCLASS_H
