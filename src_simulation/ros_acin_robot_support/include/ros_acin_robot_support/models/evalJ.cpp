#include <evalJ.h>
void evalJ (const Eigen::Ref<JointVector_t> q, Eigen::Ref<JacobianMatrix_t> J, const RobotParams& param)
{
  JacobianMatrix_t J_;
  RobotModel::evalJ(q, J_,param);
  J = J_;
}
void py_acin_evalJ(double* q, double* J, const RobotParams* param) {
    const Eigen::Map<JointVector_t> dataMap_q(q, _ROBOT_q_DIM_, 1);
    Eigen::Map<JacobianMatrix_t> resultMap_J(J, 6, _ROBOT_q_DIM_);
    if (param != NULL) {
        evalJ(dataMap_q, resultMap_J, *param);
    }
    else {
        RobotParams robot_params_;
        evalJ(dataMap_q, Eigen::Ref<JacobianMatrix_t>(resultMap_J), robot_params_);
    }
}
