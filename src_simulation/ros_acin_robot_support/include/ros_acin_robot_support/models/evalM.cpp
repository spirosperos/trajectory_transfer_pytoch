#include <evalM.h>
void evalM (const Eigen::Ref<JointVector_t> q, Eigen::Ref<JointMatrix_t> M, const RobotParams& param)
{
  JointMatrix_t M_;
  RobotModel::evalM(q, M_, param);
  M = M_;
}
void py_acin_evalM(double* q, double* M, const RobotParams* param) {
    Eigen::Map<JointVector_t> dataMap_q(q, _ROBOT_q_DIM_, 1);
    Eigen::Map<JointMatrix_t> resultMap_M(M, _ROBOT_q_DIM_, _ROBOT_q_DIM_);
    if (param != NULL) {
        evalM(dataMap_q, Eigen::Ref<JointMatrix_t>(resultMap_M), *param);
    }
    else {
        RobotParams robot_params_;
        evalM(dataMap_q, Eigen::Ref<JointMatrix_t>(resultMap_M), robot_params_);
    }
}
