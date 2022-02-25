#include <evalC.h>
static uint64_t dim = _ROBOT_q_DIM_;
void evalC (const Eigen::Ref<JointVector_t> q, const Eigen::Ref<JointVector_t> qd, Eigen::Ref<JointMatrix_t> C,const RobotParams& param)
{
  JointMatrix_t C_;
  RobotModel::evalC(q, qd, C_, param);
  C = C_;
}
void py_acin_evalC(double* q, double* qd, double* C, const RobotParams* param) {
    Eigen::Map<JointVector_t> dataMap_q(q, _ROBOT_q_DIM_, 1);
    Eigen::Map<JointVector_t> dataMap_qd(q, _ROBOT_q_DIM_, 1);
    Eigen::Map<JointMatrix_t> resultMap_C(C, _ROBOT_q_DIM_, _ROBOT_q_DIM_);
    if (param != NULL) {
        evalC(dataMap_q, dataMap_qd, resultMap_C, *param);
    }
    else {
        RobotParams robot_params_;
        evalC(dataMap_q, dataMap_qd, resultMap_C, robot_params_);
    }
}
