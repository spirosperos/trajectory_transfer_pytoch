#include <evalG.h>
void evalG (const Eigen::Ref<JointVector_t> q, Eigen::Ref<JointVector_t> G, const RobotParams& param)
{
  JointVector_t G_;
  RobotModel::evalG(q, G_,param);
  G = G_;
}

void py_acin_evalG(double* q, double* G, const RobotParams* param) {
    Eigen::Map<JointVector_t> dataMap_q(q, _ROBOT_q_DIM_, 1);
    Eigen::Map<JointVector_t> resultMap_G(G, _ROBOT_q_DIM_, 1);
    if (param != NULL) {
        evalG(dataMap_q, resultMap_G, *param);
    }
    else {
        RobotParams robot_params_;
        evalG(dataMap_q, resultMap_G, robot_params_);
    }
}
