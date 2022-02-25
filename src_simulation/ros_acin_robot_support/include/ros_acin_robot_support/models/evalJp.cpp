#include <evalJp.h>
void evalJp (const Eigen::Ref<JointVector_t> q, Eigen::Ref<JointVector_t> qd, Eigen::Ref<JacobianMatrix_t> Jp, const RobotParams& param)
{
  JacobianMatrix_t Jp_;
  RobotModel::evalJp(q, qd, Jp_, param);
  Jp = Jp_;
}
void py_evalJp(double* q, double* qp, double* Jp, const RobotParams* param) {
	Eigen::Map<JointVector_t> dataMap_q(q, _ROBOT_q_DIM_, 1);
	Eigen::Map<JointVector_t> dataMap_qp(qp, _ROBOT_q_DIM_, 1);
	Eigen::Map<JacobianMatrix_t> resultMap_Jp(Jp, 6, _ROBOT_q_DIM_);
	RobotParams robot_params_;
	evalJp(dataMap_q, dataMap_qp, resultMap_Jp, robot_params_);
}
