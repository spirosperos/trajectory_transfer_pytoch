#include <evalT0E.h>
void evalT0E (const Eigen::Ref<JointVector_t> q, Eigen::Ref<HomogeneousTransformation_t> T0E, const RobotParams& param)
{
  HomogeneousTransformation_t T0E_;
  RobotModel::forwardKinematics(q, T0E_, param);
  T0E = T0E_;
}
void py_acin_evalT0E(double* q, double* T0E, const RobotParams* param) {
    Eigen::Map<JointVector_t> dataMap_q(q, 7, 1);
    Eigen::Map<HomogeneousTransformation_t> resultMap_TOE(T0E, 4, 4);
    //printf("dataMap_q:= [%f, %f, %f, %f, %f, %f, %f]", q[0],q[1],q[2],q[3],q[4],q[5],q[6]);
    //printf("resultMap_TOE:= [%f, %f, %f, %f]", resultMap_TOE(0,3),resultMap_TOE(1,3),resultMap_TOE(2,3),resultMap_TOE(3,3));
    //printf("pointer para:= %p\n",param);
    //printf("pointer para:= %f\n",param->d1);
    //TODO some kind of evaluation, if the number of paramters in the memroy of the struct are equcal as defeind in c,..
    if (param != NULL) {
        evalT0E(dataMap_q, resultMap_TOE, *param);
        //printf("pointer para:= %f\n",param->d1);
    }
    else {
        RobotParams robot_params_;
        evalT0E(dataMap_q, resultMap_TOE, robot_params_);
        //printf("c defined para:= %f\n",robot_params_.d1);
    }
}
