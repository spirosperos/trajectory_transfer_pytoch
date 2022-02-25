#ifndef ACIN_CONTROLLER_NODE_H
#define ACIN_CONTROLLER_NODE_H
#include <ros_acin_robot_support/ros_acin_robot_support.h>

#include <time.h>
/*Eigen lib related headers*/
#include <vector>

/**/

/*unit test/ debug headers*/
#include "util_control.h"

template <int N_knots_> class ACIN_Controller_Node_C
{
    template <int> friend class ROS_ACIN_Controller_Node_C;
    //typedef pBSpline_interpolator_C<3, 3, N_knots_, false> spline_Interpolator_t;
    //typedef std::shared_ptr<spline_Interpolator_t> spline_Interpolator_sptr_t;
    //typedef pBSpline_interpolator_C<4, 3, N_knots_, true> quaternion_Interpolator_t;
    //typedef std::shared_ptr<quaternion_Interpolator_t> quaternion_Interpolator_sptr_t;
    //typedef pBSpline_interpolator_C<_ROBOT_q_DIM_, 3, N_knots_, false> joint_Interpolator_t;
    //typedef std::shared_ptr<joint_Interpolator_t> joint_Interpolator_sptr_t;
    //typedef typename spline_Interpolator_t::curve_t point_curve_t;
protected:
    enum traj_pub_t { TRAJ_DESIRED, TRAJ_ACTUEL, TRAJ_ERROR };
    enum timeBase_t { ACIN_CTRL_BASE_SEC = 0, ACIN_CTRL_BASE_mSEC, ACIN_CTRL_BASE_uSEC, ACIN_CTRL_BASE_nSEC };
    timeBase_t timeBase_setting = ACIN_CTRL_BASE_SEC;
    typedef struct {
       //rotation between the robot base coord and the world coord, extrinsic rotation x-y-z, or intrinstic Z-Y-X (Yaw - Pitch - Roll)
       double BaseRotationX = 0;// roll angle (gamma_base) between base and world coord system
       double BaseRotationY = 0;// pitch angle (beta_base)  between base and world coord system
       double BaseRotationZ = 0;//Yaw angle (alpha_base)  between base and world coord system
       // -> R_w_0 = R_z(BaseRotationZ)*R_y(BaseRotationY)*R_x(BaseRotationX)
       double dw0x = 0.0;
       double dw0y = 0.0;
       double dw0z = 0.0;
       double mE = 0;
       double sE[3] = {0,0,0};
       double T_init_0 = 5;
       JointVector_t q_d_NS_0;
       timeBase_t timeBase_setting = ACIN_CTRL_BASE_SEC;
       double T_end_lb = 0.5;//  lower bound end timewhen in the soll postion/orientation command or coord adjustment command, when desired end time when the endpose/robot coord should be reached
       double TaskSpace_PosAdj_ub_limit = 0.01;// upper bound, possible adjusment value for reaching the postion in the coord adjustment commant
       double AngleAdj_ub_limit = 5;//upper bound of the orientation(task space) or robot coordinate adjustmnet values
       double accPerc_lb_limit = 0.2;//percentace lower bound for the acceleration phase at the beginning and the deacceleration phase in the soll pose(taskspace)/soll robot angles and coord adjustmnet commands
       bool TaskSpace_ctrl_safety = true;
       double DeltaTau_p_safety = 0.5;
       bool TaskSpaceCtrl_K_endeffektor = true; //CT taskspace controller,is the stiffness given for the coordinate system of the endeffector, otherwise in base robot coord. system ?
       bool TaskSpaceCtrl_D_endeffektor = true; //CT taskspace controller,is the damping given for the coordinate system of the endefector, otherwise in base robot coord. system
    }init_acin_ctrl_t;
    ~ACIN_Controller_Node_C() {
        if (this->point_interpolator_sptr_ != NULL) {
            this->point_interpolator_sptr_.reset();
        }
        if (this->joint_interpolator_sptr_ != NULL) {
            this->joint_interpolator_sptr_.reset();
        }
        if (this->euler_interpolator_sptr_ != NULL) {
            this->euler_interpolator_sptr_.reset();
        }
        if (this->quat_interpolator_sptr_ != NULL) {
            this->quat_interpolator_sptr_.reset();
        }
    }
    ACIN_Controller_Node_C(double Ta) :Ta_(Ta) {
        this->timeBase_setting = ACIN_CTRL_BASE_SEC;
        FIRST_SENSOR_VALUE_ = false;
        Euler_interpl_flag_ = false;
        Joint_interpl_flag_ = true;
        switch_ = false;
        qN_log_barrier_flag_ = false;
        qN_max_FoM_flag_ = false;
        gravity_comp_ = true;
        safe_mode_ = false; // if the controler is in safe mode, only joint space trajectorys are allowed, setting the gravity_mode from off to on back to off will reset
        verbose_ = false;
        TaskSpaceCtrl_K_endeffektor_ = false;
        TaskSpaceCtrl_D_endeffektor_ = false;
        for (uint8_t ii = 0; ii < 3;ii++) {
            sE_[ii] = 0;
        }
        mE_ = 0;
        T_init_0_ = 2;
        q_d_NS_ = JointVector_t::Zero();
        T_end_lb_ = 0.5; //lower point of reaching the desired end value, can be either given by a coordinate adusjemnt command or an command for reching a desired end goal
        TaskSpace_PosAdj_ub_limit_ = 0.01;
        AngleAdj_ub_limit_ = 5;// Angle Adj ub limit for coord adjustment command
        TaskSpace_ctrl_safety_ = true;
        DeltaTau_p_safety_ = 0.2;
        accPerc_lb_limit_ = 0.3;
        R_w_B_ = RotationMatrix_t::Identity();
        d_w_B_ = Vector3::Zero();
        H_w_B_.block<3,3>(0,0) = R_w_B_;
        H_w_B_.block<3,1>(0,3) = d_w_B_;
        H_w_B_(3,3)  = 1;
        init_node();
    }
    //DeltaTau_p_safety
    void init_controller_cfg(double Ta,
        init_acin_ctrl_t& init_acin_ctrl
       ){//DeltaTau_p_safety_ used for task space controll safety near singularitys
        this->T_end_lb_ = init_acin_ctrl.T_end_lb;
        this->TaskSpace_PosAdj_ub_limit_ = init_acin_ctrl.TaskSpace_PosAdj_ub_limit;
        this->AngleAdj_ub_limit_ = init_acin_ctrl.AngleAdj_ub_limit;
        this->timeBase_setting = init_acin_ctrl.timeBase_setting;
        this->TaskSpace_ctrl_safety_ = init_acin_ctrl.TaskSpace_ctrl_safety;
        this->TaskSpaceCtrl_K_endeffektor_ = init_acin_ctrl.TaskSpaceCtrl_K_endeffektor;
        this->TaskSpaceCtrl_D_endeffektor_ = init_acin_ctrl.TaskSpaceCtrl_D_endeffektor;
        this->DeltaTau_p_safety_ = init_acin_ctrl.DeltaTau_p_safety;
        if (this->DeltaTau_p_safety_ > 0.5 || this->DeltaTau_p_safety_ < 0.0) {
            this->DeltaTau_p_safety_ = 0.2;
        }
        if (this->T_end_lb_ < 0) {
            this->T_end_lb_ = 0;
        }
    #if !defined(FLAG_ROS_COMPILE)
        ACIN_CTRL_ASSERT(N_knots_ == _Traj_N_Knot_);
        ACIN_CTRL_ASSERT((sizeof(Joint_t) == sizeof(double) * (_ROBOT_q_DIM_)) && "Array Sizes of type Joint_t {Joints []} not set set proberly ?");
        ACIN_CTRL_ASSERT((sizeof(Pose_t) == sizeof(double) * (3 + 4)) && "Array Sizes of type Pose_t {Postion [], Orientation[]} not set proberly ?");
        //printf("TaskSpaceTraj_t %lu vs.%lu \n",sizeof(TaskSpaceTraj_t),sizeof(double)*(N_knots_) + sizeof(acin_control_Pose_t)*(N_knots_) + sizeof(bool));
        //printf("TaskSpaceTraj_t::PoseArray %lu vs.%lu \n",sizeof(TaskSpaceTraj_t::PoseArray),sizeof(acin_control_Pose_t)*(N_knots_));
        ACIN_CTRL_ASSERT((sizeof(TaskSpaceTraj_t) == sizeof(double) * (N_knots_)+sizeof(acin_control_Pose_t) * (N_knots_)+sizeof(bool)) && "Array Sizes of type TaskSpaceTraj_t not set proberly ?");
        ACIN_CTRL_ASSERT(sizeof(JointSpaceTraj_t) == sizeof(double) * (N_knots_)+sizeof(acin_control_Joints_t) * (N_knots_)+sizeof(acin_control_Header_t) && "Array Sizes of type JointSpaceTraj_t not set proberly ?");
        ACIN_CTRL_ASSERT(sizeof(JointState_t) == sizeof(double) * (_ROBOT_q_DIM_) * 3 && "Array Sizes of type JointState_t not set proberly ?");
    #else

    #endif
    //twinCat_init(void);
    //#cur_time_handle_ =
    /*TODO SET init values !!""*/



    mE_ = init_acin_ctrl.mE;
    for (uint8_t ii = 0; ii < 3;ii++) {
        sE_[ii] = init_acin_ctrl.sE[ii];
    }
    robot_params_.m7 = robot_params_.m7_ + mE_;
    robot_params_.sp7x = (robot_params_.m7_ * robot_params_.sp7x_ + mE_ * sE_[0]) / robot_params_.m7; // m7 = m7_(link 7) +  mass of the endeffector tool
    robot_params_.sp7y = (robot_params_.m7_ * robot_params_.sp7y_ + mE_ * sE_[1]) / robot_params_.m7;
    robot_params_.sp7z = (robot_params_.m7_ * robot_params_.sp7z_ + mE_ * sE_[2]) / robot_params_.m7;
    T_init_0_ = init_acin_ctrl.T_init_0;
    q_d_NS_ = init_acin_ctrl.q_d_NS_0;
    robot_params_.alpha_base = init_acin_ctrl.BaseRotationZ*M_PI/180;
    robot_params_.beta_base = init_acin_ctrl.BaseRotationY*M_PI/180;
    robot_params_.gamma_base = init_acin_ctrl.BaseRotationX*M_PI/180;

    robot_params_.dw0x = init_acin_ctrl.dw0x;
    robot_params_.dw0y = init_acin_ctrl.dw0y;
    robot_params_.dw0z = init_acin_ctrl.dw0z;
    RotationMatrix_t R_z = RotationMatrix_t(Eigen::AngleAxisd(init_acin_ctrl.BaseRotationZ*M_PI/180, Eigen::Vector3d::UnitZ()).toRotationMatrix());
    RotationMatrix_t R_y = RotationMatrix_t(Eigen::AngleAxisd(init_acin_ctrl.BaseRotationY*M_PI/180, Eigen::Vector3d::UnitY()).toRotationMatrix());
    RotationMatrix_t R_x = RotationMatrix_t(Eigen::AngleAxisd(init_acin_ctrl.BaseRotationX*M_PI/180, Eigen::Vector3d::UnitX()).toRotationMatrix());
    RotationMatrix_t R_d = R_z * R_y * R_x;
    R_w_B_ = R_d; //Rotation between robot base and world coord system -> from world coord system to robot coord
    d_w_B_ = Vector3::Zero();
    d_w_B_(0) = init_acin_ctrl.dw0x;
    d_w_B_(1) = init_acin_ctrl.dw0y;
    d_w_B_(2) = init_acin_ctrl.dw0z;
    H_w_B_.block<3,3>(0,0) = R_w_B_;
    H_w_B_.block<3,1>(0,3) = d_w_B_;
    H_w_B_(3,3) = 1;
    init_node();
    }
  ACIN_Controller_Node_C( double Ta,
                          init_acin_ctrl_t &init_acin_ctrl
                        ) : Ta_(Ta)
  {
	  ACIN_CTRL_VERBOSE("init_controller_cfg");
	  ACIN_CTRL_VERBOSE("controller_params_ %f", controller_params_.L(0));
    this->init_controller_cfg(Ta, init_acin_ctrl);
  }
  #if !defined(ACIN_CONTROLLER_DEBUG)
    struct timespec {
        double tv_sec;
        double tv_nsec;
    };
  #endif

  virtual void v_pub_Tau(Eigen::Ref<JointVector_t> Tau_out) = 0;
  virtual void v_pub_PoseTraj_actuel(PoseState_t &PoseTraj) = 0;
  virtual void v_pub_PoseTraj_desired(PoseState_t &PoseTraj)= 0;
  virtual void v_pub_TaskSpace_error(PoseState_t &PoseState)= 0; /*publish joint space controler joint errors*/
  virtual void v_pub_JointSpace_error(JointState_t & JointState)= 0; /*publish joint space controler joint errors*/

  virtual void v_pub_JointSpace_precurve(JointState_t &JointState)= 0;
  virtual void v_pub_time_meas(calcTimeMeasurement_t &calcTimeMeasurement)= 0; /*publish task space controler pose errors*/
  virtual double get_cur_time(const timeBase_t &t_base = ACIN_CTRL_BASE_SEC) = 0;
  virtual void v_pub_motion_switch(boolMsg_t &MotionEN_msg) = 0;
  #if defined( ACIN_CONTROLLER_DEBUG)
    virtual double convert_time(time_t sec, uint64_t nsec, timeBase_t t_base)=0;
  #endif
  // ~ACIN_Controller_Node_C(){
  //   wait_for_threads();
  // }
  //##################################################################################
  //##################################################################################
  //##################################################################################
  //##################################################################################
private:
  #if defined(FLAG_ROS_COMPILE)
  std::timed_mutex ACIN_CTRL_mutex_;
  #endif
  uint64_t N_t_meas_avg_= 1;
  //typedef std::chrono::nanoseconds time_meas_resUnit_t;
  //typedef std::chrono::nanoseconds time_meas_pubUnit_t;
  const timeBase_t time_meas_PubUnit = ACIN_CTRL_BASE_nSEC;
  const timeBase_t time_meas_resUnit = ACIN_CTRL_BASE_nSEC;
  //std::chrono::duration<double, time_meas_resUnit_t::period> t_TS_ctrl_ =time_meas_resUnit_t(0);//std::chrono::high_resolution_clock::time_point(0); //time taskpace controller
  //std::chrono::duration<double, time_meas_resUnit_t::period> t_QS_ctrl_ = time_meas_resUnit_t(0);//std::chrono::high_resolution_clock::time_point(0); //time joint controller
  //std::chrono::duration<double, time_meas_resUnit_t::period> t_QS_intpl_ = time_meas_resUnit_t(0);//std::chrono::high_resolution_clock::time_point(0); //time taskpace controller
  //std::chrono::duration<double, time_meas_resUnit_t::period> t_TS_intpl_ = time_meas_resUnit_t(0);//std::chrono::high_resolution_clock::time_point(0);
  struct timespec t_TS_ctrl_ = {0,0};
  struct timespec t_QS_ctrl_ = {0,0};
  struct timespec t_QS_intpl_ = { 0,0 };
  struct timespec t_TS_intpl_ = { 0,0 };
  /*Controller Flags*/
  bool FIRST_SENSOR_VALUE_ = false;
  bool Euler_interpl_flag_ = false;
  bool Joint_interpl_flag_ = true;
  bool switch_= false;
  bool qN_log_barrier_flag_  = false;
  bool qN_max_FoM_flag_ = false;
  bool gravity_comp_ = true;
  bool safe_mode_ = false; // if the controler is in safe mode, only joint space trajectorys are allowed, setting the gravity_mode from off to on back to off will reset
  bool verbose_ =false;
  bool tau_FB_m_Flag_ =false;
  bool ctrl_motion_en_ = false;
  bool friction_comp_CTRL_flag_ =false;
  bool SingularPert_CTRL_flag_ = false;

  bool TaskSpaceCtrl_K_endeffektor_ = true;
  bool TaskSpaceCtrl_D_endeffektor_ = true;
  /*limits, safety paramters*/
  double T_end_lb_ = 0.5;
  double AngleAdj_ub_limit_ = 5;//in degree
  double TaskSpace_PosAdj_ub_limit_ = 0.25; //in m
  double DeltaTau_p_safety_;
  double accPerc_lb_limit_ = 0.2;
  bool TaskSpace_ctrl_safety_;
  /*some paramter*/
  double w_manipulability_ = 0.0;
  double t0_ = 0.0; //start time of the trajectory which is currently handled by the controller and dispatcher
  double t0_ProMP_traj_ = 0.0; //new start time of a next desired trajectory
  double t0_TaskSpace_traj_ = 0.0; //new start time of a next desired task Space trajectory
  double T_ProMP_traj_ = 0.0; //length of the current executed trajectory, for ProMP time how long the disred action will take to be executed !
  double T_TaskSpace_traj_ = 0.0; //length of the current executed trajectory, TaskSpace trajectory, spline interpolation !
  double Ta_;//controller sample time
  /*****************************************************************************/
  RotationMatrix_t R_w_B_ = RotationMatrix_t::Identity();
  Vector3 d_w_B_ = Vector3::Zero();
  /*Joint-Space Domain*/
  JointVector_t q_ = JointVector_t::Zero(); /*actuel joint values*/
  JointVector_t q_offset_ = JointVector_t::Zero();
  JointVector_t q_dot_= JointVector_t::Zero(); /*actuel joint  velocity values*/
  JointVector_t q_dot_filter_ = JointVector_t::Zero();
  JacobianMatrix_t Jp_ = JacobianMatrix_t::Zero();
  JacobianMatrix_t J_ = JacobianMatrix_t::Zero();
  HomogeneousTransformation_t H_B_E_ = HomogeneousTransformation_t::Zero();
  HomogeneousTransformation_t H_w_B_ = HomogeneousTransformation_t::Zero();
  JointVector_t e_q_ = JointVector_t::Zero(); /*actuel joint error, e = q_- q_d_*/
  JointVector_t e_q_dot_ = JointVector_t::Zero(); /*actuel joint error, e = q_dot - q_dot_d_*/

  // internal controller states:
  JointVector_t xq_I_ = JointVector_t::Zero(); /*controller integrator state*/
  // states of the torque filters:
  JointVector_t x_Tau_ = JointVector_t::Zero(); /*internal state of the torque filter*/
  JointVector_t x_Tau_p_ = JointVector_t::Zero(); /*internal state of the derivative torque filter*/
  // state of the derivative filter, dereivates of the measured robot coordinates
  JointVector_t x_q_dot_ = JointVector_t::Zero(); /*controller integrator state*/
  JointVector_t x_q_dot_hat_ = JointVector_t::Zero();

  JointVector_t q_d_ = JointVector_t::Zero(); /*spline joint values*/
  JointVector_t qDot_d_= JointVector_t::Zero(); /*spline joint vel. values*/
  JointVector_t qDotDot_d_ = JointVector_t::Zero(); /*spline joint accel_ values*/
  JointVector_t tau_d_ = JointVector_t::Zero(); /*actuel controller torque, joint space or taskspace controller output*/
  JointVector_t tau_SP_ = JointVector_t::Zero(); /*output of the singular perturbation controller for the torque tau*/
  JointVector_t tau_ = JointVector_t::Zero(); /*actuel torque from robot*/ /*measured torque*/
  JointVector_t tau_FB_m_ = JointVector_t::Zero(); /*torque, feedback controller of the motor robot*/ /*measured torque*/
  JointVector_t tau_p_filter_ =JointVector_t::Zero();
  JointVector_t tau_filter_ =JointVector_t::Zero();
  JointVector_t tau_p_ = JointVector_t::Zero(); /*actuel filtered derivativ of the robot torque */
  JointVector_t tau_friction_comp_ = JointVector_t::Zero();
  JointVector_t q_d_NS_ = JointVector_t::Zero(); /* desired nullspace joint position */
  /*****************************************************************************/
  /*Task-Space Controller Domain*/
  /*Desired Values of the Pose, point velocity pDot, */
  PoseVector_t pose_d_ = PoseVector_t::Zero(); //pose_d_  = (x, y, z, quotation w, quotation x,quotation y,quotation z)

  PoseVector_t e_pose_ = PoseVector_t::Zero(); /**/
  CartesianVector_t e_cart_vel_ = CartesianVector_t::Zero(); /**/
  CartesianVector_t e_cart_accel_ = CartesianVector_t::Zero(); /**/

  PoseVector_t pose_ = PoseVector_t::Zero(); /*actuel pose error, e = pose_- q_d_*/
  CartesianVector_t cart_vel_ = CartesianVector_t::Zero(); /*actuel joint error, e = pose_- q_d_*/
  Eigen::Vector3d pDot_d_= Eigen::Vector3d::Zero(); //(x_dot,y_dot,z_dot)
  Eigen::Vector3d omega_d_ = Eigen::Vector3d::Zero(); //(omega_x,omega_y,omega_z)
  Eigen::Vector3d pDotDot_d_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d omegaDot_d_ = Eigen::Vector3d::Zero();
  /*****************************************************************************/
  /*Interpolators*/
  // JerkHatInterpolator<N_knots_, 7> jerk_interpolator_;
  /*pieceWise Spline interpolator, for interpolation in the TaskSpace (point(x,y,z),EulerAngles(Yaw, Pitch, Roll)*/
  const static uint16_t N_PreCurve = 3; //number of points used for defining the pre curve of the interpolation, -> last N_PreCurve sensor values are used
  typedef Eigen::Matrix<double,  N_knots_, 1> time_intpl_t;
  typedef Eigen::Matrix<double,  3, N_knots_> spline_intpl_t;
  typedef Eigen::Matrix<double,  9, N_knots_> joint_intpl_t;
  typedef Eigen::Matrix<double,  4, N_knots_> quat_intpl_t;
  spline_Interpolator_sptr_t point_interpolator_sptr_;
  spline_Interpolator_sptr_t euler_interpolator_sptr_;
  quaternion_Interpolator_sptr_t quat_interpolator_sptr_;
  joint_Interpolator_sptr_t joint_interpolator_sptr_;
  //ProMPInterpolator_C ProMPInterpolator_;
  double T_traj_end_ = std::numeric_limits<double>::infinity();
  double T_init_0_ = 3.0;
  typename joint_Interpolator_t::pre_curve_t q_PreCurve_;
  typename spline_Interpolator_t::pre_curve_t p_PreCurve_;
  typename spline_Interpolator_t::pre_curve_t euler_PreCurve_;
  typename quaternion_Interpolator_t::pre_curve_t qaut_PreCurve_;
  /*****************************************************************************/
  /*paramter structes*/
  RobotParams robot_params_;
  struct ControllerParams controller_params_;
  double mE_ = 0.0; // endeffector mass
  Eigen::Vector3d sE_ = Eigen::Vector3d::Zero(); //center of gravity of the endeffector mass
  /*int  values for counting certain events*/
  uint16_t dalay_task_space_ = 0;
  /**/

  // void wait_for_threads(){
  //
  // }


  void init_node(){

    /*init actuel joint values*/
    q_ = JointVector_t::Zero(); // actuel joint values
    q_dot_ = JointVector_t::Zero(); // actuel value of the the joint velocitys
    q_d_NS_= JointVector_t::Zero(); // taskspace controller nullspace joint values
    xq_I_ = JointVector_t::Zero(); // joint space controler integrator sate
    x_q_dot_hat_ = JointVector_t::Zero();
    tau_filter_ = JointVector_t::Zero();
    tau_p_filter_ = JointVector_t::Zero();
    q_dot_filter_ = JointVector_t::Zero();
    evalT0E(q_ ,H_B_E_, robot_params_);
    evalJ(q_, J_, robot_params_);
    evalJp(q_, q_dot_, Jp_,robot_params_);
    /*init actuel torque values*/
    tau_d_ = JointVector_t::Zero();
    tau_ = JointVector_t::Zero();
    tau_FB_m_ = JointVector_t::Zero();
    tau_FB_m_Flag_ = false;
    ctrl_motion_en_ = false;
    pDot_d_ = Eigen::Vector3d::Zero();
    omega_d_ = Eigen::Vector3d::Zero();
    pDotDot_d_ = Eigen::Vector3d::Zero();
    omegaDot_d_ = Eigen::Vector3d::Zero();
    e_pose_ = PoseVector_t::Zero();
    e_cart_vel_ = CartesianVector_t::Zero();
    e_cart_accel_= CartesianVector_t::Zero();
    /**************************************************************************/
    /*****************Setup Taskspace and Joint Space Interpolation****************************/
    /**************************************************************************/
    double T_TaskSpace_0 = 5.0;
    time_intpl_t t_vec = time_intpl_t::LinSpaced(N_knots_, 0.0, T_TaskSpace_0);

    spline_intpl_t point_knots = spline_intpl_t::Zero();
    joint_intpl_t joint_knots = joint_intpl_t::Zero();
    spline_intpl_t eulerA_knots = spline_intpl_t::Zero();
    quat_intpl_t quat_knots = quat_intpl_t::Zero();


    typename joint_Interpolator_t::curve_t joint_curve;
    typename joint_Interpolator_t::pre_curve_t pre_joint_curve;
    joint_curve.pts = joint_knots;
    joint_curve.para = t_vec;
    //joint_Interpolator_t joint_interpolator = joint_Interpolator_t(joint_curve, pre_joint_curve);
    //this-> joint_interpolator_sptr_ = std::make_shared<joint_Interpolator_t>(joint_interpolator);
    this-> joint_interpolator_sptr_ = std::shared_ptr<joint_Interpolator_t>(new joint_Interpolator_t(joint_curve, pre_joint_curve));
    typename spline_Interpolator_t::curve_t point_curve;
    typename spline_Interpolator_t::pre_curve_t pre_curve;
    point_curve.pts = point_knots;
    point_curve.para = t_vec;
    //spline_Interpolator_t point_interpolator = spline_Interpolator_t(point_curve, pre_curve);
    //this-> point_interpolator_sptr_ = std::make_shared<spline_Interpolator_t>(point_interpolator);
    this-> point_interpolator_sptr_ = std::shared_ptr<spline_Interpolator_t>(new spline_Interpolator_t(point_curve, pre_curve));

    typename spline_Interpolator_t::curve_t euler_curve;
    euler_curve.pts = eulerA_knots;
    euler_curve.para = t_vec;
    //spline_Interpolator_t euler_interpolator = spline_Interpolator_t(euler_curve, pre_curve);
    //this-> euler_interpolator_sptr_ = std::make_share<spline_Interpolator_t>(euler_interpolator);
    this-> euler_interpolator_sptr_ = std::shared_ptr<spline_Interpolator_t>(new spline_Interpolator_t(euler_curve, pre_curve));

    typename quaternion_Interpolator_t::curve_t quat_curve;
    typename quaternion_Interpolator_t::pre_curve_t pre_quat_curve;
    quat_curve.pts = quat_knots;
    quat_curve.para = t_vec;
    //quaternion_Interpolator_t quat_interpolator = quaternion_Interpolator_t(quat_curve, pre_quat_curve);
    //this-> quat_interpolator_sptr_ = std::make_shared<quaternion_Interpolator_t>(quat_interpolator);
    this-> quat_interpolator_sptr_ = std::shared_ptr<quaternion_Interpolator_t>(new  quaternion_Interpolator_t(quat_curve, pre_quat_curve));
    for (uint64_t ii = 0; ii < static_cast<uint64_t>(q_PreCurve_.pts.cols()) - 1; ii++) {
      q_PreCurve_.pts.col(ii) = JointVector_t::Zero();
      q_PreCurve_.para(ii) = 0;
    }
    for (uint64_t ii = 0; ii < static_cast<uint64_t>(p_PreCurve_.pts.cols()) - 1; ii++) {
        p_PreCurve_.pts.col(ii) = Eigen::Matrix<double,3, 1>::Zero();
        p_PreCurve_.para(ii) = 0;
    }
    for (uint64_t ii = 0; ii < static_cast<uint64_t>(euler_PreCurve_.pts.cols()) - 1; ii++) {
        euler_PreCurve_.pts.col(ii) = Eigen::Matrix<double, 3, 1>::Zero();
        euler_PreCurve_.para(ii) = 0;
    }
    for (uint64_t ii = 0; ii < static_cast<uint64_t>(qaut_PreCurve_.pts.cols()) - 1; ii++) {
        qaut_PreCurve_.pts.col(ii) = Eigen::Matrix<double, 4, 1>::Zero();
        qaut_PreCurve_.para(ii) = 0;
    }
  }
  void pub_PoseTraj_( Eigen::Ref<PoseVector_t> pose_d,
                        Eigen::Ref<CartesianVector_t> cart_vel_d,
                        Eigen::Ref<CartesianVector_t> cart_accel_d,
                        traj_pub_t traj_pub)
   {
     /**************************************************************************/
     /**************************************************************************/
     /**************************************************************************/
     auto x_ProMP_traj = PoseState_t();
     double t = get_cur_time(ACIN_CTRL_BASE_SEC);
     #if defined(FLAG_ROS_COMPILE)
      x_ProMP_traj.header.stamp = ros::Time(t);
    #else
      x_ProMP_traj.header.stamp = t;
     #endif
     /* ------------------------------- */
     auto Pose =  Pose_t();
     #if defined(FLAG_ROS_COMPILE)
      Pose.position = std::vector<double>(3);
     #endif
     Pose.position[0] = pose_d(0,0);
     Pose.position[1] = pose_d(1,0);
     Pose.position[2] = pose_d(2,0);
     #ifdef FLAG_ROS_COMPILE
     Pose.orientation = std::vector<double>(4);
     #endif
     Pose.orientation[0] = pose_d(3,0); //qaut w
     Pose.orientation[1] = pose_d(4,0); //qaut x
     Pose.orientation[2] = pose_d(5,0); //qaut y
     Pose.orientation[3] = pose_d(6,0); //qaut z
     x_ProMP_traj.pose = Pose;
     /* ------------------------------- */
     auto pDot_traj =  Point_t();
     pDot_traj.x = cart_vel_d(0,0);
     pDot_traj.y = cart_vel_d(1,0);
     pDot_traj.z = cart_vel_d(2,0);
     x_ProMP_traj.p_vel = pDot_traj;
     /* ------------------------------- */
     auto omega_traj=  Point_t();
     omega_traj.x = cart_vel_d(3,0);
     omega_traj.y = cart_vel_d(4,0);
     omega_traj.z = cart_vel_d(5,0);
     x_ProMP_traj.omega = omega_traj;
     /* ------------------------------- */
     auto pDotDot_traj =  Point_t();
     pDotDot_traj.x = cart_accel_d(0,0);
     pDotDot_traj.y = cart_accel_d(1,0);
     pDotDot_traj.z = cart_accel_d(2,0);
     x_ProMP_traj.p_accl = pDotDot_traj;
     /* ------------------------------- */
     auto omegaDot_traj=  Point_t();
     omegaDot_traj.x = cart_accel_d(3,0);
     omegaDot_traj.y = cart_accel_d(4,0);
     omegaDot_traj.z = cart_accel_d(5,0);
     x_ProMP_traj.omegaDot = omegaDot_traj;
     switch (traj_pub){
        case TRAJ_DESIRED:
          v_pub_PoseTraj_desired(x_ProMP_traj);
        break;
        case TRAJ_ACTUEL:
          v_pub_PoseTraj_actuel(x_ProMP_traj);
        break;
        case TRAJ_ERROR:
          v_pub_TaskSpace_error(x_ProMP_traj);
        break;
        default:
        break;
      }
   }


   void pub_e_JointState_(){
     JointState_t precurve;
    #ifdef FLAG_ROS_COMPILE
      precurve.position = std::vector<double>(_ROBOT_q_DIM_);
      precurve.velocity = std::vector<double>(_ROBOT_q_DIM_);
      precurve.effort = std::vector<double>(_ROBOT_q_DIM_);
    #endif
    #ifndef FLAG_ROS_COMPILE
     //assert(sizeof(Joint_t) == _ROBOT_q_DIM_*sizeof(double)*3);
    #endif
     JointState_t e_JointsState;
    #ifdef FLAG_ROS_COMPILE
     e_JointsState.position = std::vector<double>(_ROBOT_q_DIM_);
     e_JointsState.velocity = std::vector<double>(_ROBOT_q_DIM_);
     e_JointsState.effort = std::vector<double>(_ROBOT_q_DIM_);
    #endif
    #ifndef FLAG_ROS_COMPILE
     //assert(sizeof(Joint_t) == _ROBOT_q_DIM_*sizeof(double)*3);
    #endif
     for (uint32_t ii = 0; ii < e_q_.rows(); ii++){
       precurve.position[ii] = q_PreCurve_.pts(ii, q_PreCurve_.para.rows()-1);
       //e_JointsState.position[ii] = e_q_[ii];
       //e_JointsState.velocity[ii] = q_d_[ii];
       //e_JointsState.effort[ii] = qDot_d_[ii];
       e_JointsState.position[ii] = q_d_[ii];
       e_JointsState.velocity[ii] = qDot_d_[ii];
       e_JointsState.effort[ii] = qDotDot_d_[ii];
     }
      v_pub_JointSpace_error(e_JointsState);
      v_pub_JointSpace_precurve(precurve);
   }
  void pub_e_poseTraj_(){
    pub_PoseTraj_(e_pose_, e_cart_vel_, e_cart_accel_, traj_pub_t::TRAJ_ERROR);
 }
#if defined(ACIN_CONTROLLER_DEBUG)
  void pub_time_meas_(){
     calcTimeMeasurement_t t_meas;
     // t_meas.t_TS_ctrl = std::chrono::duration_cast<time_meas_pubUnit_t>(t_TS_ctrl_).count();
     // t_meas.t_QS_ctrl = std::chrono::duration_cast<time_meas_pubUnit_t>(t_QS_ctrl_).count();
     // t_meas.t_QS_intpl = std::chrono::duration_cast<time_meas_pubUnit_t>(t_QS_intpl_).count();
     // t_meas.t_TS_intpl = std::chrono::duration_cast<time_meas_pubUnit_t>(t_TS_intpl_).count();
     t_meas.t_TS_ctrl = convert_time(t_TS_ctrl_.tv_sec, t_TS_ctrl_.tv_nsec, time_meas_PubUnit);
     t_meas.t_QS_ctrl = convert_time(t_QS_ctrl_.tv_sec, t_QS_ctrl_.tv_nsec, time_meas_PubUnit);
     t_meas.t_QS_intpl = convert_time(t_QS_intpl_.tv_sec, t_QS_intpl_.tv_nsec, time_meas_PubUnit);
     t_meas.t_TS_intpl = convert_time(t_TS_intpl_.tv_sec, t_TS_intpl_.tv_nsec, time_meas_PubUnit);
     v_pub_time_meas(t_meas);
   }
#endif
protected:

  void stateCallback(const JointState_t& state)
  {
    #if defined(FLAG_ROS_COMPILE)
    std::unique_lock<std::timed_mutex> lock{ACIN_CTRL_mutex_,  std::defer_lock};
    lock.lock();
    #endif
    #if defined(WITH_EXCEPTIONS)
      if (state.position.size() != _ROBOT_q_DIM_ || state.velocity.size() != _ROBOT_q_DIM_){
        throw std::runtime_error("joint stateCallback wrong dim. of state message");
      }
    #endif
    for(int i = 0; i < _ROBOT_q_DIM_; ++i)
    {
      if (std::isnan(state.position[i])) {
         FIRST_SENSOR_VALUE_ = false;
         q_ = JointVector_t::Zero();
         return;
      }
      if (!FIRST_SENSOR_VALUE_){
        if (state.position[i] > M_PI){
          uint64_t n = (uint64_t)floor((double)state.position[i]/(double)M_PI);
          q_offset_(i) = n*M_PI;
        }else{
          if(state.position[i] < -M_PI){
            uint64_t n = (uint64_t)floor((double)state.position[i]/(double)M_PI);
            q_offset_(i) = n*M_PI;
          }else{
            q_offset_(i) = 0;
          }
        }

      }
      q_(i) = state.position[i];
      if (!FIRST_SENSOR_VALUE_ || std::isnan(state.velocity[i])){
        q_dot_(i) = q_dot_filter_[i];
      }else{
        q_dot_(i) = state.velocity[i];
      }
      if(!FIRST_SENSOR_VALUE_) controller_joint_space(false);
      if(std::isnan(state.effort[i])){
        tau_(i) = tau_d_(i);
        tau_filter(true);
        tau_p_filter(true);
      }else{
        tau_(i) = state.effort[i];
      }
    }

    if(FIRST_SENSOR_VALUE_==false){
      #ifdef FLAG_ROS_COMPILE
        ACIN_CTRL_WARN("Get First sensor Value");
        ACIN_CTRL_INFO("Actual Joint Position:");
        ACIN_CTRL_INFO("q0_:= %f, q1_:= %f,q2_:= %f,q3_:= %f, q4_:= %f,q5_:= %f, q6_:= %f",
                q_(0,0),q_(1,0),q_(2,0),q_(3,0),q_(4,0),q_(5,0), q_(6,0));
        ACIN_CTRL_INFO("q0_off:= %f, q1_off:= %f,q2_off:= %f,q3_off:= %f, q4_off:= %f,q5_off:= %f, q6_off:= %f",
            q_offset_(0,0),q_offset_(1,0),q_offset_(2,0),q_offset_(3,0),q_offset_(4,0),q_offset_(5,0), q_offset_(6,0));
      #endif
      HomogeneousTransformation_t H_B_E_q_loc;
      evalT0E(q_, H_B_E_q_loc,robot_params_); //homogenus tranformation from the endeffector in to the robot base evalT0E
      RotationMatrix_t R = H_B_E_q_loc.block<3,3>(0,0);
      Eigen::Vector3d Euler_ist = R.eulerAngles(2,1,0); //YPR
      Eigen::Vector3d p_ist = H_B_E_q_loc.block<3,1>(0,3); //YPR
      Eigen::Quaternion<double> quat_loc = Eigen::Quaternion<double>(H_B_E_q_loc.block<3,3>(0,0));
      Eigen::Vector4d qaut_ist(quat_loc.w(),quat_loc.w(),quat_loc.w(),quat_loc.w()); //YPR
      pose_d_(0,0) = H_B_E_q_loc(0,3);
      pose_d_(1,0) = H_B_E_q_loc(0,3);
      pose_d_(2,0) = H_B_E_q_loc(0,3);
      pose_d_(3,0) =  quat_loc.w();
      pose_d_(4,0) =  quat_loc.x();
      pose_d_(5,0) =  quat_loc.y();
      pose_d_(6,0) =  quat_loc.z();
      double t_now = get_cur_time(timeBase_setting);
      double T_init = T_init_0_;
      double Ta_init = T_init/N_knots_;
      for(uint16_t jj = 0; jj < q_PreCurve_.pts.cols(); jj++){
        q_PreCurve_.pts.col(jj) = q_;
        q_PreCurve_.para(jj) = t_now + Ta_init*(jj-N_PreCurve);
      }
      //*TODO set precurve  in a function... task space precurves depands on the joint pre cureve*/
      get_TaskSpace_preCurve(p_PreCurve_, euler_PreCurve_, qaut_PreCurve_);
      joint_interpolator_sptr_->lin_interpolation(q_, q_, q_PreCurve_, t_now, T_init_0_/2);
      point_interpolator_sptr_->lin_interpolation(p_ist,p_ist, p_PreCurve_,t_now, T_init_0_/2);
      euler_interpolator_sptr_->lin_interpolation(Euler_ist,Euler_ist,euler_PreCurve_, t_now, T_init_0_/2);
      quat_interpolator_sptr_->lin_interpolation(qaut_ist,qaut_ist,qaut_PreCurve_, t_now, T_init_0_/2);
      T_traj_end_ = t_now + T_init_0_;
      q_dot_filter(true);
      tau_p_filter(true);
      q_dot_filter(true);
      FIRST_SENSOR_VALUE_ = true;
    }else{
      double t_now = get_cur_time(timeBase_setting);
      if(t_now  - q_PreCurve_.para(q_PreCurve_.para.rows()-1)  >= Ta_*5){
        double Ta_curve = joint_interpolator_sptr_->get_deltaT_avg();
        double t0_curve = joint_interpolator_sptr_->get_t_0();
        uint16_t N_PreCurve = q_PreCurve_.pts.cols();
        for(uint64_t ii = 0; ii < static_cast<uint64_t>(N_PreCurve)-1; ii++){
            q_PreCurve_.pts.col(ii) = q_PreCurve_.pts.col(ii + 1);
            q_PreCurve_.para(ii) = t0_curve - Ta_curve*( N_PreCurve - ii);
        }
        q_PreCurve_.pts.col(q_PreCurve_.pts.cols()-1) = q_;
        q_PreCurve_.para(q_PreCurve_.pts.cols()-1) = t0_curve - Ta_curve;
        //*TODO set precurve  in a function... task space precurves depands on the joint pre cureve*/
        get_TaskSpace_preCurve(p_PreCurve_, euler_PreCurve_, qaut_PreCurve_);
      }
    }


    /**************************************************************************/
    /**************************************************************************/
    evalT0E(q_, H_B_E_, robot_params_);
    evalJ(q_, J_, robot_params_);
    evalJp(q_, q_dot_, Jp_, robot_params_);

    pose_.block<3,1>(0,0) = H_B_E_.block<3,1>(0,3);
    //std::cout<<"pose_ "<<pose_<<std::endl;
    //std::cout<<"q_ "<<q_<<std::endl;
    Eigen::Quaternion<double> quat_actual(H_B_E_.block<3,3>(0,0));
    pose_(3,0) = quat_actual.w();
    pose_(4,0) = quat_actual.x();
    pose_(5,0) = quat_actual.y();
    pose_(6,0) = quat_actual.z();
    cart_vel_  = J_*q_dot_;

  }
  void GravityModeCallback(const boolMsg_t& gravity_mode){
    #if defined(FLAG_ROS_COMPILE)
    std::unique_lock<std::timed_mutex> lock{ACIN_CTRL_mutex_,  std::defer_lock};
    lock.lock();
    #endif
    if (gravity_mode .data == gravity_comp_){
      return;
    }
    ACIN_CTRL_WARN("switch controller gravity mode");
    ACIN_CTRL_WARN("controller gravity compensation:= %d",gravity_mode.data);
    double t_now = get_cur_time(ACIN_CTRL_BASE_SEC);
    joint_interpolator_sptr_->profiled_interpolation( q_, q_, q_PreCurve_, t_now, T_init_0_/2, 5);
    T_traj_end_ = t_now + T_init_0_;
    Joint_interpl_flag_ = true;
    switch_ = true;
    gravity_comp_ = gravity_mode.data;

  }
  void torque_Motor_feedback_Callback(const Joint_t& torque_FB_m)
  {
    #if defined(FLAG_ROS_COMPILE)
    std::unique_lock<std::timed_mutex> lock{ACIN_CTRL_mutex_,  std::defer_lock};
    lock.lock();
    #endif
    for(uint32_t ii = 0; ii< _ROBOT_q_DIM_; ii++){
      if (torque_FB_m.Joints[ii] == std::numeric_limits<double>::quiet_NaN()){
        tau_FB_m_Flag_= false;
        return;
      }
      tau_FB_m_ [ii] = torque_FB_m.Joints[ii];
    }
    tau_FB_m_Flag_= true;
  }
  void jointNSstateCallback(const JointState_t& state){
    #if defined(FLAG_ROS_COMPILE)
    std::unique_lock<std::timed_mutex> lock{ACIN_CTRL_mutex_,  std::defer_lock};
    lock.lock();
    #endif
    ACIN_CTRL_VERBOSE("Nullspace joint values set");
    #if defined(WITH_EXCEPTIONS)
      if (state.position.size() != _ROBOT_q_DIM_){
        throw std::runtime_error("jointNSstateCallback wrong dim. of state message");
      }
    #endif
    for(int i = 0; i < _ROBOT_q_DIM_; ++i)
    {
      q_d_NS_(i) = state.position[i];
    }
    qN_log_barrier_flag_  = false;
    qN_max_FoM_flag_ = false;
  }
  void ToolParam_Callback(const ToolParam_t& Tool_param){
    #if defined(FLAG_ROS_COMPILE)
    std::unique_lock<std::timed_mutex> lock{ACIN_CTRL_mutex_,  std::defer_lock};
    lock.lock();
    #endif
    ACIN_CTRL_INFO("Tool m7: %f",Tool_param.m);
    ACIN_CTRL_INFO("Tool sp7x: %f",Tool_param.spx);
    ACIN_CTRL_INFO("Tool sp7y: %f",Tool_param.spy);
    ACIN_CTRL_INFO("Tool sp7z: %f",Tool_param.spz);

    double mE = Tool_param.m;
    double sE [] = {Tool_param.spx, Tool_param.spy, Tool_param.spz};
    robot_params_.m7 = robot_params_.m7_ + mE;
    robot_params_.sp7x = (robot_params_.m7_*robot_params_.sp7x_ + mE*sE[0])/robot_params_.m7; // m7 = m7_(link 7) +  mass of the endeffector tool
    robot_params_.sp7y = (robot_params_.m7_*robot_params_.sp7y_+ mE*sE[1])/robot_params_.m7;
    robot_params_.sp7z = (robot_params_.m7_*robot_params_.sp7z_ + mE*sE[2])/robot_params_.m7;
  }

  void JointSpace_trajectoryCallback(const JointSpaceTraj_t& JointSpace_traj){
    #if defined(FLAG_ROS_COMPILE)
      std::unique_lock<std::timed_mutex> lock{ACIN_CTRL_mutex_,  std::defer_lock};
      lock.lock();
    #endif
    double t_now = get_cur_time(timeBase_setting);
    if (t_now == this->q_PreCurve_.para(q_PreCurve_.para.rows()-1)){
      t_now = t_now + this->Ta_;
    }
    #if defined(WITH_EXCEPTIONS)
      if (  JointSpace_traj.JointsArray.size() != JointSpace_traj.t_k.size() ||
            JointSpace_traj.JointsArray.size() >= N_knots_ ||
            JointSpace_traj.t_k.size() >= N_knots_){
            ACIN_CTRL_ERROR("Drop JointSpaceTraj message: Wrong number of Knots (timeKonts: %lu, JointKnots:%lu, expected:%d) given",
              JointSpace_traj.JointsArray.size(),JointSpace_traj.t_k.size(),N_knots_);
        throw std::runtime_error("JointSpace_trajectoryCallback wrong dim. of JointSpace_traj");
      }
    #endif
    if(!gravity_comp_){
      Joint_interpl_flag_ = true;
      switch_= true;
      #ifdef FLAG_ROS_COMPILE
        ACIN_CTRL_WARN("New JointSpace trajectory");
        ACIN_CTRL_WARN("Switch to JointSpace-Controller");
      #endif
      time_intpl_t t_vec = time_intpl_t::Zero();
      joint_intpl_t joint_ctrl_pts = joint_intpl_t::Zero();
      uint64_t valid_traj_pts = 0;
      double t_traj_0 =0;
      double t_traj_jj =0;
      double t_traj_prv =0;
      for(uint32_t jj = 0; jj< joint_ctrl_pts.cols(); jj++){
        #ifdef FLAG_ROS_COMPILE
          ACIN_CTRL_ASSERT(timeBase_setting == ACIN_CTRL_BASE_SEC);//TODO make this robust, other time settings ?
          if (jj == JointSpace_traj.JointsArray.size() ){
          //no more trajectorypoints in the ros message
            break;
          }
          t_traj_0 =static_cast<double>(JointSpace_traj.t_k[0].toSec());
          t_traj_jj = static_cast<double>(JointSpace_traj.t_k[jj].toSec());
          if(jj > 0){
            t_traj_prv = static_cast<double>(JointSpace_traj.t_k[jj-1].toSec());
          }
        #else
          t_traj_0 =static_cast<double>(JointSpace_traj.t_k[0]);
          t_traj_jj = static_cast<double>(JointSpace_traj.t_k[jj]);
          if(jj > 0){
            t_traj_prv = static_cast<double>(JointSpace_traj.t_k[jj-1]);
          }
        #endif
        //########################################################################################################
        //############ check if there are more valid elements within the trajectory message #####################
        //############ if the time value of the previous element is smaller or equal with the current value, break, number of valid data points are jj-1//##############################################
        //##########################################################################################################################################
        if(t_traj_jj < 0 || std::isnan(t_traj_jj)){
          break;
        }
        if(jj > 0 ){
          if (t_traj_prv >= t_traj_jj){
            break;
          }
        }
        //##############################################
        //##############################################
        t_vec(jj,0) = t_now + (t_traj_jj -  t_traj_0);

        Joint_t Joints_jj = JointSpace_traj.JointsArray[jj];
        #if defined(WITH_EXCEPTIONS)
          if (Joints_jj.Joints.size() == joint_ctrl_pts.rows()){
            throw std::runtime_error("Drop JointSpaceTraj message: Wrong number of Joints (%lu/%lu) given at time step (%u)", Joints_jj.Joints.size(), joint_ctrl_pts.rows(), jj+1);
          }
        #endif
        for(uint16_t kk = 0; kk < joint_ctrl_pts.rows(); kk ++){
          joint_ctrl_pts(kk, jj) = Joints_jj.Joints[kk];
        }
        valid_traj_pts++;
      }
      if(valid_traj_pts <2 ){//min one point set in the trajectory
        ACIN_CTRL_ERROR("Drop JointSpaceTraj message, curve dosnt have atleast one point");
        return;
      }
      for(uint32_t jj = 1; jj< valid_traj_pts; jj++){
        if (t_vec(jj - 1,0) >= t_vec(jj,0)){
          ACIN_CTRL_ERROR("< > Drop JointSpaceTraj message, curve parameter is not strictly increasing, must be ensured by the user");
          ACIN_CTRL_ERROR("parameter vector t[%u](%f) >= t[%u](%f)",jj-1,t_vec(jj - 1,0),jj,t_vec(jj,0));
          return;
        }
      }
      double t_last = t_vec(valid_traj_pts - 1,0);
      double T_last = (t_vec(valid_traj_pts - 1,0) - t_vec(valid_traj_pts - 2,0));
      if (T_last < this->Ta_){
        T_last = this->Ta_;
      }
      uint32_t N_preCurve =  q_PreCurve_.pts.cols();
      for(uint32_t jj = 0; jj< N_preCurve; jj++){
        q_PreCurve_.para(jj) = t_now - T_last*(N_preCurve-jj);
      }

      for(uint32_t jj = valid_traj_pts; jj< joint_ctrl_pts.cols(); jj++){
        t_vec(jj,0) = t_last + (jj - valid_traj_pts + 1)*T_last;

        //if not all point of the intepolator curve is set, clamp it to the last value
        for(uint16_t kk = 0; kk < joint_ctrl_pts.rows(); kk ++){
          joint_ctrl_pts(kk, jj) = joint_ctrl_pts(kk, valid_traj_pts - 1);
        }
      }
      #ifdef FLAG_ROS_COMPILE
        std::cout<<"trajectory len:= " << valid_traj_pts << std::endl;
        std::cout<<"max trajecotry len:= " << joint_ctrl_pts.cols() << std::endl;
        std::cout<<"t_now:= " << t_now << std::endl;
        std::cout<<"\tt pre curve last:= "<<q_PreCurve_.para(q_PreCurve_.para.rows()-1)<<std::endl;
        std::cout<<"Curve start:= "<<std::endl;
        std::cout<<"\tt_vec[0]:= " << t_vec(0) << std::endl;
        std::cout<<"\tq(0):= "<<joint_ctrl_pts.col(0).transpose()<<std::endl;
        std::cout<<"Curve end:="<<std::endl;
        std::cout<<"\tt_vec[t_vec.rows()-1]:= " << t_vec(t_vec.rows()-1,0) << std::endl;
        std::cout<<"\tq(t_vec.rows() - 1):= "<<joint_ctrl_pts.col(joint_ctrl_pts.cols() - 1).transpose()<<std::endl;
      #endif
      T_traj_end_ = t_vec(t_vec.rows()-1,0);
      typename joint_Interpolator_t::curve_t joint_curve;
      joint_curve.pts = joint_ctrl_pts;
      joint_curve.para = t_vec;
      //typename joint_Interpolator_t::pre_curve_t NULL_pre_curve;
      joint_interpolator_sptr_->set_curve(joint_curve, q_PreCurve_, true, false, false);
    }else{
      ACIN_CTRL_ERROR("Drop JointSpaceTraj message, gravity mode is active");
    }
  }
  void TaskSpace_trajectoryCallback(const TaskSpaceTraj_t& TaskSpace_traj)
  {
    #if defined(FLAG_ROS_COMPILE)
      std::unique_lock<std::timed_mutex> lock{ACIN_CTRL_mutex_,  std::defer_lock};
      lock.lock();
    #endif
    double t_now = get_cur_time(timeBase_setting);
    if (t_now == this->p_PreCurve_.para(p_PreCurve_.para.rows()-1)){
      t_now = t_now + this->Ta_;
    }
    #if defined(WITH_EXCEPTIONS)
      if ( TaskSpace_traj.PoseArray.size() != TaskSpace_traj.t_k.size()|| TaskSpace_traj.PoseArray.size() >= N_knots_ || TaskSpace_traj.t_k.size() >= N_knots_){
        throw std::runtime_error("TaskSpace_trajectoryCallback wrong dim. of TaskSpace_traj");
      }
    #endif
    if(!gravity_comp_ && !safe_mode_){

      Joint_interpl_flag_ = false;
      switch_=true;
      spline_intpl_t point_ctrl_pts = spline_intpl_t::Zero();
      spline_intpl_t euler_ctrl_pts = spline_intpl_t::Zero();
      quat_intpl_t quat_ctrl_pts = quat_intpl_t::Zero();
      time_intpl_t t_vec = time_intpl_t::Zero();
      this->Euler_interpl_flag_ = false;
      ACIN_CTRL_WARN("New TraskSpace trajectory");
      ACIN_CTRL_INFO("Switch to TaskSpace-Controller");
      #if defined(WITH_EXCEPTIONS)
        if (TaskSpace_traj.PoseArray[0].orientation.size()  != 4 || TaskSpace_traj.PoseArray[0].orientation.size()  != 3){
          throw std::runtime_error("TaskSpace_trajectoryCallback wrong dim. of Pose Array orientation");
        }
      #endif

      if(TaskSpace_traj.OrientationAsQaut == true){
        this->Euler_interpl_flag_ = false;
        ACIN_CTRL_WARN("Orentation is given as qauternions");
      }else{
        this->Euler_interpl_flag_ = true;
        ACIN_CTRL_WARN("Orentation is given as euler angles");
      }
      double t_traj_0 =0;
      double t_traj_jj =0;
      double t_traj_prv =0;
      uint64_t valid_traj_pts = 0;
      for(uint32_t jj = 0; jj< point_ctrl_pts.cols(); jj++){
        #ifdef FLAG_ROS_COMPILE
          ACIN_CTRL_ASSERT(timeBase_setting == ACIN_CTRL_BASE_SEC);//TODO make this robust, other time settings ?
          if (jj == TaskSpace_traj.PoseArray.size() ){
          //no more trajectorypoints in the ros message
            break;
          }
          t_traj_0 =static_cast<double>(TaskSpace_traj.t_k[0].toSec());
          t_traj_jj = static_cast<double>(TaskSpace_traj.t_k[jj].toSec());
          if(jj > 0){
            t_traj_prv = static_cast<double>(TaskSpace_traj.t_k[jj-1].toSec());
          }
        #else
          t_traj_0 =static_cast<double>(TaskSpace_traj.t_k[0]);
          t_traj_jj = static_cast<double>(TaskSpace_traj.t_k[jj]);
          if(jj > 0){
            t_traj_prv = static_cast<double>(TaskSpace_traj.t_k[jj-1]);
          }
        #endif
        //########################################################################################################
        //############ check if there are more valid elements withing the trajectory message #####################
        //############ if the time value of the previous element is smaller or equal with the current value, break, number of valid data points are jj-1//##############################################
        //##########################################################################################################################################
        if(t_traj_jj < 0 || std::isnan(t_traj_jj)){
          break;
        }
        if(jj > 0 ){
          if (t_traj_prv >= t_traj_jj){
            break;
          }
        }
        //##############################################
        //##############################################
        t_vec(jj,0) = t_now + (t_traj_jj -  t_traj_0);
        Pose_t Pose_jj= TaskSpace_traj.PoseArray[jj];
        #if defined(WITH_EXCEPTIONS)
          if (Pose_jj.position.size()  != 3){
            throw std::runtime_error("TaskSpace_trajectoryCallback wrong dim. of Pose Array position");
          }
        #endif
        for(uint8_t kk = 0; kk < 3; kk ++){
          if(kk < point_ctrl_pts.rows()){
            point_ctrl_pts(kk, jj) = Pose_jj.position[kk];
          }else{
            #ifdef FLAG_ROS_COMPILE
              ACIN_CTRL_WARN("point traj interpolator, has not the dim. of a point in the trajectory package");
            #endif
            break;
          }
        }

        Eigen::Vector3d Euler_jj;
        Eigen::Quaternion<double> quat_jj;
        if(!this->Euler_interpl_flag_){
          Eigen::Matrix<double, 4, 1> quat_mat;
          for(uint8_t kk = 0; kk < 4; kk ++){
            quat_mat[kk] =  Pose_jj.orientation[kk]; //(qw,qx,qy,qz)
          }
          quat_mat = quat_mat.normalized();
          quat_jj = Eigen::Quaternion<double>(quat_mat[0], quat_mat[1], quat_mat[2], quat_mat[3]); //(w,x,y,z)
          Euler_jj = quat_jj.toRotationMatrix().eulerAngles(2, 1, 0); //(Yaw, Pitch, Roll)
        }else{
          Euler_jj[0] = Pose_jj.orientation[0]; //Yaw
          Euler_jj[1] = Pose_jj.orientation[1]; //Pitch
          Euler_jj[2] = Pose_jj.orientation[2]; //Roll
          Eigen::Matrix3d R_jj;
          R_jj = Eigen::AngleAxisd(Euler_jj(0), Eigen::Vector3d::UnitZ())
                * Eigen::AngleAxisd(Euler_jj(1),  Eigen::Vector3d::UnitY())
                * Eigen::AngleAxisd(Euler_jj(2), Eigen::Vector3d::UnitX());
          quat_jj = Eigen::Quaternion<double>(R_jj);
        }

        euler_ctrl_pts.col(jj) = Euler_jj;
        quat_ctrl_pts.col(jj) = quat_jj.coeffs();
        valid_traj_pts++;

      }
      if(valid_traj_pts <2 ){//min one point set in the trajectory
        ACIN_CTRL_ERROR("Drop JointSpaceTraj message, curve dosnt have atleast two point");
        return;
      }
      for(uint32_t jj = 1; jj< valid_traj_pts; jj++){
        if (t_vec(jj - 1,0) >= t_vec(jj,0)){
          ACIN_CTRL_ERROR("< > Drop message, curve parameter is not strictly increasing, must be ensured by the user");
          ACIN_CTRL_ERROR("parameter vector t[%u] >= t[%u]",jj-1,jj);
          return;
        }
      }
      double t_last = t_vec(valid_traj_pts - 1,0);
      double T_last = (t_vec(valid_traj_pts - 1,0) - t_vec(valid_traj_pts - 2,0));
      if (T_last < this->Ta_){
        T_last = this->Ta_;
      }
      uint32_t N_preCurve =  p_PreCurve_.pts.cols();
      for(uint32_t jj = 0; jj< N_preCurve; jj++){
        p_PreCurve_.para(jj) = t_now - T_last*(N_preCurve-jj);
        euler_PreCurve_.para(jj) = p_PreCurve_.para(jj);
        qaut_PreCurve_.para(jj) = p_PreCurve_.para(jj);
      }
      for(uint32_t jj = valid_traj_pts; jj< point_ctrl_pts.cols(); jj++){
        //if not all point of the intepolator curve is set, clamp it to the last value
        t_vec(jj,0) = t_last + (jj - valid_traj_pts + 1)*T_last;
        point_ctrl_pts.col(jj) = point_ctrl_pts.col(valid_traj_pts-1);
        euler_ctrl_pts.col(jj) = euler_ctrl_pts.col(valid_traj_pts-1);
        quat_ctrl_pts.col(jj) = quat_ctrl_pts.col(valid_traj_pts-1);
      }
      HomogeneousTransformation_t H_B_E_jj;
      evalT0E(q_PreCurve_.pts.col(0), H_B_E_jj, robot_params_);
      Eigen::Vector3d p_jj = H_B_E_jj.block<3,1>(0,3);
      //ACIN_CTRL_WARN("Setup Taskspace Trajectory Curve interpolator");
      T_traj_end_ = t_vec(t_vec.rows() - 1, 0);
      typename spline_Interpolator_t::curve_t point_curve;
      point_curve.pts = point_ctrl_pts;
      point_curve.para = t_vec;
      point_interpolator_sptr_->set_curve(point_curve, p_PreCurve_, true, false, false);
      if(this->Euler_interpl_flag_){
        typename spline_Interpolator_t::curve_t Euler_curve;
        Euler_curve.pts = euler_ctrl_pts;
        Euler_curve.para = t_vec;
        euler_interpolator_sptr_->set_curve(Euler_curve, euler_PreCurve_, true, false, false);
      }else{
        typename  quaternion_Interpolator_t::curve_t quat_curve;
        quat_curve.pts = quat_ctrl_pts;
        quat_curve.para = t_vec;
        quat_interpolator_sptr_->set_curve(quat_curve, qaut_PreCurve_, true, false, false);
      }
      #ifdef FLAG_ROS_COMPILE
        std::cout<<"trajecotry len:= " << valid_traj_pts << std::endl;
        std::cout<<"max trajecotry len:= " << euler_ctrl_pts.cols() << std::endl;
        std::cout<<"t_now:=" << t_now << std::endl;
        std::cout<<"Curve start:="<<std::endl;
        std::cout<<"\tt_vec[0]:=" << t_vec(0) << std::endl;
        std::cout<<"\tp_soll start "<< point_ctrl_pts.col(0).transpose() <<std::endl;
        std::cout<<"\teuler_soll start "<< euler_ctrl_pts.col(0).transpose() <<std::endl;
        std::cout<<"Curve end:="<<std::endl;
        std::cout<<"\tt_vec[t_vec.rows()-1]:=" << t_vec(t_vec.rows()-1,0) << std::endl;
        std::cout<<"\tp_soll end "<< point_ctrl_pts.col(t_vec.rows()-1).transpose() <<std::endl;
        std::cout<<"\teuler_soll end "<< point_ctrl_pts.col(t_vec.rows()-1).transpose() <<std::endl;
        std::cout<<"\tp_preCurve "<< p_PreCurve_.pts.col(p_PreCurve_.pts.cols()-1).transpose() <<std::endl;
        std::cout<<"\teuler_pre_curve "<< euler_PreCurve_.pts.col(p_PreCurve_.pts.cols()-1).transpose() <<std::endl;
      #endif
    }
  }




private:
  void join_space_pt_pt_(const Eigen::Ref<JointVector_t> q_E, double T_end, double acc_perc = 0.2){

    uint16_t N_knots_acc = (uint16_t)((double)N_knots_*acc_perc/2.0);
    double t_now = get_cur_time();
    ACIN_CTRL_INFO("new joint space end point given!");
    ACIN_CTRL_INFO("%f; %f; %f; %f; %f;  %f; %f; %f; %f", q_(0),q_(1),q_(2),q_(3),q_(4),q_(5),q_(6),q_(7),q_(8));
    ACIN_CTRL_INFO("%f; %f; %f; %f; %f;  %f; %f, %f, %f", q_E(0),q_E(1),q_E(2),q_E(3),q_E(4),q_E(5),q_E(6),q_E(7),q_E(8));
    ACIN_CTRL_INFO("t_now: = %f", t_now);
    ACIN_CTRL_INFO("T_end: = %f", T_end);
    ACIN_CTRL_INFO("N_knots_acc: = %lu", (unsigned long)N_knots_acc);
    double Ta_Curve = T_end/N_knots_;
    //double Ta_Curve = this->Ta_*10;
    uint32_t N_PreCurve = q_PreCurve_.pts.cols();
    for (uint32_t ii = 0; ii < N_PreCurve; ii++){
      q_PreCurve_.para(ii) = t_now - Ta_Curve*(N_PreCurve - ii);
    }
    joint_interpolator_sptr_->profiled_interpolation(q_, q_E, q_PreCurve_, t_now, T_end, N_knots_acc);
    T_traj_end_ = t_now + T_end;
    Joint_interpl_flag_ = true;
  }
  void joint_space_adjustment_(const CoordAdj_t &coord_adj){
    #if defined(FLAG_ROS_COMPILE)
    std::unique_lock<std::timed_mutex> lock{ACIN_CTRL_mutex_,  std::defer_lock};
    lock.lock();
    #endif
    uint64_t joint_idx = coord_adj.coord_idx;
    double adj_val = coord_adj.adj_val;
    double T_end = coord_adj.T_traj; //time to reach the new steady state point
    double acc_perc = coord_adj.acc_perc; //percentage of the trajectory which is used for acceleration and deceleration
    if (joint_idx < 0 || joint_idx >=_ROBOT_q_DIM_){
      ACIN_CTRL_WARN("WRONG joint coordinate (%lu)to increment", joint_idx);
      return;
    }else{
      ACIN_CTRL_WARN("adjust Joint %lu position by %f!", joint_idx, adj_val);
    }
    //TODO better limit to joint velocity
    //TODO seceond spline interplolator for checking the avaiable of the trajetory ?
    if(T_end < this->T_end_lb_ && this->T_end_lb_ >0){
       T_end = this->T_end_lb_;
    }

    if (acc_perc < accPerc_lb_limit_ && accPerc_lb_limit_ > 0) {
        acc_perc = accPerc_lb_limit_;
    }
    //the acceleration and deacceleration secection  can be max. 95% precentage of the total time
    if (acc_perc > 0.95) {
        acc_perc = 0.95;
    }
    JointVector_t qE = q_;
    double joint_Limit_safety_margin = 0.05;
    double q_limit_ub = robot_params_.q_limit_upper[joint_idx]*(1 - joint_Limit_safety_margin);
    double q_limit_lb = robot_params_.q_limit_lower[joint_idx]*(1 - joint_Limit_safety_margin);
    qE(joint_idx) = qE(joint_idx) + adj_val;
    if (qE(joint_idx)  > q_limit_ub){
      qE(joint_idx) = q_limit_ub;
    }else{
      if(qE(joint_idx) < q_limit_lb){
        qE(joint_idx) = q_limit_lb;
      }
    }
    join_space_pt_pt_(qE, T_end, acc_perc);
    switch_ = true;
  }
  void jointSpace_SollPos_(const acinSollPosMsg_t &sollPosMsg){
    #if defined(FLAG_ROS_COMPILE)
    std::unique_lock<std::timed_mutex> lock{ACIN_CTRL_mutex_,  std::defer_lock};
    lock.lock();
    #endif
    double T_end = sollPosMsg.T_traj;
    double acc_perc = sollPosMsg.acc_perc;
    if(T_end < T_end_lb_ && T_end_lb_ > 0){
      T_end = T_end_lb_;
    }
    if(acc_perc < accPerc_lb_limit_ && accPerc_lb_limit_ > 0){
      acc_perc = accPerc_lb_limit_;
    }
    //the acceleration and deacceleration secection  can be max. 95% precentage of the total time
    if (acc_perc > 0.95) {
        acc_perc = 0.95;
    }
    JointVector_t qE = JointVector_t::Zero();
    double joint_Limit_safety_margin = 0.05;
    for(uint16_t ii = 0; ii<_ROBOT_q_DIM_; ii++){
      if (sollPosMsg.coord[ii] != 0 ){
        if (sollPosMsg.sollPos[ii] > robot_params_.q_limit_upper[ii] * (1-joint_Limit_safety_margin)){
          qE[ii] = robot_params_.q_limit_upper[ii] - robot_params_.q_limit_upper[ii]*joint_Limit_safety_margin;
        }else{
          if(sollPosMsg.sollPos[ii] < robot_params_.q_limit_lower[ii]*(1-joint_Limit_safety_margin)){
            qE[ii] = robot_params_.q_limit_lower[ii] - robot_params_.q_limit_lower[ii]*joint_Limit_safety_margin;
          }else{
            qE[ii] = sollPosMsg.sollPos[ii];
          }
        }
      }else{
        qE[ii] = q_[ii];
      }
    }
    join_space_pt_pt_(qE, T_end, acc_perc);
  }
  void task_space_pt_pt_(CartesianVector_t y_E, double T_end, double acc_perc = 0.2){
    uint16_t N_knots_acc = (uint16_t)((double)N_knots_*acc_perc/2.0);
    Eigen::Vector3d p_0 = pose_.block<3,1>(0,0);
    Eigen::Vector3d p_E = y_E.block<3,1>(0,0);

    Eigen::Quaternion<double> quat_0 = Eigen::Quaternion<double>(pose_[3], pose_[4], pose_[5], pose_[6]); //(w,x,y,z);
    Eigen::Vector4d quat_0_knots;
    quat_0_knots << pose_[3], pose_[4], pose_[5], pose_[6];

    Eigen::Vector3d Euler_0;
    Euler_0 = quat_0.toRotationMatrix().eulerAngles(2, 1, 0); //(Yaw, Pitch, Roll)
    Eigen::Vector3d Euler_E = y_E.block<3,1>(3,0);
    RotationMatrix_t R_z_E = RotationMatrix_t(Eigen::AngleAxisd(y_E(3,0), Eigen::Vector3d::UnitZ()).toRotationMatrix());
    RotationMatrix_t R_y_E = RotationMatrix_t(Eigen::AngleAxisd(y_E(4,0),  Eigen::Vector3d::UnitY()).toRotationMatrix());
    RotationMatrix_t R_x_E = RotationMatrix_t(Eigen::AngleAxisd(y_E(5,0), Eigen::Vector3d::UnitX()).toRotationMatrix());
    RotationMatrix_t R_d_E = R_z_E*R_y_E*R_x_E;
    Eigen::Quaternion<double> quat_E =  Eigen::Quaternion<double>(R_d_E);
    Eigen::Vector4d quat_E_knots;
    quat_E_knots << quat_E.w(), quat_E.x(), quat_E.y(), quat_E.z();
    double t_now = get_cur_time();
    //double Ta_curve = this->Ta_*10;
    double Ta_curve = T_end/N_knots_;
    uint64_t N_PreCurve = p_PreCurve_.pts.cols();
    for (uint32_t ii = 0; ii < N_PreCurve; ii++){
      p_PreCurve_.para(ii) = t_now - Ta_curve*(N_PreCurve - ii);
      euler_PreCurve_.para(ii) = p_PreCurve_.para(ii);
      qaut_PreCurve_.para(ii) = p_PreCurve_.para(ii);
    }
    point_interpolator_sptr_->profiled_interpolation(p_0, p_E, p_PreCurve_, t_now, T_end, N_knots_acc);
    euler_interpolator_sptr_->profiled_interpolation(Euler_0, Euler_E, euler_PreCurve_, t_now, T_end, N_knots_acc);
    quat_interpolator_sptr_->profiled_interpolation(quat_0_knots, quat_E_knots, qaut_PreCurve_, t_now, T_end, N_knots_acc);

    //get_euler_curve_manipulability(q_, point_interpolator_sptr_, euler_interpolator_sptr_);
    T_traj_end_ = t_now + T_end;
    Joint_interpl_flag_ = false;
    this->Euler_interpl_flag_ = true;
  }

  void taskSpace_SollPos_(const acinSollPosMsg_t &sollPosMsg){
    #if defined(FLAG_ROS_COMPILE)
      std::unique_lock<std::timed_mutex> lock{ACIN_CTRL_mutex_,  std::defer_lock};
      lock.lock();
    #endif
	  ACIN_CTRL_ALWAYS("taskSpace_SollPos_");
    double T_end = sollPosMsg.T_traj;
    double acc_perc = sollPosMsg.acc_perc;
    if(T_end < T_end_lb_ && T_end_lb_> 0){
      ACIN_CTRL_WARN("LIMIT desired end time is set to %.4f second, for safety, can be set in the instantation of the controller", T_end_lb_);
      T_end = T_end_lb_;
    }
    if (acc_perc < accPerc_lb_limit_ && accPerc_lb_limit_ > 0) {
        acc_perc = accPerc_lb_limit_;
    }
    //the acceleration and deacceleration secection  can be max. 95% precentage of the total time
    if (acc_perc > 0.95) {
        acc_perc = 0.95;
    }

    Eigen::Quaternion<double> quat_0 = Eigen::Quaternion<double>(pose_[3], pose_[4], pose_[5], pose_[6]); //(w,x,y,z);
    Eigen::Vector3d Euler_0;
    Euler_0 = quat_0.toRotationMatrix().eulerAngles(2, 1, 0); //(Yaw, Pitch, Roll)
    //TODO better limit joint velocity
    //TODO seceond spline interplolator for checking the avaiable of the trajetory ?
    //TODO limit of the joints, through inverse kinematic!
    HomogeneousTransformation_t H_B_E_loc;
    evalT0E(q_, H_B_E_loc, robot_params_);
    PoseVector_t pose_a;
    pose_a.block<3,1>(0,0) = H_B_E_loc.block<3,1>(0,3);
    Eigen::Quaternion<double> quat_actual(H_B_E_loc.block<3,3>(0,0));
    pose_a(3,0) = quat_actual.w();
    pose_a(4,0) = quat_actual.x();
    pose_a(5,0) = quat_actual.y();
    pose_a(6,0) = quat_actual.z();
    ACIN_CTRL_WARN("profiled taskspace interpolation");
    //temp<<"pose_a:= "<<pose_a;
    //ACIN_CTRL_WARN_STREAM(temp);
    CartesianVector_t y0 = CartesianVector_t::Zero();
    y0.block<3,1>(0,0) = pose_.block<3,1>(0,0);
    y0.block<3,1>(3,0) = Euler_0;

    CartesianVector_t yE = CartesianVector_t::Zero();
    for(uint16_t ii = 0; ii < 6; ii++){
      if(sollPosMsg.coord[ii] !=0){
        yE[ii] = sollPosMsg.sollPos[ii];
      }else{
        yE[ii] = y0[ii];
      }
    }
    ACIN_CTRL_ERROR("set taskSpace soll pos. not implemented, additional safety things should be implemented");
    task_space_pt_pt_(yE, T_end, acc_perc);
    return;

  }

  void task_space_adjustment_(const CoordAdj_t &coord_adj){
    #if defined(FLAG_ROS_COMPILE)
      std::unique_lock<std::timed_mutex> lock{ACIN_CTRL_mutex_,  std::defer_lock};
      lock.lock();
    #endif
    uint64_t TS_coord_idx = coord_adj.coord_idx;
    double adj_val = coord_adj.adj_val;
    double T_end = coord_adj.T_traj;
    double acc_perc = coord_adj.acc_perc;
    if(T_end < T_end_lb_ && T_end_lb_ != 0){
      ACIN_CTRL_WARN("<task_space_adjustment> LIMIT desired end time is set to %.4f second, for safety, can be set in the instantation of the controller", T_end_lb_);
      T_end = T_end_lb_;
    }
    if (TS_coord_idx >= 0 && TS_coord_idx < 3){
      if(std::abs(adj_val) > TaskSpace_PosAdj_ub_limit_){
        ACIN_CTRL_WARN("<task_space_adjustment> LIMIT desired task space adjust value to %.4f m, for safety purpose, can be set in the instantation of the controller", TaskSpace_PosAdj_ub_limit_);
        if(adj_val > 0){
          adj_val = TaskSpace_PosAdj_ub_limit_;
        }else{
          adj_val = - TaskSpace_PosAdj_ub_limit_;
        }
      }
    }else{
      if(std::abs(adj_val)*180/M_PI > AngleAdj_ub_limit_){//not greater than 5 degree
        ACIN_CTRL_WARN("LIMIT desired task space adjust value angle to %.4f m, for safety purpose, can be set in the instantation of the controller", AngleAdj_ub_limit_);
        if(adj_val > 0){
          adj_val = AngleAdj_ub_limit_*M_PI/180;
        }else{
          adj_val =- AngleAdj_ub_limit_*M_PI/180;
        }
      }
    }
    if (acc_perc < accPerc_lb_limit_ && accPerc_lb_limit_ > 0) {
        acc_perc = accPerc_lb_limit_;
    }
  //the acceleration and deacceleration secection  can be max. 95% precentage of the total time
    if (acc_perc > .95){
        acc_perc = 0.95;
    }
	  ACIN_CTRL_ALWAYS("task_space_adjustment_");
    if (TS_coord_idx < 0 || TS_coord_idx >= 6){
      ACIN_CTRL_WARN("WRONG task coord to increment, possible coord indices: (0):\"x\",(1): \"y\" (2): \"z\" (3): \"YAW\" (4): \"PITCH\" (5): \"ROLL\" ");
      return;
    }else{
      ACIN_CTRL_WARN("adjust task space position %lu by %f!", TS_coord_idx, adj_val);
    }
    //std::ostringstream temp;

    Eigen::Quaternion<double> quat_0 = Eigen::Quaternion<double>(pose_[3], pose_[4], pose_[5], pose_[6]); //(w,x,y,z);
    Eigen::Vector3d Euler_0;
    Euler_0 = quat_0.toRotationMatrix().eulerAngles(2, 1, 0); //(Yaw, Pitch, Roll)()
    //TODO better limit joint velocity
    //TODO seceond spline interplolator for checking the avaiable of the trajetory ?
    //TODO limit of the joints, through inverse kinematic!
    HomogeneousTransformation_t H_B_E_loc;
    evalT0E(q_, H_B_E_loc, robot_params_);
    PoseVector_t pose_a;
    pose_a.block<3,1>(0,0) = H_B_E_loc.block<3,1>(0,3);
    Eigen::Quaternion<double> quat_actual(H_B_E_loc.block<3,3>(0,0));
    pose_a(3,0) = quat_actual.w();
    pose_a(4,0) = quat_actual.x();
    pose_a(5,0) = quat_actual.y();
    pose_a(6,0) = quat_actual.z();
    ACIN_CTRL_WARN("profiled taskspace interpolation");
    //temp<<"pose_a:= "<<pose_a;
    //ACIN_CTRL_WARN_STREAM(temp);
    CartesianVector_t yE = CartesianVector_t::Zero();
    yE.block<3,1>(0,0) = pose_.block<3,1>(0,0);
    yE.block<3,1>(3,0) = Euler_0;
    ACIN_CTRL_ALWAYS("x(0):=%f, y(0):=%f, z(0):=%f", yE(0, 0), yE(1, 0), yE(2, 0));
    ACIN_CTRL_ALWAYS("Y(0):=%f, P(0):=%f, R(0):=%f", yE(3, 0)*180/M_PI, yE(4, 0) * 180 / M_PI, yE(5, 0) * 180 / M_PI);
    //temp<<"y0:= "<<yE;
    //ACIN_CTRL_WARN_STREAM(temp);
    yE(TS_coord_idx) = yE(TS_coord_idx) + adj_val;
	  ACIN_CTRL_ALWAYS("x(T):=%f, y(T):=%f, z(T):=%f", yE(0, 0), yE(1, 0), yE(2, 0));
    ACIN_CTRL_ALWAYS("Y(T):=%f, P(T):=%f, R(T):=%f", yE(3, 0) * 180 / M_PI, yE(4, 0) * 180 / M_PI, yE(5, 0) * 180 / M_PI);
    //temp<<"yE:= "<<yE;
    //ACIN_CTRL_WARN_STREAM(temp);
    task_space_pt_pt_(yE, T_end);
    switch_ = true;
  }

  void get_TaskSpace_preCurve( typename spline_Interpolator_t::pre_curve_t &p_PreCurve,
                          typename spline_Interpolator_t::pre_curve_t &euler_PreCurve,
                          typename quaternion_Interpolator_t::pre_curve_t &qaut_PerCurve){
    ACIN_CTRL_ASSERT(p_PreCurve.pts.cols() ==euler_PreCurve.pts.cols() );
    ACIN_CTRL_ASSERT(qaut_PerCurve.pts.cols() == euler_PreCurve.pts.cols() );
    double t_now = get_cur_time(timeBase_setting);
    double Ta = 0.0;
    uint64_t NpreCurve = p_PreCurve.pts.cols();
    for(uint16_t jj = 1; jj < NpreCurve; jj++){
      Ta += (q_PreCurve_.para(jj) - q_PreCurve_.para(jj-1));
    }
    Ta = Ta/(NpreCurve-1);
    for(uint16_t jj = 0; jj < NpreCurve; jj++){
      HomogeneousTransformation_t H_B_E_jj;
      int64_t q_preCurve_idx_ = (int64_t)q_PreCurve_.pts.cols() - (int64_t)p_PreCurve.pts.cols() + (int64_t)jj;
      if (q_preCurve_idx_ >= 0){
        evalT0E(q_PreCurve_.pts.col(jj), H_B_E_jj, robot_params_);
      }else{
        evalT0E(q_PreCurve_.pts.col(0), H_B_E_jj, robot_params_);
      }
      Eigen::Vector3d p_jj = H_B_E_jj.block<3,1>(0,3);
      p_PreCurve.pts.col(jj) = p_jj;

      p_PreCurve.para(jj) = t_now + Ta*(jj-N_PreCurve);

      RotationMatrix_t R_jj = H_B_E_jj.block<3,3>(0,0);
      Eigen::Vector3d Euler_jj = R_jj.eulerAngles(2,1,0); //YPR
      euler_PreCurve.pts.col(jj) = Euler_jj;
      euler_PreCurve.para(jj) = p_PreCurve.para(jj);
      Eigen::Quaternion<double> qaut_jj = Eigen::Quaternion<double>(R_jj);
      qaut_PerCurve.pts.col(jj) = qaut_jj.coeffs();
      qaut_PerCurve.para(jj) = p_PreCurve.para(jj);
    }
  }
  void dispatchDesired()
  {
    double t_norm = 0.0;
    //static ProMPInterpolator_C::TrajPoint_t ProMPtraj_offset(6,1);
    /*set actuel pos of the end effector*/
    Eigen::Vector3d p = H_B_E_.block<3,1>(0,3);
    RotationMatrix_t R = H_B_E_.block<3,3>(0,0);
    Eigen::Vector3d Euler_ist = R.eulerAngles(2,1,0); //YPR
    Eigen::Quaternion<double> quat_ist(R);
    //
    double t_now = get_cur_time(timeBase_setting);
    /**************************************************************************/
    /**************************************************************************/
    #if defined(ACIN_CONTROLLER_DEBUG)
      timespec t1;
      timespec t2;
      clock_gettime(CLOCK_REALTIME,&t1);
    #endif
    if(!Joint_interpl_flag_){
      pub_spline_traj(t_now);
      #if defined(ACIN_CONTROLLER_DEBUG)
        clock_gettime(CLOCK_REALTIME,&t2);
        timespec_diff(t2,t1,t_TS_intpl_);
      #endif
    }else{
      pub_joint_spline_traj(t_now);
      #if defined(ACIN_CONTROLLER_DEBUG)
        clock_gettime(CLOCK_REALTIME,&t2);
        timespec_diff(t2,t1,t_QS_intpl_);
      #endif
    }
    //TODO
    //accel_actual_ = CartesianVector_t::Zero();// Jd*q_dot_ +  J*q_dot_dot_;
#ifdef ACIN_CONTROLLER_DEBUG_MSG
    if(switch_){
      JointVector_t e_q_loc = (q_ - q_d_);
      PoseVector_t e_pose_loc = pose_ - pose_d_;
      ACIN_CTRL_INFO("-------------------------------------------------------------------------------------");
      ACIN_CTRL_INFO("new trajectory to dispatch");
      ACIN_CTRL_INFO("Task Space trajectory := %lu",(uint64_t)(!Joint_interpl_flag_));
      double t_0 = point_interpolator_sptr_->get_t_0();
      double t_end = point_interpolator_sptr_->get_t_end();
      ACIN_CTRL_INFO("t_now := %f, t_0:=%f, t_end:= %f, T length:= %f", t_now, t_0, t_end, t_end - t_0);
      ACIN_CTRL_INFO("---Joint Space---");
      ACIN_CTRL_INFO("e_q_loc:= %f, e_q_loc:= %f,e_q_loc:= %f,e_q_loc:= %f, e_q_loc:= %f,e_q_loc:= %f, e_q_loc:= %f",
                e_q_loc(0,0),e_q_loc(1,0),e_q_loc(2,0),e_q_loc(3,0),e_q_loc(4,0),e_q_loc(5,0), e_q_loc(6,0));
      ACIN_CTRL_INFO("e_q_ (0):= %f, e_q_ (1)::= %f,e_q_ (2)::= %f,e_q_ (3)::= %f, e_q_ (4)::= %f,e_q_ (5)::= %f, e_q_ (6)::= %f\n\n",
                qDot_d_(0,0),qDot_d_(1,0),qDot_d_(2,0),qDot_d_(3,0),qDot_d_(4,0),qDot_d_(5,0), qDot_d_(6,0));
      ACIN_CTRL_INFO("qDotDot_d_:= %f, qDotDot_d_:= %f,qDotDot_d_:= %f,qDotDot_d_:= %f, qDotDot_d_:= %f,qDotDot_d_:= %f, qDotDot_d_:= %f\n\n",
                qDotDot_d_(0,0),qDotDot_d_(1,0),qDotDot_d_(2,0),qDotDot_d_(3,0),qDotDot_d_(4,0),qDotDot_d_(5,0), qDotDot_d_(6,0));
      ACIN_CTRL_INFO("---Task Space---");
      typename spline_Interpolator_t::ctrlpt_t point_0 = point_interpolator_sptr_->get_ctrlpt_0();
      typename spline_Interpolator_t::ctrlpt_t point_end = point_interpolator_sptr_->get_ctrlpt_end();
      ACIN_CTRL_INFO("ctrl p[0](0):= %f, ctrl p[0](1):= %f, ctrl p[0](2):= %f",
                point_0(0,0),point_0(1,0),point_0(2,0));
      ACIN_CTRL_INFO("ctrl p[end](0):= %f, ctrl p[end](1):= %f,ctrl p[end](2):= %f",
                      point_end(0,0),point_end(1,0),point_end(2,0));
      ACIN_CTRL_INFO("e_pose_(0):= %f, e_pose_(1):= %f,e_pose_(2):= %f,e_pose_(3) [q_w]:= %f, e_pose_(4)[q_x]:= %f,e_pose_(5)[q_y]:= %f, e_pose_(6)[q_z]:= %f",
                e_pose_loc(0,0),e_pose_loc(1,0),e_pose_loc(2,0),e_pose_loc(3,0),e_pose_loc(4,0),e_pose_loc(5,0), e_pose_loc(6,0));
      ACIN_CTRL_INFO("pDotDot_d_:= %f, pDotDot_d_:= %f,pDotDot_d_:= %f",
                pDotDot_d_(0,0),pDotDot_d_(1,0),pDotDot_d_(2,0));
      ACIN_CTRL_INFO("omegaDot_d_:= %f, omegaDot_d_:= %f,omegaDot_d_:= %f",
                omegaDot_d_(0,0),omegaDot_d_(1,0),omegaDot_d_(2,0));
      ACIN_CTRL_INFO("-------------------------------------------------------------------------------------");
      switch_ = false;
    }
#endif
  }

  void set_dest_q(const Eigen::Ref<JointVector_t> q_d){
    q_d_ = q_d;
  }
  void set_dest_qp(const Eigen::Ref<JointVector_t> qDot_d){
    qDot_d_ = qDot_d;
  }
  void set_dest_qpp(const Eigen::Ref<JointVector_t> qDotDot_d){
    qDotDot_d_ = qDotDot_d;
  }
  void set_dest_cart_pose(const Eigen::Ref<PoseVector_t> pose_d){
    pose_d_ = pose_d;
  }
  void set_dest_cart_vel(const Eigen::Ref<CartesianVector_t> cart_vel_d){
    pDot_d_ = cart_vel_d.block<3,1>(0,0);
    omega_d_ = cart_vel_d.block<3,1>(3,0);
  }
  void set_dest_cart_accel(const Eigen::Ref<CartesianVector_t> cart_accel_d){
   pDotDot_d_ = cart_accel_d.block<3,1>(0,0);
   omegaDot_d_ = cart_accel_d.block<3,1>(3,0);
  }
  void set_dest_q_nullspace(const Eigen::Ref<JointVector_t> q_d_NS){
    q_d_NS_ = q_d_NS;
  }
  /*taskspace spline trajectory interpolation*/
  void pub_spline_traj(double t_now){
    double t_loc = t_now;
    std::vector<uint8_t> dd_vec(3);
    dd_vec[0] = 0;
    dd_vec[1] = 1;
    dd_vec[2] = 2;
    typename spline_Interpolator_t::eval_t point_eval; // matrix, evlulation of the spline at mutiple time stemps possible, rows define the time (x knots), col value at the evaluation knot, desiresd derivatives
    ACIN_CTRL_ASSERT(point_eval.rows()==1);
    ACIN_CTRL_ASSERT(point_eval.cols()==_TaskSpaceInterpolator_deg_ + 1);
    /*evaluate the spline*/
    std::vector<double> t_tmp_vec(1);
    t_tmp_vec[0] = t_loc;
    point_interpolator_sptr_->evaluate(point_eval, t_tmp_vec , dd_vec);
    /****************************************/
    /*set cartesian pose "pose_d" from spline evaluation*/
    /****************************************/
    typename spline_Interpolator_t::ctrlpt_t p_spline = point_eval(0, 0);
    PoseVector_t pose_d;
    pose_d.block<3,1>(0,0) = p_spline;

    /* first derivative of interpolation piece wise spline*/
    /* Set velocities cart_vel_d from spline evaluation*/
    /* ------------------------------- */
    CartesianVector_t cart_vel_d;
    typename spline_Interpolator_t::ctrlpt_t pDot_d_spline = point_eval(0, 1);
    cart_vel_d.block<3,1>(0,0) = pDot_d_spline;
    /* ------------------------------- */
    /* seceond derivative of interpolation piece wise spline*/
    /* Set accelerations*/
    CartesianVector_t cart_accel_d;
    typename spline_Interpolator_t::ctrlpt_t pDotDot_d_spline = point_eval(0, 2);
    cart_accel_d.block<3,1>(0,0) = pDotDot_d_spline;

    if(this->Euler_interpl_flag_){
      typename spline_Interpolator_t::eval_t euler_eval; // matrix, evlulation of the spline at mutiple time stemps possible, rows define the time (x knots), col value at the evaluation knot, desiresd derivatives
      ACIN_CTRL_ASSERT(euler_eval.rows()==1);
      ACIN_CTRL_ASSERT(euler_eval.cols()==_TaskSpaceInterpolator_deg_+1);
      euler_interpolator_sptr_->evaluate(euler_eval, t_tmp_vec , dd_vec);
      typename spline_Interpolator_t::ctrlpt_t Euler_spline;
      Eigen::Quaternion<double> quat_0 = Eigen::Quaternion<double>(pose_[3], pose_[4], pose_[5], pose_[6]);
      typename spline_Interpolator_t::ctrlpt_t Euler_0 = quat_0.toRotationMatrix().eulerAngles(2, 1, 0);
      Euler_spline = Euler_0;
      //Euler_spline = euler_eval(0, 0);
      Eigen::Matrix<double,3,3> R_euler_spline;
      R_euler_spline = Eigen::AngleAxisd(Euler_spline(0), Eigen::Vector3d::UnitZ())
            * Eigen::AngleAxisd(Euler_spline(1),  Eigen::Vector3d::UnitY())
            * Eigen::AngleAxisd(Euler_spline(2), Eigen::Vector3d::UnitX());
      Eigen::Quaternion<double> quat_spline(R_euler_spline);

      pose_d(3,0) = quat_spline.w();
      pose_d(4,0) = quat_spline.x();
      pose_d(5,0) = quat_spline.y();
      pose_d(6,0) = quat_spline.z();
      // Eigen::Vector3d pDot_d_spline = pd_interp;
      /***

      */
      Eigen::Vector3d omega_d_spline;
      Eigen::Vector3d EulerDot = euler_eval(0, 1);
      eulerDot2omega (Euler_spline, EulerDot, omega_d_spline);
      cart_vel_d.block<3,1>(3,0) = omega_d_spline;
      /****************************************/
      // Eigen::Vector3d pDotDot_d_spline = pdd_interp;
      /***

      */
      Eigen::Vector3d omegaDot_d_spline;
      Eigen::Vector3d EulerDotDot = euler_eval(0,2);
      eulerDotDot2omegaDot (Euler_spline, EulerDot, EulerDotDot, omegaDot_d_spline);
      cart_accel_d.block<3,1>(3,0) = omegaDot_d_spline;
    }else{
      typename quaternion_Interpolator_t::eval_t quat_eval; // matrix, evlulation of the spline at mutiple time stemps possible, rows define the time (x knots), col value at the evaluation knot, desiresd derivatives
      ACIN_CTRL_ASSERT(quat_eval.rows()==1);
      ACIN_CTRL_ASSERT(quat_eval.cols()==_TaskSpaceInterpolator_deg_+1);
      quat_interpolator_sptr_->evaluate(quat_eval, t_tmp_vec , dd_vec);
      typename quaternion_Interpolator_t::ctrlpt_t qaut_eval = quat_eval(0,0);
      Eigen::Quaternion<double> quat_interpl(qaut_eval);
      Eigen::Matrix<double,3,3> R_quat_interpl;
      R_quat_interpl = quat_interpl.toRotationMatrix();
      Vector3d Euler_qaut_interpl =  R_quat_interpl.eulerAngles(2, 1, 0);

      pose_d(3,0) = quat_interpl.w();
      pose_d(4,0) = quat_interpl.x();
      pose_d(5,0) = quat_interpl.y();
      pose_d(6,0) = quat_interpl.z();
      /***

      */
      Eigen::Vector3d omega_d_QautSpline;
      Eigen::Vector4d quatDot = quat_eval(0, 1);
      qautDot2omega(quat_eval(0,0), quatDot, omega_d_QautSpline);
      cart_vel_d.block<3,1>(3,0) = omega_d_QautSpline;
      Eigen::Vector3d omegaDot_d_Qautspline;
      Eigen::Vector4d quatDotDot = quat_eval(0, 2);
      qautDotDot2omegaDot (qaut_eval, quatDot, quatDotDot, omegaDot_d_Qautspline);
      cart_accel_d.block<3,1>(3,0) = omegaDot_d_Qautspline;
    }
    set_dest_cart_pose(pose_d);
    set_dest_cart_vel(cart_vel_d);
    set_dest_cart_accel(cart_accel_d);

  }
  void pub_joint_spline_traj(double t_now){
    double t_loc = t_now;
    std::vector<uint8_t> dd_vec(3);
    dd_vec[0] = 0;
    dd_vec[1] = 1;
    dd_vec[2] = 2;
    //3 is because of the degree of the spline polynomial
    typename joint_Interpolator_t::eval_t joint_eval; // matrix, evlulation of the spline at mutiple time N_knots_ possible, rows define the time (x knots), col value at the evaluation knot, desiresd derivatives
    ACIN_CTRL_ASSERT(joint_eval.cols() == _JointSpaceInterpolator_deg_ + 1);
    /*evaluate the spline*/
    std::vector<double> t_tmp_vec(1);
    t_tmp_vec[0] = t_loc;
    static bool print_verbose = false;
    joint_interpolator_sptr_->evaluate(joint_eval, t_tmp_vec , dd_vec);
    if (t_loc > joint_interpolator_sptr_->get_t_end()){
      print_verbose = false;
    }else{
      // std::cout<< t_tmp_vec[0] << std::endl;
      // std::cout<< joint_eval(0, 0).transpose() << std::endl;
      print_verbose = true;
    }

    /****************************************/
    /*set desired joint values from spline interpolation*/
    /****************************************/
    typename joint_Interpolator_t::ctrlpt_t q_spline = joint_eval(0, 0);
    typename joint_Interpolator_t::ctrlpt_t qp_spline = joint_eval(0, 1);
    typename joint_Interpolator_t::ctrlpt_t qpp_spline = joint_eval(0, 2);
    #ifdef ACIN_CONTROLLER_DEBUG_MSG
      bool not_NAN = true;
      for (uint16_t ii = 0; ii < _ROBOT_q_DIM_; ii++) {
        if (std::isnan(q_spline[ii])) {
            not_NAN = false;
            break;
        }
      }
      if (!not_NAN) {
        ACIN_CTRL_ERROR("q_spline state nan value");
        ACIN_CTRL_ERROR("Ta:= %f", this->Ta_);
        ACIN_CTRL_ERROR("t_loc:= %f", t_loc);
        ACIN_CTRL_ERROR("[%f, %f, %f, %f, %f, %f, %f]", q_spline[0], q_spline[1], q_spline[2], q_spline[3], q_spline[4], q_spline[5], q_spline[6]);
        q_spline = JointVector_t::Zero();
      }
    #endif
    set_dest_q(q_spline);
    set_dest_qp(qp_spline);
    set_dest_qpp(qpp_spline);
    /****************************************/
    /*set desired task space values from the joint spline interpolation*/
    /****************************************/
    /*calculate pose with the HomogeneousTransformation_t, forward kinematik*/
    PoseVector_t pose_d;
    HomogeneousTransformation_t H_B_E_d;
    evalT0E(q_spline, H_B_E_d, robot_params_);
    // position vector for current and desired
    CartesianPositionVector_t p_B_E_d = H_B_E_d.block<3, 1>(0, 3);
    RotationMatrix_t R_B_E_d = H_B_E_d.block<3, 3>(0, 0);
    pose_d(0,0) = p_B_E_d(0,0);
    pose_d(1,0) = p_B_E_d(1,0);
    pose_d(2,0) = p_B_E_d(2,0);
    Eigen::Quaternion<double> quat_B_E_d = Eigen::Quaternion<double>(R_B_E_d);
    pose_d(3,0) = quat_B_E_d.w();
    pose_d(4,0) = quat_B_E_d.x();
    pose_d(5,0) = quat_B_E_d.y();
    pose_d(6,0) = quat_B_E_d.z();
    /*calculate the cartesian velocitys with geometric Jacobian, forward kinematik*/
    CartesianVector_t cart_vel_d;
    JacobianMatrix_t J_d;
    evalJ(q_spline, J_d, robot_params_);
    cart_vel_d = J_d*qp_spline;
    /*calculate the cartesian accelerations with geometric Jacobian, forward kinematik*/
    CartesianVector_t cart_accel_d;
    JacobianMatrix_t JDot_d;
    evalJp(q_spline, qp_spline, JDot_d, robot_params_);
    cart_accel_d = JDot_d*qp_spline + J_d*qpp_spline;
    /**/
    set_dest_cart_pose(pose_d);
    set_dest_cart_vel(cart_vel_d);
    set_dest_cart_accel(cart_accel_d);
    /***/
    /***/
    pub_PoseTraj_(pose_d, cart_vel_d, cart_accel_d, traj_pub_t::TRAJ_DESIRED );
  }


  void controller_task_space()
  {

    JointMatrix_t C;
    evalC(q_, q_dot_, C, robot_params_);
    JointMatrix_t M;
    evalM(q_, M, robot_params_);
	  if (this->SingularPert_CTRL_flag_) {
        JointMatrix_t B_mat = JointVector_t(robot_params_.B).asDiagonal();
        JointMatrix_t tmp_K_inv = (JointMatrix_t::Identity() + controller_params_.K_tau_sp).inverse();
        M = M + tmp_K_inv * B_mat;
    }
    JointVector_t G;
    evalG(q_, G, robot_params_);


    // Jacobian Matrix for current state
    JacobianMatrix_t J = J_;
    //diff of the actuel Jacobian Matrix
    JacobianMatrix_t J_dot = Jp_;
    //evalJp(q_, q_dot_, J_dot, robot_params_);


    JacobianInverseMatrix_t J_inv;
    double w_manipulability = (J*J.transpose()).determinant();
    w_manipulability_ = w_manipulability;
    double lambda =0.0;
    // if(w_manipulability < 0.00001){
      //
      // Eigen::MatrixXd J_temp(6,7);
      // for(uint16_t ii = 0; ii < J.rows(); ii++){
      //   for(uint16_t jj = 0; jj < J.cols(); jj++){
      //     J_temp(ii,jj) = J(ii,jj);
      //   }
      // }
      //
      // // Eigen::JacobiSVD< JacobianMatrix_t> svd = J_temp.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV);
      // Eigen::BDCSVD< Eigen::MatrixXd> svd(J_temp, Eigen::ComputeThinU | Eigen::ComputeThinV);
      // Eigen::Matrix<double, 6, 6>  U = svd.matrixU();
      // Eigen::Matrix<double, 6, 7> U_;
      // U_ << U, Eigen::Matrix<double, 6, 1>::Zero();
      // std::cout<<svd.matrixV().rows()<<std::endl;
      // std::cout<<svd.matrixV().cols()<<std::endl;
      // Eigen::Matrix<double, 7, 6> V = svd.matrixV();
      // Eigen::Matrix<double, 7, 7> V_;
      // V_ << V, Eigen::Matrix<double, 7, 1>::Zero();
      // Eigen::Matrix<double, 6, 1> svd_values = svd.singularValues();
      // Eigen::Matrix<double, 6,1> svd_inv;
      // double max_svd = std::numeric_limits<double>::min();
      // double min_svd = std::numeric_limits<double>::max();
      // uint32_t ii_max = 0;
      // uint32_t ii_min = 0;
      //
      // for(uint64_t ii = 0; ii < 6; ii++){
      //   if(svd_values[ii] > max_svd){
      //     max_svd = svd_values[ii];
      //     ii_max = ii;
      //   }
      //   if(svd_values[ii] < min_svd){
      //     min_svd = svd_values[ii];
      //     ii_min = ii;
      //   }
      //   svd_inv[ii] = 1/svd_values[ii];
      // }
      // // svd_inv[ii_min] = svd_inv[5];
      //
      // if (max_svd == 0.0 && min_svd ==0.0){
      //   throw std::range_error("matrix fully singular, stop task space controller");
      // }
      //
      // Eigen::Matrix<double, 7,1> temp;
      // temp << svd_inv,0.0;
      // Eigen::Matrix<double, 7,7> S_inv = temp.asDiagonal();
      // // J_inv = V_*S_inv*U_.transpose();
      // double w_manipulability_new = (J_tmp*J_tmp.transpose()).determinant();
      // ACIN_CTRL_ERROR("TaskSpace Controller operates near singularity, w_mani. old:= %f ",w_manipulability);
      // std::cout<<"svd := " << svd_values << std::endl;
      // std::cout<<"V :=" << V << std::endl;
      // std::cout<<"V :=" << V*svd_inv << std::endl;
      // ROS_ERROR_STREAM();
      // ROS_ERROR_STREAM("V :=" << V << std::endl);
    // }else{
      // lambda = 0.004;
       J_inv = J.transpose()*(J*J.transpose()+(lambda*lambda)*Eigen::Matrix<double,6,6>::Identity()).inverse();
    // }

    // static uint64_t t_m_ii = 0;
    // static double duration = 0;
    // auto t1 = std::chrono::high_resolution_clock::now();

    //   auto t2 = std::chrono::high_resolution_clock::now();
    // duration += std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count();
    // if(t_m_ii == t_m_itr- 1){
    //   std::cout<<"SVD calculation time avg. over "<<t_m_itr<<" iterations "<< duration/t_m_itr<< " ms\n\n";
    //   t_m_ii = 0;
    //   duration = 0;
    // }else{
    //   t_m_ii++;
    // }


    CartesianVector_t x_d;
    x_d.block<3,1>(0,0) = pose_d_.block<3,1>(0,0); // (x,y,z)
    x_d.block<3,1>(3,0) = pose_d_.block<3,1>(4,0);// set vector part of quaternion q.x, q.y,q.z
    CartesianVector_t x_d_dot; //(xDot,yDot,zDot,omega_x,omega,y,omega_z)
    x_d_dot.block<3,1>(0,0) = pDot_d_;
    x_d_dot.block<3,1>(3,0) = omega_d_;
    CartesianVector_t x_d_dot_dot;//(xDotDot,yDotDot,zDotDot,omega_xDot,omega_yDot,omega_zDot)
    x_d_dot_dot.block<3,1>(0,0) = pDotDot_d_;
    x_d_dot_dot.block<3,1>(3,0) = omegaDot_d_;

    Eigen::Vector3d p_d = x_d.block<3,1>(0,0);
    Eigen::Quaternion<double> quat_d(pose_d_(3,0),pose_d_(4,0),pose_d_(5,0),pose_d_(6,0));
    Eigen::Matrix3d R_d = quat_d.toRotationMatrix();
    pose_d_(3,0) = quat_d.w();
    pose_d_(4,0) = quat_d.x();
    pose_d_(5,0) = quat_d.y();
    pose_d_(6,0) = quat_d.z();
    // rotation matrix for current and desired
    // rotation matrix for current

    RotationMatrix_t R= H_B_E_.block<3, 3>(0, 0);
    Eigen::Quaternion<double> quat = Eigen::Quaternion<double>(R);
    // position vector for current joint state
    CartesianPositionVector_t p = H_B_E_.block<3, 1>(0, 3);
    // cartesian error
    CartesianVector_t e;
    e.block<3, 1>(0, 0) = p - p_d;
    auto e_r = Eigen::Quaternion<double>(R*R_d.transpose());
    e(3) = e_r.x();
    e(4) = e_r.y();
    e(5) = e_r.z();
    this->e_pose_ = pose_ - pose_d_;
    this->e_q_ = JointVector_t::Zero();
    /*calculate the cartesian velocitys with geometric Jacobian, forward kinematik*/
    CartesianVector_t x_dot = J * q_dot_;
    CartesianVector_t e_dot = x_dot - x_d_dot;
    this->e_cart_vel_ = e_dot;
    this->e_q_dot_ = JointVector_t::Zero();

    this->e_cart_accel_ = CartesianVector_t::Zero();
    //static std::vector<double> q_m(q_.rows(), 0);

    // tau_d_ = G + C * q_dot_
    //        + M * (J_inv * (x_d_dot_dot_ - J_dot * q_dot_ - controller_params_.K1_cart * e_dot - controller_params_.K0_cart * e)
    //        + (JointMatrix_t::Identity() - J_inv * J) * (-controller_params_.K1_N_cart * q_dot_ - controller_params_.K0_N_cart * q_));
    //static JointVector_t grad_W_mFoM = JointVector_t::Zero();
    //static JointVector_t qN_log_barrier = JointVector_t::Zero();
    //grad_mFoM(grad_W_mFoM, q_, robot_params_.q_limit_lower.data(), robot_params_.q_limit_upper.data(), q_m);
    //log_barrier(qN_log_barrier, q_, robot_params_.q_limit_lower, robot_params_.q_limit_upper);
    JointVector_t e_qN = (q_ - q_d_NS_);
    JointVector_t eqN_K0 = JointVector_t::Zero();
    if (qN_log_barrier_flag_){
      //eqN_K0 = qN_log_barrier;
    }else{
      if (qN_max_FoM_flag_){
        //eqN_K0 = grad_W_mFoM;
      }else{
        eqN_K0 = e_qN;
      }
    }
    JointVector_t eqN_K1 = q_dot_;
    JointVector_t tau_Nullspace = M*(JointMatrix_t::Identity() - J_inv * J)*
                                  (-controller_params_.K1_N_cart * eqN_K1 - controller_params_.K0_N_cart * eqN_K0);
    JointVector_t  tau_d_prv = tau_d_;
    /*CT task space behaviour, spring damper system for the task space coordinates,
    the stiffness and the damping can be either defined based on endeffektor postion/orientation in the base coord system or in the coord system of the endeffector it self(stiffness and damping independt of the relative position to the robot base)*/
    CartesianVector_t  tau_K = CartesianVector_t::Zero();//general task space force, spring behaviour, force and torque in the base coord system of the robot
    CartesianVector_t  tau_D = CartesianVector_t::Zero();//general task space force, damping behaviour, force and torque in the base coord system of the robot
    RotationMatrix_t R_inv = R.transpose();
    CartesianMatrix_t K0_cart = controller_params_.K0_cart;

    if (this->TaskSpaceCtrl_K_endeffektor_) {
        K0_cart.block<3, 3>(0, 0) = R * K0_cart.block<3, 3>(0, 0)* R_inv;
        K0_cart.block<3, 3>(3, 3) = R * K0_cart.block<3, 3>(3, 3)* R_inv;
        tau_K = K0_cart * e;
    }else {
        tau_K = K0_cart * e;
    }
    CartesianMatrix_t K1_cart = controller_params_.K1_cart;
    if (this->TaskSpaceCtrl_D_endeffektor_) {
        //K1_cart.block<3, 3>(0, 0) = R * K1_cart.block<3, 3>(0, 0);
        //K1_cart.block<3, 3>(3, 3) = R * K1_cart.block<3, 3>(3, 3);
        /*TODO not yet implemented*/
        tau_D = K1_cart * e_dot;
    }else {
        tau_D = K1_cart* e_dot;
    }
    tau_d_ = G + C * q_dot_
        + M * J_inv * (x_d_dot_dot - J_dot * q_dot_ - tau_D - tau_K)
        + tau_Nullspace;
/**/
  if(this->TaskSpace_ctrl_safety_){
      for(int32_t ii_tau =0; ii_tau < _ROBOT_q_DIM_; ii_tau++){
          double Tau_ub_ii = robot_params_.Tau_ub[ii_tau];
          double Tau_lb_ii = robot_params_.Tau_lb[ii_tau];
          double Tau_d_ii = tau_d_(ii_tau);
          double Tau_d_prv_ii = tau_d_prv(ii_tau);
          if(std::isnan(Tau_d_ii)){
            safe_mode_ = true;
            for (int32_t ii_row= 0; ii_row < J_dot.rows(); ii_row++)
            {
                for (int32_t ii_q = 0; ii_q < J_dot.cols(); ii_q++) {
                    if (std::isnan(J_dot(ii_row, ii_q))) {
                        ACIN_CTRL_ERROR("J_dot(%i,%i)", ii_row, ii_q);
                    }
                    if (std::isnan(J_inv(ii_row, ii_q))) {
                        ACIN_CTRL_ERROR("J_inv(%i,%i)", ii_row, ii_q);
                    }
                }
                if (std::isnan(e(ii_row))) {
                    ACIN_CTRL_ERROR("e(%i)", ii_row);
                }
                if (std::isnan(e_dot(ii_row))) {
                    ACIN_CTRL_ERROR("e_dot(%i)", ii_row);
                }
                if (std::isnan(x_d_dot_dot(ii_row))) {
                    ACIN_CTRL_ERROR("x_d_dot_dot(%i)", ii_row);
                }
            }
            break;
          }
          if((w_manipulability < 1e-6) && (Tau_d_ii > Tau_ub_ii)){
            safe_mode_ = true;
            break;
          }
          if((w_manipulability < 1e-6) && (Tau_d_ii < Tau_lb_ii)){
            safe_mode_ = true;
            break;
          }
          if((w_manipulability < 1e-6) && ( std::abs(Tau_d_ii - Tau_d_prv_ii) > (std::abs(Tau_ub_ii - Tau_lb_ii)*this->DeltaTau_p_safety_))){
            safe_mode_ = true;
            break;
          }
      }
     if(safe_mode_){
        #if defined (ACIN_CTRL_EXECEPTIONS)
           std::cout << q_<<std::endl;
           std::cout<< "near singularity, can't operate in TaskSpace"<<std::endl;
           throw std::range_error("near singularity, can't operate in TaskSpace");
        #endif
        ACIN_CTRL_ERROR("SAFE_MODE active, taskspace is operating near singularity");
        ACIN_CTRL_ERROR("w_manipulability:= %f", w_manipulability);
        ACIN_CTRL_ERROR("tau_d_:= [%f, %f, %f, %f, %f, %f, %f]", tau_d_(0), tau_d_(1), tau_d_(2), tau_d_(3), tau_d_(4), tau_d_(5), tau_d_(6));
        ACIN_CTRL_ERROR("tau_N_:= [%f, %f, %f, %f, %f, %f, %f]", tau_Nullspace(0), tau_Nullspace(1), tau_Nullspace(2), tau_Nullspace(3), tau_Nullspace(4), tau_Nullspace(5), tau_Nullspace(6));
        double t_now = get_cur_time(timeBase_setting);
        joint_interpolator_sptr_->lin_interpolation(q_, q_, q_PreCurve_, t_now, T_init_0_/2);
        Joint_interpl_flag_ = true;
        pub_joint_spline_traj(t_now);
        controller_joint_space(false);
        boolMsg_t motion_EN_msg;
        motion_EN_msg.data = !this->ctrl_motion_en_;
        v_pub_motion_switch(motion_EN_msg);
        //ctrl_motion_disable();
        return;
     }

   }
  }
  bool flag_CT_joint_sapce_First = false;
  void controller_joint_space(const bool &reset){

    JointMatrix_t C;
    JointMatrix_t M;
    JointVector_t G;
    evalC(q_, q_dot_, C, robot_params_);
    evalM(q_, M, robot_params_);
    JointMatrix_t B_mat = JointVector_t(robot_params_.B).asDiagonal();
    JointMatrix_t tmp_K_inv = (JointMatrix_t::Identity() + controller_params_.K_tau_sp).inverse();
    JointMatrix_t M_mod = M + tmp_K_inv * B_mat;
    if(this->SingularPert_CTRL_flag_){
        M = M_mod;
    }
    evalG(q_, G, robot_params_);
    JointVector_t q_d = q_d_;
    JointVector_t qDot_d = qDot_d_;
    JointVector_t qDotDot_d = qDotDot_d_;
    if (flag_CT_joint_sapce_First == false) {
        JointMatrix_t tmp = this->controller_params_.K0_joint;
        ACIN_CTRL_ALWAYS("### joint space trajectory controller parameter#####");
        ACIN_CTRL_ALWAYS("K0_joint [%f, %f, %f, %f, %f, %f, %f]", tmp(0, 0), tmp(1, 1), tmp(2, 2), tmp(3, 3), tmp(4, 4), tmp(5, 5), tmp(6, 6));
        tmp = this->controller_params_.K1_joint;
        ACIN_CTRL_ALWAYS("K1_joint [%f, %f, %f, %f, %f, %f, %f]", tmp(0, 0), tmp(1, 1), tmp(2, 2), tmp(3, 3), tmp(4, 4), tmp(5, 5), tmp(6, 6));
        tmp = this->controller_params_.KI_joint;
        ACIN_CTRL_ALWAYS("KI_joint [%f, %f, %f, %f, %f, %f, %f]", tmp(0, 0), tmp(1, 1), tmp(2, 2), tmp(3, 3), tmp(4, 4), tmp(5, 5), tmp(6, 6));
        tmp = M_mod;
        ACIN_CTRL_ALWAYS("M mod diag [%f, %f, %f, %f, %f, %f, %f]", tmp(0, 0), tmp(1, 1), tmp(2, 2), tmp(3, 3), tmp(4, 4), tmp(5, 5), tmp(6, 6));
        tmp = M;
        ACIN_CTRL_ALWAYS("M diag [%f, %f, %f, %f, %f, %f, %f]", tmp(0, 0), tmp(1, 1), tmp(2, 2), tmp(3, 3), tmp(4, 4), tmp(5, 5), tmp(6, 6));
        flag_CT_joint_sapce_First = true;
    }
    if (reset){
      xq_I_=JointVector_t::Zero();
    }
    JointVector_t T_I = -M*controller_params_.KI_joint*(xq_I_);
    for(uint64_t ii = 0; ii < static_cast<uint64_t>(q_.size()); ii++){
      if(T_I[ii]>robot_params_.Tau_ub[ii]/10.0){
        // ACIN_CTRL_WARN("Tau_I  of joint %lu is greater than 10%% of the upper limmit",ii);
      }else{
        if(T_I[ii]<robot_params_.Tau_lb[ii]/10.0){
          // ACIN_CTRL_WARN("Tau_I of joint %lu is greater than 10%% of the lower limmit",ii);
        }
      }

    }
    JointVector_t e_q = (q_ - q_d);
    JointVector_t e_q_dot = (q_dot_ - qDot_d);
    JointVector_t v = qDotDot_d - controller_params_.K0_joint * e_q - controller_params_.K1_joint * e_q_dot - controller_params_.KI_joint*(xq_I_);
    // if (switch_){
    //   std::cout<<"Tau last task space controller:="<<std::endl;
    //   std::cout<<tau_d_.transpose()<<std::endl;
    // }
    if (gravity_comp_){
      tau_d_ = G - controller_params_.KD_grav*q_dot_;
    }else{
      xq_I_ = xq_I_ +  Ta_*e_q;
      tau_d_ = M * v + C * q_dot_ + G;
    }
    // if (switch_){
    //     std::cout<<"Tau new"<<std::endl;
    //   std::cout<<tau_d_.transpose()<<std::endl;
    //   switch_ =false;
    // }
    this->e_q_ = e_q;
    this->e_q_dot_ = e_q_dot;
    this->e_pose_ = PoseVector_t::Zero();
  }


  //low pass filter for measured robot joint torque
  void q_dot_filter(const bool &reset){
    JointVector_t T1_q_dot = controller_params_.T1_q_dot;
    for (uint8_t ii = 0; ii < this->q_dot_.size(); ii++){
      double alpha_q_dot_f = exp(-Ta_/T1_q_dot(ii));
      double V_q_dot_f = 1/Ta_*(1 - alpha_q_dot_f);
      if (reset){
        x_q_dot_(ii) = -V_q_dot_f*this->q_(ii);
        this->q_dot_filter_(ii) = 0;

      }else{
        this->q_dot_filter_(ii) = x_q_dot_(ii) + V_q_dot_f*this->q_(ii);
        x_q_dot_(ii) = alpha_q_dot_f*x_q_dot_(ii) + (alpha_q_dot_f - 1)*V_q_dot_f*this->q_(ii);
      }
    }
  }
  //low pass filter for measured robot joint torque
  void tau_filter(const bool &reset){
    JointVector_t T1_tau = controller_params_.T1_tau;
    for (uint8_t ii = 0; ii < this->tau_.size(); ii++){
      double alpha_tau_f = exp(-Ta_/T1_tau(ii));
      if (reset){
        x_Tau_(ii) = 1/(1-alpha_tau_f)*this->tau_(ii);
        tau_filter_(ii) = this->tau_(ii);
      }else{
          tau_filter_(ii) =(1-alpha_tau_f)*x_Tau_(ii);
          x_Tau_(ii) = alpha_tau_f*x_Tau_(ii)+this->tau_(ii);
      }
    }
  }
  //low pass derivative filter to get derivatives of the robot joint torques for the singular pertubation
  void tau_p_filter(const bool &reset){
    JointVector_t T1_tau_p = controller_params_.T1_tau_p;
    for (uint8_t ii = 0; ii < this->tau_.size(); ii++){
      double alpha_tau_p_f = exp(-Ta_/T1_tau_p(ii));
      double V_tau_p_f = 1/Ta_*(1 - alpha_tau_p_f);
      if (reset){
        x_Tau_p_(ii) = -V_tau_p_f*this->tau_(ii);
        this->tau_p_filter_(ii) = 0;
      }else{
        this->tau_p_filter_(ii) = x_Tau_p_(ii) + V_tau_p_f*this->tau_(ii);
        x_Tau_p_(ii) = alpha_tau_p_f*x_Tau_p_(ii) + (alpha_tau_p_f - 1)*V_tau_p_f*this->tau_(ii);
      }
    }
  }
  void singular_pertubation_Tau(){
    JointVector_t tau_SP_K = controller_params_.K_tau_sp*(tau_filter_-tau_d_);
    JointVector_t tau_SP_D = controller_params_.D_tau_sp*(tau_p_filter_);
    tau_SP_ =  - tau_SP_K - tau_SP_D;

  }
  JointMatrix_t B_loc_inv =JointMatrix_t::Zero();
  void friction_compensation(bool reset){
    JointMatrix_t L_fComp = controller_params_.L;
    //Moments of inertia of the rotors
    JointMatrix_t B_loc = JointVector_t(robot_params_.B).asDiagonal();
    JointVector_t tau_feedback_m = JointVector_t::Zero();
    if (tau_FB_m_Flag_){
      tau_feedback_m  = tau_FB_m_;
    }else{
      tau_feedback_m  = tau_filter_;
    }
    if (reset|| !ctrl_motion_en_){
      tau_friction_comp_ = JointVector_t::Zero();
      x_q_dot_hat_ = JointVector_t::Zero();
    }else{
      tau_friction_comp_ =-L_fComp*B_loc*(q_dot_filter_ - x_q_dot_hat_);
      x_q_dot_hat_ = x_q_dot_hat_ + Ta_*B_loc.inverse()*(tau_feedback_m - tau_filter_ - tau_friction_comp_);
    }
  }

  void calculate_CTRL_effort(const bool &reset){
      #if defined(FLAG_ROS_COMPILE)
        std::unique_lock<std::timed_mutex> lock{ACIN_CTRL_mutex_,  std::defer_lock};
        lock.lock();
      #endif

      static bool gravity_comp_last = gravity_comp_;
      static bool limits_torque = false;//if in the last iteration the torque limit was reached
      //static bool friction_comp_last = friction_comp_CTRL_flag_;
      /*ATTENTION-> Calculation order matters,e.g. singular pertubation needs the desired effort of the CT controller*/
      /*Filter for reducing noise of the torque measurement*/
      //if (gravity_comp_last == gravity_comp_){
      tau_filter(reset);
      tau_p_filter(reset);
      //}else{
      //  tau_filter(true);
      //  tau_p_filter(true);
      //}
      /*differntation Filter, getting q_dot, if a measurement is avaiable more structional things must be done TODO*/
      q_dot_filter(reset);
      /*first calculate desired torque for CT*/
      /***********Calculated CT Torque, task space or jointspace**********/
      if( !gravity_comp_  && !Joint_interpl_flag_){
        /*CT Taskspace controller*/
        #if defined(ACIN_CONTROLLER_DEBUG)
          timespec t1;
          timespec t2;
          clock_gettime(CLOCK_REALTIME,&t1);
        #endif
        controller_task_space();
        #if defined(ACIN_CONTROLLER_DEBUG)
          clock_gettime(CLOCK_REALTIME,&t2);
          timespec_diff(t2,t1,t_TS_ctrl_);
        #endif
      }else{
        #if defined(ACIN_CONTROLLER_DEBUG)
          timespec t1;
          timespec t2;
          clock_gettime(CLOCK_REALTIME,&t1);
        #endif
        /*CT jointspace controller*/
        controller_joint_space(reset || limits_torque);
        #if defined(ACIN_CONTROLLER_DEBUG)
          clock_gettime(CLOCK_REALTIME,&t2);
          timespec_diff(t2,t1,t_QS_ctrl_);
        #endif
        //std::cout<<t_QS_ctrl_.tv_sec<<" sec; "<<t_QS_ctrl_.tv_nsec<<" nsec"<<std::endl<<std::endl;
      }
      /***********end**********/

      singular_pertubation_Tau();
      friction_compensation(reset);
      JointVector_t Tau_out = JointVector_t::Zero();
      Tau_out = tau_d_;
      if (this->friction_comp_CTRL_flag_){
        bool not_NAN = true;
        for(uint16_t ii=0; ii < _ROBOT_q_DIM_; ii++){
            if (std::isnan(tau_friction_comp_[ii])){
              not_NAN = false;
              break;
            }
        }
        if(not_NAN){
          Tau_out += tau_friction_comp_;
        }else{
          x_q_dot_hat_ = JointVector_t::Zero();
          tau_friction_comp_ = JointVector_t::Zero();
          ACIN_CTRL_ERROR("friction compensation results in NAN values");
        }

      }
      if (this->SingularPert_CTRL_flag_){
        Tau_out += tau_SP_;
      }
      limits_torque = false;
      for(uint16_t ii = 0; ii < _ROBOT_q_DIM_; ii++){
        if(Tau_out(ii) > this->robot_params_.Tau_ub[ii]){
          // ACIN_CTRL_ERROR("Torque Limit axis %i",ii);
          // ACIN_CTRL_ERROR("calc. Torque %f",Tau_out(ii));
          // ACIN_CTRL_ERROR("Torque set upper bound %f",this->robot_params_.Tau_ub[ii]);
          Tau_out(ii) =this->robot_params_.Tau_ub[ii];
          limits_torque = true;
        }
        if(Tau_out(ii) < this->robot_params_.Tau_lb[ii]) {
          // ACIN_CTRL_ERROR("Torque Limit axis %i",ii);
          // ACIN_CTRL_ERROR("calc. Torque %f",Tau_out(ii));
          // ACIN_CTRL_ERROR("Torque set lower bound %f",this->robot_params_.Tau_lb[ii]);
          Tau_out(ii) = this->robot_params_.Tau_lb[ii];
          limits_torque = true;
        }
      }
      v_pub_Tau(Tau_out);
      gravity_comp_last = gravity_comp_;
      //friction_comp_last = friction_comp_CTRL_flag_;
  }


protected:

  void get_Tau_f(Eigen::Ref<JointVector_t> Tau_f){
    Tau_f = tau_filter_;
  }
  void get_Tau_p_f(Eigen::Ref<JointVector_t> Tau_p_f){
    Tau_p_f = tau_p_filter_;
  }
  void get_q_dot_f(Eigen::Ref<JointVector_t> q_dot_f){
    q_dot_f = q_dot_filter_;
  }
  void get_Tau_SP(Eigen::Ref<JointVector_t> Tau_SP){
    Tau_SP = tau_SP_;
  }
  void get_Tau_friction(Eigen::Ref<JointVector_t> Tau_friction){
    Tau_friction = tau_friction_comp_;
  }
  void get_Tau_CT(Eigen::Ref<JointVector_t> Tau_CT){
    Tau_CT = tau_d_;
  }
  void CTRL_friction_comp_enable(){
    friction_compensation(true);
    this->friction_comp_CTRL_flag_ = true;
  }
  void CTRL_friction_comp_disable(){
    this->friction_comp_CTRL_flag_ = false;
  }
  void CTRL_SingularPert_enable(){
    tau_filter(true);
    tau_p_filter(true);
    this->SingularPert_CTRL_flag_ = true;
  }
  void CTRL_SingularPert_disable(){
    this->SingularPert_CTRL_flag_ = false;
  }
  void ctrl_motion_enable(){
    this->xq_I_ = JointVector_t::Zero();
    ctrl_motion_en_ = true;
  }
  void ctrl_motion_disable(){
    controller_reset();
    ctrl_motion_en_ = false;
  }
  void desired_sollPosCallback(const acinSollPosMsg_t &sollPosMsg){
    if (FIRST_SENSOR_VALUE_){
      if(sollPosMsg.TaskSpace){
        taskSpace_SollPos_(sollPosMsg);
      }else{
        jointSpace_SollPos_(sollPosMsg);
      }
    }else{
      double t = get_cur_time();
      double t_0 = point_interpolator_sptr_->get_t_0();
      double t_end = point_interpolator_sptr_->get_t_end();
      ACIN_CTRL_WARN("<sollPosMsg function> current time system %.4f", t);
      ACIN_CTRL_WARN("<sollPosMsg function> t0 point spline interpolation %.4f", t_0);
      ACIN_CTRL_WARN("<sollPosMsg function> tend point spline interpolation %.4f", t_end);
      ACIN_CTRL_WARN("<sollPosMsg function> sollPos: [%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f] ", sollPosMsg.sollPos[0],sollPosMsg.sollPos[1],sollPosMsg.sollPos[2],sollPosMsg.sollPos[3],sollPosMsg.sollPos[4],sollPosMsg.sollPos[5],sollPosMsg.sollPos[6]);
      ACIN_CTRL_WARN("<sollPosMsg function> coord flag [%u,%u,%u,%u,%u,%u,%u]", (uint32_t)sollPosMsg.coord[0],sollPosMsg.coord[1],sollPosMsg.coord[2],sollPosMsg.coord[3],sollPosMsg.coord[4],sollPosMsg.coord[5],sollPosMsg.coord[6]);
      ACIN_CTRL_WARN("<sollPosMsg function> tend point spline interpolation %.4f", t_end);
    }
  }
  void coord_adj(const CoordAdj_t &coord_adj){

    if (FIRST_SENSOR_VALUE_){
      if(coord_adj.TaskSpace){
        task_space_adjustment_(coord_adj);
      }else{
        joint_space_adjustment_(coord_adj);
      }
    }else{
      double t = get_cur_time();
      double t_0 = point_interpolator_sptr_->get_t_0();
      double t_end = point_interpolator_sptr_->get_t_end();
      ACIN_CTRL_WARN("<coord_adj function> current time system %.4f", t);
      ACIN_CTRL_WARN("<coord_adj function> t0 point spline interpolation %.4f", t_0);
      ACIN_CTRL_WARN("<coord_adj function> t0 point spline interpolation %.4f", t_end);
    }

  }
public:
  void iterate()
  {
    #if defined(ACIN_CONTROLLER_DEBUG)
      timespec t1__;
      timespec t2__;
      timespec dispatch_t__;
    #endif
    static bool ctrl_motion_en_last = ctrl_motion_en_;
    if (FIRST_SENSOR_VALUE_){
      /***********Calculated CT Torque, task space or jointspace**********/
      dispatchDesired();
      /***********end**********/
      if (ctrl_motion_en_last == false && ctrl_motion_en_ == true){
        calculate_CTRL_effort(true);
      }else{
        calculate_CTRL_effort(false);
      }
      ctrl_motion_en_last = ctrl_motion_en_;
    }else{
      calculate_CTRL_effort(true);
      ctrl_motion_en_last = false;
    }
    #if defined(ACIN_CONTROLLER_DEBUG)
      clock_gettime(CLOCK_REALTIME,&t2__);
      timespec_diff(t2__,t1__,dispatch_t__);
    #endif

  }
  void pub_status_information(){
    pub_e_JointState_();
    CartesianVector_t accel_actual = CartesianVector_t::Zero();
    pub_PoseTraj_(pose_, cart_vel_, accel_actual, traj_pub_t::TRAJ_ACTUEL);
    CartesianVector_t cart_vel_d;
    CartesianVector_t cart_accel_d;

    cart_vel_d.block<3,1>(0,0) = pDot_d_;
    cart_vel_d.block<3,1>(3,0) = omega_d_;
    cart_accel_d.block<3,1>(0,0) = pDotDot_d_;
    cart_accel_d.block<3,1>(3,0) = omegaDot_d_;
    pub_PoseTraj_(pose_d_, cart_vel_d, cart_accel_d, traj_pub_t::TRAJ_DESIRED);
    pub_e_poseTraj_();
    #if defined(ACIN_CONTROLLER_DEBUG)
      pub_time_meas_();
    #endif
  }
  void setControllerParam(const struct ControllerParams &controller_params){
    if(ctrl_motion_en_ == false){
      ACIN_CTRL_ALWAYS("## setControllerParam ##");
      this->controller_params_ = controller_params;
      JointMatrix_t tmp = this->controller_params_.K0_joint;
      std::string K0_joint_str = EigenToString(tmp);
      ACIN_CTRL_ALWAYS("K0_joint:= \n%s",K0_joint_str.c_str());

    }
  }
public:
  void setworldBaseTransformation(const Eigen::Ref<HomogeneousTransformation_t> H_w_B_new){
    if(ctrl_motion_en_ == false){
      ACIN_CTRL_ALWAYS("## setworldBaseTransformationParam ##");
      //      this->controller_params_ = controller_params;
      //JointMatrix_t tmp = this->controller_params_.K0_joint;
      //std::string K0_joint_str = EigenToString(tmp);
      this->H_w_B_ = H_w_B_new;
      this->R_w_B_ = H_w_B_new.block<3,3>(0,0);
      this->d_w_B_ = H_w_B_new.block<3,1>(0,3);
      Vector3f RPY = R_w_B_.eulerAngles(0, 1, 2);
      this->robot_params_.gamma_base =RPY(0);
      this->robot_params_.beta_base =RPY(1);
      this->robot_params_.alpha_base =RPY(2);
      this->robot_params_.dw0x =d_w_B_(0);
      this->robot_params_.dw0y =d_w_B_(1);
      this->robot_params_.dw0z =d_w_B_(2);
    }
  }
  void getSampleTime(double &Ta){
    Ta = this->Ta_;
  }
  void setSampleTime(double &Ta){
    if(Ta <= 0){
      return;
    }
    this->Ta_ = Ta;
    this->controller_reset();
  }
  static inline void timespec_diff(struct timespec &a, struct timespec &b,
      struct timespec &result) {
      result.tv_sec  = a.tv_sec  - b.tv_sec;
      result.tv_nsec = a.tv_nsec - b.tv_nsec;
      if (result.tv_nsec < 0) {
          result.tv_sec -= 1;
          result.tv_nsec += 1000000000L;
      }
  }
  private:
    void controller_reset(){
      this->xq_I_ = JointVector_t::Zero();
      this->x_Tau_ = JointVector_t::Zero();
      this->x_Tau_p_ = JointVector_t::Zero();
      this->x_q_dot_ = JointVector_t::Zero();
      this->x_q_dot_hat_ = JointVector_t::Zero();
      FIRST_SENSOR_VALUE_ = false;
      Euler_interpl_flag_ = false;
      Joint_interpl_flag_ = true;
      tau_FB_m_Flag_ = false;
      switch_= false;
      gravity_comp_ = true;
      safe_mode_ = false; // if the controler is in safe mode, only joint space trajectorys are allowed, setting the gravity_mode from off to on back to off will reset
      ctrl_motion_en_ = false;
      SingularPert_CTRL_flag_ =false;
      friction_comp_CTRL_flag_ = false;
    }
};

#endif // ACIN_CONTROLLER_NODE_H
