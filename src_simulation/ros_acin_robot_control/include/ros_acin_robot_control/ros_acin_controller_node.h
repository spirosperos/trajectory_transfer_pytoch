#ifndef ACIN_CONTROLLER_NODE_ROS_H
#define ACIN_CONTROLLER_NODE_ROS_H
#include "acin_controller_node.h"
#include <ros_acin_robot_support/acin_ctrl_UI.h>
template <int N_knots_> class ROS_ACIN_Controller_Node_C:public ACIN_Controller_Node_C<N_knots_>
{
  public:
    typedef typename ACIN_Controller_Node_C<N_knots_>::init_acin_ctrl_t init_acin_ctrl_t;
    typedef typename ACIN_Controller_Node_C<N_knots_>::timeBase_t timeBase_t;
    ROS_ACIN_Controller_Node_C(ros::NodeHandle& nh, double Ta) : nh_(nh), nh_private_("~"), ACIN_Controller_Node_C<N_knots_>(Ta)
    {
      ros_init();
      ros_t0_ = ros::Time (get_cur_time());
    }
    void v_pub_motion_switch(boolMsg_t& MotionEN_msg){
      MotionENCallback_ROS(MotionEN_msg);
    }
    void v_pub_PoseTraj_actuel(PoseState_t &PoseTraj_actuel){
      ros_pub_TaskSpace_traj_actual_.publish(PoseTraj_actuel);
    }
    void v_pub_PoseTraj_desired(PoseState_t &PoseTraj_desired){
      ros_pub_TaskSpace_traj_desired_.publish(PoseTraj_desired);
    }
    void v_pub_TaskSpace_error(PoseState_t & TaskSpace_error){
      ros_pub_TaskSpace_error_.publish(TaskSpace_error);
    } /*publish joint space controler joint errors*/
    void v_pub_JointSpace_error(JointState_t &JointSpace_error){
      ros_pub_JointSpace_error_.publish(JointSpace_error);
    } /*publish joint space controler joint errors*/
    void v_pub_JointSpace_precurve(JointState_t &JointSpace_precurve){
      ros_pub_JointSpace_precurve_.publish(JointSpace_precurve);
    }
    void v_pub_time_meas(calcTimeMeasurement_t &calcTime){
      #if defined(ACIN_CONTROLLER_DEBUG)
        ros_pub_time_meas_.publish(calcTime);
      #endif
    }
    void pub_Tau_Debug_(){
        auto Tau_f_msg = Joint_t();
        auto Tau_p_f_msg = Joint_t();
        auto Tau_CT_msg = Joint_t();
        auto Tau_SP_msg = Joint_t();
        auto Tau_friction_msg = Joint_t();
        auto q_p_f_msg = Joint_t();
        Tau_f_msg.Joints = std::vector<double>(_ROBOT_q_DIM_);
        Tau_p_f_msg.Joints = std::vector<double>(_ROBOT_q_DIM_);
        Tau_CT_msg.Joints = std::vector<double>(_ROBOT_q_DIM_);
        Tau_SP_msg.Joints = std::vector<double>(_ROBOT_q_DIM_);
        Tau_friction_msg.Joints= std::vector<double>(_ROBOT_q_DIM_);
        q_p_f_msg.Joints=std::vector<double>(_ROBOT_q_DIM_);
        JointVector_t tau_filter;
        this->get_Tau_f(tau_filter);
        JointVector_t tau_p_filter;
        this->get_Tau_p_f(tau_p_filter);
        JointVector_t tau_d;
        this->get_Tau_CT(tau_d);
        JointVector_t tau_SP;
        this->get_Tau_SP(tau_SP);
        JointVector_t tau_friction_comp;
        this->get_Tau_friction(tau_friction_comp);
        JointVector_t q_dot_filter;
        this->get_q_dot_f(q_dot_filter);
       for(int i = 0; i < _ROBOT_q_DIM_; ++i)
       {
         Tau_f_msg.Joints[i] = tau_filter[i];
         Tau_p_f_msg.Joints[i] = tau_p_filter[i];
         Tau_CT_msg.Joints[i] = tau_d[i];
         Tau_SP_msg.Joints[i] = tau_SP[i];
         Tau_friction_msg.Joints[i] = tau_friction_comp[i];
         q_p_f_msg.Joints[i] = q_dot_filter[i];
       }
       ros_pub_tau_f_.publish(Tau_f_msg);
       ros_pub_tau_p_f_.publish(Tau_p_f_msg);
       ros_pub_tau_CT_.publish(Tau_CT_msg);
       ros_pub_tau_SP_.publish(Tau_SP_msg);
       ros_pub_tau_friction_.publish(Tau_friction_msg);
       ros_pub_q_p_f_.publish(q_p_f_msg);
    }
    void v_pub_Tau(Eigen::Ref<JointVector_t> Tau_out){

      // publish
    auto tau_ctrl_out_msg = Joint_t();
     tau_ctrl_out_msg.Joints = std::vector<double>(_ROBOT_q_DIM_);
     #if defined(WITH_EXCEPTIONS)
       if (tau_ctrl_out_msg.data.size() != _ROBOT_q_DIM_){
         throw std::runtime_error("");
       }
     #endif
      for(int i = 0; i < _ROBOT_q_DIM_; ++i)
      {

        tau_ctrl_out_msg.Joints[i] = static_cast<double>(Tau_out(i));

      }

      ros_pub_tau_.publish(tau_ctrl_out_msg);
      pub_Tau_Debug_();


    }
    double convert_time(time_t sec, uint64_t nsec, timeBase_t t_base){
      double t = 0.0;
      switch (t_base){
        case timeBase_t::ACIN_CTRL_BASE_SEC:
          t = (double)sec + (double)nsec/1000000000.0;
          break;
        case timeBase_t::ACIN_CTRL_BASE_mSEC:
          t = (double)sec*1000.0 + (double)nsec/1000000.0;
          break;
        case timeBase_t::ACIN_CTRL_BASE_uSEC:
          t = (double)sec*1000000.0 + (double)nsec/1000.0;
          break;
        case timeBase_t::ACIN_CTRL_BASE_nSEC:
          t = (double)sec*1000000000.0 + (double)nsec;
          break;
        default:
          throw std::runtime_error("ACIN_CTRL: Unkown time base");
      }
      if ((sec != 0 || nsec != 0) && t == 0.0){
        ROS_WARN("convert_time: time unit (enum = %lu) might not be wisly chosen, time before conversion %lu sec, %lu nsec",(uint64_t)t_base, sec, nsec);
      }
      return t;
    }
    double get_cur_time(const timeBase_t &t_base = timeBase_t::ACIN_CTRL_BASE_SEC) {
        ros::Time ros_time = ros::Time::now();
        time_t sec = ros_time.sec;
        uint64_t nsec = ros_time.nsec;
        double t = convert_time(sec, nsec, t_base);
        double t0 = convert_time(ros_t0_.sec, ros_t0_.nsec, t_base);
        ACIN_CTRL_ASSERT(t0 <= t);
        return t - t0;
    }
  protected:
      ros::Time ros_t0_ = ros::Time (0.0);//motion start time;
      //void v_pub_e_JointSpace(JointState_t &) = 0; /*publish joint space controler joint errors*/
      //void v_pub_e_TaskSpace(PoseState_t &) = 0; /*publish joint space controler joint errors*/
      //void v_pub_JointState_precurve(JointState_t &) = 0;
      //void v_pub_time_meas(calcTimeMeasurement_t &) = 0; /*publish task spa
      /*ROS node handler*/
      ros::NodeHandle nh_;
      ros::NodeHandle nh_private_;
      /*ROS Publisher*/
      ros::Publisher ros_pub_tau_;

      ros::Publisher ros_pub_tau_f_;
      ros::Publisher ros_pub_tau_p_f_;
      ros::Publisher ros_pub_tau_CT_;
      ros::Publisher ros_pub_tau_SP_;
      ros::Publisher ros_pub_tau_friction_;
      ros::Publisher ros_pub_q_p_f_;
      ros::Publisher ros_pub_TaskSpace_traj_actual_;
      ros::Publisher ros_pub_TaskSpace_traj_desired_;
      ros::Publisher ros_pub_jointState_actuel_;
      //ros::Publisher ros_pub_ProMP_traj_;

      ros::Publisher ros_pub_JointSpace_error_; /*publish joint space controler joint errors*/
      ros::Publisher ros_pub_TaskSpace_error_; /*publish task space controler pose errors*/
      ros::Publisher ros_pub_JointSpace_precurve_;
      #if defined(ACIN_CONTROLLER_DEBUG)
        ros::Publisher ros_pub_time_meas_; /*publish task space controler pose errors*/
      #endif
      ros::Publisher ros_pub_motion_EN_;
      /*ROS Subscriber*/
      ros::Subscriber sub_state_;
      //ros::Subscriber sub_ProMP_traj_;
      ros::Subscriber sub_Task_traj_;
      ros::Subscriber sub_Joint_traj_;
      ros::Subscriber sub_Tool_param_;
      ros::Subscriber sub_NS_state_;
      ros::Subscriber sub_gravity_mode_;
      ros::Subscriber sub_coordAdjustemt_;
      ros::Subscriber sub_sollPos_;
      ros::Subscriber sub_motion_EN_;
      ros::Subscriber sub_friction_comp_;
      ros::Subscriber sub_singularPert_;
      /*ROS Timer*/
      ros::Timer timer_report_;
      ros::Timer timer_status_report_;

      /* flags*/
      bool friction_comp_ROS_flag_ = false;
      //bool motion_EN_in_flag_;
      bool SingularPert_ROS_flag_ = false;
      void TaskSpace_trajectoryCallback_ROS(const TaskSpaceTraj_t& TaskSpace_traj){
        this->TaskSpace_trajectoryCallback(TaskSpace_traj);
      }
      void stateCallback_ROS(const JointState_t& state){
        ros_pub_jointState_actuel_.publish(state);
        this->stateCallback(state);
        Joint_t tau_FB_m = Joint_t();
        tau_FB_m.Joints=std::vector<double>(_ROBOT_q_DIM_,0);
        for(uint16_t ii=0; ii < _ROBOT_q_DIM_ ; ii++ ){
          tau_FB_m.Joints[ii]  = state.effort[ii];
        }

        this->torque_Motor_feedback_Callback(tau_FB_m);
      }
      void JointSpace_trajectoryCallback_ROS(const JointSpaceTraj_t& JointSpace_traj){
        this->JointSpace_trajectoryCallback(JointSpace_traj);
      }
      void ToolParam_Callback_ROS(const ToolParam_t& Tool_param){
        this->ToolParam_Callback(Tool_param);
      }


      void jointNSstateCallback_ROS(const JointState_t& state){
        this->jointNSstateCallback(state);
      }
      void GravityModeCallback_ROS(const boolMsg_t& gravity_mode){
        this->GravityModeCallback(gravity_mode);
      }
      void MotionENCallback_ROS(const boolMsg_t& motion_EN_msg){
        ros_pub_motion_EN_.publish(motion_EN_msg);
        if (this->ctrl_motion_en_ == false && motion_EN_msg.data == true){
          this->ctrl_motion_enable();
        }
        if (this->ctrl_motion_en_ == true && motion_EN_msg.data == false){
           this->ctrl_motion_disable();
        }
        //motion_EN_in_flag_ = motion_EN_msg.data;
      }
      void coordAdj_ROS(const CoordAdj_t &coord_adj){
        this->coord_adj(coord_adj);
      }
      void acinSollPosCallback_ROS(const acinSollPosMsg_t& sollPosMsg){
        this->desired_sollPosCallback(sollPosMsg);
      }
      void FrictionCompCallback_ROS(const boolMsg_t &frictionCompMsg){
        if (this->friction_comp_ROS_flag_ == false  && frictionCompMsg.data == true){
          ROS_WARN("Enable friction Compensation ROS");
          this->CTRL_friction_comp_enable();
        }else{
          if (this->friction_comp_ROS_flag_ == true  && frictionCompMsg.data == false){
            ROS_WARN("Disable friction Compensation");
            this->CTRL_friction_comp_disable();
          }
        }
        this->friction_comp_ROS_flag_ =  frictionCompMsg.data;

      }
      void SingularPertCallback_ROS(const boolMsg_t &SingularPertMsg){
        if (this->SingularPert_ROS_flag_ == false  && SingularPertMsg.data == true){
          ROS_WARN("Enable Singular Pertubation");
          this->CTRL_SingularPert_enable();
        }else{
          if (this->SingularPert_ROS_flag_ == true  && SingularPertMsg.data == false){
            ROS_WARN("Disable Singular Pertubation");
            this->CTRL_SingularPert_disable();
          }
        }
        this->SingularPert_ROS_flag_ =  SingularPertMsg.data;

      }

      void ros_init(){
        RobotParams &robot_params_ = this->robot_params_;
        double &mE_ = this->mE_;
        Eigen::Vector3d &sE_ = this->sE_;
        /*define publish topics*/
        ros_pub_tau_ = nh_.advertise<Joint_t>("/ros_acin_gazebo_robot_plugin/joint_effort", 1); // actuel effort, singular pertubation as well as friction compensation if activated
        ros_pub_tau_f_ = nh_.advertise<Joint_t>("Tau_f", 1);
        ros_pub_tau_p_f_ = nh_.advertise<Joint_t>("Tau_p_f", 1);
        ros_pub_tau_CT_ = nh_.advertise<Joint_t>("Tau_CT", 1);// CT controller calculated effort
        ros_pub_tau_SP_ = nh_.advertise<Joint_t>("Tau_SP", 1);//singular pertubation torque controller
        ros_pub_tau_friction_ = nh_.advertise<Joint_t>("Tau_friction", 1);// CT controller calculated effort
        ros_pub_q_p_f_ = nh_.advertise<Joint_t>("q_p_f", 1);
        ros_pub_TaskSpace_traj_actual_ = nh_.advertise<PoseState_t>("TaskSpace_traj_actual", 1); //actuel taskspace trajectory of the robot endeffector, calculated from the current senesor values of the joints
        ros_pub_TaskSpace_traj_desired_ = nh_.advertise<PoseState_t>("TaskSpace_traj_desired", 1); //trajectory interpolated for the current controller, bspline spline trajectory,
        ros_pub_TaskSpace_error_ = nh_.advertise<PoseState_t>("TaskSpace_error", 1);
  //      ros_pub_ProMP_traj_ = nh_.advertise<PoseState_t>("ctrl_ProMP_intpl_traj", 1);
        ros_pub_JointSpace_error_ = nh_.advertise<JointState_t>("JointSpace_error", 1);
        ros_pub_JointSpace_precurve_ = nh_.advertise<JointState_t>("JointSpace_precurve", 1);
        ros_pub_jointState_actuel_ = nh_.advertise<JointState_t>("joint_state", 1);
        #if defined(ACIN_CONTROLLER_DEBUG)
          ros_pub_time_meas_ = nh_.advertise<calcTimeMeasurement_t>("calcTimeMeasurement",1); /*publish task space controler pose errors*/
        #endif
        ros_pub_motion_EN_ = nh_.advertise<boolMsg_t>(std::string("/ros_acin_gazebo_robot_plugin/") + "motion_EN", 1);
        /*define subscribe topics*/
        sub_state_ = nh_.subscribe("/ros_acin_gazebo_robot_plugin/joint_state", 1, &ROS_ACIN_Controller_Node_C<N_knots_>::stateCallback_ROS, this);
        //sub_ProMP_traj_ = nh_.subscribe("/ProMP_trajectory", 1, &ACIN_Controller_Node_C::ProMP_trajectoryCallback, this);
        sub_Task_traj_ = nh_.subscribe("desired_TaskSpace_traj", 1, &ROS_ACIN_Controller_Node_C<N_knots_>::TaskSpace_trajectoryCallback_ROS, this);
        sub_Joint_traj_ = nh_.subscribe("desired_JointSpace_traj", 1, &ROS_ACIN_Controller_Node_C<N_knots_>::JointSpace_trajectoryCallback_ROS, this);
        sub_Tool_param_ = nh_.subscribe("set_Tool_param", 1, &ROS_ACIN_Controller_Node_C<N_knots_>::ToolParam_Callback_ROS,this);
        sub_NS_state_ = nh_.subscribe("desired_Nullspace_state", 1, &ROS_ACIN_Controller_Node_C<N_knots_>::jointNSstateCallback_ROS, this);
        sub_gravity_mode_ = nh_.subscribe("gravity_comp_mode", 1, &ROS_ACIN_Controller_Node_C<N_knots_>::GravityModeCallback_ROS, this);
        sub_coordAdjustemt_ = nh_.subscribe("coordAdjustment", 1, &ROS_ACIN_Controller_Node_C<N_knots_>::coordAdj_ROS, this);
        sub_motion_EN_ = nh_.subscribe("motion_EN", 1, &ROS_ACIN_Controller_Node_C<N_knots_>::MotionENCallback_ROS, this);
        sub_friction_comp_ = nh_.subscribe("friction_comp", 1, &ROS_ACIN_Controller_Node_C<N_knots_>::FrictionCompCallback_ROS, this);
        sub_singularPert_ = nh_.subscribe("SingularPert", 1, &ROS_ACIN_Controller_Node_C<N_knots_>::SingularPertCallback_ROS, this);
        sub_sollPos_ = nh_.subscribe("SetSollPos",1,&ROS_ACIN_Controller_Node_C<N_knots_>::acinSollPosCallback_ROS,this);
        //######
        bool hanging;

        nh_.param<bool>("hanging", hanging, false); // node paramters are set from argc,argv (via launch file or via comandline args)
        if(hanging){
          robot_params_.g = -robot_params_.g;
        }


        std::string robot_base_position_str;
        nh_.param<std::string>("robot_base_position", robot_base_position_str, "-x 0.0 -y 0.0 -z 0.0 -Y 0.0 -P 0.0 -R 0.0"); // node paramters are set from argc,argv (via launch file or via comandline args)
        boost::erase_all ( robot_base_position_str, " " );
        std::vector<std::string> robot_base_position_str_split;

        std::vector<double> robot_base_pose(6,0.0);
        const std::vector<char> pose_coord_str = {'x','y','z','Y','P','R'};
        //std::cout<<"robot_base_position_str"<<robot_base_position_str<<std::endl;
        boost::split ( robot_base_position_str_split, robot_base_position_str, boost::is_any_of ( "-"));
        for(uint64_t ii = 0; ii < pose_coord_str.size(); ii++){
          robot_base_pose[ii] = 0.0;
          try{
            bool found_pose = false;
            for(uint64_t jj= 0; jj <robot_base_position_str_split.size();jj++){
              if(robot_base_position_str_split[jj].size()>=2){
                if (pose_coord_str[ii] == robot_base_position_str_split[jj][0]){
                  robot_base_pose[ii] = std::stod(robot_base_position_str_split[jj].substr(1));
                  found_pose = true;
                  break;
                }
              }
            }
            if(!found_pose){
              ACIN_CTRL_WARN("cant get cant find robot pose coord \"%c\",set default to 0.0", pose_coord_str[ii]);
            }
          }catch(const std::invalid_argument& ia){
            ROS_WARN_STREAM("Invalid argument: " << ia.what() << '\n');
            ACIN_CTRL_WARN("cant get robot_base_pose coord \"%c\", convert error of found pose, set default to 0.0", pose_coord_str[ii]);
            robot_base_pose[ii] = 0.0;
          }
        }
       HomogeneousTransformation_t H_w_B_new;
        RotationMatrix_t R_z = RotationMatrix_t(Eigen::AngleAxisd(robot_base_pose[3], Eigen::Vector3d::UnitZ()).toRotationMatrix());
        RotationMatrix_t R_y = RotationMatrix_t(Eigen::AngleAxisd(robot_base_pose[4], Eigen::Vector3d::UnitY()).toRotationMatrix());
        RotationMatrix_t R_x = RotationMatrix_t(Eigen::AngleAxisd(robot_base_pose[5], Eigen::Vector3d::UnitX()).toRotationMatrix());
        RotationMatrix_t R_d = R_z * R_y * R_x;
        H_w_B_new.block<3,3>(0,0) = R_d;
        H_w_B_new(0,3) = robot_base_pose[0];
        H_w_B_new(1,3) = robot_base_pose[1];
        H_w_B_new(2,3) = robot_base_pose[2];
        H_w_B_new(3,3) = 1;
        ROS_WARN_STREAM("H_w_B"<<H_w_B_new<<"\n");
        this->H_w_B_ = H_w_B_new;
        this->R_w_B_ = H_w_B_new.block<3,3>(0,0);
        this->d_w_B_ = H_w_B_new.block<3,1>(0,3);

        //######
        //######
        /*####################################################*/
        /* mass compensation in the model of the robot by simple adding discrete mass to the endeffector of the robot, !*/
        /*####################################################*/
        /* set mass*/
        nh_.param<double>("mE", this->mE_, 0.0); // node paramters are set from argc,argv (via launch file or via comandline args)
        /*set new  center of gravity*/
        std::string sE_str;
        nh_.param<std::string>("sE", sE_str, "0.0,0.0,0.0"); // node paramters are set from argc,argv (via launch file or via comandline args)
        boost::erase_all ( sE_str, " " );
        std::vector<std::string> sE_split;
        boost::split ( sE_split, sE_str, boost::is_any_of ( "," ) );
        for(uint64_t ii = 0; ii < sE_.size(); ii++){
          if(ii < sE_split.size()){
            try{
              this->sE_[ii] = std::stod(sE_split[ii]);
            }catch(const std::invalid_argument& ia){
              ROS_WARN_STREAM("Invalid argument: " << ia.what() << '\n');
              ACIN_CTRL_WARN("coordinate (%lu,%ld) from CoG of the endeffector mass not well defined, set to default 0.0",ii+1, sE_.size());
              this->sE_[ii] = 0.0;
            }
          }else{
            ACIN_CTRL_WARN("coordinate (%lu,%ld) from CoG of the endeffector mass not well defined, set to default 0.0",ii+1, sE_.size());
            this->sE_[ii] = 0.0;
          }

        }
        ROS_INFO_STREAM("Mass link 7:= "<<robot_params_.m7_);
        ROS_INFO_STREAM("CoG link 7:=\n\tx:"<< robot_params_.sp7x_<<", y:"<<robot_params_.sp7y_<<", z:"<< robot_params_.sp7z_);
        robot_params_.m7 = robot_params_.m7_ + mE_;
        robot_params_.sp7x = (robot_params_.m7_*robot_params_.sp7x_ + mE_*sE_[0])/robot_params_.m7; // m7 = m7_(link 7) +  mass of the endeffector tool
        robot_params_.sp7y = (robot_params_.m7_*robot_params_.sp7y_ + mE_*sE_[1])/robot_params_.m7;
        robot_params_.sp7z = (robot_params_.m7_*robot_params_.sp7z_ + mE_*sE_[2])/robot_params_.m7;
        ROS_INFO_STREAM("Mass link 7 with tool:= "<<robot_params_.m7);
        ROS_INFO_STREAM("CoG link 7 with tool:= \n\tx:"<< robot_params_.sp7x<<", y:"<<robot_params_.sp7y<<", z:"<< robot_params_.sp7z);
        /*####################################################*/
        /*init Postion of the robot, !*/
        /*####################################################*/
        /* set q0 */
        std::string joint_null_space_init_pos_str;
        nh_.param<std::string>("q0", joint_null_space_init_pos_str, "1.5708, 0.2266, -0.0000, 1.7712, -0.0000, 1.5446, -1.5708"); // node paramters are set from argc,argv (via launch file or via comandline args)
        boost::erase_all ( joint_null_space_init_pos_str, " " );
        std::vector<std::string> joint_null_space_init_pos_split;
        boost::split ( joint_null_space_init_pos_split, joint_null_space_init_pos_str, boost::is_any_of ( "," ) );
        JointVector_t q_init_0;
        for(uint64_t ii = 0; ii < q_init_0.size(); ii++){
          if(ii < joint_null_space_init_pos_split.size()){
            try{
              q_init_0[ii] = std::stod(joint_null_space_init_pos_split[ii]);
            }catch(const std::invalid_argument& ia){
              ROS_WARN_STREAM("Invalid argument: " << ia.what() << '\n');
              ACIN_CTRL_WARN("init. joint null space position (%lu,%ld) value not well defined, set to default 0.0",ii+1, q_init_0.size());
              q_init_0[ii] = 0.0;
            }
          }else{
            ACIN_CTRL_WARN("init. joint position (%lu,%ld) value not defined, set to default 0.0",ii+1, q_init_0.size());
            q_init_0[ii] = 0.0;
          }
        }
        ACIN_CTRL_INFO("\nRobot Initial Joint Space Configuration used if no y0 is given:");
        ACIN_CTRL_INFO("\n\t\tq0_init:= %f, q1_init:= %f, q2_init:= %f, q3_init:= %f, q4_init:= %f, q5_init:= %f, q6_init:= %f",
                  q_init_0(0,0),q_init_0(1,0),q_init_0(2,0),q_init_0(3,0),q_init_0(4,0),q_init_0(5,0), q_init_0(6,0));
        nh_.param<double>("T_init", this->T_init_0_, 10.0); // node paramters are set from argc,argv (via launch file or via comandline args)
        ACIN_CTRL_INFO("\n\t\t desired time for reaching this joint space position:= %f [s]",this->T_init_0_);

        /*####################################################*/
        /*init Postion of the robot, !*/
        /*####################################################*/
        /* set q0 */
        JointVector_t &q_d_NS_ = this->q_d_NS_;
        std::string joint_init_pos_str;
        nh_.param<std::string>("q0_d_NS", joint_init_pos_str, "0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0"); // node paramters are set from argc,argv (via launch file or via comandline args)
        boost::erase_all ( joint_init_pos_str, " " );
        std::vector<std::string> joint_init_pos_split;
        boost::split ( joint_init_pos_split, joint_init_pos_str, boost::is_any_of ( "," ) );
        for(uint64_t ii = 0; ii < q_d_NS_.size(); ii++){
          if(ii < joint_init_pos_split.size()){
            try{
              q_d_NS_[ii] = std::stod(joint_init_pos_split[ii]);
            }catch(const std::invalid_argument& ia){
              ROS_WARN_STREAM("Invalid argument: " << ia.what() << '\n');
              ACIN_CTRL_WARN("init. joint position (%lu,%ld) value not well defined, set to default 0.0",ii+1, q_d_NS_.size());
              q_d_NS_[ii] = 0.0;
            }
          }else{
            ACIN_CTRL_WARN("init. joint position (%lu,%ld) value not defined, set to default 0.0",ii+1, q_d_NS_.size());
            q_d_NS_[ii] = 0.0;
          }
        }
        std::ostringstream ros_print_s("");
        ros_print_s << "\nRobot Initial Joint Null Space Configuration:";
        ros_print_s << string_sprintf("\n\t\tq0_d_NS_init:= %f, q1_d_NS_init:= %f,q2_d_NS_init:= %f,q3_d_NS_init:= %f, q4_d_NS_init:= %f,q5_d_NS_init:= %f, q6_d_NS_init:= %f",
                  q_d_NS_(0,0),q_d_NS_(1,0),q_d_NS_(2,0),q_d_NS_(3,0),q_d_NS_(4,0),q_d_NS_(5,0), q_d_NS_(6,0));
        ACIN_CTRL_INFO_STREAM(ros_print_s);
        this->qN_log_barrier_flag_  = false;
        this->qN_max_FoM_flag_ = false;
        /*####################################################*/
        /*Pose Start Postion !*/
        /*####################################################*/
        CartesianVector_t y_start_ =CartesianVector_t::Zero();
        HomogeneousTransformation_t H_q_init_0;
        evalT0E(q_init_0,H_q_init_0,robot_params_);
        Eigen::Quaternion<double> quat_init_0 = Eigen::Quaternion<double>(H_q_init_0.block<3,3>(0,0));
        auto euler_init_0 = quat_init_0.toRotationMatrix().eulerAngles(2, 1, 0);
        std::string y_init_pos_str;
        std::ostringstream y_0_default;
        y_0_default << H_q_init_0(0,3)<<",";
        y_0_default << H_q_init_0(1,3)<<",";
        y_0_default << H_q_init_0(2,3)<<",";
        y_0_default << euler_init_0(0)<<",";
        y_0_default << euler_init_0(1)<<",";
        y_0_default << euler_init_0(2);
        //#############
        ros_print_s << "\n y_0_default(form q_0_init_):\n\t";
        ros_print_s << y_0_default.str();
        ACIN_CTRL_INFO_STREAM(ros_print_s);
        //#############
        nh_.param<std::string>("y0", y_init_pos_str, y_0_default.str());
        //#############
        ros_print_s << "\n y_init:\n\t";
        ros_print_s << y_init_pos_str<<std::endl;
        ACIN_CTRL_INFO_STREAM(ros_print_s);
        //#############
        boost::erase_all ( y_init_pos_str, " " );
        std::vector<std::string> y_init_pos_split;
        boost::split ( y_init_pos_split, y_init_pos_str, boost::is_any_of ( "," ) );
        for(uint64_t ii = 0; ii < y_start_.size(); ii++){
          if(ii < y_init_pos_split.size()){
            try{
              y_start_[ii] = std::stod(y_init_pos_split[ii]);
            }catch(const std::invalid_argument& ia){
              ROS_WARN_STREAM("Invalid argument: " << ia.what() << '\n');
              ACIN_CTRL_WARN("init. y0 (%lu,%ld) value not well defined, set to defaul 0.0",ii+1, y_start_.size());
              y_start_[ii] = 0.0;
            }
          }else{
            ACIN_CTRL_WARN("init. y0 (%lu,%ld) value not defined, set to default 0.0",ii+1, y_start_.size());
            y_start_[ii] = 0.0;
          }
        }

        /*####################################################*/
        /*####################################################*/
        /*####################################################*/
        timer_status_report_ = nh_.createTimer(ros::Duration(10), boost::bind(&ROS_ACIN_Controller_Node_C::status_report_cb, this, _1),false);
        struct ControllerParams ControllerParams_ros;
        ControllerParams_ros.K0_joint(2, 2) = 4000;
        ControllerParams_ros.K0_joint(3, 3) = 4000;
        this->setControllerParam(ControllerParams_ros);
      }
      void sensor_timer_cb(const ros::TimerEvent&){
        this->FIRST_SENSOR_VALUE_ = false;
        this->INIT_POS_REACHED_ = false;
        this->xq_I_ = JointVector_t::Zero();//reset joint space controler integrator state
        this->dalay_task_space_ = 0;
        ACIN_CTRL_WARN("Lost Sensor Signal for joint states and velocities");
      }
      void status_report_cb(const ros::TimerEvent&){
        if (this->verbose_){
          ACIN_CTRL_WARN("gravity mode_ %d",this->gravity_comp_);
          ACIN_CTRL_WARN("safe_mode_ %d",this->safe_mode_);
          // double t_now = get_cur_time();
          // ACIN_CTRL_INFO("time:= %f",t_now);
          // ACIN_CTRL_INFO("eq0:= %f, eq1:= %f,eq2:= %f,eq3:= %f, eq4:= %f,eq5:= %f, eq6:= %f",
                     //e_q_(0,0),e_q_(1,0),e_q_(2,0),e_q_(3,0),e_q_(4,0),e_q_(5,0), e_q_(6,0));
          // ACIN_CTRL_INFO("q0:= %f, q1:= %f,q2:= %f,q3:= %f, q4:= %f,q5:= %f, q6:= %f",
          //               q_d_(0,0),q_(1,0),q_(2,0),q_(3,0),q_(4,0),q_(5,0), q_(6,0));
          ACIN_CTRL_INFO("q0:= %f, q1:= %f,q2:= %f,q3:= %f, q4:= %f,q5:= %f, q6:= %f",
                       this->q_d_(0,0),this->q_d_(1,0),this->q_d_(2,0),this->q_d_(3,0),this->q_d_(4,0),this->q_d_(5,0), this->q_d_(6,0));
          // ACIN_CTRL_ERROR("e_pose_(0):= %f, e_pose_(1):= %f,e_pose_(2):= %f,e_pose_(3) [q_w]:= %f, e_pose_(4)[q_x]:= %f,e_pose_(5)[q_y]:= %f, e_pose_(6)[q_z]:= %f",
          //                 e_pose_(0,0),e_pose_(1,0),e_pose_(2,0),e_pose_(3,0),e_pose_(4,0),e_pose_(5,0), e_pose_(6,0));
          // ACIN_CTRL_INFO("pose_.block<3,1>(0,0)-pose_init_.block<3,1>(0,0)).norm() := %1.8f",(pose_.block<3,1>(0,0)-pose_init_.block<3,1>(0,0)).norm());
          for(int i = 0; i < this->q_.rows(); ++i)
          {
            if(this->q_(i)>this->robot_params_.q_limit_upper[i]){
              ACIN_CTRL_WARN("joint %i over the upper limit (q:=%f ,ub:= %f)",i,this->q_(i),this->robot_params_.q_limit_upper[i]);
            }
            if(this->q_(i)<this->robot_params_.q_limit_lower[i]){
              ACIN_CTRL_WARN("joint %i under the lower limit (q:=%f ,lb:= %f)",i,this->q_(i),this->robot_params_.q_limit_lower[i]);
            }
          }
        }
      }

};
#endif
