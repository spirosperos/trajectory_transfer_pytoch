#include "ros_acin_gazebo_robot_plugin/ros_acin_gazebo_robot_plugin.h"
#include <boost/bind.hpp>
#include <boost/algorithm/string/erase.hpp>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string.hpp>
#include <stdio.h>
#include <iostream>
/*msg type*/

#define HOLD_FRAMES 100

namespace gazebo
{

  ModelPush::~ModelPush(){
    init_pose_set_ = false;
    gravity_mode_off_ = false;
    apply_ctrl_torque_ = false;
    apply_last_torque_ = false;
    nh_.shutdown();
    update_connection_ = NULL;

  }
  void ModelPush::reset_simulation(){
    init_pose_set_ = false;
    gravity_mode_off_ =true;
    apply_ctrl_torque_ = false;
    apply_last_torque_ = false;
    reset_thread_started_ = false;
    nh_.shutdown();
    nh_ = ros::NodeHandle("ros_acin_gazebo_robot_plugin");
    //topics on which are published
    joint_state_pub_ = nh_.advertise<sensor_msgs::JointState>("joint_state", 1);
    //topics on which are subscribed
    joint_effort_sub_ = nh_.subscribe("joint_effort", 1, &ModelPush::JointEffortCallback, this);
    dis_FT_sub_ = nh_.subscribe("disturbance_FT", 1, &ModelPush::disturbanceFTCallback, this);
    motion_EN_sub_ = nh_.subscribe("motion_EN", 1, &ModelPush::MotionENCallback, this);
    pauseGazebo_ = nh_.serviceClient<std_srvs::Empty>("/gazebo/pause_physics");
    unpauseGazebo_ = nh_.serviceClient<std_srvs::Empty>("/gazebo/unpause_physics");
    past_last_torque_counter_ = HOLD_FRAMES;

  }
  void ModelPush::stopMotion(){
    model_->SetGravityMode(false);
    apply_ctrl_torque_ = false;
    apply_last_torque_ = false;
    gravity_mode_off_ = true;
    for (int i = 0; i < joints_.size(); i++)
    {
      joints_[i]->SetForce(0, 0.0);
      joints_[i]->SetVelocity(0, 0.0);

    }
  }
  void ModelPush::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
  {
    reset_simulation();

    // Store the pointer to the model
    model_ = _parent;
    model_sdf_ptr_ = _sdf;

    world_ = model_->GetWorld();
    robot_namespace_ = model_->GetName ();
    // Listen to the update event. This event is broadcast every
    // simulation iteration.
    update_connection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&ModelPush::OnUpdate, this, _1));
    //set ros node handler
    //#######################################

    this->ModelLinks_ = _parent->GetLinks (); //Link_V type-> std_vector<LinkPtr> boost pointer to link object
    for ( uint64_t ii = 0; ii< ModelLinks_.size(); ii++ ) {
        // link_.push_back ( this->parent_->GetLink ( link_names_[i] ) );
        ROS_INFO_NAMED("ros_acin_gazebo_robot_plugin", "links: %s", ModelLinks_[ii]->GetName().c_str() );
    }
    //#######################################
    this->ModelJoints_ = _parent->GetJoints (); //Link_V type-> std_vector<LinkPtr> boost pointer to link object
    for ( uint64_t ii = 0; ii< ModelJoints_.size(); ii++ ) {
        // link_.push_back ( this->parent_->GetLink ( link_names_[i] ) );
        ROS_INFO_NAMED("ros_acin_gazebo_robot_plugin", "joints: %s", ModelJoints_[ii]->GetName().c_str() );
    }
    //#######################################
    this->jointTorque_feq_ = 0.0;
    if ( !_sdf->HasElement ( "JointTorque_updateRate" ) ) {
        ROS_WARN_NAMED("ros_acin_gazebo_robot_plugin", "Gazebo Robot Plugin (ns = %s) missing <JointTorque_updateRate>, defaults to %f",
                   this->robot_namespace_.c_str(), this->jointTorque_feq_ );
    } else {
        this->jointTorque_feq_ = _sdf->GetElement ( "updateRate" )->Get<double>();
    }
    // Initialize update rate stuff
    if ( this->jointTorque_feq_ > 0.0 ) {
        this->jointTorque_T_ = 1.0 / this->jointTorque_feq_;
    } else {
        this->jointTorque_T_ = 0.0;
    }
    std::string joint_prefix = "";
    if ( !_sdf->HasElement ( "jointPrefix" ) ) {
      joint_prefix = "iiwa_joint_";
      ROS_WARN_NAMED("ros_acin_gazebo_robot_plugin", "No \"jointPrefix\" given in sdf, use default \"%s\"",joint_prefix.c_str());
    }else{
      sdf::ElementPtr element = _sdf->GetElement ( "jointPrefix" );
      joint_prefix = element->Get<std::string>();
    }
    if ( !_sdf->HasElement ( "jointNames" ) ){
      ROS_WARN_NAMED("ros_acin_gazebo_robot_plugin", "No jointNames given, use default:");
      for(uint16_t ii = 0 ; ii < 7; ii++){
        joint_names_.emplace_back(joint_prefix + std::to_string(ii+1));
        ROS_WARN_NAMED("ros_acin_gazebo_robot_plugin","\t%s",joint_names_[ii].c_str());
      }

    }else{
        sdf::ElementPtr element = _sdf->GetElement ( "jointNames" ) ;
        std::string joint_names = element->Get<std::string>();
        boost::erase_all ( joint_names, " " );
        boost::split ( joint_names_, joint_names, boost::is_any_of ( "," ) );
    }
    if ( !_sdf->HasElement ( "joint_init_pos" ) ) {
        ROS_WARN_NAMED("ros_acin_gazebo_robot_plugin", "No \"joint_init_pos\" given in sdf,  use default:");
        for(uint16_t ii = 0 ; ii < 7; ii++){
          joint_init_pos_.emplace_back(0.0);
          ROS_WARN_NAMED("ros_acin_gazebo_robot_plugin","\t%s(t=0): %f",joint_names_[ii].c_str(),joint_init_pos_[ii]);
        }
    }else{
        sdf::ElementPtr element = _sdf->GetElement ( "joint_init_pos" ) ;
        std::string joint_init_pos_str = element->Get<std::string>();
        boost::erase_all ( joint_init_pos_str, " " );
        std::vector<std::string> joint_init_pos_split;
        boost::split ( joint_init_pos_split, joint_init_pos_str, boost::is_any_of ( "," ) );
        ROS_INFO("init. pos:=\n");
        for(uint16_t ii = 0 ; ii < joint_init_pos_split.size(); ii++){
          printf("\t\t%i.joint:=%s\n",ii,joint_init_pos_split[ii].c_str());
          double q0_ii =0.0;
          try{
            q0_ii = std::stod(joint_init_pos_split[ii]);
            joint_init_pos_.emplace_back(q0_ii);
          }catch(const std::invalid_argument& ia){
            ROS_WARN_STREAM("Invalid argument: " << ia.what() << '\n');
            ROS_WARN("init. joint postion (%d,%ld) value not well defined, set to defaul %f",ii+1, joint_init_pos_split.size(), q0_ii);
            joint_init_pos_.emplace_back(q0_ii);
          }
        }

    }
    for (uint32_t ii = 0; ii< joint_names_.size(); ii++ ){
      physics::JointPtr _joint_ptr = model_->GetJoint( joint_names_[ii] );
      if(_joint_ptr == NULL){
        ROS_ERROR("Invalid joint, Joint named \"%s\" not loaded in the model",joint_names_[ii].c_str());
        exit(-1);
      }
      // printf("%s\n",_joint_ptr->GetName().c_str());
      joints_.emplace_back( _joint_ptr );
    }
    printf("%lu\n",joints_.size());
    //----------------
    ROS_INFO("Finished loading IIWA Plugin.");
  }
  void ModelPush::disturbanceFTCallback(const ros_acin_gazebo_robot_plugin::FTdisturbance::ConstPtr& _msg){
    if(_msg->JointLinkName.size() != _msg->ForceTorque.size()){
      return;
    }
    past_FT_disturbance_.clear();
    past_FT_dis_LinkJoint_Ptr_idx_.clear();
    past_FT_link_.clear();
    for(uint64_t kk = 0; kk< _msg->JointLinkName.size(); kk++ ){
      bool jointLink_valid = false;
      for(uint64_t ii  = 0 ; ii < ModelLinks_.size(); ii++){
        if(ModelLinks_[ii]->GetName() == _msg->JointLinkName[kk]){
          if(jointLink_valid ==true){
            ROS_ERROR_NAMED("ros_acin_gazebo_robot_plugin","not unique Link names, apply disturbance detected links with the same name");
          }
          past_FT_dis_LinkJoint_Ptr_idx_.push_back(ii);
          past_FT_link_.push_back(true);
          jointLink_valid = true;
        }
      }
      for(uint64_t ii  = 0 ; ii < ModelJoints_.size(); ii++){
        if(ModelJoints_[ii]->GetName() == _msg->JointLinkName[kk]){
          if(jointLink_valid ==true){
            ROS_ERROR_NAMED("ros_acin_gazebo_robot_plugin","not unique joint names, apply disturbance detected joints with the same name");
          }
          past_FT_dis_LinkJoint_Ptr_idx_.push_back(ii);
          past_FT_link_.push_back(false);
          jointLink_valid = true;
        }
      }
      if(jointLink_valid == true){
        std::vector<double> FT_disturbance(6, 0.0);
        //ROS_ASSERT(_msg->ForceTorque[kk].data.size() == 6);
        if(_msg->ForceTorque[kk].data.size() < 5){
          ROS_WARN("Not all FT values given in the MSG");
        }
        for (uint64_t ii = 0; ii < _msg->ForceTorque[kk].data.size(); ii++)
        {
          if (ii < 6){
            FT_disturbance[ii] = _msg->ForceTorque[kk].data[ii];
          }else{
              break;
          }

        }
        past_FT_disturbance_.push_back(FT_disturbance);
      }
    }

    FT_dis_Duration_.nsec = _msg->T.data.nsec;
    FT_dis_Duration_.sec = _msg->T.data.sec;
    FT_dis_start_ = this->world_->SimTime();
    FT_dis_apply_count_ = 0;
    ROS_ASSERT(past_FT_disturbance_.size() == past_FT_link_.size());
    ROS_INFO("get new disturbance Message at SimTime %i sec, %i nsec\n",FT_dis_start_.sec,FT_dis_start_.nsec);
    ROS_INFO("Duration of Force application %i sec, %i nsec\n",FT_dis_Duration_.sec,FT_dis_Duration_.nsec);
    ROS_INFO("Number of valid joint/link disturbances %lu in the message\n",past_FT_link_.size());
  }
  void ModelPush::JointEffortCallback(const ros_acin_robot_support::Joints::ConstPtr& _msg)
  {
    //ROS_INFO("NEW_JOINTEFFORT\n");
    boost::unique_lock<boost::timed_mutex> lock{EffortMutex_, boost::try_to_lock};
    if(lock.owns_lock() || lock.try_lock_for(boost::chrono::microseconds{1})){
      apply_ctrl_torque_ = false;
      ctrl_torque_buf_.clear();

      // ctrl_torque_buf_.push_back(0);
      for (int i = 0; i < _msg->Joints.size(); i++)
      {
        ctrl_torque_buf_.push_back(_msg->Joints[i]);
      }
      if (ctrl_torque_buf_.size() != joints_.size()){
        ROS_ERROR("Number of commands recived from JointEffort Callback are not equal with the number of joints");
        exit(-1);
      }
      apply_ctrl_torque_ = true;
    }else{
      ROS_WARN("JointEffortCallback can't aquire lock");
    }


  }
  void ModelPush::MotionENCallback(const boolMsg_t::ConstPtr& _msg){
    ROS_WARN_NAMED("ros_acin_gazebo_robot_plugin","Motion Controll Callback");
    if (_msg->data){
      ROS_WARN_NAMED("ros_acin_gazebo_robot_plugin","Enable Motion !");
      motion_enable_ = true;
    }else{
      ROS_WARN_NAMED("ros_acin_gazebo_robot_plugin","Stop Motion !");
      motion_enable_ = false;
      stopMotion();
    }
  }

  void ModelPush::OnUpdate(const common::UpdateInfo& _info)
  {
    //ROS_INFO("onupdate\n");
     if (!reset_thread_started_){
       (void) pthread_create(&std_input_tId, 0, &ModelPush::std_input_event_thread, this);
       reset_thread_started_ = true;
     }
    if(std_input_tId != -1 and pthread_tryjoin_np(std_input_tId, NULL) == 0){
         reset_simulation();
    }
    if(!init_pose_set_){
      init_pose_set_ = true;
      model_->SetGravityMode(false);
      gravity_mode_off_ = true;
      motion_enable_ = false;
      for (uint32_t ii = 0; ii< joints_.size(); ii++ ){
        physics::JointControllerPtr JointController = model_->GetJointController ();
        JointController->SetJointPosition(joints_[ii], joint_init_pos_[ii]);
      }
      ROS_INFO_NAMED("ROS_ERROR_NAMED --","Simulation Gravity set to off, init pos. set!");
      // std_srvs::Empty pauseSrv;
      // pauseGazebo_.call(pauseSrv);
    }
    common::Time current_time = this->world_->SimTime();
    if ((current_time - FT_dis_start_) < FT_dis_Duration_ || FT_dis_Duration_.Double() == 0){
      if(past_FT_link_.size()!=0){
        if(FT_dis_apply_count_ == 0){
          printf("start applying FT disturbance @ %i sec - %i nsec\n",current_time.sec,current_time.nsec);
        }
        ModelPush::applyFTdisturbance();
      }
    }else{
      if(past_FT_link_.size()!=0){
        printf("end applying FT disturbance, Duratation was := %i sec and %i nsec\n",FT_dis_Duration_.sec,FT_dis_Duration_.nsec);
        past_FT_disturbance_.clear();
        past_FT_dis_LinkJoint_Ptr_idx_.clear();
        past_FT_link_.clear();
      }
    }
    ModelPush::applyJointTorque();
    ModelPush::publishJointStates(_info);
    ros::spinOnce();
  }
  void ModelPush::applyFTdisturbance(){
    for(uint64_t ii = 0; ii < past_FT_link_.size(); ii++){
      uint64_t LinkJoint_idx = past_FT_dis_LinkJoint_Ptr_idx_[ii];
      std::vector<double> FT =  past_FT_disturbance_[ii];
      if(past_FT_link_[ii] == true){
        physics::LinkPtr LinkPtr = ModelLinks_[LinkJoint_idx];//physics::Link_V
        // ROS_INFO("Apply Force/Torque to link %s",LinkPtr->GetName().c_str() );
      }else{
        physics::JointPtr JointPtr = ModelJoints_[LinkJoint_idx]; //physics::Joint_V
        // ROS_INFO("Apply Force/Torque to joint %s",JointPtr->GetName().c_str() );

      }
      // ROS_INFO("Force/Torque:= [%f, %f, %f, %f, %f,%f]",FT[0],FT[1],FT[2],FT[3],FT[4],FT[5]);

    }
    FT_dis_apply_count_ ++;
    if(FT_dis_apply_count_%300 ==0){
      common::Time current_time = this->world_->SimTime();
      printf("apply_count := %lu sec\n",FT_dis_apply_count_);
      printf("Time now:= %i sec\n",current_time.sec);
      printf("start Time := %i sec\n",FT_dis_start_.sec);
      printf("duration := %i sec\n",FT_dis_Duration_.sec);
    }
  }
  void ModelPush::applyJointTorque() {
    boost::unique_lock<boost::timed_mutex> lock{EffortMutex_, boost::try_to_lock};
    static bool motion_enable_last_ = false;
    if (motion_enable_ && apply_ctrl_torque_ && (lock.owns_lock() || lock.try_lock_for(boost::chrono::microseconds{50})))
    {
      past_last_torque_counter_ = 0;
      apply_last_torque_ = true;

      if (joints_.size()  != ctrl_torque_buf_.size()){
        ROS_ERROR("joints_.size() {%lu}  != ctrl_torque_buf_.size() {%lu} \n",joints_.size(), ctrl_torque_buf_.size());
        throw std::range_error("Toruq");
      }
      ROS_ASSERT(joints_.size()  == ctrl_torque_buf_.size());
      last_torque_buf_.clear();
      for (int i = 0; i < joints_.size() && i < ctrl_torque_buf_.size(); i++)
      {
        // Force is additive (multiple calls to SetForce to the same joint in the same time step
        // will accumulate forces on that Joint)?
        joints_[i]->SetForce(0, ctrl_torque_buf_[i]);
        last_torque_buf_.push_back(ctrl_torque_buf_[i]);
      }
      if (gravity_mode_off_ == true and apply_ctrl_torque_){
        model_->SetGravityMode(true);
        ROS_INFO("Gravity has benn enabled, mode_on");
        // std_srvs::Empty unpauseSrv;
        // unpauseGazebo_.call(unpauseSrv);
        gravity_mode_off_ = false;
      }
      apply_ctrl_torque_ = false;
    }else{
      past_last_torque_counter_ += 1;
      if (past_last_torque_counter_ >= HOLD_FRAMES)
      {
        apply_last_torque_ = false;
      }
      if (apply_last_torque_){
        //ROS_INFO("Hold Torque");
        for (int i = 0; i < joints_.size() && i < ctrl_torque_buf_.size(); i++)
        {
          // Force is additive (multiple calls to SetForce to the same joint in the same time step
          // will accumulate forces on that Joint)?
          //printf("%s -  Torque:= %f",joints_[i]->GetName().c_str(),ctrl_torque_buf_[i]);
          joints_[i]->SetForce(0, last_torque_buf_[i]);
        }
      }else{
        if(!lock.owns_lock()){
          ROS_ERROR("applyJointTorque can't aquire lock and exceeded hold frames, slow computer hardware/desyn, fallback, set gravity off and apply no Torques ?");
          for (int i = 0; i < joints_.size(); i++)
          {
            // Force is additive (multiple calls to SetForce to the same joint in the same time step
            // will accumulate forces on that Joint)?
            //printf("%s -  Torque:= %f",joints_[i]->GetName().c_str(),ctrl_torque_buf_[i]);
            joints_[i]->SetForce(0, 0);
          }
        }
      }
    }
    motion_enable_last_= motion_enable_;
  }
  void ModelPush::publishJointStates(const common::UpdateInfo& _info) {
    std::vector<double> ang_arr;
    std::vector<double> vel_arr;
    std::vector<double> effort_arr;
    std::vector<double> state_arr;
    for (int i = 0; i < joints_.size(); i++)
    {
      ang_arr.push_back(joints_[i]->Position(0));
      vel_arr.push_back(joints_[i]->GetVelocity(0));
      // get joint torque
      physics::JointWrench wrench = joints_[i]->GetForceTorque(0);
      ignition::math::Vector3d measured_torque;
      ignition::math::Vector3d measured_force;
      measured_torque = wrench.body2Torque;//torque applied to the child link defined in the child link frame
      measured_force = wrench.body2Force;//force applied to the child link deinfed in the child link frame
      ignition::math::Quaterniond rotation_child = (joints_[i]->GetChild()->InitialRelativePose()).Rot();//get rotation (R_li_qi) from the link frame (li) to current child links joint (qi) as a quaternion
      auto rot = ignition::math::Matrix3d(rotation_child.Inverse()); // TODO: not sure about this
      measured_torque = rot * measured_torque;
      //effort_arr.push_back((-1.0) * measured_torque.Z());//-1 because GetForceTorque
      if(gravity_mode_off_){
        effort_arr.push_back(std::numeric_limits<double>::quiet_NaN());
      }else{
        effort_arr.push_back(-1.0*measured_torque.Z());//-1 because GetForceTorque
      }
    }
    sensor_msgs::JointState msg;
    msg.header.stamp = ros::Time(_info.simTime.Double());
    msg.position = ang_arr;
    msg.velocity = vel_arr;
    msg.effort = effort_arr;
    joint_state_pub_.publish(msg);
  }


}
