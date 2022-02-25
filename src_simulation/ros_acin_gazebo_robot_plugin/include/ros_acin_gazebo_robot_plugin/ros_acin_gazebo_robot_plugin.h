#ifndef IIWA_GAZEBO_PLUGIN_H
#define IIWA_GAZEBO_PLUGIN_H

#include <ros/ros.h>
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Float64MultiArray.h"
#include <ros_acin_gazebo_robot_plugin/FTdisturbance.h>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <std_srvs/Empty.h>
#include <sensor_msgs/JointState.h>
#include <ros_acin_robot_support/Joints.h>
#include <std_msgs/Bool.h>
namespace gazebo
{
class ModelPush : public ModelPlugin
{
  typedef std_msgs::Bool boolMsg_t;
public:
  void JointEffortCallback(const ros_acin_robot_support::Joints::ConstPtr& _msg);
  void disturbanceFTCallback(const ros_acin_gazebo_robot_plugin::FTdisturbance::ConstPtr& _msg);
  void MotionENCallback(const boolMsg_t::ConstPtr& _msg);
public:
  void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);
  // Called by the world update start event
public:
  void OnUpdate(const common::UpdateInfo& _info);
  // Pointer to the model
private:
  void publishJointStates(const common::UpdateInfo& _info);
  void applyJointTorque();
  void applyFTdisturbance(void);
  void stopMotion(void);
  static void* std_input_event_thread(void* context){
    bool done = false;
    while (!done)
    {
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        char key_press = std::cin.get();
        switch (key_press)
        {
            case 'r':
              std::cout << "keypress deteceted r"<<std::endl;
              done = true;
              break;
            default:
              std::cout << "keypress deteceted:=" << key_press << std::endl;
              break;
        }
    }
    pthread_exit(0);
  }
  void reset_simulation(void);
  ~ModelPush();
private:

  ros::NodeHandle nh_;
  ros::Publisher joint_state_pub_;
  ros::Subscriber joint_effort_sub_;
  ros::Subscriber dis_FT_sub_;
  ros::Subscriber motion_EN_sub_;
  ros::ServiceClient pauseGazebo_;
  ros::ServiceClient unpauseGazebo_;

  sdf::ElementPtr model_sdf_ptr_;
  physics::ModelPtr model_;
  physics::WorldPtr world_;
  physics::Joint_V ModelJoints_; //physics::Joint_V
  physics::Link_V ModelLinks_; //physics::Link_V
  physics::Joint_V joints_;
  event::ConnectionPtr update_connection_;

  common::Time FT_dis_Duration_;
  common::Time FT_dis_start_;

  std::string robot_namespace_;
  std::vector<std::string> joint_names_;
  std::vector<double> joint_init_pos_;
  std::vector<float> ctrl_torque_buf_;
  std::vector<float> last_torque_buf_;
  // subscribe on force/torque topic which causes a disturbance on a desired link or in a disred joint, disred joint/link is encoded in the message as a string
  // appling d
  std::vector<std::vector<double>> past_FT_disturbance_;
  std::vector<uint64_t> past_FT_dis_LinkJoint_Ptr_idx_;
  std::vector<bool> past_FT_link_;


  bool gravity_mode_off_;
  bool init_pose_set_;
  bool apply_ctrl_torque_;// apply joint effort signal unitl a new one is recived or past_command_counter_ exceeds HOLD_FRAMES
  bool apply_last_torque_;
  bool apply_past_FT_disturbance_;
  bool motion_enable_;
  int past_last_torque_counter_;// count the number of applications of the passt recived command, HOLD_FRAMES is a fixed value at complie time
  uint64_t FT_dis_apply_count_;
  double jointTorque_feq_; //update frequency
  double jointTorque_T_; //update Period
  double disFT_updateRate_;

  boost::timed_mutex EffortMutex_;
  boost::mutex FTdisMutex_;


  pthread_t std_input_tId =-1;
  bool reset_thread_started_;
  // Pointer to the update event connection
};

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(ModelPush)
}  // namespace gazebo


#endif // IIWA_GAZEBO_PLUGIN_H
