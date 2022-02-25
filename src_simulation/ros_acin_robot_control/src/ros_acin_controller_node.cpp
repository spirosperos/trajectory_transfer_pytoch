#define FLAG_ROS_COMPILE
#include <thread>
#include <mutex>
#include <ros/ros.h>
#include <getopt.h>
#include <boost/thread/locks.hpp>
#include <boost/thread/mutex.hpp>

#if !defined(FLAG_ROS_COMPILE)
  #include "ros_acin_robot_control/acin_controller_node.h"
  template <int N_knots_> class ACIN_Controller_Node_C_twincat:public ACIN_Controller_Node_C<N_knots_>
  {

    public:
      typedef typename ACIN_Controller_Node_C<N_knots_>::init_acin_ctrl_t init_acin_ctrl_t;
      typedef typename ACIN_Controller_Node_C<N_knots_>::timeBase_t timeBase_t;
      ACIN_Controller_Node_C_twincat(double Ta, init_acin_ctrl_t &init_acin_ctrl):
        ACIN_Controller_Node_C<N_knots_>(Ta, init_acin_ctrl){
        return;
      }
      void v_pub_Tau(Joint_t &Tau){

      }
      void v_pub_PoseTraj_actuel(PoseState_t &PoseTraj_actuel){

      }
      void v_pub_PoseTraj_desired(PoseState_t &PoseTraj_desired){

      }
      void v_pub_TaskSpace_error(PoseState_t & TaskSpace_error){

      } /*publish joint space controler joint errors*/
      void v_pub_JointSpace_error(JointState_t &JointSpace_error){

      } /*publish joint space controler joint errors*/
      void v_pub_JointSpace_precurve(JointState_t &JointSpace_precurve){

      }
      void v_pub_time_meas(calcTimeMeasurement_t &calcTime){

      }
      double get_cur_time(const timeBase_t &t_base = timeBase_t::ACIN_CTRL_BASE_SEC) {
    			// UTC at time of call in 100 ns intervals since 1.1.1601, pentium time -> proccessor time
  //  			LONGLONG time_current = 0;
          // UTC at "begin of task" in 100 ns intervals since 1.1.1601 must be fetch from *ipTask pointer of the cycleupdate, ipTask->GetCurrentSysTime(&LONG LONG TaskSysTime);
    			//m_spRTime->GetCurPentiumTime(&time_current);//number of 100ns intervalls of UTC
    			double t_ret = 0.0;
          timespec start;
          clock_gettime(CLOCK_REALTIME, &start);
          time_t sec = start.tv_sec;
          uint64_t nsec = start.tv_nsec;
    			switch (t_base) {
    			case timeBase_t::ACIN_CTRL_BASE_SEC:
    				t_ret = (double) (sec + (double)nsec / 1000000000);
    				break;
    			case timeBase_t::ACIN_CTRL_BASE_mSEC:
    				t_ret = (double) sec*1000 + (double) nsec / 1000000;
    				break;
    			case timeBase_t::ACIN_CTRL_BASE_uSEC:
    				t_ret = (double) sec*1000000 + (double) nsec / 1000;
    				break;
    			case timeBase_t::ACIN_CTRL_BASE_nSEC:
    				t_ret = (double) sec*1000000000 + (double) nsec;
    				break;
    			default:
    				t_ret = (double) sec*1000000000 + (double) nsec;
    			}
    			if (round(t_ret) == 0 && sec != 0 && nsec != 0) {
    			     fprintf(stderr,"get time rounding to base time, time get was not zero but rounded value is");
    			}
    			return t_ret;
    	}
  };

  typedef ACIN_Controller_Node_C_twincat<_Traj_N_Knot_> ros_acin_ctrl_t;
  //void ACIN_Controller_Node_C<_Traj_N_Knot_>::v_pub_Tau(Joint_t &Tau) {return;}
  //void v_pub_PoseTraj_actuel(PoseState_t &PoseTraj){}
  //void v_pub_PoseTraj_desired(PoseState_t &PoseTraj){}
  //void v_pub_TaskSpace_error(PoseState_t &PoseTraj){} /*publish joint space controler joint errors*/
  //void v_pub_JointSpace_error(JointState_t &JointState){} /*publish joint space controler joint errors*/
  //void v_pub_JointSpace_precurve(JointState_t &JointState){}
  //void v_pub_time_meas(calcTimeMeasurement_t &calcTimeMeasurement){} /*publish task space controler pose errors*/
#else
  #include "ros_acin_robot_control/ros_acin_controller_node.h"
  typedef ROS_ACIN_Controller_Node_C<_Traj_N_Knot_> ros_acin_ctrl_t;
#endif
typedef std::shared_ptr<ros_acin_ctrl_t> ros_acin_ctrl_sptr_t;
// #include "ros_acin_robot_support/acin_ctrl_UI.h"
void usage(const char *prgname){

  ACIN_CTRL_ERROR("Usage: %s [-f <hz>|-T <ms>]",prgname);

}
boost::timed_mutex iterateMutex_;

ros_acin_ctrl_sptr_t acin_ctrl_ROS_sptr;
void controller_status_information(const ros::TimerEvent&){
  boost::unique_lock<boost::timed_mutex> iteratelock{iterateMutex_, boost::try_to_lock};
  if(acin_ctrl_ROS_sptr!=NULL && iteratelock.owns_lock() ){
    acin_ctrl_ROS_sptr->pub_status_information();
    iteratelock.unlock();
  }

}
void controller_iterate(const ros::TimerEvent&){
  boost::unique_lock<boost::timed_mutex> iteratelock{iterateMutex_, boost::try_to_lock};
  if (!iteratelock.owns_lock()){
    return;
  }
  iteratelock.lock();
  acin_ctrl_ROS_sptr->iterate();
  iteratelock.unlock();
  ros::spinOnce();
}
int main(int argc, char** argv)
{
  ros::init(argc, argv, "acin_robot_control_node");

  ros::NodeHandle nh("~");
  ros::Publisher grav_pub= nh.advertise<ros_boolMsg_t>("gravity_comp_mode", 1);
  double rate = 1000.0;
  double Ta = 1.0 / rate;
  if(nh.hasParam("f")){
    nh.getParam("f", rate);

    if (rate <= 0 or rate >8000){
      ACIN_CTRL_ERROR("rate should be greater than 0 Hz and smaller equal to 8000 Hz");
      exit(-1);
    }else{
      Ta = 1.0/rate;
    }
  }
  ACIN_CTRL_INFO("acin_robot_control_node control frequency:= %f Hz\n", rate);
  // if(!nh.hasParam("q0")){
  //   nh.setParam("q0", "1.5708,0.2266,-0.0000,1.7712,-0.0000,1.5446,-1.5708");
  // }

  // nh.param<std::string>("q0", q0_str, "1.5708,0.2266,-0.0000,1.7712,-0.0000,1.5446,-1.5708");
  #if defined(FLAG_ROS_COMPILE)
    //ACIN_Controller_Node_C<_Traj_N_Knot_> ACIN_Controller_Node(nh, Ta);
    acin_ctrl_ROS_sptr = ros_acin_ctrl_sptr_t(new ros_acin_ctrl_t(nh, Ta));
  #else
    ACIN_Controller_Node_C_twincat<_Traj_N_Knot_>::init_acin_ctrl_t init_acin_ctrl;
    init_acin_ctrl.hanging = true;
    init_acin_ctrl.mE = 0;
    init_acin_ctrl.sE[0]= 0;
    init_acin_ctrl.sE[1]= 0;
    init_acin_ctrl.sE[2]= 0;
    init_acin_ctrl.q_0 = JointVector_t::Zero();
    init_acin_ctrl.T_init_0 = 5;
    init_acin_ctrl.q_d_NS_0 = JointVector_t::Zero();
    acin_ctrl_ROS_sptr = ros_acin_ctrl_sptr_t(new ros_acin_ctrl_t(Ta,init_acin_ctrl));
    //ACIN_Controller_Node_C_twincat<_Traj_N_Knot_> ACIN_Controller_Node(Ta, init_acin_ctrl);
  #endif

  UIArg_t UIArg;
  UIArg.finished_flag = 0;
  UIArg.grav_pub = nh.advertise<ros_boolMsg_t>("gravity_comp_mode", 1);
  UIArg.coordAdjustment_pub = nh.advertise<ros_CoordAdj_t>("coordAdjustment", 1);
  UIArg.ToolParam_pub = nh.advertise<ros_ToolParam_t>("set_Tool_param", 1);
  UIArg.motion_EN_pub = nh.advertise<ros_boolMsg_t>("motion_EN", 1);
  UIArg.taskSpaceTraj_pub = nh.advertise<ros_TaskSpaceTraj_t>("desired_TaskSpace_traj", 1);
  UIArg.JointSpaceTraj_pub = nh.advertise<JointSpaceTraj_t>("desired_JointSpace_traj", 1);
  UIArg.FrictionComp_pub = nh.advertise<ros_boolMsg_t>("friction_comp", 1);
  UIArg.singularPert_pub = nh.advertise<ros_boolMsg_t>("SingularPert", 1);
  UIArg.sollPos_pub = nh.advertise<ros_acinSollPosMsg_t>("SetSollPos", 1);
  //UIArg.acin_ctrl_sptr = static_cast<acin_ctrl_sptr_t> (acin_ctrl_ROS_sptr);
  int qt_argc = 0;
  char **qt_argv = 0;
  QApplication qt_app_( qt_argc, qt_argv );
  // //std::cout<<"acin_ctrl_sptr:--- "<<acin_ctrl_ROS_sptr<<std::endl;
  ACIN_ctrl_UI_C ACIN_ctrl_UI_obj(UIArg, nh);
  ACIN_ctrl_UI_obj.show();
  ros::Rate r(rate);
  ros::Timer ctrl_status_timer = nh.createTimer(ros::Duration(0.1), controller_status_information);
  //ros::Timer ctrl_iterate_timer_ = nh.createTimer(ros::Rate, controller_iterate);


  boost::unique_lock<boost::timed_mutex> iteratelock{iterateMutex_,boost::defer_lock};
  while(ros::ok() && UIArg.finished_flag == 0)
  {
    static uint64_t ii_loop  =0;
    // do something
    iteratelock.lock();
    acin_ctrl_ROS_sptr->iterate();
    iteratelock.unlock();

    ros::spinOnce();
    if(!r.sleep())
    {
        // ROS_WARN("In %s: Loop missed desired rate of %.4fs (loop actually took %.10fs)", ros::this_node::getName().c_str(),
               // r.expectedCycleTime().toSec(), r.cycleTime().toSec());
    }
    if(ii_loop == 10000){
      ROS_INFO("<ros_acin_controller_node> controller_loop alive");
      ii_loop = 0;
    }else{
      ii_loop++;
    }
    qt_app_.processEvents();
  }
  if(UIArg.finished_flag == 1){
    std::cout<<"quit send to input stream!"<<std::endl;
  }
  return 0;
}
