#ifndef ROS_ACIN_ROBOT_SUPPORT_H
#define ROS_ACIN_ROBOT_SUPPORT_H
#ifdef _Traj_N_Knot_
#error "_Traj_N_Knot_: Should not be defined, include file defines it"
#endif
#define _Traj_N_Knot_ (500)
#include "ros_acin_robot_support/models/robot_model.h"
#include "ros_acin_robot_support/models/evalJ.h"
#include "ros_acin_robot_support/models/evalJp.h"
#include "ros_acin_robot_support/models/evalT0E.h"
#include "ros_acin_robot_support/models/evalM.h"
#include "ros_acin_robot_support/models/evalC.h"
#include "ros_acin_robot_support/models/evalG.h"


#if !defined(TWINCAT_COMPILE)
  #define FLAG_ROS_COMPILE
#endif
#if !defined(TWINCAT_COMPILE)
  #include <ros_acin_robot_support/ct_controller_twinCatServicesv0.h>
#else
  #include "ct_controller_twinCat.h"
#endif
#if defined(FLAG_ROS_COMPILE)
  #define SPLINE_EXCEPTIONS
  #define ACIN_CONTROLLER_DEBUG
  #define EIGEN_NO_MALLOC
  #include <thread>
  #include <mutex>
  #include <ros/ros.h>
  #include <ros/package.h>
  #include <boost/algorithm/string/erase.hpp>
  #include <boost/algorithm/string/split.hpp>
  #include <boost/algorithm/string.hpp>
  #include <boost/thread/mutex.hpp>
  /*msg types*/
  #include <std_msgs/Header.h>
  #include <std_msgs/Float32MultiArray.h>
  #include <std_msgs/Bool.h>
  #include <std_msgs/Time.h>
  #include <sensor_msgs/JointState.h>
  #include <geometry_msgs/PoseArray.h>
  #include <ros_acin_robot_support/ProMP.h>
  #include <ros_acin_robot_support/PoseState.h>
  #include <ros_acin_robot_support/TaskSpaceTraj.h>
  #include <ros_acin_robot_support/JointSpaceTraj.h>
  #include <ros_acin_robot_support/ToolParam.h>
  #include <ros_acin_robot_support/calcTimeMeasurement.h>
  #include <ros_acin_robot_support/CoordAdj.h>
  #include <ros_acin_robot_support/acinSollPos.h>
  typedef sensor_msgs::JointState JointState_t;
  typedef std_msgs::Bool boolMsg_t;
  typedef std_msgs::Time rosTime_t;
  typedef ros_acin_robot_support::calcTimeMeasurement calcTimeMeasurement_t;
  typedef ros_acin_robot_support::ToolParam ToolParam_t;
  typedef ros_acin_robot_support::Pose Pose_t;
  typedef ros_acin_robot_support::Joints Joint_t;
  typedef ros_acin_robot_support::PoseState PoseState_t;
  typedef ros_acin_robot_support::JointSpaceTraj JointSpaceTraj_t;
  typedef ros_acin_robot_support::TaskSpaceTraj TaskSpaceTraj_t;
  typedef ros_acin_robot_support::ProMP ProMP_t;
  typedef ros_acin_robot_support::ProMP::_basis_fcn_type basis_fcn_msg_t;
  typedef ros_acin_robot_support::CoordAdj CoordAdj_t;
  typedef ros_acin_robot_support::acinSollPos acinSollPosMsg_t;
  //typedef Joint_t robot_array_t;
  typedef geometry_msgs::Point Point_t;
  //#include <acin_robot_control/debug_fcn.h>
  #define ACIN_CTRL_ERROR(...) ROS_ERROR(__VA_ARGS__)
  #define ACIN_CTRL_WARN(...) ROS_WARN( __VA_ARGS__)
  #define ACIN_CTRL_INFO(...) ROS_INFO( __VA_ARGS__)
  #define ACIN_CTRL_VERBOSE(...) ROS_INFO( __VA_ARGS__)
  #define ACIN_CTRL_ALWAYS(...) ROS_INFO( __VA_ARGS__)

  #define ACIN_CTRL_ASSERT(...) assert(__VA_ARGS__)
  template< typename... Args >
  std::string string_sprintf( const char* format, Args... args ) {
    int length = std::snprintf( nullptr, 0, format, args... );
    ACIN_CTRL_ASSERT( length >= 0 );

    char* buf = new char[length + 1];
    std::snprintf( buf, length + 1, format, args... );

    std::string str( buf );
    delete[] buf;
    return str;
  }
  void ACIN_CTRL_INFO_STREAM(std::ostringstream& stream){
    ROS_INFO_STREAM(""<<stream.str());
    stream.str("");
    stream.clear();
  }
  void ACIN_CTRL_WARN_STREAM(std::ostringstream& stream){
    ROS_WARN_STREAM(""<<stream.str());
    stream.str("");
    stream.clear();
  }

  template<typename ... Args>
  std::string stringf( const std::string& format, Args ... args )
  {
      int size = snprintf( nullptr, 0, format.c_str(), args ... ) + 1; // Extra space for '\0'
      if( size <= 0 ){ throw std::runtime_error( "Error during formatting." ); }
      std::unique_ptr<char[]> buf( new char[ size ] );
      snprintf( buf.get(), size, format.c_str(), args ... );
      return std::string( buf.get(), buf.get() + size - 1 ); // We don't want the '\0' inside
  }
  static std::string EigenToString(const Eigen::Ref<Eigen::Matrix<double,-1,-1>> mat)
  {
      std::string mat_str = "[";
      std::cout<<mat<<std::endl;
      for (int64_t ii = 0; ii< mat.rows(); ii++){
        for (int64_t jj = 0; jj< mat.cols(); jj++){
            mat_str = mat_str + stringf("%.5f",mat(ii,jj));
            if (jj != mat.cols()-1){
              mat_str = mat_str + ",";
            }
        }
        mat_str = mat_str + ";\n";
      }
      mat_str = mat_str+ "]";
      return mat_str;
  }


#else
    #if defined(TWINCAT_COMPILE)
        #define ACIN_CTRL_ERROR(format,...) if(ACIN_Controller_Node_twincat_C_uptr !=nullptr) ACIN_Controller_Node_twincat_C_uptr->m_Trace_ptr->Log(tlError, FLEAVEA "<acin_robot_ctrl> ERROR:\n\t" format, ##__VA_ARGS__)
        #define ACIN_CTRL_WARN(format,...) if(ACIN_Controller_Node_twincat_C_uptr !=nullptr) ACIN_Controller_Node_twincat_C_uptr->m_Trace_ptr->Log(tlWarning,FLEAVEA "<acin_robot_ctrl> WARNING:\n\t" format, ##__VA_ARGS__)
        #define ACIN_CTRL_INFO(format,...) if(ACIN_Controller_Node_twincat_C_uptr !=nullptr) ACIN_Controller_Node_twincat_C_uptr->m_Trace_ptr->Log(tlInfo, FLEAVEA "<acin_robot_ctrl> INFO:\n\t" format, ##__VA_ARGS__)
        #define ACIN_CTRL_VERBOSE(format,...) if(ACIN_Controller_Node_twincat_C_uptr !=nullptr) ACIN_Controller_Node_twincat_C_uptr->m_Trace_ptr->Log(tlVerbose, FLEAVEA "<acin_robot_ctrl> VERBOSE:\n\t" format, ##__VA_ARGS__)
        #define ACIN_CTRL_ALWAYS(format,...) if(ACIN_Controller_Node_twincat_C_uptr !=nullptr) ACIN_Controller_Node_twincat_C_uptr->m_Trace_ptr->Log(tlAlways, "<acin_robot_ctrl> ALWAYS:\n\t" format, ##__VA_ARGS__)
    #endif
    template<typename ... Args>
    std::string stringf( const std::string& format, Args ... args )
    {
      int size = snprintf( nullptr, 0, format.c_str(), args ... ) + 1; // Extra space for '\0'
      size = 500;
      if (size <= 0) { return std::string(""); }
      //if( size <= 0 ){ throw std::runtime_error( "Error during formatting." ); }
      std::unique_ptr<char[]> buf( new char[ size ] );
      size = snprintf( buf.get(), size, format.c_str(), args ... );
      if (size <= 0) { return std::string(""); }
      return std::string( buf.get(), buf.get() + size - 1 ); // We don't want the '\0' inside
    }
    static std::string EigenToString(const Eigen::Ref<Eigen::Matrix<double, -1, -1>> mat)
    {
        std::string mat_str = "[";
        for (int64_t ii = 0; ii < mat.rows(); ii++) {
            for (int64_t jj = 0; jj < mat.cols(); jj++) {
                mat_str = mat_str + stringf(std::string("%.5f"), mat(ii, jj));
                if (jj != mat.cols() - 1) {
                    mat_str = mat_str + ",";
                }
            }
            mat_str = mat_str + ";\n";
        }
        mat_str = mat_str + "]";
        return mat_str;
    }
  typedef acin_control_JointState_t JointState_t;
  typedef acin_control_BoolMsg_t boolMsg_t;
  typedef acin_control_calcTimeMeasurement_t calcTimeMeasurement_t;
  typedef acin_control_ToolParam_t ToolParam_t;
  typedef acin_control_Pose_t Pose_t;
  typedef acin_control_Joints_t Joint_t;
  typedef acin_control_JointSpaceTraj_t JointSpaceTraj_t;
  typedef acin_control_TaskSpaceTraj_t TaskSpaceTraj_t;
  typedef acin_control_PoseState_t PoseState_t;
  typedef acin_control_CartPoint_t Point_t;
  typedef acin_control_CoordAdj_t CoordAdj_t;
  typedef acin_control_acinSollPos_t acinSollPosMsg_t;

  #if defined(ACIN_CONTROLLER_DEBUG)
    #if defined(TWINCAT_COMPILE)

    #else
      #define ANSI_COLOR_RED     "\x1b[31m"
      #define ANSI_COLOR_GREEN   "\x1b[32m"
      #define ANSI_COLOR_YELLOW  "\x1b[33m"
      #define ANSI_COLOR_BLUE    "\x1b[34m"
      #define ANSI_COLOR_MAGENTA "\x1b[35m"
      #define ANSI_COLOR_CYAN    "\x1b[36m"
      #define ANSI_COLOR_RESET   "\x1b[0m"
      #define ACIN_CTRL_ERROR(format,...) printf(ANSI_COLOR_RED format ANSI_COLOR_RESET ANSI_COLOR_RESET, ##__VA_ARGS__)
      #define ACIN_CTRL_INFO(format,...) printf(ANSI_COLOR_BLUE format ANSI_COLOR_RESET ANSI_COLOR_RESET, ##__VA_ARGS__)
      #define ACIN_CTRL_VERBOSE(format,...) printf(ANSI_COLOR_BLUE format ANSI_COLOR_RESET ANSI_COLOR_RESET, ##__VA_ARGS__)
      #define ACIN_CTRL_ALWAYS(format,...) printf(ANSI_COLOR_BLUE format ANSI_COLOR_RESET ANSI_COLOR_RESET, ##__VA_ARGS__)
      #define ACIN_CTRL_WARN(format,...) printf(ANSI_COLOR_YELLOW format ANSI_COLOR_RESET ANSI_COLOR_RESET, ##__VA_ARGS__)
      #define ACIN_CTRL_ASSERT(...) assert(__VA_ARGS__)
    #endif

  #else

    //#define ACIN_CTRL_ERROR(...) do {} while(0)
    #if !defined(ACIN_CTRL_INFO)
        #define ACIN_CTRL_INFO(...) do {} while(0)
    #endif
    #if !defined(ACIN_CTRL_WARN)
        #define ACIN_CTRL_WARN(...) do {} while(0)
    #endif
    #if !defined(ACIN_CTRL_ASSERT)
        #define ACIN_CTRL_ASSERT(...) do {} while(0)
    #endif
    #if !defined(ACIN_CTRL_VERBOSE)
        #define ACIN_CTRL_VERBOSE(...) do {} while(0)
    #endif
    #if !defined(ACIN_CTRL_INFO)
        #define ACIN_CTRL_ALWAYS(...) do {} while(0)
    #endif
  #endif

#endif

#include "ros_acin_robot_support/interpolators/pBSpline_interpolation.h"

typedef Eigen::Quaternion<double> qa_t;
typedef Eigen::Matrix<double,4,1> qa_vec_t;

typedef pBSpline_interpolator_C<3, _TaskSpaceInterpolator_deg_, _Traj_N_Knot_, false> spline_Interpolator_t;//change name, ev. 3D_spline_Interpolator
typedef std::shared_ptr<spline_Interpolator_t> spline_Interpolator_sptr_t;
typedef pBSpline_interpolator_C<4, _TaskSpaceInterpolator_deg_, _Traj_N_Knot_, true> quaternion_Interpolator_t;
typedef std::shared_ptr<quaternion_Interpolator_t> quaternion_Interpolator_sptr_t;
typedef pBSpline_interpolator_C<_ROBOT_q_DIM_, _JointSpaceInterpolator_deg_, _Traj_N_Knot_, false> joint_Interpolator_t;
typedef std::shared_ptr<joint_Interpolator_t> joint_Interpolator_sptr_t;

typedef spline_Interpolator_t::curve_t point_curve_t;
typedef joint_Interpolator_t::curve_t joint_curve_t;
typedef Eigen::Matrix<double,_Traj_N_Knot_,1> w_curve_t;
#endif // ROS_ACIN_ROBOT_SUPPORT_H
