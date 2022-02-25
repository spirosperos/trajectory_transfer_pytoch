#ifndef ACIN_CONTROLLER_UI_H
#define ACIN_CONTROLLER_UI_H
#include "ros_acin_robot_support/ros_acin_robot_support.h"
#include <sys/stat.h>
#include <QMainWindow>
#include <qapplication.h>
#include <qinputdialog.h>
#include <QPushButton>
#include <QObject>
#include <QCloseEvent>
#include <QMessageBox>
#include <QComboBox>
#include <QLabel>
#include <QColor>
#include <QIcon>
#include <QSlider>
#include <QFileDialog>
#include <QCheckBox>
#include <QGridLayout>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGroupBox>
#include <QMap>


//typedef ACIN_Controller_Node_C<_Traj_N_Knot_> acin_ctrl_t;
//typedef std::shared_ptr<acin_ctrl_t> acin_ctrl_sptr_t;

typedef std_msgs::Bool ros_boolMsg_t;
typedef std_msgs::Header ros_Header_t;
typedef sensor_msgs::JointState ros_JointState_t;
typedef ros_acin_robot_support::CoordAdj ros_CoordAdj_t;
typedef ros_acin_robot_support::ToolParam ros_ToolParam_t;
typedef ros_acin_robot_support::ToolParam ros_ToolParam_t;
typedef ros_acin_robot_support::TaskSpaceTraj ros_TaskSpaceTraj_t;
typedef ros_acin_robot_support::acinSollPos ros_acinSollPosMsg_t;
typedef ros_acin_robot_support::JointSpaceTraj ros_JointSpaceTraj_t;
typedef ros_acin_robot_support::Pose ros_Pose_t;
typedef ros_acin_robot_support::Joints ros_Joints_t;
typedef ros_acin_robot_support::PoseState ros_PoseState_t;

typedef struct{
  //acin_ctrl_sptr_t acin_ctrl_sptr;
  ros::Publisher grav_pub;
  ros::Publisher coordAdjustment_pub;
  ros::Publisher ToolParam_pub;
  ros::Publisher motion_EN_pub;
  ros::Publisher taskSpaceTraj_pub;
  ros::Publisher JointSpaceTraj_pub;
  ros::Publisher motor_EN_pub;
  ros::Publisher FrictionComp_pub;
  ros::Publisher singularPert_pub;
  ros::Publisher sollPos_pub;
  uint8_t finished_flag;
}UIArg_t;

class ACIN_ctrl_UI_C: public QMainWindow{

protected:
    void closeEvent(QCloseEvent *event) override;
private:

  const uint16_t label_heigth = 50;
  const uint16_t label_width = 50;
  const uint16_t tf_x_pos = 40;
  const uint16_t tf_y_pos = 0;
  const uint16_t testTrajectory_x_pos = 0;
  const uint16_t testTrajectory_y_pos = 200;
  const uint16_t x_0 = 0;
  const uint16_t y_0 = 0;
  const uint16_t SMALL_BUT_X =50;
  const uint16_t SMALL_BUT_Y =25;
  const uint16_t L_BUT_X =100;
  const uint16_t L_BUT_Y =50;
  const uint16_t SMALL_TF_X =50;
  const uint16_t SMALL_TF_Y =50;
  const uint16_t TINY_TF_Y =25;
  const uint16_t LARGE_BUT_X =100;
  const uint16_t LARGE_BUT_Y =50;
  /* state parameters sof the acin_controller*/
  ros_boolMsg_t s_gravity_comp_;
  ros_boolMsg_t s_motion_EN_DIS_;
  ros_ToolParam_t s_ToolParam_ ;
  ros_boolMsg_t s_singularPert_;
  ros_Joints_t s_motor_EN_;
  ros_boolMsg_t s_frictionComp_;
  bool joint_space_inc_ = true;
  bool reject_userUIcmd =false;
  std::shared_ptr<UIArg_t> UIArg_sptr_ = NULL;
  double T_end_= 2;
  double UserAccelPerc_ = 0.80;//used by the controller to peform profiled interpolation between actuel Pose/configuration coords and adjustment value/end posistion
  double adjust_value_= 1.1;
  uint32_t coord_= 0;
  QMap<QString, QPushButton *> ButtonMap;
  QMap<QString, QComboBox *> ComboBoxMap;


  //QLineEdit *coord_TF_;
  //QComboBox *coord_CB_;
  QGridLayout *mainLayout;
  QLineEdit *T_TF_;
  QLineEdit *accel_TF_;
  QLineEdit *adjust_TF_;
  QMap<QString, QLineEdit *> LineEdit_Map_;
  QMap<QString, QCheckBox *> CheckBox_Map_;

  //QLabel *coord_label_;
  //QLabel *time_label_;
  //QLabel *adjust_label_;

  QMap<QString, QLabel *> LabelMap;
  QMap<QString, QSlider *> SliderMap;


  //QComboBox *coord_1_traj_CB_;
  //QComboBox *coord_2_traj_CB_;
  //QComboBox *Traj_mode_CB_;
  //QComboBox *rqt_plot_CB_;
  std::string rqt_multiplot_XML_path="./";
  std::unique_ptr<QWidget> ui_area_;
  const char taskSpace_coord_labels[6][6] = {"x","y","z","Yaw","Pitch","Roll"};
  const char taskSpace_Qaut_coord_labels[7][3] = {"x","y","z","Qw","Qx","Qy","Qz"};
  //const char *taskSpace_coord_labels[] = {"x","y",'z','R','P','Y'};



  ros::NodeHandle nh_;
  ros::master::V_TopicInfo topic_infos;
  ros::Subscriber sub_Task_actual_;
  void TaskSpaceTrajActual_Callback(const ros_PoseState_t &Pose_a);
  bool valid_actuel_Pose_ = false;
  std::thread valid_pose_watch_dog_;
  boost::timed_mutex PoseMutex_;
  ros_PoseState_t Pose_actuel_;
  double Pose_soll_[6]; //("x","y","z","R","P","Y")

  ros::Subscriber sub_Joint_actual_;
  void JointSpaceTrajActual_Callback(const ros_JointState_t &Joint_a);
  bool valid_actuel_Joint_ = false;
  std::thread valid_joint_watch_dog_;
  boost::timed_mutex JointMutex_;
  ros_Joints_t Joint_actuel_;
  ros_Joints_t Joint_soll_;
  #if _ROBOT_q_DIM_ > 6
    bool sollCheckbox_[_ROBOT_q_DIM_];
  #else
    bool sollCheckbox_[6];
  #endif


  void init_variables(void){
    s_gravity_comp_.data = true;
    s_motion_EN_DIS_.data = false;
    s_singularPert_.data = false;
    s_frictionComp_.data = false;
    s_ToolParam_.m = 0.4759;
    s_ToolParam_.spx = -0.0045;
    s_ToolParam_.spy = -0.0057;
    s_ToolParam_.spz =  0.145;
    Pose_actuel_.pose.position = std::vector<double>(3, 0.0);
    Pose_actuel_.pose.orientation = std::vector<double>(3, 0.0);
    Pose_actuel_.p_vel.x = 0;
    Pose_actuel_.p_vel.y = 0;
    Pose_actuel_.p_vel.z = 0;
    Pose_actuel_.omega.x = 0;
    Pose_actuel_.omega.y = 0;
    Pose_actuel_.omega.z = 0;
    Pose_actuel_.p_accl.x = 0;
    Pose_actuel_.p_accl.y = 0;
    Pose_actuel_.p_accl.z = 0;
    Pose_actuel_.omegaDot.x = 0;
    Pose_actuel_.omegaDot.y = 0;
    Pose_actuel_.omegaDot.z = 0;
    Joint_actuel_.Joints = std::vector<double>(_ROBOT_q_DIM_, 0.0);
    s_motor_EN_.Joints = std::vector<double>(_ROBOT_q_DIM_, 0.0);
    this->Joint_soll_.Joints = std::vector<double>(_ROBOT_q_DIM_,0.0);
    for(uint16_t ii=0; ii < _ROBOT_q_DIM_; ii++){
      Joint_actuel_.Joints[ii] = 0.0;
      s_motor_EN_.Joints[ii] = 0.0;
    }
    for(uint16_t ii=0; ii < 6; ii++){
      Pose_soll_[ii] = 0; //x y z, R P Y
    }
    adjust_value_ = 1.1;
  }
  void setButtonState(QPushButton *pButton, const bool &State);
  void setSliderState(QSlider *pButton, const bool &State);
  void set_frictionComp_button_state(void);
  void set_gravComp_button_state(void );
  void set_singularPert_button_state(void );
  void fetch_userUIinput(void);

public:
  ACIN_ctrl_UI_C(UIArg_t &acin_UIArg, ros::NodeHandle &nh, QWidget *parent = nullptr);
  void mousePressEvent(QMouseEvent *e);
  void add_topics_to_CB(QComboBox *CB, const std::string &dataType){
    CB->clear();
    ros::master::V_TopicInfo topic_infos;
    ros::master::getTopics(topic_infos);
    for (uint32_t ii = 0; ii < topic_infos.size(); ii++){
      if (dataType == topic_infos[ii].datatype){
        CB-> addItem(tr(topic_infos[ii].name.c_str()), QVariant(ii));
      }
    }
  }
  bool isClosed();
  ~ACIN_ctrl_UI_C(){
    //QMap<QString, QLineEdit *> LineEdit_Map_;
    qDeleteAll( LineEdit_Map_ );  //  deletes all the values stored in "map"
    LineEdit_Map_.clear();
    qDeleteAll( CheckBox_Map_ );  //  deletes all the values stored in "map"
    CheckBox_Map_.clear();
    qDeleteAll( LabelMap );  //  deletes all the values stored in "map"
    LabelMap.clear();
    qDeleteAll( SliderMap );  //  deletes all the values stored in "map"
    SliderMap.clear();
    qDeleteAll( ButtonMap );
    ButtonMap.clear();
    qDeleteAll( ComboBoxMap );
    ComboBoxMap.clear();
  }
  public:
    void gravComp_change();
    void motion_disable_Callback();
  private slots:
    void handleButton_inc();
    void handleButton_dec();
    void handleButton_grav_mode();
    void handleButton_singular_perturbation();
    void handleButton_frictionComp();
    void handleButton_setToolParam();
    void handleButton_motion_EN();
    void handleButton_motion_DIS();
    void handleButton_motor_sw();
    void handleButton_TestTraj();
    void handleButton_RQTFolder();
    void handleButton_sollPos();
    void handleComboBox_RQTPlot(int);
    void handleCB_Traj_mode(int);
    void handleCB_TaskTraj_index(int);
    void handleCB_traj_topic(int);
    void handleCB_JointState_topic(int);
    void valid_pose_watch_dog_Callback();
    void valid_joint_watch_dog_Callback();
    void slider_setValue_Callback(int);
};
template<typename ... Args>
std::string string_format( const std::string& format, Args ... args )
{
    int size = snprintf( nullptr, 0, format.c_str(), args ... ) + 1; // Extra space for '\0'
    if( size <= 0 ){ throw std::runtime_error( "Error during formatting." ); }
    std::unique_ptr<char[]> buf( new char[ size ] );
    snprintf( buf.get(), size, format.c_str(), args ... );
    return std::string( buf.get(), buf.get() + size - 1 ); // We don't want the '\0' inside
}

#endif
