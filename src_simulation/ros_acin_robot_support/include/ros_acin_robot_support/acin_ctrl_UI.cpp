#include "ros_acin_robot_support/acin_ctrl_UI.h"

ACIN_ctrl_UI_C::ACIN_ctrl_UI_C( UIArg_t &acin_UIArg,
                                ros::NodeHandle &nh, QWidget *parent)
                                :nh_(nh),
                                QMainWindow(parent){

    init_variables();
    std::string ICON_path = ros::package::getPath ("ros_acin_robot_support");
    ICON_path = ICON_path+"/logo"+"/ACIN_ctrl_logo.png";
    struct stat buffer;
    bool file_exists = (stat (ICON_path.c_str(), &buffer) == 0);
    if (file_exists)  this->setWindowIcon(QIcon(ICON_path.c_str()));
    //QMainWindow_ = new QMainWindow()

    UIArg_sptr_ = std::make_shared<UIArg_t>(acin_UIArg);
    ui_area_ = std::unique_ptr<QWidget>(new QWidget(this));
    this->setCentralWidget(ui_area_.get());
    this->setWindowTitle("acin UI Main window");
    ui_area_->resize(600, 512);
    this->resize(600, 512);
    //QGroupBox *MainGroupBox = new QGroupBox(this);
    mainLayout = new QGridLayout();
    ui_area_->setLayout(mainLayout);
    //w = new QWidget();
    /***************************************************************************/
    /***************************************************************************/
    { // Groupbox for moving the robot
      // --- set desired end time of the movement
      // --- set adjustment value to perform a movement of one desired coordinate
      // --- peform lissajous curve with set parameter and in the choosen coordinates
      QGroupBox *MoveRobotGroupBox = new QGroupBox(this);
      QGridLayout *MoveRobotLayout = new QGridLayout;
      MoveRobotGroupBox->setStyleSheet("QGroupBox{padding-top:15px; margin-top:15px}");

      { //trajecotry end time
        QLabel *time_label_ = new QLabel("T:=",MoveRobotGroupBox);
        MoveRobotLayout->addWidget(time_label_, 0,0,1,1);
        LabelMap["time_label"] = time_label_;

        T_TF_ = new QLineEdit(MoveRobotGroupBox);
        T_TF_->setPlaceholderText(QString::number(T_end_));
        MoveRobotLayout->addWidget(T_TF_, 0,1,1,1);

        QComboBox *time_label_unit_ = new QComboBox(MoveRobotGroupBox);
        time_label_unit_-> addItem(tr("s"), QVariant(0));
        time_label_unit_->setStyleSheet("font-size: 10px;");
        MoveRobotLayout->addWidget(time_label_unit_, 0,2,1,1);
        ComboBoxMap["time_label_unit"] = time_label_unit_;
      }
      { //adjust value ->  peform movement from current pos. added with adjust value
        /******/
        /******/
        /******/
        QLabel* adjust_label_ = new QLabel("adjust value:=",MoveRobotGroupBox);
        MoveRobotLayout->addWidget(adjust_label_, 1,0,1,1);
        LabelMap["adjust_label"] = adjust_label_;
        adjust_TF_ = new QLineEdit();
        MoveRobotLayout->addWidget(adjust_TF_, 1,1,1,1);
        adjust_TF_ -> setPlaceholderText(QString::number(adjust_value_));
        LineEdit_Map_["adjust"] = adjust_TF_;
        QComboBox *adjust_unit_  = new QComboBox(MoveRobotGroupBox);

        MoveRobotLayout->addWidget(adjust_unit_, 1,2,1,1);
        adjust_unit_-> addItem(tr("°"), QVariant(0));
        adjust_unit_-> addItem(tr("rad"), QVariant(1));
        adjust_unit_->setStyleSheet("font-size: 10px;");
        ComboBoxMap["adjust_unit"] = adjust_unit_;

      }

      {
        QComboBox *coord_CB_  = new QComboBox();
        MoveRobotLayout->addWidget(coord_CB_, 2,1,1,1);
        for (uint8_t ii =0; ii < _ROBOT_q_DIM_; ii++ ){
          std::string id_str = std::to_string(ii);;
          coord_CB_-> addItem(tr(id_str.c_str()), QVariant(ii));
        }
        ComboBoxMap["coord"] = coord_CB_;
      }
      {
        QPushButton *inc_button_ = new QPushButton("+");
        MoveRobotLayout->addWidget(inc_button_,2,2,1,1);
        connect(inc_button_, &QPushButton::released, this, &ACIN_ctrl_UI_C::handleButton_inc);
        ButtonMap["inc_button"] = inc_button_;
      }
      {
        QPushButton *dec_button_= new QPushButton("-");
        MoveRobotLayout->addWidget(dec_button_, 2,3,1,1);
        connect(dec_button_, &QPushButton::released, this, &ACIN_ctrl_UI_C::handleButton_dec);
        ButtonMap["dec_button"] = dec_button_;
      }
      {
        QComboBox *Traj_mode_CB_  = new QComboBox();
        MoveRobotLayout->addWidget(Traj_mode_CB_, 0,3,1,2);
        Traj_mode_CB_-> addItem(tr("JointSpace"), QVariant(0));
        Traj_mode_CB_-> addItem(tr("TaskSpace"), QVariant(1));
        Traj_mode_CB_->setStyleSheet("font-size: 10px;");
        QObject::connect(Traj_mode_CB_,  static_cast<void (QComboBox::*)(int)>(&QComboBox::activated),this, &ACIN_ctrl_UI_C::handleCB_Traj_mode);
        ComboBoxMap["Traj_mode_CB"]= Traj_mode_CB_;
      }
      { // define the percentace of the profiled interpolation moevement which is used for the acceleration and deacceleration of the robot
        QLabel *accel_label_ = new QLabel("accel. Perc:=",MoveRobotGroupBox);
        MoveRobotLayout->addWidget(accel_label_, 3,0,1,1);
        LabelMap["accel_label"] = accel_label_;

        accel_TF_ = new QLineEdit(MoveRobotGroupBox);
        accel_TF_->setPlaceholderText(QString::number(UserAccelPerc_*100));
        MoveRobotLayout->addWidget(accel_TF_, 3,1,1,1);

        QLabel *accel_label_unit_ = new QLabel("%",MoveRobotGroupBox);
        accel_label_unit_->setStyleSheet("font-size: 10px;");
        MoveRobotLayout->addWidget(accel_label_unit_, 3,2,1,1);
        LabelMap["accel_label_unit"] = accel_label_unit_;
      }
    /**************************************************************************/
    /*test trajectory*/
    /**************************************************************************/
    {
      //TODO mutiple test trajectorys
      //testTrajectory_label_ = new QLabel("adjust value:=",this);
      //testTrajectory_label_ -> setGeometry(QRect(QPoint(a_x_pos, a_y_pos), QSize(10, label_heigth)));
      QPushButton  *testTrajectory_button_ = new QPushButton("lissajous\n curve dispatch");
      MoveRobotLayout->addWidget(testTrajectory_button_, 1,3,1,1);
      testTrajectory_button_->setStyleSheet("font-size: 10px;");
      connect(testTrajectory_button_, &QPushButton::released, this, &ACIN_ctrl_UI_C::handleButton_TestTraj);
      ButtonMap["testTrajectory_button"] = testTrajectory_button_;
      QComboBox *coord_1_traj_CB_  = new QComboBox();
      MoveRobotLayout->addWidget(coord_1_traj_CB_, 1,4,1,1);
      QComboBox *coord_2_traj_CB_  = new QComboBox();
      MoveRobotLayout->addWidget(coord_2_traj_CB_, 2,4,1,1);
      for (uint8_t ii =0; ii < 7; ii++ ){
        std::string id_str =std::to_string(ii);;
        if(ii == 0){
          coord_1_traj_CB_-> addItem(tr(id_str.c_str()), QVariant(ii));
        }else{
          if(ii == 1){
            coord_2_traj_CB_-> addItem(tr(id_str.c_str()), QVariant(ii));
          }else{
            coord_1_traj_CB_-> addItem(tr(id_str.c_str()), QVariant(ii));
            coord_2_traj_CB_-> addItem(tr(id_str.c_str()), QVariant(ii));
          }
        }
      }
      coord_1_traj_CB_->setStyleSheet("font-size: 10px;");
      coord_2_traj_CB_->setStyleSheet("font-size: 10px;");
      ComboBoxMap["coord_1_traj_CB"] = coord_1_traj_CB_;
      ComboBoxMap["coord_2_traj_CB"] = coord_2_traj_CB_;
      QObject::connect(coord_1_traj_CB_,  static_cast<void (QComboBox::*)(int)>(&QComboBox::activated),this, &ACIN_ctrl_UI_C::handleCB_TaskTraj_index);
      QObject::connect(coord_2_traj_CB_,  static_cast<void (QComboBox::*)(int)>(&QComboBox::activated),this, &ACIN_ctrl_UI_C::handleCB_TaskTraj_index);
    }
    /**************************************************************************/
    /**************************************************************************/
    mainLayout->addWidget(MoveRobotGroupBox, 0, 0, 1, 6);
    MoveRobotLayout-> setHorizontalSpacing(2);
    MoveRobotLayout-> setVerticalSpacing(2);
    MoveRobotLayout-> setContentsMargins(1,1,1,1);//left top right bottom);
    MoveRobotGroupBox->setLayout(MoveRobotLayout);
    MoveRobotGroupBox->show();



    QGroupBox *TopicGroupBox = new QGroupBox(this);
    QGridLayout *TopicLayout = new QGridLayout;
    TopicGroupBox->setStyleSheet("QGroupBox{padding-top:15px; margin-top:-15px}");
    /*##########################################################################*/
    /*combo Box for selection of getting pose data for the current robot position */
    /*##########################################################################*/
    {
      QLabel* JointTopic_label_ = new QLabel("Joint actual Topic");
      TopicLayout->addWidget(JointTopic_label_, 0,0,1,1);
      JointTopic_label_->setStyleSheet("font-size: 10px;");
      LabelMap["JointTopic_label"] = JointTopic_label_;
      QComboBox *JointState_topic_CB_  = new QComboBox();
      TopicLayout->addWidget(JointState_topic_CB_, 0,1,1,1);
      JointState_topic_CB_ -> setStyleSheet("font-size: 10px;");
      ComboBoxMap["JointState_topic_CB"] = JointState_topic_CB_;
      QObject::connect(JointState_topic_CB_,  static_cast<void (QComboBox::*)(int)>(&QComboBox::activated),this, &ACIN_ctrl_UI_C::handleCB_JointState_topic);
      //this->add_topics_to_CB(Traj_topic_CB_,"acin_robot_support/PoseState");
      sub_Joint_actual_ = nh_.subscribe("/ros_acin_gazebo_robot_plugin/joint_state", 1, &ACIN_ctrl_UI_C::JointSpaceTrajActual_Callback, this);
    }
    {
      QLabel* PoseTopic_label_ = new QLabel("Pose actual Topic");
      TopicLayout->addWidget(PoseTopic_label_, 1,0,1,1);
      PoseTopic_label_->setStyleSheet("font-size: 10px;");
      LabelMap["PoseTopic_label"] = PoseTopic_label_;

      QComboBox *Traj_topic_CB_  = new QComboBox();
      TopicLayout->addWidget(Traj_topic_CB_, 1,1,1,1);
      Traj_topic_CB_->setStyleSheet("font-size: 10px;");
      ComboBoxMap["Traj_topic_CB"] = Traj_topic_CB_;
      QObject::connect(Traj_topic_CB_,  static_cast<void (QComboBox::*)(int)>(&QComboBox::activated),this, &ACIN_ctrl_UI_C::handleCB_traj_topic);
      //this->add_topics_to_CB(Traj_topic_CB_,"acin_robot_support/PoseState");
      sub_Task_actual_ = nh_.subscribe("/ros_acin_robot_control/TaskSpace_traj_actual", 1, &ACIN_ctrl_UI_C::TaskSpaceTrajActual_Callback, this);
    }
    mainLayout->addWidget(TopicGroupBox, 0, 6, 1, 2);
    TopicLayout-> setHorizontalSpacing(2);
    TopicLayout-> setVerticalSpacing(2);
    TopicLayout-> setContentsMargins(1,1,1,1);//left top right bottom);
    TopicGroupBox->setLayout(TopicLayout);
    TopicGroupBox->show();
  }


  /*##########################################################################*/
  /*##########################################################################*/
  {
    QGroupBox *SollPosGroupBox = new QGroupBox(this);
    QGridLayout *SollPosLayout = new QGridLayout;
    SollPosGroupBox->setStyleSheet("QGroupBox{padding-top:15px; margin-top:-15px}");

   /************************************************************************/
    /************************************************************************/
    /****************UI elements for setting the desired soll position*****/
    /************************************************************************/
    /************************************************************************/

    #if _ROBOT_q_DIM_ > 6
      const uint16_t MaxNumCoords = _ROBOT_q_DIM_;
    #else
      const uint16_t MaxNumCoords = 6;
    #endif
    for(uint16_t ii = 0; ii< MaxNumCoords; ii++){
        {
          QLineEdit *soll_LE = new QLineEdit(SollPosGroupBox);
          std::string soll_str ="soll_"+std::to_string(ii);

          if (ii < _ROBOT_q_DIM_){
            soll_LE->setPlaceholderText(QString::number(Joint_soll_.Joints[ii],'f',2));
          }
          SollPosLayout->addWidget(soll_LE, ii, 0,1,2);
          LineEdit_Map_[tr(soll_str.c_str())] = soll_LE;

          std::string label_CB_str ="";
          if (ii < _ROBOT_q_DIM_){
            label_CB_str ="q_"+std::to_string(ii);
          }
          QCheckBox *soll_checkB = new QCheckBox(tr(label_CB_str.c_str()), SollPosGroupBox);
          SollPosLayout->addWidget(soll_checkB, ii, 2,1,1);
          soll_checkB-> setStyleSheet("font-size: 10px;");
          CheckBox_Map_[tr(soll_str.c_str())] = soll_checkB;
        }
    }

    QPushButton *sollPos_button_ = new QPushButton("Set Soll Pos.", SollPosGroupBox);
    SollPosLayout->addWidget(sollPos_button_, MaxNumCoords, 0, 1, 2);
    sollPos_button_-> setStyleSheet("font-size: 10px;");
    connect(sollPos_button_, &QPushButton::released, this, &ACIN_ctrl_UI_C::handleButton_sollPos);
    ButtonMap["sollPos_button"] = sollPos_button_;


    QComboBox *unit_soll_Pos_  = new QComboBox(SollPosGroupBox);
    SollPosLayout->addWidget(unit_soll_Pos_, 0, 3, 1, 1);
    unit_soll_Pos_-> addItem(tr("°"), QVariant(0));
    unit_soll_Pos_-> addItem(tr("rad"), QVariant(1));
    unit_soll_Pos_->setStyleSheet("font-size: 10px;");
    ComboBoxMap["unit_soll_Pos"] = unit_soll_Pos_;

    mainLayout->addWidget(SollPosGroupBox, 0, _ROBOT_q_DIM_+1, 2, 3);
    SollPosGroupBox->setLayout(SollPosLayout);
    SollPosLayout-> setHorizontalSpacing(1);
    SollPosLayout-> setVerticalSpacing(1);
    SollPosLayout-> setContentsMargins(1,1,1,1);//left top right bottom);
    SollPosGroupBox->show();
  }

  /*****************************************************************************/
  /*****************************************************************************/
  /*****************************************************************************/
  {
    //QMainWindow_->setAttribute(Qt::WA_QuitOnClose, true);
    QGroupBox *ButtonGroupBox = new QGroupBox(this);
    QGridLayout *ButtonGroupBoxLayout = new QGridLayout;
    //ButtonGroupBox->setGeometry(QRect(QPoint(x_pos_loc, y_pos_loc), QSize(L_BUT_X, L_BUT_Y)));
    ButtonGroupBox->setStyleSheet("QGroupBox{padding-top:15px; margin-top:-15px}");
    {
      QPushButton *gravity_button_= new QPushButton("gravity\ncomp.",ButtonGroupBox);
      ButtonGroupBoxLayout->addWidget(gravity_button_,0,0,1,1);
      gravity_button_->setStyleSheet("font-size: 10px");
      connect(gravity_button_, &QPushButton::released, this, &ACIN_ctrl_UI_C::handleButton_grav_mode);
      setButtonState(gravity_button_,true);
      ButtonMap["gravity_button"] = gravity_button_;
      s_gravity_comp_.data = true;
    }
    {
      QPushButton *motion_EN_button_ = new QPushButton("motion enable",ButtonGroupBox);
      ButtonGroupBoxLayout->addWidget(motion_EN_button_,0,1,1,1);
      connect(motion_EN_button_, &QPushButton::released, this, &ACIN_ctrl_UI_C::handleButton_motion_EN);
      ButtonMap["motion_EN_button"] = motion_EN_button_;
      QPushButton *motion_DIS_button_ = new QPushButton("motion disable",ButtonGroupBox);
      ButtonGroupBoxLayout->addWidget(motion_DIS_button_,0,2,1,1);
      connect(motion_DIS_button_, &QPushButton::released, this, &ACIN_ctrl_UI_C::handleButton_motion_DIS);
      ButtonMap["motion_DIS_button"] = motion_DIS_button_;
      s_motion_EN_DIS_.data = false;
      setButtonState(motion_EN_button_,s_motion_EN_DIS_.data);
      setButtonState(motion_DIS_button_,!s_motion_EN_DIS_.data);
      handleButton_motion_DIS();
    }
    {
      QPushButton *frictionComp_button_ = new QPushButton("friction comp.",ButtonGroupBox);
      ButtonGroupBoxLayout->addWidget(frictionComp_button_,0,3,1,1);
      connect(frictionComp_button_, &QPushButton::released, this, &ACIN_ctrl_UI_C::handleButton_frictionComp);
      ButtonMap["frictionComp_button"] = frictionComp_button_;
      s_frictionComp_.data = false;
      setButtonState(frictionComp_button_,s_frictionComp_.data);
    }
    {
      QPushButton *singularPert_button_ = new QPushButton("singular pert.",ButtonGroupBox);
      ButtonGroupBoxLayout->addWidget(singularPert_button_,1,3,1,1);
      connect(singularPert_button_, &QPushButton::released, this, &ACIN_ctrl_UI_C::handleButton_singular_perturbation);
      ButtonMap["singularPert_button"] = singularPert_button_;
      s_singularPert_.data = false;
      setButtonState(singularPert_button_, s_singularPert_.data);
    }

    {
      /*##########################################################################*/
      /*combo Box for selection of getting pose data for the current robot position */
      /*##########################################################################*/

      for(uint16_t ii = 0; ii< _ROBOT_q_DIM_; ii++){
        QSlider *moter_enable_silder_ = new QSlider(Qt::Horizontal, this);
        std::string motor_enable_str ="motor_enable_"+std::to_string(ii);
        moter_enable_silder_ -> setStyleSheet("font-size: 10px;");
        ButtonGroupBoxLayout->addWidget(moter_enable_silder_,2,ii,1,1);
        //moter_enable_silder_ -> setGeometry(QRect(QPoint(400, 25 + 25 *ii), QSize(50, 25)));
        moter_enable_silder_ -> setTickInterval(1);
        moter_enable_silder_ -> setSingleStep(1);
        moter_enable_silder_ -> setMinimum(0);
        moter_enable_silder_ -> setMaximum(1);
        s_motor_EN_.Joints[ii] = 0;
        moter_enable_silder_ -> setValue(s_motor_EN_.Joints[ii]);
        setSliderState(moter_enable_silder_,s_motor_EN_.Joints[ii] ==1);
        SliderMap[motor_enable_str.c_str()] = moter_enable_silder_;
        connect(moter_enable_silder_, &QSlider::valueChanged, this, &ACIN_ctrl_UI_C::slider_setValue_Callback);

      }
      QPushButton *motor_DIS_button_ = new QPushButton("motor lock", this);
      ButtonGroupBoxLayout->addWidget(motor_DIS_button_,2,_ROBOT_q_DIM_,1,1);
      connect(motor_DIS_button_, &QPushButton::released, this, &ACIN_ctrl_UI_C::handleButton_motor_sw);
      ButtonMap["motor_DIS_button"] = motor_DIS_button_;
      setButtonState(ButtonMap["motor_DIS_button"],false);
    }
    {
      /**************************************************************************/
      /*rqt .xml file, get .xml files for mutlplot, define folder, handle check box activation, which .xml file should be loaded, handle mutiplot lunch in the callback handle*/
      /**************************************************************************/
      QComboBox * rqt_plot_CB_  = new QComboBox(ButtonGroupBox);
      ButtonGroupBoxLayout->addWidget(rqt_plot_CB_,1, 4,1,1);
      // Using decltype to avoid figuring out the ugly pointer-to-member-function syntax.
      // Assumes all signals have the same arguments.
      QObject::connect(rqt_plot_CB_,  static_cast<void (QComboBox::*)(int)>(&QComboBox::activated),this, &ACIN_ctrl_UI_C::handleComboBox_RQTPlot);
      ComboBoxMap["rqt_plot_CB"]=rqt_plot_CB_;
      QPushButton *select_RQT_folder_button_ = new QPushButton("",ButtonGroupBox);
      ButtonGroupBoxLayout->addWidget(select_RQT_folder_button_,1, 5,1,1);
      select_RQT_folder_button_->setIcon(QIcon::fromTheme("edit-find")); //list-add
      connect(select_RQT_folder_button_, &QPushButton::released, this, &ACIN_ctrl_UI_C::handleButton_RQTFolder);
      ButtonMap["RQT_folder"] =select_RQT_folder_button_;
    }
    mainLayout->addWidget(ButtonGroupBox, 1, 0, 1, _ROBOT_q_DIM_+1);
    ButtonGroupBoxLayout-> setHorizontalSpacing(1);
    ButtonGroupBoxLayout-> setVerticalSpacing(1);
    ButtonGroupBoxLayout-> setContentsMargins(1,1,1,1);//left top right bottom);
    ButtonGroupBox->setLayout(ButtonGroupBoxLayout);
    ButtonGroupBox->show();
    //mainLayout->addWidget(ButtonGroupBox,0,0,1,5);
    //this->w->setLayout(mainLayout);
    //this->w->setWindowTitle("Grid Layouts (3x4)");
    //this->w->show();
    //this->setLayout(mainLayout);
  }
  /*****************************************************************************/
  /*****************************************************************************/
  /*****************************************************************************/
  {
    QGroupBox *SetCTRLConfigGroupBox = new QGroupBox(this);
    QGridLayout *SetCTRLConfigLayout = new QGridLayout;
    SetCTRLConfigGroupBox->setStyleSheet("QGroupBox{padding-top:15px; margin-top:-15px}");
    {
      /*Tool parameter set*/
      QPushButton *ToolParam_button_= new QPushButton("set ToolParam.", this);
      SetCTRLConfigLayout->addWidget(ToolParam_button_,0,0,1,2);
      //ToolParam_button_->setGeometry(QRect(QPoint(x_pos_loc, y_pos_loc), QSize(L_BUT_X, L_BUT_Y)));
      //y_pos_loc += L_BUT_Y;
      ToolParam_button_->setStyleSheet("font-size: 10px");
      connect(ToolParam_button_, &QPushButton::released, this, &ACIN_ctrl_UI_C::handleButton_setToolParam);
      ButtonMap["ToolParam_button"] = ToolParam_button_;
      {
        QLabel *m_label_ = new QLabel("m",this);
        SetCTRLConfigLayout->addWidget(m_label_,1,0,1,1);
        m_label_->setStyleSheet("font-size: 10px");
        //m_label_ -> setGeometry(QRect(QPoint(x_pos_loc, y_pos_loc), QSize(SMALL_TF_X, 15)));
        LabelMap["m_label"] = m_label_;

        QLineEdit *m_TF_ = new QLineEdit(this);
        m_TF_ -> setPlaceholderText(QString::number(s_ToolParam_.m));
        SetCTRLConfigLayout->addWidget(m_TF_, 2,0,1,1);
        //m_TF_ -> setGeometry(QRect(QPoint(x_pos_loc, y_pos_loc+15), QSize(SMALL_TF_X, TINY_TF_Y)));
        LineEdit_Map_["m"] = m_TF_;
      }
      {
        QLabel *spx_label_ = new QLabel("spx",this);
        spx_label_->setStyleSheet("font-size: 10px");
        //spx_label_ -> setGeometry(QRect(QPoint(x_pos_loc, y_pos_loc), QSize(SMALL_TF_X, 15)));
        SetCTRLConfigLayout->addWidget(spx_label_, 1,1,1,1);
        LabelMap["spx_label"] = spx_label_;
        QLineEdit *spx_TF_ = new QLineEdit(this);
        spx_TF_ -> setPlaceholderText(QString::number(s_ToolParam_.spx));
        SetCTRLConfigLayout->addWidget(spx_TF_, 2,1,1,1);
        //spx_TF_ -> setGeometry(QRect(QPoint(x_pos_loc, y_pos_loc+15), QSize(SMALL_TF_X, TINY_TF_Y)));
        LineEdit_Map_["spx"] = spx_TF_;
      }
      {
        QLabel *spy_label_ = new QLabel("spy",this);
        spy_label_->setStyleSheet("font-size: 10px");
        SetCTRLConfigLayout->addWidget(spy_label_, 3,0,1,1);
        //spy_label_ -> setGeometry(QRect(QPoint(x_pos_loc, y_pos_loc), QSize(SMALL_TF_X, 15)));
        LabelMap["spy_label"] = spy_label_;
        QLineEdit *spy_TF_ = new QLineEdit(this);
        spy_TF_ -> setPlaceholderText(QString::number(s_ToolParam_.spy));
        SetCTRLConfigLayout->addWidget(spy_TF_, 4,0,1,1);
        //spy_TF_ -> setGeometry(QRect(QPoint(x_pos_loc, y_pos_loc+15), QSize(SMALL_TF_X, TINY_TF_Y)));
        LineEdit_Map_["spy"] = spy_TF_;
      }
      {
        QLabel *spz_label_ = new QLabel("spz",this);
        spz_label_->setStyleSheet("font-size: 10px");
        SetCTRLConfigLayout->addWidget(spz_label_, 3,1,1,1);
        //spz_label_ -> setGeometry(QRect(QPoint(x_pos_loc, y_pos_loc), QSize(SMALL_TF_X, 15)));
        LabelMap["spz_label"] = spz_label_;
        QLineEdit *spz_TF_ = new QLineEdit(this);
        spz_TF_ -> setPlaceholderText(QString::number(s_ToolParam_.spz));
        SetCTRLConfigLayout->addWidget(spz_TF_, 4,1,1,1);
        //spz_TF_ -> setGeometry(QRect(QPoint(x_pos_loc, y_pos_loc+15), QSize(SMALL_TF_X, TINY_TF_Y)));
        LineEdit_Map_["spz"] = spz_TF_;
      }
    }
    mainLayout->addWidget(SetCTRLConfigGroupBox, 2, 0, 1, 2);
    SetCTRLConfigGroupBox->setLayout(SetCTRLConfigLayout);
    SetCTRLConfigLayout-> setHorizontalSpacing(2);
    SetCTRLConfigLayout-> setVerticalSpacing(2);
    SetCTRLConfigLayout-> setContentsMargins(2,2,2,2);//left top right bottom);
    SetCTRLConfigGroupBox->show();
  }
  //this->setLayout(mainLayout);
  this->show();
}



void ACIN_ctrl_UI_C::TaskSpaceTrajActual_Callback(const PoseState_t &Pose_a){
  boost::unique_lock<boost::timed_mutex> lock{PoseMutex_, boost::try_to_lock};
  if(lock.owns_lock() || lock.try_lock_for(boost::chrono::microseconds{1})){
    this->Pose_actuel_ = Pose_a;
  }
  valid_actuel_Pose_ = true;
  try{
    valid_pose_watch_dog_.detach();

    valid_pose_watch_dog_ = std::thread(&ACIN_ctrl_UI_C::valid_pose_watch_dog_Callback,this);
  }catch(std::exception e){

  }
}
void ACIN_ctrl_UI_C::JointSpaceTrajActual_Callback(const ros_JointState_t &Joint_a){
  boost::unique_lock<boost::timed_mutex> lock{JointMutex_, boost::try_to_lock};
  if(lock.owns_lock() || lock.try_lock_for(boost::chrono::microseconds{1})){
    for(uint16_t ii = 0; ii < _ROBOT_q_DIM_; ii++){
      this->Joint_actuel_.Joints[ii] = Joint_a.position[ii];
    }
  }
  valid_actuel_Joint_ = true;
  // try{
  //   valid_joint_watch_dog_.detach();
  //
  // //valid_joint_watch_dog_ = std::thread(&ACIN_ctrl_UI_C::valid_joint_watch_dog_Callback,this);
  // }catch(std::exception e){
  //
  // }
}
void ACIN_ctrl_UI_C::valid_joint_watch_dog_Callback(){
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  valid_actuel_Joint_ = false;
}
void ACIN_ctrl_UI_C::valid_pose_watch_dog_Callback(){
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  valid_actuel_Pose_ = false;
}

void ACIN_ctrl_UI_C::handleCB_traj_topic(int CB_indx){
  //printf("##handleCB_traj_topic##\n");
  QVariant cur_ele = ComboBoxMap["Traj_topic_CB"]->currentData();
  QString cur_text = ComboBoxMap["Traj_topic_CB"]->currentText();
  this->add_topics_to_CB(ComboBoxMap["Traj_topic_CB"], "ros_acin_robot_support/PoseState");
  int indx = ComboBoxMap["Traj_topic_CB"]->findData(cur_ele);
  if(indx !=-1 ){
    ComboBoxMap["Traj_topic_CB"]->setCurrentIndex(indx);
  }else{
    return;
  }
  std::string topic_str = cur_text.toStdString();
  //std::cout<< "ComboBoxMap[\"Traj_topic_CB\"] " << cur_text.toStdString()<<std::endl;
  sub_Task_actual_ = nh_.subscribe(topic_str.c_str(), 1, &ACIN_ctrl_UI_C::TaskSpaceTrajActual_Callback, this);
  valid_actuel_Pose_ = false;
}
void ACIN_ctrl_UI_C::handleCB_JointState_topic(int CB_indx){
  //printf("##handleCB_JointState_topic##\n");
  QVariant cur_ele = ComboBoxMap["JointState_topic_CB"]->currentData();
  QString cur_text = ComboBoxMap["JointState_topic_CB"]->currentText();
  this->add_topics_to_CB(ComboBoxMap["JointState_topic_CB"], "sensor_msgs/JointState");
  int indx = ComboBoxMap["JointState_topic_CB"]->findData(cur_ele);
  if(indx !=-1 ){
    ComboBoxMap["JointState_topic_CB"]->setCurrentIndex(indx);
  }else{
    return;
  }
  std::string topic_str = cur_text.toStdString();
  //std::cout<< "ComboBoxMap[\"JointState_topic_CB\"] " << cur_text.toStdString()<<std::endl;
  sub_Joint_actual_ = nh_.subscribe(topic_str.c_str(), 1, &ACIN_ctrl_UI_C::JointSpaceTrajActual_Callback, this);
  valid_actuel_Joint_ = false;
}

// callback if trajactory mode combobox changed, Traj_mode... new index if 0 joint space controller will be active when a trajetory is dispatched,
// edit unit labels, change comboboxs assoicated to it with coordinates to adjust
void ACIN_ctrl_UI_C::handleCB_Traj_mode(int Traj_mode){
  //QVariant Traj_mode = Traj_mode_CB_ ->currentData();
  QComboBox *coord_CB_ = ComboBoxMap["coord"];
  coord_CB_->clear();
  ComboBoxMap["adjust_unit"]->clear();
  ComboBoxMap["unit_soll_Pos"]->clear();
  /***************************************************************************/
  /*******************Joint Space Mode***********************************/
  /***************************************************************************/
  if(Traj_mode == 0){
    ComboBoxMap["adjust_unit"]-> addItem(tr("°"), QVariant(0));
    ComboBoxMap["adjust_unit"]-> addItem(tr("rad"), QVariant(1));
    ComboBoxMap["unit_soll_Pos"]-> addItem(tr("°"), QVariant(0));
    ComboBoxMap["unit_soll_Pos"]-> addItem(tr("rad"), QVariant(1));
    joint_space_inc_ = true;
    for (uint8_t ii =_ROBOT_q_DIM_; ii < 6; ii++ ){
      std::string soll_str ="soll_"+std::to_string(ii);
      std::string label_CB_str ="";
      label_CB_str ="q_"+std::to_string(ii);
      QCheckBox * q_CB = CheckBox_Map_[tr(soll_str.c_str())];
      if(q_CB!=NULL){
        q_CB ->setText("");
      }
    }
    for (uint8_t ii =0; ii < _ROBOT_q_DIM_; ii++ ){
      std::string id_str =  "q" + std::to_string(ii);
      coord_CB_-> addItem(tr(id_str.c_str()), QVariant(ii));
      std::string soll_str ="soll_"+std::to_string(ii);
      std::string label_CB_str ="";
      label_CB_str ="q_"+std::to_string(ii);
      QCheckBox * q_CB = CheckBox_Map_[tr(soll_str.c_str())];
      if(q_CB!=NULL){
        q_CB ->setText(QString(label_CB_str.c_str()));
      }
    }
  }
  /***************************************************************************/
  /*******************Task Space Mode***********************************/
  /***************************************************************************/
  if(Traj_mode == 1){
    ComboBoxMap["adjust_unit"]-> addItem(tr("m\\°"), QVariant(0));
    ComboBoxMap["adjust_unit"]-> addItem(tr("m\\rad"), QVariant(1));
    ComboBoxMap["adjust_unit"]-> addItem(tr("mm\\°"), QVariant(2));
    ComboBoxMap["adjust_unit"]-> addItem(tr("mm\\rad"), QVariant(3));
    ComboBoxMap["adjust_unit"]->setCurrentIndex(ComboBoxMap["adjust_unit"]->findText("mm\\°"));
    ComboBoxMap["unit_soll_Pos"]-> addItem(tr("m\\°"), QVariant(0));
    ComboBoxMap["unit_soll_Pos"]-> addItem(tr("m\\rad"), QVariant(1));
    ComboBoxMap["unit_soll_Pos"]-> addItem(tr("mm\\°"), QVariant(2));
    ComboBoxMap["unit_soll_Pos"]-> addItem(tr("mm\\rad"), QVariant(3));
    this->adjust_value_ = 0;
    adjust_TF_->setPlaceholderText(QString::number(this->adjust_value_,'f',2));
    adjust_TF_->clear();
    for (uint8_t ii =6; ii < _ROBOT_q_DIM_; ii++ ){
      std::string soll_str ="soll_"+std::to_string(ii);
      std::string label_CB_str ="";
      label_CB_str ="q_"+std::to_string(ii);
      QCheckBox * q_CB = CheckBox_Map_[tr(soll_str.c_str())];
      if(q_CB!=NULL){
        q_CB ->setText("");
      }
    }
    for (uint8_t ii =0; ii < 6; ii++ ){
      coord_CB_-> addItem(tr(this->taskSpace_coord_labels[ii]), QVariant(ii));
      std::string soll_str ="soll_"+std::to_string(ii);
      std::string label_CB_str ="q_"+std::to_string(ii);
      QCheckBox * q_CB = CheckBox_Map_[tr(soll_str.c_str())];
      if(q_CB!=NULL){
        q_CB ->setText(QString(taskSpace_coord_labels[ii]));
      }
    }
    joint_space_inc_ = false;
  }
  this->handleCB_TaskTraj_index(-1);
}

void ACIN_ctrl_UI_C::mousePressEvent(QMouseEvent *e){
  this->add_topics_to_CB(ComboBoxMap["Traj_topic_CB"],"ros_acin_robot_support/PoseState");
  this->add_topics_to_CB(ComboBoxMap["JointState_topic_CB"],"sensor_msgs/JointState");
}

//get all text fields of the application, read in user input
void ACIN_ctrl_UI_C::fetch_userUIinput(){
    bool ok;
    this->reject_userUIcmd = false;


    /*get the legnth of a Trajecotry, duration of a movement,...*/
    if (accel_TF_->isModified()){
      QString accelPerc_qstr = accel_TF_->text();
      double accelPerc_tmp = accelPerc_qstr.toDouble(&ok);
      if(ok){
        this->UserAccelPerc_ = accelPerc_tmp/100.0;
      }else{
        accelPerc_qstr = accel_TF_->placeholderText();
        this->UserAccelPerc_ = accelPerc_qstr.toDouble(&ok)/100.0;
        ACIN_CTRL_WARN("Can't parse desired acceleration percentage from UI, use default value, %f %%",this->UserAccelPerc_);
        accel_TF_->clear();
      }
    }

    /*get the legnth of a Trajecotry, duration of a movement,...*/
    if (T_TF_->isModified()){
      QString T_qstr = T_TF_->text();
      double T_tmp = T_qstr.toDouble(&ok);
      if(ok){
        this->T_end_ = T_tmp;
      }else{
        T_qstr = T_TF_->placeholderText();
        this->T_end_ = T_qstr.toDouble(&ok);
        ACIN_CTRL_WARN("Can't parse T_end from UI, use default value");
        T_TF_->clear();
      }
    }
    /*value for adjusting only a coordinate, amplitude of test lissajous trajectory,...*/
    if(adjust_TF_->isModified()){
      QString adjust_qstr = adjust_TF_->text();
      double adjust_tmp = adjust_qstr.toDouble(&ok);
      if(ok){
        QVariant Traj_mode = ComboBoxMap["Traj_mode_CB"] ->currentData();
        this->adjust_value_ = adjust_tmp;
      }else{
        ACIN_CTRL_WARN("Can't parse adjust value from UI, use default value");
        QString adjust_qstr = adjust_TF_->placeholderText();
        this->adjust_value_ = adjust_qstr.toDouble(&ok);
        adjust_TF_->clear();
      }
    }else{
      QString adjust_qstr = adjust_TF_->placeholderText();
      this->adjust_value_ = adjust_qstr.toDouble(&ok);
    }
    /*get desired postion based on the last call of handleCB_Traj_mode (TaskSpace or JointSpace Mode),...*/
    uint16_t numSollPos = _ROBOT_q_DIM_;
    if (!joint_space_inc_){
      numSollPos = 6;
    }
    double scale_angle =1.0;
    double scale_pos = 1.0;
    if (ComboBoxMap["unit_soll_Pos"]!=nullptr){
      if (((ComboBoxMap["unit_soll_Pos"]->currentData().toInt(&ok)) % 2) == 0){
        if(!ok) ACIN_CTRL_ASSERT("CAN't convert qvariant of the soll positions unit");
        scale_angle = M_PI/180.0;
        ACIN_CTRL_ERROR("SCALE readin angle of the soll orientation., from deg to rad");
      }
      if ( (ComboBoxMap["unit_soll_Pos"]->currentData() > 1) && (ComboBoxMap["unit_soll_Pos"]->currentData()  < 4)){
        scale_pos = 1/1000.0;
        ACIN_CTRL_ERROR("SCALE readin position of soll pos., from mm to m");
      }
    }else{
      ACIN_CTRL_WARN("Cant get scale value for the soll Postion, UI Comobo Box Element not set ?");
    }
    for(uint16_t ii = 0; ii< numSollPos; ii ++){
      std::string soll_str ="soll_"+std::to_string(ii);
      QLineEdit *soll_ii_LE = LineEdit_Map_[tr(soll_str.c_str())];
      QCheckBox *soll_ii_CB = CheckBox_Map_[tr(soll_str.c_str())];

      if (soll_ii_LE == NULL || soll_ii_CB == NULL){
        ACIN_CTRL_WARN("<ACIN_CTRL_UI> can't get object to parse user input desired soll position %u",(uint32_t)ii);
        continue;
      }
      sollCheckbox_[ii] = soll_ii_CB->isChecked();
      double sollValue_ii = 0;
      if (soll_ii_LE->text().isEmpty()){
        sollValue_ii = soll_ii_LE->placeholderText().toDouble(&ok);
      }else{
        sollValue_ii = soll_ii_LE->text().toDouble(&ok);
      }
      if(ok){
        if (joint_space_inc_){
          this->Joint_soll_.Joints[ii] = sollValue_ii*scale_angle;
        }else{
          if(ii < 3){//x y z postions
            this->Pose_soll_[ii] = sollValue_ii*scale_pos;
          }else{//euler angles Roll pitch yaw
            this->Pose_soll_[ii] = sollValue_ii*scale_angle;
          }

        }
      }else{
        ACIN_CTRL_WARN("Can't parse soll Value [%u] from UI, don't use this soll value",(uint32_t)ii);
        sollCheckbox_[ii] = false;
        CheckBox_Map_[soll_str.c_str()] -> setChecked(false);
        if (joint_space_inc_){
          this->Joint_soll_.Joints[ii] = 0;
        }else{
          ACIN_CTRL_ERROR("not implemented task Space soll position set, handling of singularitys ?");
          ACIN_CTRL_ASSERT(0==1);
        }
        //QString q_ii_default_qstr = q_soll_ii->placeholderText();
        //this->Joint_soll_.Joints[ii] = q_ii_default_qstr.toDouble(&ok);
        //q_soll_ii->clear();
      }
    }
    {
      QLineEdit *m_LE =LineEdit_Map_["m"];
      if(m_LE->isModified()){
        double m = m_LE->text().toDouble(&ok);
        if(ok){
          s_ToolParam_.m = m;
        }else{
          ACIN_CTRL_WARN("Can't parse mass from UI, use default value");
          s_ToolParam_.m = m_LE->placeholderText().toDouble(&ok);
          m_LE->clear();
        }
      }
    }
    {
      QLineEdit *spx_LE =LineEdit_Map_["spx"];
      if(spx_LE->isModified()){
        double spx = spx_LE->text().toDouble(&ok);
        if(ok){
          s_ToolParam_.spx = spx;
        }else{
          ACIN_CTRL_WARN("Can't parse spx from UI, use default value");
          s_ToolParam_.spx = spx_LE->placeholderText().toDouble(&ok);
          spx_LE->clear();
        }
      }
    }
    {
      QLineEdit *spy_LE =LineEdit_Map_["spy"];
      if(spy_LE->isModified()){
        double spy = spy_LE->text().toDouble(&ok);
        if(ok){
          s_ToolParam_.spy = spy;
        }else{
          ACIN_CTRL_WARN("Can't parse spy from UI, use default value");
          s_ToolParam_.spy = spy_LE->placeholderText().toDouble(&ok);
          spy_LE->clear();
        }
      }
    }
    {
      QLineEdit *spz_LE =LineEdit_Map_["spz"];
      if(spz_LE->isModified()){
        double spz = spz_LE->text().toDouble(&ok);
        if(ok){
          s_ToolParam_.spz = spz;
        }else{
          ACIN_CTRL_WARN("Can't parse mass from UI, use default value");
          s_ToolParam_.spz = spz_LE->placeholderText().toDouble(&ok);
          spz_LE->clear();
        }
      }
    }


  //}
  if (joint_space_inc_){
    if (ComboBoxMap["adjust_unit"]->currentData() == 0){
      this->adjust_value_ = this->adjust_value_*M_PI/180.0; // from ° to rad, controller assumes rad values
    }
  }else{
    if (ComboBoxMap["coord"]->currentData() < 3){
      if (ComboBoxMap["adjust_unit"]->currentData() > 1){
        this->adjust_value_ = this->adjust_value_/1000.0; //from mm to m, controller assumes m values
        ACIN_CTRL_WARN("taskSpace position adjustment, scale mm value to m");
      }
    }else{
      if (((ComboBoxMap["adjust_unit"]->currentData().toInt(&ok)) % 2) == 0){
        this->adjust_value_ = this->adjust_value_*M_PI/180; //from ° to rad, controller assumes rad values for angles
        ACIN_CTRL_WARN("taskSpace orientation adjustment, scale ° value to rad");
      }
    }
  }
  std::string label_adjust_unit = ComboBoxMap["adjust_unit"]->itemData(ComboBoxMap["adjust_unit"]->currentIndex()).toString().toStdString();
  QVariant coord_qVar = ComboBoxMap["coord"] ->currentData();
  double coord_tmp = coord_qVar.toDouble(&ok);
  if(ok){
     this->coord_ = coord_tmp;
   }else{
     ACIN_CTRL_WARN("Can't parse coordinate from UI, use default value");
     this->coord_ = 0;
   }





  std::cout << string_format("<acin_ctrl_UI> fetched User Inputs: \n\tcoord: %lu, adjust_value_: %0.8f, T end:%1.2f, Mode:= %i",
                              this->coord_,
                              this->adjust_value_,
                              this->T_end_,
                              (uint32_t) joint_space_inc_) << std::endl;
}



void ACIN_ctrl_UI_C::setSliderState(QSlider *pButton, const bool &State){
  pButton->setAutoFillBackground(true);
  if (State){
    pButton->setStyleSheet("border: 1px solid green");
    //pButton->setStyleSheet("QSlider::groove:horizontal {font-size: 10px; color: blue; background-color: green}");
  }else{
    pButton->setStyleSheet("border: 2px solid red");//
  }
  pButton->show();
}

bool ACIN_ctrl_UI_C::isClosed(){
  return UIArg_sptr_->finished_flag==1;
}
/* ----------------------------------------------------------------------------*/
/******************************** BUTTON CALLBACKs *****************************/
/* ----------------------------------------------------------------------------*/
void ACIN_ctrl_UI_C::setButtonState(QPushButton *pButton, const bool &State){
  if (pButton == nullptr) return;
  pButton->setAutoFillBackground(true);
  if (State){
    pButton->setStyleSheet("font-size: 10px; color: blue; background-color: green");
  }else{
    pButton->setStyleSheet("font-size: 10px; color: blue; background-color: red");
  }
  pButton->show();
}

void ACIN_ctrl_UI_C::set_frictionComp_button_state(void){
  try{
    UIArg_sptr_->FrictionComp_pub.isLatched();
  }catch(ros::Exception e){
    ACIN_CTRL_WARN("<acin_ctrl_UI> can't publish frictionComp");
    return;
  }
  UIArg_sptr_->FrictionComp_pub.publish(s_frictionComp_);
  setButtonState(ButtonMap["frictionComp_button"],s_frictionComp_.data);
}
void ACIN_ctrl_UI_C::gravComp_change(){
  s_gravity_comp_.data = !s_gravity_comp_.data;
  this->set_gravComp_button_state();
}
void ACIN_ctrl_UI_C::motion_disable_Callback(){
  handleButton_motion_DIS();
}
void ACIN_ctrl_UI_C::set_gravComp_button_state(void ){
  try{
    UIArg_sptr_->grav_pub.isLatched();
  }catch(ros::Exception e){
    ACIN_CTRL_WARN("<acin_ctrl_UI> can't publish gravity mode");
    return;
  }
  UIArg_sptr_->grav_pub.publish(s_gravity_comp_);
  setButtonState(ButtonMap["gravity_button"],s_gravity_comp_.data);
}
void ACIN_ctrl_UI_C::set_singularPert_button_state(void ){
  try{
    UIArg_sptr_->singularPert_pub.isLatched();
  }catch(ros::Exception e){
    ACIN_CTRL_WARN("<acin_ctrl_UI> can't publish singular perturbation state");
    return;
  }
  UIArg_sptr_->singularPert_pub.publish(s_singularPert_);
  setButtonState(ButtonMap["singularPert_button"], s_singularPert_.data);
}
void ACIN_ctrl_UI_C::ACIN_ctrl_UI_C::handleButton_inc(){
  this->fetch_userUIinput();
  if (this->reject_userUIcmd){
    return;
  }
  std::cout << "inc button pressed" << std::endl;
  ros_CoordAdj_t coord_adj;
  coord_adj.coord_idx = coord_;
  coord_adj.adj_val = adjust_value_;
  coord_adj.T_traj = T_end_; //time to reach the new steady state point
  coord_adj.acc_perc = this->UserAccelPerc_;
  coord_adj.TaskSpace = !joint_space_inc_;
  try{
    UIArg_sptr_-> coordAdjustment_pub.isLatched();
  }catch(ros::Exception e){
    ACIN_CTRL_WARN("<acin_ctrl_UI> can't publish coordAdjustment in the dec. button");
    return;
  }
  UIArg_sptr_ -> coordAdjustment_pub.publish(coord_adj);
}

void ACIN_ctrl_UI_C::handleButton_dec(){
  this->fetch_userUIinput();
  if (this->reject_userUIcmd){
    return;
  }
  std::cout<<"dec button pressed"<<std::endl;
  ros_CoordAdj_t coord_adj;
  coord_adj.coord_idx = coord_;
  coord_adj.adj_val = -1.0*adjust_value_;
  coord_adj.T_traj = T_end_; //time to reach the new steady state point
  coord_adj.acc_perc = this->UserAccelPerc_;
  coord_adj.TaskSpace = !joint_space_inc_;
  try{
    UIArg_sptr_->grav_pub.isLatched();
  }catch(ros::Exception e){
    ACIN_CTRL_WARN("<acin_ctrl_UI> can't publish coordAdjustment in the dec. button");
    return;
  }
  UIArg_sptr_ -> coordAdjustment_pub.publish(coord_adj);
}

void ACIN_ctrl_UI_C::handleButton_grav_mode(){
  std::cout << "Switch - Gravity Mode"<<std::endl;
  if (s_gravity_comp_.data){
    std::cout << "Disable - Gravity Comp. "<<std::endl;
    s_gravity_comp_.data = false;
  }else{
    std::cout << "Enable - Gravity Comp. "<<std::endl;
    s_gravity_comp_.data = true;
  }
  set_gravComp_button_state();
}
void ACIN_ctrl_UI_C::handleButton_frictionComp(){
  std::cout << "Switch - Friction compensation mode"<<std::endl;
  if (s_frictionComp_.data){
    std::cout << "Disable - Friction compensation "<<std::endl;
    s_frictionComp_.data = false;
  }else{
    std::cout << "Enable - Friction compensation"<<std::endl;
    s_frictionComp_.data = true;
  }
  set_frictionComp_button_state();
}

void ACIN_ctrl_UI_C::handleButton_setToolParam(){
  this->fetch_userUIinput();
  try{
    UIArg_sptr_->ToolParam_pub.isLatched();
  }catch(ros::Exception e){
    ACIN_CTRL_WARN("<acin_ctrl_UI> can't publish coordAdjustment in the dec. button");
    return;
  }
  std::ostringstream s;
  s << "<acin_ctrl_UI> Set Tool Paramters: \n\t m=" << s_ToolParam_.m << ", spx="<< s_ToolParam_.spx <<"spy="<<s_ToolParam_.spy<<"spz="<<s_ToolParam_.spz<<std::endl;
  ACIN_CTRL_INFO_STREAM(s);
  UIArg_sptr_->ToolParam_pub.publish(s_ToolParam_);
}
void ACIN_ctrl_UI_C::handleButton_motion_EN(){

  s_motion_EN_DIS_.data=true;
  try{
    UIArg_sptr_->motion_EN_pub.isLatched();
  }catch(ros::Exception e){
    ACIN_CTRL_WARN("<acin_ctrl_UI> can't publish motion enable");
    return;
  }
  UIArg_sptr_->motion_EN_pub.publish(s_motion_EN_DIS_);
  setButtonState(ButtonMap["motion_EN_button"],s_motion_EN_DIS_.data);
  setButtonState(ButtonMap["motion_DIS_button"],!s_motion_EN_DIS_.data);
}
void ACIN_ctrl_UI_C::handleButton_motion_DIS(){
  s_motion_EN_DIS_.data=false;
  s_frictionComp_.data=false;
  set_frictionComp_button_state();
  s_gravity_comp_.data=true;
  set_gravComp_button_state();
  s_singularPert_.data=false;
  set_singularPert_button_state();
  try{
    UIArg_sptr_->motion_EN_pub.isLatched();
  }catch(ros::Exception e){
    ACIN_CTRL_WARN("<acin_ctrl_UI> can't publish motion disable");
    return;
  }
  UIArg_sptr_->motion_EN_pub.publish(s_motion_EN_DIS_);
  setButtonState(ButtonMap["motion_EN_button"],s_motion_EN_DIS_.data);
  setButtonState(ButtonMap["motion_DIS_button"],!s_motion_EN_DIS_.data);
}
void ACIN_ctrl_UI_C::handleButton_singular_perturbation(){
  std::cout << "<acin_ctrl_UI> Switch - singular perturbation mode"<<std::endl;
  if (s_singularPert_.data){
    std::cout << "Disable - singular perturbation "<<std::endl;
    s_singularPert_.data = false;
  }else{
    std::cout << "Enable - singular perturbation"<<std::endl;
    s_singularPert_.data = true;
  }
  set_singularPert_button_state();

}
void ACIN_ctrl_UI_C::handleButton_motor_sw(){
  ros_Joints_t motor_enable = ros_Joints_t();
  motor_enable.Joints =std::vector<double>(_ROBOT_q_DIM_,0.0);
  uint16_t motor_status = 1;
  for(uint16_t ii=0;  ii < _ROBOT_q_DIM_ ; ii++){
      std::string motor_enable_str = "motor_enable_" + std::to_string(ii);
      if (this->SliderMap[tr(motor_enable_str.c_str())] == NULL){
        ACIN_CTRL_WARN("can't get motor switch  %u  with key %s",(uint32_t)ii, motor_enable_str.c_str());
        continue;
      }
      uint16_t moter_enable_ii_flag = this->SliderMap[tr(motor_enable_str.c_str())]->value();
      if(moter_enable_ii_flag != 0){
        motor_status = 0;
      }
  }
  for(uint16_t ii=0;  ii < _ROBOT_q_DIM_ ; ii++){
    std::string motor_enable_str = "motor_enable_" + std::to_string(ii);
    if (this->SliderMap[tr(motor_enable_str.c_str())] == NULL){
      //ACIN_CTRL_WARN("can't get motor switch %u with key %s",(uint32_t)ii, motor_enable_str.c_str());
      continue;
    }

    this->SliderMap[tr(motor_enable_str.c_str())]->setValue(motor_status);
    motor_enable.Joints[ii] =motor_status;
    setSliderState(this->SliderMap[tr(motor_enable_str.c_str())],motor_status==1 );
  }
  setButtonState(ButtonMap["motor_DIS_button"],motor_status == 1);
  try{
    UIArg_sptr_->motor_EN_pub.isLatched();
  }catch(ros::Exception e){
    ACIN_CTRL_WARN("<acin_ctrl_UI> can't publish motion enable");
    return;
  }
  UIArg_sptr_->motor_EN_pub.publish(motor_enable);
}
void ACIN_ctrl_UI_C::slider_setValue_Callback(int value){

  ros_Joints_t motor_enable = ros_Joints_t();
  motor_enable.Joints =std::vector<double>(_ROBOT_q_DIM_,0.0);
  for(uint16_t ii=0;  ii < _ROBOT_q_DIM_ ; ii++){
      std::string motor_enable_str = "motor_enable_" + std::to_string(ii);
      if(this->SliderMap[motor_enable_str.c_str()] == nullptr){
        ACIN_CTRL_WARN("can't get motor switch  %u  with key %s",(uint32_t)ii, motor_enable_str.c_str());
        continue;
      }
      uint16_t moter_enable_ii_flag = this->SliderMap[motor_enable_str.c_str()]->value();
      setSliderState(this->SliderMap[motor_enable_str.c_str()],moter_enable_ii_flag==1 );
      motor_enable.Joints[ii]= (double)moter_enable_ii_flag;
  }
  try{
    UIArg_sptr_->motor_EN_pub.isLatched();
  }catch(ros::Exception e){
    ACIN_CTRL_WARN("<acin_ctrl_UI> can't publish motor enable");
    return;
  }
  UIArg_sptr_->motor_EN_pub.publish(motor_enable);
}
void ACIN_ctrl_UI_C::handleButton_TestTraj(){


  this->fetch_userUIinput();
  if(this->reject_userUIcmd){
    return;
  }
  ros_Header_t header;
  double T_traj = T_end_;
  const uint64_t Traj_N_Knot_loc = _Traj_N_Knot_-3;
  ACIN_CTRL_ASSERT(Traj_N_Knot_loc <= _Traj_N_Knot_);
  std::vector<ros::Time> t_k(Traj_N_Knot_loc);
  Eigen::Matrix<double,Traj_N_Knot_loc,1> t_k_eig = Eigen::Matrix<double,Traj_N_Knot_loc,1>::LinSpaced(Traj_N_Knot_loc, 0.0, T_end_);
  QVariant index_1 = ComboBoxMap["coord_1_traj_CB"]->currentData();
  QVariant index_2 = ComboBoxMap["coord_2_traj_CB"]->currentData();
  uint8_t id_1 = index_1.toInt();
  uint8_t id_2 = index_2.toInt();
  double omega_1 = (2*M_PI)/T_traj;
  double v = 1/2.0; //lissajous figure frequency relationship TODO
  double omega_2 = omega_1*v;
  if(!joint_space_inc_){
    boost::unique_lock<boost::timed_mutex> lock{PoseMutex_, boost::try_to_lock};
    std::vector<ros_Pose_t> PoseArray(Traj_N_Knot_loc);
    bool OrientationAsQaut = false;
    bool orientation_1 = index_1 > 2;
    bool orientation_2 = index_2 > 2;
    if (orientation_1){
      id_1 = id_1 - 2;
    }
    if (orientation_2){
      id_2 =  id_2 - 2;
    }
    ROS_WARN("Dispatch TaskSpace test trajectory");
    std::vector<double> position_offset (3,0.0);
    std::vector<double> orientation_offset (4,0.0);
    if( valid_actuel_Pose_ &&
        (lock.owns_lock() || lock.try_lock_for(boost::chrono::microseconds{1}))){
          for(uint32_t ii = 0; ii < 3; ii++){
            position_offset[ii] =Pose_actuel_.pose.position[ii];
            orientation_offset[ii] =Pose_actuel_.pose.orientation[ii];
          }
          orientation_offset[3] =Pose_actuel_.pose.orientation[3];
    }else{
      ACIN_CTRL_WARN("can't dispatch valid Task Space trajectory, no vaild actuel pose of the robot found or cant aquire lock");
      return;
    }

    for(uint32_t ii = 0; ii < Traj_N_Knot_loc; ii++){
      PoseArray[ii].position = position_offset;
      PoseArray[ii].orientation = orientation_offset;


      if(orientation_1){
        PoseArray[ii].orientation[id_1] +=  adjust_value_*std::sin(omega_1*t_k_eig[ii]);
      }else{
        PoseArray[ii].position[id_1] +=  adjust_value_*std::sin(omega_1*t_k_eig[ii]);

      }
      if(orientation_2){
        PoseArray[ii].orientation[id_2] +=  adjust_value_*std::sin(omega_2*t_k_eig[ii]);
      }else{
        PoseArray[ii].position[id_2] +=  adjust_value_*std::sin(omega_2*t_k_eig[ii]);
      }
      t_k[ii] =  ros::Time(t_k_eig[ii]);
    }

    // for(uint32_t ii = Traj_N_Knot_loc; ii < _Traj_N_Knot_; ii++){
    //   t_k[ii] =  ros::Time(-1);
    //   PoseArray[ii].orientation = PoseArray[Traj_N_Knot_loc-1].orientation;
    //   PoseArray[ii].position = PoseArray[Traj_N_Knot_loc-1].position;
    // }
    ros_TaskSpaceTraj_t TestTrajectory;
    TestTrajectory.OrientationAsQaut = OrientationAsQaut;
    TestTrajectory.PoseArray = PoseArray;
    TestTrajectory.t_k = t_k;
    try{
      UIArg_sptr_->taskSpaceTraj_pub.isLatched();
    }catch(ros::Exception e){
      ACIN_CTRL_WARN("<acin_ctrl_UI> can't publish trail taskspace trajectory");
      return;
    }
    UIArg_sptr_->taskSpaceTraj_pub.publish(TestTrajectory);
  }else{
    ROS_WARN("Dispatch JointSpace test trajectory");
    boost::unique_lock<boost::timed_mutex> lock{JointMutex_, boost::try_to_lock};
    std::vector<ros_Joints_t> JointArray(Traj_N_Knot_loc);
    std::vector<double> joint_offset (_ROBOT_q_DIM_,0.0);
    if( valid_actuel_Joint_ &&
        (lock.owns_lock() || lock.try_lock_for(boost::chrono::microseconds{1}))){
          joint_offset = Joint_actuel_.Joints;
    }else{
      ACIN_CTRL_WARN("can't dispatch valid Joint Space trajectory, no vaild actuel joint data of the robot found or cant aquire lock");
      return;
    }
    for(uint32_t ii = 0; ii < Traj_N_Knot_loc; ii++){
      JointArray[ii].Joints = joint_offset;
      JointArray[ii].Joints[id_1] +=  adjust_value_*std::sin(omega_1*t_k_eig[ii]);
      JointArray[ii].Joints[id_2] +=  adjust_value_*std::sin(omega_2*t_k_eig[ii]);
      t_k[ii] =  ros::Time(t_k_eig[ii]);
    }
    // for(uint32_t ii = Traj_N_Knot_loc; ii < _Traj_N_Knot_; ii++){
    //   t_k[ii] =  ros::Time(-1);
    //   JointArray[ii].Joints = std::vector<double>(_ROBOT_q_DIM_,0.0);
    //   for (uint64_t jj =0; jj < _ROBOT_q_DIM_; jj++){
    //     JointArray[ii].Joints[jj]= JointArray[Traj_N_Knot_loc-1].Joints[jj];
    //   }
    // }
    ACIN_CTRL_WARN("JointArray.size():=%lu",JointArray.size());
    ros_JointSpaceTraj_t JointTestTrajectory;
    JointTestTrajectory.header = ros_Header_t();
    JointTestTrajectory.JointsArray = JointArray;
    JointTestTrajectory.t_k = t_k;
    try{
      UIArg_sptr_->JointSpaceTraj_pub.isLatched();
    }catch(ros::Exception e){
      ACIN_CTRL_WARN("<acin_ctrl_UI> can't publish Joint Space trajectory, is not latched ?");
      return;
    }
    UIArg_sptr_->JointSpaceTraj_pub.publish(JointTestTrajectory);
    ACIN_CTRL_WARN("Finished Dispatch JointSpace test trajectory");
  }
}
void ACIN_ctrl_UI_C::handleButton_sollPos(void){
  this->fetch_userUIinput();
  if (this->reject_userUIcmd){
    return;
  }
  ros_acinSollPosMsg_t sollPos;
  /*
  bool[] coord
  float64[] sollPos

  */
  sollPos.T_traj = T_end_;
  sollPos.acc_perc = this->UserAccelPerc_;
  sollPos.TaskSpace = !joint_space_inc_;
  sollPos.planAlgoSel = 0;
  if(joint_space_inc_){
      sollPos.coord = std::vector<unsigned char>(_ROBOT_q_DIM_,0);
      sollPos.sollPos = std::vector<double>(_ROBOT_q_DIM_,0.0);
      for(uint16_t ii = 0; ii < _ROBOT_q_DIM_;ii++){
        sollPos.sollPos[ii] = Joint_soll_.Joints[ii];
        sollPos.coord [ii] = sollCheckbox_[ii];
      }
  }else{
    sollPos.coord = std::vector<unsigned char>(6, 0);
    sollPos.sollPos = std::vector<double>(6, 0.0);
    for(uint16_t ii = 0; ii < 6; ii++){
      std::cout<<"valid_actuel_Pose_ := "<<valid_actuel_Pose_<<std::endl;
      if (valid_actuel_Pose_ == false){
        ACIN_CTRL_ERROR("Can't accept pose trajectory soll postion, no valid actual position given, Reject User input Command ");
        this->reject_userUIcmd = true;
        break;
      }else{
          if(ii < 3){//x y z postio
              if (std::abs(this->Pose_soll_[ii] - this->Pose_actuel_.pose.position[ii])>= 0.5e-2){
                ACIN_CTRL_ERROR("Allowed Pose offset from pos. [%i] to large (>= 0.5 cm), Reject User input Command",ii);
                this->reject_userUIcmd = true;
                break;
              }
          }else{
              if(std::abs(this->Pose_soll_[ii] - this->Pose_actuel_.pose.orientation[ii-3]) >= 1*M_PI/180){
                ACIN_CTRL_ERROR("Allowed Pose offset from pos. [%i] to large (>= 1° ), Reject User input Command",ii);
                this->reject_userUIcmd = true;
                break;
              }
          }
      }
      sollPos.sollPos[ii] = Pose_soll_[ii];
      sollPos.coord[ii] = sollCheckbox_[ii];
    }
  }


  try{
    UIArg_sptr_->sollPos_pub.isLatched();
  }catch(ros::Exception e){
    ACIN_CTRL_WARN("<acin_ctrl_UI> can't publish soll position");
    return;
  }
  UIArg_sptr_->sollPos_pub.publish(sollPos);
}
void ACIN_ctrl_UI_C::handleCB_TaskTraj_index(int index){
  QVariant index_1;
  QVariant index_2;
  if(index ==-1){
    index_1 = 0;
    index_2 = 1;
  }else{
    index_1 = ComboBoxMap["coord_1_traj_CB"]->currentData();
    index_2 = ComboBoxMap["coord_2_traj_CB"]->currentData();
  }
  ComboBoxMap["coord_1_traj_CB"]->clear();
  ComboBoxMap["coord_2_traj_CB"]->clear();
  std::cout<<"index_1 "<<index_1.toString().toStdString()<<std::endl;
  std::cout<<"index_2 "<<index_2.toString().toStdString()<<std::endl;
  uint8_t numCoord = _ROBOT_q_DIM_;
  if (!joint_space_inc_){
    numCoord = 6;
  }
  for (uint8_t ii =0; ii < numCoord; ii++ ){
    std::string id_str ="";
    if(joint_space_inc_){
      id_str = "q"+std::to_string(ii);
    }else{
      id_str = "" + std::string(taskSpace_coord_labels[ii]);
    }

    if(ii == index_1){
      ComboBoxMap["coord_1_traj_CB"]-> addItem(tr(id_str.c_str()), QVariant(ii));
    }else{
      if(ii == index_2){
        ComboBoxMap["coord_2_traj_CB"]-> addItem(tr(id_str.c_str()), QVariant(ii));
      }else{
        ComboBoxMap["coord_1_traj_CB"]-> addItem(tr(id_str.c_str()), QVariant(ii));
        ComboBoxMap["coord_2_traj_CB"]-> addItem(tr(id_str.c_str()), QVariant(ii));
      }
    }
  }
  int idx_1 = ComboBoxMap["coord_1_traj_CB"]->findData(index_1);
  if(idx_1 !=-1){
    ComboBoxMap["coord_1_traj_CB"]->setCurrentIndex(idx_1);
  }else{
    assert(1==0);
  }
  int idx_2 = ComboBoxMap["coord_2_traj_CB"]->findData(index_2);
  if(idx_2 !=-1){
    ComboBoxMap["coord_2_traj_CB"]->setCurrentIndex(idx_2);
  }else{
    assert(1==0);
  }

}
void ACIN_ctrl_UI_C::handleComboBox_RQTPlot(int index){
  QVariant rqt_plot_variant = ComboBoxMap["rqt_plot_CB"] ->currentData();
  std::cout << "RQT_combobox  index: " <<index <<" variant"<< rqt_plot_variant.toString().toStdString() << std::endl;
}
void ACIN_ctrl_UI_C::handleButton_RQTFolder(){
  QString dir_rqt_xml = QFileDialog::getExistingDirectory(this, tr("Select rqt_multiplot config directory"),
                                             "/home",
                                             QFileDialog::ShowDirsOnly
                                             | QFileDialog::DontResolveSymlinks);
  QDir directory(dir_rqt_xml);
  QStringList xmlFiles = directory.entryList(QStringList() << "*.xml" << "*.XML",QDir::Files);
  ComboBoxMap["rqt_plot_CB"]->clear();
  ComboBoxMap["rqt_plot_CB"]->addItems(xmlFiles);
}

void ACIN_ctrl_UI_C::closeEvent (QCloseEvent *event)
{
    QMessageBox::StandardButton resBtn = QMessageBox::question( this, "x",
                                                                tr("Are you sure?\n"),
                                                                QMessageBox::Cancel | QMessageBox::No | QMessageBox::Yes,
                                                                QMessageBox::Yes);
    if (resBtn != QMessageBox::Yes) {
        event->ignore();
    } else {
        event->accept();
        UIArg_sptr_->finished_flag = 1;
        handleButton_motion_DIS();
    }

}
// QSize ACIN_ctrl_UI_C::sizeHint()
// {
//    return QSize(100, 110);
// }
