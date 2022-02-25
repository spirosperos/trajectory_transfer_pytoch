#include <stdio.h>

#include <QLineEdit>
#include <QTabWidget>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGroupBox>
#include <QWidget>
#include <QPushButton>
#include <QLabel>
#include <QSpinBox>
#include <QDoubleSpinBox>
#include <QSlider>
#include <QComboBox>
#include <QListView>
#include <QCheckBox>
#include <QFileInfo>
#include <QColorDialog>
#include <QProgressBar>
#include <QTextEdit>
#include <QStringList>
#include <QStringListModel>
#include <QFileDialog>
#include <QPainter>
#include <QFontMetrics>
#include <QSizePolicy>
#include <QRadioButton>
#include <QIcon>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/RobotState.h>
#include <moveit_msgs/GetPositionIK.h>

#include <Eigen/Geometry>

#include <cmath>

#include "object_reconstruction_panel.h"
#include <acin_reconstruction/Transform.h>
#include <acin_reconstruction/Modify.h>
#include <acin_reconstruction/Register.h>
#include <acin_reconstruction/MultiwayRegister.h>
#include <acin_reconstruction/ScanList.h>
#include <acin_reconstruction/Load.h>
#include <acin_reconstruction/Scan.h>
#include <acin_reconstruction/Export.h>
#include <acin_reconstruction/Connect.h>
#include <acin_reconstruction/BoundingBox.h>
#include <acin_reconstruction/SetReference.h>

#include "visualization_msgs/Marker.h"
#include "geometry_msgs/Transform.h"
#include "geometry_msgs/PoseStamped.h"
#include "tf2/transform_datatypes.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/transform_listener.h"
#include <std_srvs/Trigger.h>

#include <acin_automation/ExecuteScriptAction.h>
#include <acin_automation/PlanViewsAction.h>
#include <actionlib/client/simple_action_client.h>

namespace acin_gui {

LabelEllipsis::LabelEllipsis(QWidget *parent)
  : QLabel(parent)
{
    this->setSizePolicy(QSizePolicy::MinimumExpanding, QSizePolicy::MinimumExpanding);
}

void LabelEllipsis::paintEvent(QPaintEvent *event)
{
    QPainter painter(this);
    QFontMetrics metrics = this->fontMetrics();
    QString elided = metrics.elidedText(this->text(), Qt::ElideRight, this->width());
    painter.drawText(this->rect(), this->alignment(), elided);
}

QSize LabelEllipsis::minimumSizeHint() const
{
    return this->sizeHint();
}

QSize LabelEllipsis::sizeHint() const
{
    int left, top, right, bottom;
    this->getContentsMargins(&left, &top, &right, &bottom);
    int margin = 2 * this->margin();
    return QSize(left + right + margin, top + bottom + margin);
}

ObjectReconstructionPanel::ObjectReconstructionPanel(QWidget *parent)
  : rviz::Panel(parent)
{
    fileCounter_ = 0;
    
    QStringList list;
    model_ = new QStringListModel(this);
    
    // Create tab widget
    QTabWidget *tabWidget = new QTabWidget;
    QWidget *tab[5];
    std::string tabName[5] = {"Automate", "Plan", "Scan", "Register", "Reconstruct"};
    for (int i = 0; i < 5; ++i) {
        tab[i] = new QWidget();
        tabWidget->addTab(tab[i], QString::fromStdString(tabName[i]));
    }
    
    // Automate tab
    QVBoxLayout *automateLayout = new QVBoxLayout();
    
    // Create groups
    QGroupBox *automateGroup[4];
    std::string automateGroupName[4] = {"Commands", "View Planning", "Script", "Progress"};
    for (int i = 0; i < 4; ++i) {
        automateGroup[i] = new QGroupBox(QString::fromStdString(automateGroupName[i]));
        automateLayout->addWidget(automateGroup[i]);
    }
    tab[0]->setLayout(automateLayout);
    
    // Set queue size of action clients to ensure all feedback messages are received
    nh_.setParam("plan_views/actionlib_client_sub_queue_size", 50);
    nh_.setParam("execute_script/actionlib_client_sub_queue_size", 50);
    
    // Create action clients
    viewPlanningActionClient_ = new actionlib::SimpleActionClient<acin_automation::PlanViewsAction>("plan_views", false);
    scriptActionClient_ = new actionlib::SimpleActionClient<acin_automation::ExecuteScriptAction>("execute_script", false);
    
    // Commands group
    {
        // Create label for status
        statusLabel_ = new QLabel();
        
        // Create and connect buttons
        QIcon play =  QIcon::fromTheme("media-playback-start");
        startViewPlanningBtn_ = new QPushButton(play, "Scan");
        startScriptBtn_ = new QPushButton(play, "Script");
        stopBtn_ = new QPushButton("Stop");
        stopBtn_->setDisabled(true);
        
        connect(startViewPlanningBtn_, &QPushButton::clicked, [=]() {
            if (viewPlanningActionClient_->isServerConnected()) {
                acin_automation::PlanViewsGoal goal;
                goal.distance = viewPlanningDistanceSpinBox_->value();
                goal.scans = viewPlanningNumberSpinBox_->value();
                viewPlanningActionClient_->sendGoal(goal,
                                                    boost::bind(&ObjectReconstructionPanel::viewPlanningDoneCb, this, _1, _2),
                                                    boost::bind(&ObjectReconstructionPanel::activeCb, this),
                                                    boost::bind(&ObjectReconstructionPanel::viewPlanningFeedbackCb, this, _1));
                statusLabel_->setText(QString::fromStdString("Performing scan..."));
                startViewPlanningBtn_->setDisabled(true);
                startScriptBtn_->setDisabled(true);
                chooseBtn_->setDisabled(true);
                scriptLineEdit_->setDisabled(true);
                stopBtn_->setDisabled(false);
                automationMode = "viewPlanning";
            } else
                statusLabel_->setText("No connection");
        });
        
        connect(startScriptBtn_, &QPushButton::clicked, [=]() {
            QFileInfo script(scriptLineEdit_->text());
            if (!script.exists()) {
                statusLabel_->setText("File does not exist");
                return;
            }
            if (scriptActionClient_->isServerConnected()) {
                acin_automation::ExecuteScriptGoal goal;
                goal.filename = scriptLineEdit_->text().toUtf8().constData();
                scriptActionClient_->sendGoal(goal,
                                              boost::bind(&ObjectReconstructionPanel::scriptDoneCb, this, _1, _2),
                                              boost::bind(&ObjectReconstructionPanel::activeCb, this),
                                              boost::bind(&ObjectReconstructionPanel::scriptFeedbackCb, this, _1));
                statusLabel_->setText(QString::fromStdString("Executing script..."));
                startViewPlanningBtn_->setDisabled(true);
                startScriptBtn_->setDisabled(true);
                chooseBtn_->setDisabled(true);
                scriptLineEdit_->setDisabled(true);
                stopBtn_->setDisabled(false);
                automationMode = "script";
            } else
                statusLabel_->setText("No connection");
        });
        
        connect(stopBtn_, &QPushButton::clicked, [=]() {
            bool success = false;
            
            if (automationMode == "script" && scriptActionClient_->isServerConnected()) {
                scriptActionClient_->cancelGoal();
                success = true;
            }
            
            if (automationMode == "viewPlanning" && viewPlanningActionClient_->isServerConnected()) {
                viewPlanningActionClient_->cancelGoal();
                success = true;
            }
            
            if (success) {
                stopBtn_->setDisabled(true);
                statusLabel_->setText("Stopping...");
                startViewPlanningBtn_->setDisabled(false);
                startScriptBtn_->setDisabled(false);
                chooseBtn_->setDisabled(false);
                scriptLineEdit_->setDisabled(false);
            } else {
                statusLabel_->setText("No connection");
            }
        });
        
        // Create and set layout
        QHBoxLayout *hBoxLayout = new QHBoxLayout();
        hBoxLayout->addWidget(startViewPlanningBtn_);
        hBoxLayout->addWidget(startScriptBtn_);
        hBoxLayout->addWidget(stopBtn_);
        hBoxLayout->addWidget(statusLabel_);
        hBoxLayout->addStretch();
        automateGroup[0]->setLayout(hBoxLayout);
    }
    
    // View planning group
    {
        // Create and connect spin boxes
        viewPlanningDistanceSpinBox_ = new QDoubleSpinBox();
        viewPlanningDistanceSpinBox_->setSingleStep(0.1);
        viewPlanningNumberSpinBox_ = new QSpinBox();
        viewPlanningNumberSpinBox_->setMaximum(999);
        viewPlanningNumberSpinBox_->setMinimum(1);
        
        connect(viewPlanningDistanceSpinBox_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), [=]() {
            Q_EMIT configChanged();
        });
        
        connect(viewPlanningNumberSpinBox_, QOverload<int>::of(&QSpinBox::valueChanged), [=]() {
            Q_EMIT configChanged();
        });
        
        // Create and set layout
        QHBoxLayout *hBoxLayout = new QHBoxLayout();
        hBoxLayout->addWidget(new QLabel("Distance (m):"));
        hBoxLayout->addWidget(viewPlanningDistanceSpinBox_);
        hBoxLayout->addWidget(new QLabel("Scans:"));
        hBoxLayout->addWidget(viewPlanningNumberSpinBox_);
        hBoxLayout->addStretch();
        automateGroup[1]->setLayout(hBoxLayout);
    }
    
    // Script group
    {
        // Create and connect line edit
        scriptLineEdit_ = new QLineEdit();
        connect(scriptLineEdit_, &QLineEdit::textChanged, [=](){Q_EMIT configChanged();});
        
        // Create and connect button
        chooseBtn_ = new QPushButton("Choose");
        connect(chooseBtn_, &QPushButton::clicked, [=]() {
            QString dir = scriptLineEdit_->text();
            QString script = QFileDialog::getOpenFileName(this, "Select a script", dir, "XML Files (*.xml *.xacro)");
            if (!script.isNull())
                scriptLineEdit_->setText(script);
        });
        
        // Create and set layout
        QHBoxLayout *hBoxLayout = new QHBoxLayout();
        hBoxLayout->addWidget(scriptLineEdit_);
        hBoxLayout->addWidget(chooseBtn_);
        automateGroup[2]->setLayout(hBoxLayout);
    }
    
    // Progress group
    {
        // Create progress bar
        progressBar_ = new QProgressBar();
        
        // Create text edit
        statusTextEdit_ = new QTextEdit();
        statusTextEdit_->setReadOnly(true);
        
        // Create and set layout
        QVBoxLayout *vBoxLayout = new QVBoxLayout();
        vBoxLayout->addWidget(progressBar_);
        vBoxLayout->addWidget(statusTextEdit_);
        automateGroup[3]->setLayout(vBoxLayout);
    }
    
    // Plan tab
    QGridLayout *planLayout = new QGridLayout();
    
    // Create groups
    QGroupBox *planGroup[5];
    std::string planGroupName[5] = {"Reference", "Bounding Box", "Sphere", "Pose on Sphere", "Plan cartesian path"};
    for (int i = 0; i < 5; ++i)
        planGroup[i] = new QGroupBox(QString::fromStdString(planGroupName[i]));
    planLayout->addWidget(planGroup[0], 0, 0);
    planLayout->addWidget(planGroup[1], 0, 1);
    planLayout->addWidget(planGroup[2], 1, 0, 1, 2);
    planLayout->addWidget(planGroup[3], 2, 0, 1, 2);
    planLayout->addWidget(planGroup[4], 3, 0, 1, 2);


    tab[1]->setLayout(planLayout);
    
    // Reference group
    {
        // Create label
        updateLabel_ = new QLabel("");
        updateLabel_->setEnabled(false);
        
        // Create and connect (radio) buttons
        worldRadioBtn_ = new QRadioButton("World");
        worldRadioBtn_->setChecked(true);
        markerRadioBtn_ = new QRadioButton("Marker");
        QPushButton *updateBtn = new QPushButton("Update");
        updateBtn->setEnabled(false);
        
        connect(worldRadioBtn_, &QRadioButton::clicked, [=]() {
            updateBtn->setEnabled(false);
            updateLabel_->setEnabled(false);
        });
        
        connect(markerRadioBtn_, &QRadioButton::clicked, [=]() {
            updateBtn->setEnabled(true);
            updateLabel_->setEnabled(true);
        });
        
        connect(updateBtn, &QPushButton::clicked, [=]() {
            // Spawn a new thread to not block the GUI
            // (and for MoveIt to be able to receive the current robot state)
            updateLabel_->setText("");
            boost::thread t(boost::bind(&ObjectReconstructionPanel::updateReference, this));
        });
        
        // Create and set layout
        QVBoxLayout *vBoxLayout = new QVBoxLayout();
        vBoxLayout->addWidget(worldRadioBtn_);
        vBoxLayout->addWidget(markerRadioBtn_);
        vBoxLayout->addWidget(updateBtn);
        vBoxLayout->addWidget(updateLabel_);
        vBoxLayout->addStretch();
        planGroup[0]->setLayout(vBoxLayout);
    }
    
    // Bounding box group
    {
        // Create and connect buttons
        QPushButton *addBtn = new QPushButton("Add");
        connect(addBtn, &QPushButton::clicked, [=]() {
            // Read bounding box
            double lx = lowerBoundSpinBox_[0]->value();
            double ly = lowerBoundSpinBox_[1]->value();
            double lz = lowerBoundSpinBox_[2]->value();
            double ux = upperBoundSpinBox_[0]->value();
            double uy = upperBoundSpinBox_[1]->value();
            double uz = upperBoundSpinBox_[2]->value();
            
            // Create box
            shape_msgs::SolidPrimitive primitive;
            primitive.type = primitive.BOX;
            primitive.dimensions.resize(3);
            primitive.dimensions[0] = ux-lx;
            primitive.dimensions[1] = uy-ly;
            primitive.dimensions[2] = uz-lz;
            
            geometry_msgs::Pose boxPose;
            boxPose.orientation.w = 1;
            boxPose.position.x = (lx+ux) / 2;
            boxPose.position.y = (ly+uy) / 2;
            boxPose.position.z = (lz+uz) / 2;
            
            // Send bounding box to reconstruction node
            acin_reconstruction::BoundingBox srv;
            srv.request.operation = srv.request.ADD;
            srv.request.center.x = boxPose.position.x;
            srv.request.center.y = boxPose.position.y;
            srv.request.center.z = boxPose.position.z;
            srv.request.extent.x = primitive.dimensions[0];
            srv.request.extent.y = primitive.dimensions[1];
            srv.request.extent.z = primitive.dimensions[2];
            boundingBoxClient_.call(srv);
            
            // Transform bounding box to world frame
            tf2_ros::Buffer tfBuffer;
            tf2_ros::TransformListener tfListener(tfBuffer);
            geometry_msgs::TransformStamped markerTransform;
            markerTransform = tfBuffer.lookupTransform("world", "marker", ros::Time(0), ros::Duration(1));
            tf2::Transform transformTf1;
            tf2::Transform transformTf2;
            tf2::Transform transformTf3;
            tf2::fromMsg(markerTransform.transform, transformTf1);
            tf2::fromMsg(boxPose, transformTf2);
            transformTf3 = transformTf1 * transformTf2;
            geometry_msgs::Transform boxTransform;
            tf2::convert(transformTf3, boxTransform);
            boxPose.position.x = boxTransform.translation.x;
            boxPose.position.y = boxTransform.translation.y;
            boxPose.position.z = boxTransform.translation.z;
            boxPose.orientation = boxTransform.rotation;
            
            // Create collision object
            moveit_msgs::CollisionObject collisionObject;
            collisionObject.header.frame_id = "world";
            collisionObject.id = "bounding_box";
            collisionObject.primitives.push_back(primitive);
            collisionObject.primitive_poses.push_back(boxPose);
            collisionObject.operation = collisionObject.ADD;
            
            // Send collision object to MoveIt
            std::vector<moveit_msgs::CollisionObject> collisionObjects;
            collisionObjects.push_back(collisionObject);
            moveit::planning_interface::PlanningSceneInterface planningSceneInterface;
            planningSceneInterface.addCollisionObjects(collisionObjects);
        });
        
        QPushButton *removeBtn = new QPushButton("Remove");
        connect(removeBtn, &QPushButton::clicked, [=]() {
            // Create collision object
            moveit_msgs::CollisionObject collisionObject;
            collisionObject.id = "bounding_box";
            collisionObject.operation = collisionObject.REMOVE;
            
            // Remove collision object in MoveIt
            std::vector<moveit_msgs::CollisionObject> collisionObjects;
            collisionObjects.push_back(collisionObject);
            moveit::planning_interface::PlanningSceneInterface planningSceneInterface;
            planningSceneInterface.addCollisionObjects(collisionObjects);
            
            // Remove bounding box in reconstruction node
            acin_reconstruction::BoundingBox srv;
            srv.request.operation = srv.request.REMOVE;
            boundingBoxClient_.call(srv);
        });
        
        // Create layout
        QGridLayout *gridLayout = new QGridLayout();
        
        // Create and connect spin boxes
        for (int i = 0; i < 3; ++i) {
            // Lower bounds
            lowerBoundSpinBox_[i] = new QDoubleSpinBox();
            lowerBoundSpinBox_[i]->setRange(-3, 3);
            lowerBoundSpinBox_[i]->setDecimals(4);
            lowerBoundSpinBox_[i]->setSingleStep(0.01);
            connect(lowerBoundSpinBox_[i], QOverload<double>::of(&QDoubleSpinBox::valueChanged), [=](double value) {
                upperBoundSpinBox_[i]->setMinimum(value);
                Q_EMIT configChanged();
            });
            gridLayout->addWidget(lowerBoundSpinBox_[i], 1, i);
            
            // Upper bounds
            upperBoundSpinBox_[i] = new QDoubleSpinBox();
            upperBoundSpinBox_[i]->setRange(-3, 3);
            upperBoundSpinBox_[i]->setDecimals(4);
            upperBoundSpinBox_[i]->setSingleStep(0.01);
            connect(upperBoundSpinBox_[i], QOverload<double>::of(&QDoubleSpinBox::valueChanged), [=](double value) {
                lowerBoundSpinBox_[i]->setMaximum(value);
                Q_EMIT configChanged();
            });
            gridLayout->addWidget(upperBoundSpinBox_[i], 3, i);
        }
        
        // Set layout
        gridLayout->addWidget(new QLabel("Lower Bound:"), 0, 0, 1, 0);
        gridLayout->addWidget(new QLabel("Upper Bound:"), 2, 0, 1, 0);
        QHBoxLayout *hBoxLayout = new QHBoxLayout();
        hBoxLayout->addWidget(addBtn);
        hBoxLayout->addWidget(removeBtn);
        hBoxLayout->addStretch();
        QVBoxLayout *vBoxLayout = new QVBoxLayout();
        vBoxLayout->addLayout(gridLayout);
        vBoxLayout->addLayout(hBoxLayout);
        vBoxLayout->addStretch();
        planGroup[1]->setLayout(vBoxLayout);
        
    }
    
    // Sphere group
    {
        // Create layout
        QHBoxLayout *hBoxLayout = new QHBoxLayout();
        hBoxLayout->addWidget(new QLabel("xyz (m):"));
        
        // Create and connect spin boxes
        for (int i = 0; i < 4; ++i) {
            sphereSpinBox_[i] = new QDoubleSpinBox();
            sphereSpinBox_[i]->setRange(-3, 3);
            sphereSpinBox_[i]->setDecimals(3);
            sphereSpinBox_[i]->setSingleStep(0.02);
            connect(sphereSpinBox_[i], QOverload<double>::of(&QDoubleSpinBox::valueChanged), [=]() {
                if (sphereSpinBox_[i]->isVisible()) {
                    geometry_msgs::PoseStamped pose;
                    getPoseTarget(pose);
                    bool solution = setGoal(pose);
                    updateMarker(pose, solution);
                }
                Q_EMIT configChanged();
            });
            if (i == 3)
                hBoxLayout->addWidget(new QLabel("r (m):"));
            hBoxLayout->addWidget(sphereSpinBox_[i]);
        }
        hBoxLayout->addStretch();
        
        // Set layout
        planGroup[2]->setLayout(hBoxLayout);
    }
    
    // Pose group
    {
        // Create and set layout
        QGridLayout *gridLayout = new QGridLayout();
        gridLayout->addWidget(new QLabel("Azimuth Angle:"), 0, 0, 1, 2);
        gridLayout->addWidget(new QLabel("Polar Angle:"), 2, 0, 1, 2);
        gridLayout->addWidget(new QLabel("Roll of Camera:"), 4, 0, 1, 2);
        planGroup[3]->setLayout(gridLayout);
        
        // Create sliders and spin boxes
        QSlider *slider[3];
        for (int i = 0; i < 3; ++i) {
            // Create slider
            slider[i] = new QSlider(Qt::Horizontal);
            slider[i]->setRange(-180, 180);
            gridLayout->addWidget(slider[i], 2*i+1, 0);
            
            // Create spin box
            poseSpinBox_[i] = new QDoubleSpinBox();
            poseSpinBox_[i]->setRange(-180, 180);
            poseSpinBox_[i]->setDecimals(1);
            poseSpinBox_[i]->setSuffix("°");
            gridLayout->addWidget(poseSpinBox_[i], 2*i+1, 1);
        }
        
        slider[1]->setRange(0, 180);
        slider[1]->setValue(45);
        poseSpinBox_[1]->setRange(0, 180);
        poseSpinBox_[1]->setValue(45);
        
        // Connect sliders and spin boxes
        for (int i = 0; i < 3; ++i) {
            // Connect slider to spin box
            connect(slider[i], &QSlider::valueChanged, [=](int value) {
                poseSpinBox_[i]->setValue(static_cast<double>(value));
            });
            
            // Connect spin box to slider and updater method
            connect(poseSpinBox_[i], QOverload<double>::of(&QDoubleSpinBox::valueChanged), [=](double value) {
                slider[i]->setValue(static_cast<int>(value));
                if (slider[i]->isVisible()) {
                    geometry_msgs::PoseStamped pose;
                    getPoseTarget(pose);
                    bool solution = setGoal(pose);
                    updateMarker(pose, solution);
                }
            });
        }




    }
    // Cartesian path group
    {
        QPushButton *cartBtn = new QPushButton("Plan a cartesian path");

        connect(cartBtn, &QPushButton::clicked, [=]() {
    /*
            std::vector<geometry_msgs::Pose> waypoints;
    robot_state::RobotState start_state(*move_group.getCurrentState());
    geometry_msgs::Pose start_pose;
    start_pose.orientation.y = sqrt(2) / 2;
    start_pose.orientation.w = sqrt(2) / 2;
    start_pose.position.x = 0.3;
    start_pose.position.y = 0.15;
    start_pose.position.z = 0.3;
    waypoints.push_back(start_pose);

    geometry_msgs::Pose target_pose = start_pose;

    target_pose.position.y -= 0.15;
    target_pose.position.z -= 0.15;
    waypoints.push_back(target_pose); // down right

    target_pose.position.y -= 0.15;
    target_pose.position.z += 0.15;
    waypoints.push_back(target_pose); // up right

    target_pose.orientation.y = 0;
    target_pose.orientation.w = 1;
    target_pose.position.y += 0.15;
    target_pose.position.z += 0.15;
    waypoints.push_back(target_pose); // up left

    target_pose.position.y += 0.15;
    target_pose.position.z -= 0.15;
    waypoints.push_back(target_pose); // down left

    //planning
    moveit_msgs::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.01;
    double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    ROS_INFO_NAMED("move_cartesian", "Visualizing plan through waypoints (%.2f%% acheived)", fraction * 100.0);

    // Visualize the plan in RViz
    visual_tools.deleteAllMarkers();
    visual_tools.publishText(text_pose, "Waypoints", rvt::WHITE, rvt::XLARGE);
    visual_tools.publishPath(waypoints, rvt::MAGENTA, rvt::SMALL);
    for (std::size_t i = 0; i < waypoints.size(); ++i)
        visual_tools.publishAxisLabeled(waypoints[i], "pt" + std::to_string(i), rvt::MEDIUM);
    visual_tools.trigger();
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue with path execution");

    //path execution
    move_group.execute(trajectory);
    visual_tools.prompt("Press 'next' to exit the program");

    */



















        });

        // Create and set layout
        QVBoxLayout *vBoxLayout = new QVBoxLayout();
        vBoxLayout->addWidget(cartBtn);
        planGroup[4]->setLayout(vBoxLayout);

    }



    
    // Scan tab
    QGridLayout *scanLayout = new QGridLayout();
    
    // Create groups
    QGroupBox *scanGroup[5];
    std::string scanGroupName[5] = {"Scans", "Current Scan", "PhoXi Control", "Import", "Options"};
    for (int i = 0; i < 5; ++i)
        scanGroup[i] = new QGroupBox(QString::fromStdString(scanGroupName[i]));
    scanLayout->addWidget(scanGroup[0], 0, 0, 2, 1);
    scanLayout->addWidget(scanGroup[1], 0, 1);
    scanLayout->addWidget(scanGroup[2], 1, 1);
    scanLayout->addWidget(scanGroup[3], 2, 0, 1, 2);
    scanLayout->addWidget(scanGroup[4], 3, 0, 1, 2);
    tab[2]->setLayout(scanLayout);
    
    // Scans group
    {
        // Create label for status messages
        scanLabel_ = new LabelEllipsis();
        
        // Create and connect button
        QPushButton *scanBtn = new QPushButton("Scan");
        connect(scanBtn, &QPushButton::clicked, [=]() {
            // Spawn a new thread to not block the GUI
            // (and for MoveIt to be able to receive the current robot state)
            boost::thread t(boost::bind(&ObjectReconstructionPanel::scan, this));
        });
        
        scanListView_ = new QListView();
        scanListView_->setModel(model_);
        
        // Create and set layout
        QHBoxLayout *hBoxLayout = new QHBoxLayout();
        hBoxLayout->addWidget(scanBtn);
        hBoxLayout->addWidget(scanLabel_);
        QVBoxLayout *vBoxLayout = new QVBoxLayout();
        vBoxLayout->addWidget(scanListView_);
        vBoxLayout->addLayout(hBoxLayout);
        scanGroup[0]->setLayout(vBoxLayout);
    }
    
    // Current scan group
    {
        // Create and connect buttons
        QPushButton *showBtn = new QPushButton("Show");
        QPushButton *hideBtn = new QPushButton("Hide");
        QPushButton *colorBtn = new QPushButton("Color");
        QPushButton *resetBtn = new QPushButton("Reset");
        QPushButton *deleteBtn = new QPushButton("Delete");
        connect(showBtn, &QPushButton::clicked, [=]() {
            acin_reconstruction::Modify srv;
            srv.request.operation = srv.request.SHOW;
            QModelIndex index = scanListView_->currentIndex();
            srv.request.fragment = index.data().toString().toUtf8().constData();
            srv.request.custom_color = false;
            modifyClient_.call(srv);
        });
        connect(hideBtn, &QPushButton::clicked, [=]() {
            acin_reconstruction::Modify srv;
            srv.request.operation = srv.request.HIDE;
            QModelIndex index = scanListView_->currentIndex();
            srv.request.fragment = index.data().toString().toUtf8().constData();
            modifyClient_.call(srv);
        });
        connect(colorBtn, &QPushButton::clicked, [=]() {
            QColor color = QColorDialog::getColor(Qt::green, this, "Select Color");
            if (color.isValid()) {
                acin_reconstruction::Modify srv;
                srv.request.operation = srv.request.COLOR;
                int r, g, b;
                color.getRgb(&r, &g, &b);
                srv.request.color.r = r / 255.0;
                srv.request.color.g = g / 255.0;
                srv.request.color.b = b / 255.0;
                srv.request.color.a = 1;
                QModelIndex index = scanListView_->currentIndex();
                srv.request.fragment = index.data().toString().toUtf8().constData();
                srv.request.custom_color = true;
                modifyClient_.call(srv);
            }
        });
        connect(resetBtn, &QPushButton::clicked, [=]() {
            acin_reconstruction::Modify srv;
            srv.request.operation = srv.request.COLOR;
            QModelIndex index = scanListView_->currentIndex();
            srv.request.fragment = index.data().toString().toUtf8().constData();
            srv.request.custom_color = false;
            modifyClient_.call(srv);
        });
        connect(deleteBtn, &QPushButton::clicked, [=]() {
            acin_reconstruction::Modify srv;
            srv.request.operation = srv.request.DELETE;
            QModelIndex index = scanListView_->currentIndex();
            srv.request.fragment = index.data().toString().toUtf8().constData();
            modifyClient_.call(srv);
        });
        
        // Create and set layout
        QGridLayout *gridLayout = new QGridLayout();
        gridLayout->addWidget(showBtn, 0, 0);
        gridLayout->addWidget(hideBtn, 0, 1);
        gridLayout->addWidget(colorBtn, 1, 0);
        gridLayout->addWidget(resetBtn, 1, 1);
        gridLayout->addWidget(deleteBtn, 2, 0);
        QVBoxLayout *vBoxLayout = new QVBoxLayout();
        vBoxLayout->addLayout(gridLayout);
        vBoxLayout->addStretch();
        scanGroup[1]->setLayout(vBoxLayout);
    }
    
    // PhoXi group
    {
        QLabel *connectionLabel = new LabelEllipsis();
        
        // Create and connect buttons
        QPushButton *connectBtn = new QPushButton("Connect");
        QPushButton *disconnectBtn = new QPushButton("Disconnect");
        disconnectBtn->setEnabled(false);
        
        connect(connectBtn, &QPushButton::clicked, [=]() {
            acin_reconstruction::Connect srv;
            srv.request.operation = srv.request.CONNECT;
            if (connectClient_.call(srv)) {
                if (srv.response.success) {
                    connectBtn->setEnabled(false);
                    disconnectBtn->setEnabled(true);
                }
                connectionLabel->setText(QString::fromStdString(srv.response.message));
            } else {
                connectionLabel->setText("Service not available");
            }
        });
        
        connect(disconnectBtn, &QPushButton::clicked, [=]() {
            acin_reconstruction::Connect srv;
            srv.request.operation = srv.request.DISCONNECT;
            if (connectClient_.call(srv)) {
                if (srv.response.success) {
                    disconnectBtn->setEnabled(false);
                    connectBtn->setEnabled(true);
                }
                connectionLabel->setText(QString::fromStdString(srv.response.message));
            } else {
                connectionLabel->setText("Service not available");
            }
        });
        
        // Create and set layout
        QVBoxLayout *vBoxLayout = new QVBoxLayout();
        vBoxLayout->addWidget(connectBtn);
        vBoxLayout->addWidget(disconnectBtn);
        vBoxLayout->addWidget(connectionLabel);
        vBoxLayout->addStretch();
        scanGroup[2]->setLayout(vBoxLayout);
    }
    
    // Import group
    {
        // Create and connect buttons
        QPushButton *importRgbdBtn = new QPushButton("RGBD Images");
        connect(importRgbdBtn, &QPushButton::clicked, [=]() {
            QString dir;
            
            // Select color images
            dir = pathLineEdit_->text();
            QStringList colorImages = QFileDialog::getOpenFileNames(this, "Select one or more color images", dir, "Image Files (*.png)");
            
            if (!colorImages.isEmpty()) {
                // Select depth images
                QStringList depthImages = QFileDialog::getOpenFileNames(this, "Select one or more depth images", dir, "Image Files (*.png)");
                
                this->importRGBD(colorImages, depthImages);
            }
        });
        QPushButton *importPointCloudBtn = new QPushButton("Point Clouds");
        connect(importPointCloudBtn, &QPushButton::clicked, [=]() {
            QStringList files = QFileDialog::getOpenFileNames(this, "Select one or more point clouds", "/home", "Point Cloud (*.ply)");
            this->importPointCloud(files);
        });
        
        // Create and set layout
        QHBoxLayout *hBoxLayout = new QHBoxLayout();
        hBoxLayout->addWidget(importRgbdBtn);
        hBoxLayout->addWidget(importPointCloudBtn);
        hBoxLayout->addStretch();
        scanGroup[3]->setLayout(hBoxLayout);
    }
    
    // Options group
    {
        // Create and connect check boxes
        saveColorCheckBox_ = new QCheckBox("Texture");
        saveDepthCheckBox_ = new QCheckBox("Depth");
        savePoseCheckBox_ = new QCheckBox("Pose");
        connect(saveColorCheckBox_, &QCheckBox::stateChanged, [=]() {
            Q_EMIT configChanged();
        });
        connect(saveDepthCheckBox_, &QCheckBox::stateChanged, [=]() {
            Q_EMIT configChanged();
        });
        connect(savePoseCheckBox_, &QCheckBox::stateChanged, [=]() {
            Q_EMIT configChanged();
        });
        
        // Create layout
        QHBoxLayout *hBoxLayout1 = new QHBoxLayout();
        hBoxLayout1->addWidget(new QLabel("Save: "));
        hBoxLayout1->addWidget(saveColorCheckBox_);
        hBoxLayout1->addWidget(saveDepthCheckBox_);
        hBoxLayout1->addWidget(savePoseCheckBox_);
        hBoxLayout1->addStretch();
        
        // Create and connect line edit
        pathLineEdit_ = new QLineEdit();
        connect(pathLineEdit_, &QLineEdit::textChanged, [=]() {
            Q_EMIT configChanged();
        });
        
        // Create and connect button
        QPushButton *chooseBtn = new QPushButton("Choose");
        connect(chooseBtn, &QPushButton::clicked, [=]() {
            QString oldPath = pathLineEdit_->text();
            QString newPath = QFileDialog::getExistingDirectory(this, QString::fromStdString("Select directory"), oldPath);
            if (!newPath.isNull())
                pathLineEdit_->setText(newPath);
        });
        
        // Create layout
        QHBoxLayout *hBoxLayout2 = new QHBoxLayout();
        hBoxLayout2->addWidget(pathLineEdit_);
        hBoxLayout2->addWidget(chooseBtn);
        
        // Set layout
        QVBoxLayout *vBoxLayout = new QVBoxLayout();
        vBoxLayout->addLayout(hBoxLayout1);
        vBoxLayout->addLayout(hBoxLayout2);
        scanGroup[4]->setLayout(vBoxLayout);
    }
    
    // Register tab
    QGridLayout *registerLayout = new QGridLayout();
    
    QGroupBox *registerGroup[3];
    std::string registerGroupName[3] = {"Scans", "Local Registration", "Transformation"};
    for (int i = 0; i < 3; ++i)
        registerGroup[i] = new QGroupBox(QString::fromStdString(registerGroupName[i]));
    tab[3]->setLayout(registerLayout);
    registerLayout->addWidget(registerGroup[0], 0, 0);
    registerLayout->addWidget(registerGroup[1], 1, 0);
    registerLayout->addWidget(registerGroup[2], 2, 0);
    
    // Fragments group
    {
        registerListView_ = new QListView();
        registerListView_->setModel(model_);
        QVBoxLayout *vBoxLayout = new QVBoxLayout();
        vBoxLayout->addWidget(registerListView_);
        registerGroup[0]->setLayout(vBoxLayout);
    }
    
    // Registration group
    {
        // Create line edit
        targetLineEdit_ = new QLineEdit();
        
        // Create and connect combo box
        methodComboBox_ = new QComboBox();
        methodComboBox_->addItem("Point-to-point ICP");
        methodComboBox_->addItem("Point-to-plane ICP");
        connect(methodComboBox_, QOverload<int>::of(&QComboBox::currentIndexChanged), [=]() {
            Q_EMIT configChanged();
        });
        
        // Create and connect spin box
        thresholdSpinBox_ = new QDoubleSpinBox();
        thresholdSpinBox_->setRange(0, 99);
        thresholdSpinBox_->setDecimals(1);
        connect(thresholdSpinBox_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), [=]() {
            Q_EMIT configChanged();
        });
        
        // Create and connect buttons
        QPushButton *getTransformationBtn = new QPushButton("Get Transformation");
        connect(getTransformationBtn, &QPushButton::clicked, [=]() {
            this->callLocalRegistration();
        });
        QPushButton *multiwayRegistrationBtn = new QPushButton("Multiway Registration");
        connect(multiwayRegistrationBtn, &QPushButton::clicked, [=]() {
            acin_reconstruction::MultiwayRegister srv;
            int idx = methodComboBox_->currentIndex();
            if (idx == 0)
                srv.request.method = srv.request.POINT_TO_POINT_ICP;
            else if (idx == 1)
                srv.request.method = srv.request.POINT_TO_PLANE_ICP;
            
            srv.request.threshold = thresholdSpinBox_->value()/1000;
            multiwayRegistrationClient_.call(srv);
        });
        
        // Create and set layout
        QGridLayout *gridLayout = new QGridLayout();
        gridLayout->setColumnStretch(1, 1);
        gridLayout->addWidget(new QLabel("Target:"), 0, 0);
        gridLayout->addWidget(targetLineEdit_, 0, 1);
        gridLayout->addWidget(new QLabel("Threshold (mm):"), 0, 2);
        gridLayout->addWidget(thresholdSpinBox_, 0, 3);
        gridLayout->addWidget(new QLabel("Method:"), 1, 0);
        gridLayout->addWidget(methodComboBox_, 1, 1, 1, 3);
        
        QHBoxLayout *hBoxLayout = new QHBoxLayout();
        hBoxLayout->addWidget(getTransformationBtn);
        hBoxLayout->addWidget(multiwayRegistrationBtn);
        hBoxLayout->addStretch();
        
        QVBoxLayout *vBoxLayout = new QVBoxLayout();
        vBoxLayout->addLayout(gridLayout);
        vBoxLayout->addLayout(hBoxLayout);
        registerGroup[1]->setLayout(vBoxLayout);
    }
    
    // Transformation group
    {
        // Create layout
        QGridLayout *gridLayout = new QGridLayout();
        
        // Create spin boxes
        for (int i = 0; i < 3; ++i) {
            translationSpinBox_[i] = new QDoubleSpinBox();
            translationSpinBox_[i]->setDecimals(2);
            translationSpinBox_[i]->setRange(-99, 99);
            gridLayout->addWidget(translationSpinBox_[i], 0, i+1);
            
            rotationSpinBox_[i] = new QDoubleSpinBox();
            rotationSpinBox_[i]->setRange(-360, 360);
            rotationSpinBox_[i]->setDecimals(3);
            gridLayout->addWidget(rotationSpinBox_[i], 1, i+1);
            gridLayout->setColumnStretch(i+1, 1);
        }
        
        // Create and connect buttons
        QPushButton *forwardBtn = new QPushButton("Forward Transform");
        QPushButton *inverseBtn = new QPushButton("Inverse Transform");
        connect(forwardBtn, &QPushButton::clicked, [=](){this->callTransform(true);});
        connect(inverseBtn, &QPushButton::clicked, [=](){this->callTransform(false);});
        
        // Set layout
        gridLayout->addWidget(new QLabel("Translation (mm):"), 0, 0);
        gridLayout->addWidget(new QLabel("Rotation (deg):"), 1, 0);
        
        QHBoxLayout *hBoxLayout = new QHBoxLayout();
        hBoxLayout->addWidget(forwardBtn);
        hBoxLayout->addWidget(inverseBtn);
        hBoxLayout->addStretch();
        QVBoxLayout *TransformationLayout = new QVBoxLayout();
        TransformationLayout->addLayout(gridLayout);
        TransformationLayout->addLayout(hBoxLayout);
        registerGroup[2]->setLayout(TransformationLayout);
    }
    
    // Reconstruct tab
    QGridLayout *reconstructLayout = new QGridLayout();
    
    QGroupBox *reconstructGroup[4];
    std::string reconstructGroupName[4] = {"Scans", "Add", "Downsample", "Export"};
    for (int i = 0; i < 4; ++i)
        reconstructGroup[i] = new QGroupBox(QString::fromStdString(reconstructGroupName[i]));
    tab[4]->setLayout(reconstructLayout);
    reconstructLayout->addWidget(reconstructGroup[0], 0, 0);
    reconstructLayout->addWidget(reconstructGroup[1], 1, 0);
    reconstructLayout->addWidget(reconstructGroup[2], 2, 0);
    reconstructLayout->addWidget(reconstructGroup[3], 3, 0);
    
    // Scans group
    {
        reconstructListView_ = new QListView();
        reconstructListView_->setModel(model_);
        
        // Create and set layout
        QVBoxLayout *vBoxLayout = new QVBoxLayout();
        reconstructGroup[0]->setLayout(vBoxLayout);
        vBoxLayout->addWidget(reconstructListView_);
    }
    
    // Adding group
    {
        // Create line edit
        QLineEdit *targetLineEdit = new QLineEdit();
        
        // Create and connect button
        QPushButton *addBtn = new QPushButton("Add");
        connect(addBtn, &QPushButton::clicked, [=]() {
            acin_reconstruction::Modify srv;
            QModelIndex index = reconstructListView_->currentIndex();
            srv.request.fragment = index.data().toString().toUtf8().constData();
            srv.request.target = targetLineEdit->text().toUtf8().constData();
            srv.request.operation = srv.request.ADD;
            modifyClient_.call(srv);
        });
        
        // Create and set layout
        QHBoxLayout *hBoxLayout = new QHBoxLayout();
        hBoxLayout->addWidget(new QLabel("Target:"));
        hBoxLayout->addWidget(targetLineEdit);
        hBoxLayout->addWidget(addBtn);
        hBoxLayout->addStretch();
        reconstructGroup[1]->setLayout(hBoxLayout);
    }
    
    // Downsampling group
    {
        // Create and connect spin box
        downsampleSpinBox_ = new QDoubleSpinBox();
        downsampleSpinBox_->setDecimals(4);
        downsampleSpinBox_->setRange(0.0001, 0.1);
        downsampleSpinBox_->setSingleStep(0.0001);
        downsampleSpinBox_->setValue(1);
        connect(downsampleSpinBox_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), [=]() {
            Q_EMIT configChanged();
        });
        
        // Create and connect button
        QPushButton *downsampleBtn = new QPushButton("Downsample");
        connect(downsampleBtn, &QPushButton::clicked, [=]() {
            acin_reconstruction::Modify srv;
            QModelIndex index = reconstructListView_->currentIndex();
            srv.request.fragment = index.data().toString().toUtf8().constData();
            srv.request.operation = srv.request.DOWNSAMPLE;
            srv.request.voxel_size = downsampleSpinBox_->value();
            modifyClient_.call(srv);
        });
        
        // Create and set layout
        QHBoxLayout *hBoxLayout = new QHBoxLayout();
        hBoxLayout->addWidget(new QLabel("Voxel size:"));
        hBoxLayout->addWidget(downsampleSpinBox_);
        hBoxLayout->addWidget(downsampleBtn);
        hBoxLayout->addStretch();
        reconstructGroup[2]->setLayout(hBoxLayout);
    }
    
    // Export group
    {
        exportLabel_ = new QLabel("");
        
        // Create and connect buttons
        QPushButton *exportMeshBtn = new QPushButton("Export Mesh");
        connect(exportMeshBtn, &QPushButton::clicked, [=]() {
            exportLabel_->setText("");
            QModelIndex index = reconstructListView_->currentIndex();
            QString name = index.data().toString();
            QString initFileName = QString("/home/")+name+QString(".ply");
            QString fileName = QFileDialog::getSaveFileName(this, "Save File", initFileName, "Mesh (*.ply)");
            if (fileName.isNull())
                return;
            acin_reconstruction::Export srv;
            srv.request.path = fileName.toUtf8().constData();
            srv.request.fragment = index.data().toString().toUtf8().constData();
            srv.request.type = srv.request.MESH;
            if (exportClient_.call(srv) && srv.response.success)
                exportLabel_->setText("Successful");
            else
                exportLabel_->setText("Failed");
        });
        QPushButton *exportPointCloudBtn = new QPushButton("Export Point Cloud");
        connect(exportPointCloudBtn, &QPushButton::clicked, [=]() {
            exportLabel_->setText("");
            QModelIndex index = reconstructListView_->currentIndex();
            QString name = index.data().toString();
            QString initFileName = QString("/home/")+name+QString(".ply");
            QString fileName = QFileDialog::getSaveFileName(this, "Save File", initFileName, "Point Cloud (*.ply)");
            if (fileName.isNull())
                return;
            acin_reconstruction::Export srv;
            srv.request.path = fileName.toUtf8().constData();
            srv.request.fragment = index.data().toString().toUtf8().constData();
            srv.request.type = srv.request.POINT_CLOUD;
            if (exportClient_.call(srv) && srv.response.success)
                exportLabel_->setText("Successful");
            else
                exportLabel_->setText("Failed");
        });
        
        // Create and set layout
        QHBoxLayout *hBoxLayout = new QHBoxLayout();
        hBoxLayout->addWidget(exportMeshBtn);
        hBoxLayout->addWidget(exportPointCloudBtn);
        hBoxLayout->addWidget(exportLabel_);
        hBoxLayout->addStretch();
        reconstructGroup[3]->setLayout(hBoxLayout);
    }
    
    // Connect tab widget
    connect(tabWidget, &QTabWidget::currentChanged, [=](int index) {
        if (index == 1) {
            geometry_msgs::PoseStamped pose;
            getPoseTarget(pose);
            bool solution = setGoal(pose);
            updateMarker(pose, solution);
        } else {
            // Create marker message
            visualization_msgs::Marker marker;
            marker.header.stamp = ros::Time::now();
            marker.header.frame_id = "world";
            marker.action = marker.DELETE;
            marker.ns = "marker";
            marker.id = 0;
            
            // Publish marker message
            markerPub_.publish(marker);
        }
    });
    
    // Layout
    QVBoxLayout *layout = new QVBoxLayout;
    layout->addWidget(tabWidget);
    setLayout(layout);
    
    // Service clients
    transformClient_ = nh_.serviceClient<acin_reconstruction::Transform>("transform");
    localRegistrationClient_ = nh_.serviceClient<acin_reconstruction::Register>("register");
    multiwayRegistrationClient_ = nh_.serviceClient<acin_reconstruction::MultiwayRegister>("multiway_register");
    loadClient_ = nh_.serviceClient<acin_reconstruction::Load>("load");
    scanClient_ = nh_.serviceClient<acin_reconstruction::Scan>("perform_scan");
    modifyClient_ = nh_.serviceClient<acin_reconstruction::Modify>("modify");
    displayClient_ = nh_.serviceClient<std_srvs::Trigger>("display_scans");
    exportClient_ = nh_.serviceClient<acin_reconstruction::Export>("export");
    connectClient_ = nh_.serviceClient<acin_reconstruction::Connect>("connect");
    boundingBoxClient_ = nh_.serviceClient<acin_reconstruction::BoundingBox>("bounding_box");
    setReferenceClient_ = nh_.serviceClient<acin_reconstruction::SetReference>("set_reference");
    getIkClient_ = nh_.serviceClient<moveit_msgs::GetPositionIK>("compute_ik");
    
    // Subscribers
    scanListSub_ = nh_.subscribe("scan_list", 1, &ObjectReconstructionPanel::updateScanListROS, this);
    
    // Publisher
    markerPub_ = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 1);
    poseTargetPub_ = nh_.advertise<geometry_msgs::Pose>("mypose", 1);
    goalStatePub_ = nh_.advertise<moveit_msgs::RobotState>("/rviz/moveit/update_custom_goal_state", 1);
    
    // MoveIt
    moveGroup_ = new moveit::planning_interface::MoveGroupInterface("arm");
}

void ObjectReconstructionPanel::updateReference()
{
    acin_reconstruction::SetReference srv;
    srv.request.absolute_pose = moveGroup_->getCurrentPose("sensor_link").pose;
    setReferenceClient_.call(srv);
    
    if (srv.response.success)
        updateLabel_->setText("Successful");
    else
        updateLabel_->setText("Failed");
}

void ObjectReconstructionPanel::scan()
{
    QString message = QString("Scanning...");
    scanLabel_->setText(message);
    
    // Prepare service request
    acin_reconstruction::Scan srv;
    
    bool success = true;
    QString path = pathLineEdit_->text();
    QString number = QStringLiteral("%1").arg(fileCounter_, 3, 10, QLatin1Char('0'));
    srv.request.name = (QString("scan") + number).toUtf8().constData();
    if (!path.endsWith("/"))
        path.append("/");
    
    if (saveColorCheckBox_->isChecked()) {
        QString filename = path + QString("color") + number + QString(".png");
        QFileInfo file(filename);
        if (file.exists())
            success = false;
        else
            srv.request.path_color = filename.toUtf8().constData();
    }
    
    if (saveDepthCheckBox_->isChecked()) {
        QString filename = path + QString("depth") + number + QString(".png");
        QFileInfo file(filename);
        if (file.exists())
            success = false;
        else
            srv.request.path_depth = filename.toUtf8().constData();
    }
    
    if (savePoseCheckBox_->isChecked()) {
        QString filename = path + QString("pose") + number + QString(".yaml");
        QFileInfo file(filename);
        if (file.exists())
            success = false;
        else
            srv.request.path_pose = filename.toUtf8().constData();
    }
    
    if (success) {
        // Call service
        if (scanClient_.call(srv)) {
            message = QString::fromStdString(srv.response.message);
            // Publish scans
            std_srvs::Trigger trigger;
            displayClient_.call(trigger);
        } else {
            message = QString("Service call failed");
        }
    } else {
        message = QString("File already exists");
    }
    
    scanLabel_->setText(message);
}

bool ObjectReconstructionPanel::setGoal(const geometry_msgs::PoseStamped& pose) {
    static moveit_msgs::RobotState previousSolution;
    
    // Get parameters
    bool avoidCollisions = false;
    nh_.getParam("/shared/collision_aware_ik", avoidCollisions);
    
    // Compute IK solution
    moveit_msgs::GetPositionIK srv;
    srv.request.ik_request.group_name = "arm";
    srv.request.ik_request.robot_state = previousSolution;
    srv.request.ik_request.avoid_collisions = avoidCollisions;
    srv.request.ik_request.pose_stamped = pose;
    srv.request.ik_request.timeout = ros::Duration(0.1);
    
    if (getIkClient_.call(srv) && srv.response.error_code.val == srv.response.error_code.SUCCESS) {
        // Set goal state in Rviz
        goalStatePub_.publish(srv.response.solution);
        previousSolution = srv.response.solution;
        return true;
    } else {
        return false;
    }
}

void ObjectReconstructionPanel::getPoseTarget(geometry_msgs::PoseStamped& pose)
{
    // Read sphere
    double x = sphereSpinBox_[0]->value();
    double y = sphereSpinBox_[1]->value();
    double z = sphereSpinBox_[2]->value();
    double r = sphereSpinBox_[3]->value();
    
    // Read angles
    double p = poseSpinBox_[0]->value()*M_PI/180;
    double t = poseSpinBox_[1]->value()*M_PI/180;
    double a = poseSpinBox_[2]->value()*M_PI/180;
    
    // Compute quaternion from rotation matrix
    Eigen::Matrix3d R;
    R(0, 0) = cos(p)*cos(t)*sin(a) - sin(p)*cos(a);
    R(1, 0) = sin(p)*cos(t)*sin(a) + cos(p)*cos(a);
    R(2, 0) = -sin(t)*sin(a);
    R(0, 1) = cos(p)*cos(t)*cos(a) + sin(p)*sin(a);
    R(1, 1) = sin(p)*cos(t)*cos(a) - cos(p)*sin(a);
    R(2, 1) = -sin(t)*cos(a);
    R(0, 2) = -cos(p)*sin(t);
    R(1, 2) = -sin(p)*sin(t);
    R(2, 2) = -cos(t);
    Eigen::Quaterniond quaternion(R);
    Eigen::Vector4d coeffs = quaternion.coeffs();
    
    // Create pose target
    pose.pose.orientation.x = coeffs(0);
    pose.pose.orientation.y = coeffs(1);
    pose.pose.orientation.z = coeffs(2);
    pose.pose.orientation.w = coeffs(3);
    pose.pose.position.x = x+r*sin(t)*cos(p);
    pose.pose.position.y = y+r*sin(t)*sin(p);
    pose.pose.position.z = z+r*cos(t);
    
    if (markerRadioBtn_->isChecked())
        pose.header.frame_id = "marker";
    else
        pose.header.frame_id = "world";
}

void ObjectReconstructionPanel::updateMarker(const geometry_msgs::PoseStamped& pose, bool solution)
{
    // Create marker
    visualization_msgs::Marker marker;
    marker.header.stamp = ros::Time::now();
    marker.header.frame_id = pose.header.frame_id;
    marker.action = marker.ADD;
    marker.ns = "marker";
    marker.id = 0;
    marker.type = marker.ARROW;
    marker.pose.orientation.w = 1;
    marker.color.a = 1;
    if (solution)
        marker.color.g = 1;
    else
        marker.color.r = 1;
    marker.scale.x = 0.015;
    marker.scale.y = 0.040; // Width of arrowhead
    marker.scale.z = 0.070; // Height of arrowhead
    
    // Get end point of marker
    geometry_msgs::Point end;
    end.x = sphereSpinBox_[0]->value();
    end.y = sphereSpinBox_[1]->value();
    end.z = sphereSpinBox_[2]->value();
    
    // Add start and end point to marker
    marker.points.push_back(pose.pose.position);
    marker.points.push_back(end);
    
    // Publish marker
    markerPub_.publish(marker);
}

void ObjectReconstructionPanel::importRGBD(const QStringList& colorImages, const QStringList& depthImages)
{
    int sizeColor = colorImages.size();
    int sizeDepth = depthImages.size();
    if (sizeColor != sizeDepth)
        return;
    
    // Create service request
    std::string fileColor;
    std::string fileDepth;
    acin_reconstruction::Load srv;
    srv.request.type = srv.request.RGBD_IMAGE;
    for (int i = 0; i < sizeColor; ++i) {
        fileColor = colorImages.at(i).toUtf8().constData();
        fileDepth = depthImages.at(i).toUtf8().constData();
        QString name = QString("scan").append(QStringLiteral("%1").arg(i+fileCounter_, 3, 10, QLatin1Char('0')));
        
        srv.request.path_color = fileColor;
        srv.request.path_depth = fileDepth;
        srv.request.name = name.toUtf8().constData();
        geometry_msgs::Transform transform;
        transform.rotation.w = 1;
        srv.request.transform = transform;
    
        // Call service
        loadClient_.call(srv);
    }
    
    // Publish scans
    //std_srvs::Trigger trigger;
    //displayClient_.call(trigger);
}

void ObjectReconstructionPanel::importPointCloud(const QStringList &files)
{
    int size = files.size();
    
    // Create service request
    std::string file;
    acin_reconstruction::Load srv;
    srv.request.type = srv.request.POINT_CLOUD;
    for (int i = 0; i < size; ++i) {
        file = files.at(i).toUtf8().constData();
        QString name = QString("scan").append(QStringLiteral("%1").arg(i+fileCounter_, 3, 10, QLatin1Char('0')));
        
        srv.request.path_point_cloud = file;
        srv.request.name = name.toUtf8().constData();
        geometry_msgs::Transform transform;
        transform.rotation.w = 1;
        srv.request.transform = transform;
        
        // Call service
        loadClient_.call(srv);
    }
    
    // Publish scans
    //std_srvs::Trigger trigger;
    //displayClient_.call(trigger);
}

void ObjectReconstructionPanel::updateScanListROS(const acin_reconstruction::ScanList::ConstPtr& msg)
{
    // Number of scans before update
    int before = model_->stringList().size();
    
    // Update scan list
    QStringList list;
    int size = msg->scans.size();
    for (int i = 0; i < size; ++i) {
        QString element = QString::fromStdString(msg->scans[i]);
        list.append(element);
    }
    model_->setStringList(list);
    
    // Number of scans after update
    int after = model_->stringList().size();
    
    // Increase file counter
    if (after > before)
        fileCounter_ += after-before;
}

void ObjectReconstructionPanel::callTransform(bool forward)
{
    acin_reconstruction::Transform srv;
    QModelIndex index = registerListView_->currentIndex();
    srv.request.fragment = index.data().toString().toUtf8().constData();
    srv.request.forward = forward;
    
    // Insert translation
    srv.request.transform.translation.x = translationSpinBox_[0]->value()/1000;
    srv.request.transform.translation.y = translationSpinBox_[1]->value()/1000;
    srv.request.transform.translation.z = translationSpinBox_[2]->value()/1000;
    
    // Insert rotation
    float roll  = rotationSpinBox_[0]->value()*M_PI/180;
    float pitch = rotationSpinBox_[1]->value()*M_PI/180;
    float yaw   = rotationSpinBox_[2]->value()*M_PI/180;
    Eigen::Quaternionf quaternion;
    quaternion = Eigen::AngleAxisf(roll,Eigen::Vector3f::UnitX())*
                 Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitY())*
                 Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ());
    srv.request.transform.rotation.w = quaternion.w();
    srv.request.transform.rotation.x = quaternion.x();
    srv.request.transform.rotation.y = quaternion.y();
    srv.request.transform.rotation.z = quaternion.z();
    
    // Call service
    transformClient_.call(srv);
    
    // Publish scans
    std_srvs::Trigger trigger;
    displayClient_.call(trigger);
}

void ObjectReconstructionPanel::callLocalRegistration()
{
    acin_reconstruction::Register srv;
    int idx = methodComboBox_->currentIndex();
    if (idx == 0)
        srv.request.method = srv.request.POINT_TO_POINT_ICP;
    else if (idx == 1)
        srv.request.method = srv.request.POINT_TO_PLANE_ICP;
    
    QModelIndex index = registerListView_->currentIndex();
    srv.request.source = index.data().toString().toUtf8().constData();
    srv.request.target = targetLineEdit_->text().toUtf8().constData();
    
    srv.request.threshold = thresholdSpinBox_->value()/1000;
    if (localRegistrationClient_.call(srv)) {
        // Set translation
        translationSpinBox_[0]->setValue(srv.response.transform.translation.x*1000);
        translationSpinBox_[1]->setValue(srv.response.transform.translation.y*1000);
        translationSpinBox_[2]->setValue(srv.response.transform.translation.z*1000);
        
        // Set roation
        double qw = srv.response.transform.rotation.w;
        double qx = srv.response.transform.rotation.x;
        double qy = srv.response.transform.rotation.y;
        double qz = srv.response.transform.rotation.z;
        Eigen::Quaterniond quaternion = Eigen::Quaterniond(qw, qx, qy, qz);
        Eigen::Vector3d angles = quaternion.toRotationMatrix().eulerAngles(0, 1, 2);
        
        for (int i = 0; i < 3; ++i)
            rotationSpinBox_[i]->setValue(angles(i)*180/M_PI);
    } else {
        ROS_ERROR("Failed to call service");
    }
}

// Called once when the goal becomes active
void ObjectReconstructionPanel::activeCb()
{
    progressBar_->setValue(0);
    statusTextEdit_->clear();
}

// Called once when the goal completes
void ObjectReconstructionPanel::scriptDoneCb(const actionlib::SimpleClientGoalState& state,
                                             const acin_automation::ExecuteScriptResultConstPtr& result)
{
    QString stateString = QString::fromStdString(state.toString());
    QString message = QString("Script ").append(stateString.toLower());
    statusLabel_->setText(message);
    startViewPlanningBtn_->setDisabled(false);
    startScriptBtn_->setDisabled(false);
    chooseBtn_->setDisabled(false);
    scriptLineEdit_->setDisabled(false);
    stopBtn_->setDisabled(true);
}

// Called every time feedback is received for the goal
void ObjectReconstructionPanel::scriptFeedbackCb(const acin_automation::ExecuteScriptFeedbackConstPtr& feedback)
{
    int progress = 100*feedback->progress;
    progressBar_->setValue(progress);
    statusTextEdit_->append(QString::fromStdString(feedback->message));
}

// Called once when the goal completes
void ObjectReconstructionPanel::viewPlanningDoneCb(const actionlib::SimpleClientGoalState& state,
                                                const acin_automation::PlanViewsResultConstPtr& result)
{
    QString stateString = QString::fromStdString(state.toString());
    QString message = QString("Scan ").append(stateString.toLower());
    statusLabel_->setText(message);
    startViewPlanningBtn_->setDisabled(false);
    startScriptBtn_->setDisabled(false);
    chooseBtn_->setDisabled(false);
    scriptLineEdit_->setDisabled(false);
    stopBtn_->setDisabled(true);
}

// Called every time feedback is received for the goal
void ObjectReconstructionPanel::viewPlanningFeedbackCb(const acin_automation::PlanViewsFeedbackConstPtr& feedback)
{
    int progress = 100*feedback->progress;
    progressBar_->setValue(progress);
    statusTextEdit_->append(QString::fromStdString(feedback->message));
}

// Save Rviz configuration
void ObjectReconstructionPanel::save(rviz::Config config) const {
    rviz::Panel::save(config);
    config.mapSetValue("x", sphereSpinBox_[0]->value());
    config.mapSetValue("y", sphereSpinBox_[1]->value());
    config.mapSetValue("z", sphereSpinBox_[2]->value());
    config.mapSetValue("r", sphereSpinBox_[3]->value());
    config.mapSetValue("threshold", thresholdSpinBox_->value());
    config.mapSetValue("save_path", pathLineEdit_->text());
    config.mapSetValue("script_path", scriptLineEdit_->text());
    config.mapSetValue("registration_method", methodComboBox_->currentIndex());
    config.mapSetValue("bounding_lower_x", lowerBoundSpinBox_[0]->value());
    config.mapSetValue("bounding_lower_y", lowerBoundSpinBox_[1]->value());
    config.mapSetValue("bounding_lower_z", lowerBoundSpinBox_[2]->value());
    config.mapSetValue("bounding_upper_x", upperBoundSpinBox_[0]->value());
    config.mapSetValue("bounding_upper_y", upperBoundSpinBox_[1]->value());
    config.mapSetValue("bounding_upper_z", upperBoundSpinBox_[2]->value());
    config.mapSetValue("voxel_size", downsampleSpinBox_->value());
    config.mapSetValue("save_color", saveColorCheckBox_->isChecked());
    config.mapSetValue("save_depth", saveDepthCheckBox_->isChecked());
    config.mapSetValue("save_pose", savePoseCheckBox_->isChecked());
    config.mapSetValue("number_scans", viewPlanningNumberSpinBox_->value());
    config.mapSetValue("camera_distance", viewPlanningDistanceSpinBox_->value());
}

// Load Rviz configuration
void ObjectReconstructionPanel::load(const rviz::Config& config) {
    rviz::Panel::load(config);
    int i;
    if (config.mapGetInt("registration_method", &i))
        methodComboBox_->setCurrentIndex(i);
    if (config.mapGetInt("number_scans", &i))
        viewPlanningNumberSpinBox_->setValue(i);
    
    float f;
    if (config.mapGetFloat("x", &f))
        sphereSpinBox_[0]->setValue(f);
    if (config.mapGetFloat("y", &f))
        sphereSpinBox_[1]->setValue(f);
    if (config.mapGetFloat("z", &f))
        sphereSpinBox_[2]->setValue(f);
    if (config.mapGetFloat("r", &f))
        sphereSpinBox_[3]->setValue(f);
    if (config.mapGetFloat("threshold", &f))
        thresholdSpinBox_->setValue(f);
    else
        thresholdSpinBox_->setValue(5);
    if (config.mapGetFloat("bounding_lower_x", &f))
        lowerBoundSpinBox_[0]->setValue(f);
    if (config.mapGetFloat("bounding_lower_y", &f))
        lowerBoundSpinBox_[1]->setValue(f);
    if (config.mapGetFloat("bounding_lower_z", &f))
        lowerBoundSpinBox_[2]->setValue(f);
    if (config.mapGetFloat("bounding_upper_x", &f))
        upperBoundSpinBox_[0]->setValue(f);
    if (config.mapGetFloat("bounding_upper_y", &f))
        upperBoundSpinBox_[1]->setValue(f);
    if (config.mapGetFloat("bounding_upper_z", &f))
        upperBoundSpinBox_[2]->setValue(f);
    if (config.mapGetFloat("voxel_size", &f))
        downsampleSpinBox_->setValue(f);
    if (config.mapGetFloat("camera_distance", &f))
        viewPlanningDistanceSpinBox_->setValue(f);
    
    QString s;
    if (config.mapGetString("save_path", &s))
       pathLineEdit_->setText(s);
    else
        pathLineEdit_->setText("/home");
    if (config.mapGetString("script_path", &s))
        scriptLineEdit_->setText(s);
    else
        scriptLineEdit_->setText("/home");
    
    bool b = false;
    // Do the opposite first to ensure a state change
    config.mapGetBool("save_color", &b);
    saveColorCheckBox_->setChecked(!b);
    saveColorCheckBox_->setChecked(b);
    config.mapGetBool("save_depth", &b);
    saveDepthCheckBox_->setChecked(!b);
    saveDepthCheckBox_->setChecked(b);
    config.mapGetBool("save_pose", &b);
    savePoseCheckBox_->setChecked(!b);
    savePoseCheckBox_->setChecked(b);
}

} // End of namespace

// Export class for pluginlib
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(acin_gui::ObjectReconstructionPanel, rviz::Panel)
