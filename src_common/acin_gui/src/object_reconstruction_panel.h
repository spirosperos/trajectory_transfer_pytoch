#ifndef OBJECT_RECONSTRUCTION_PANEL_H
#define OBJECT_RECONSTRUCTION_PANEL_H

#ifndef Q_MOC_RUN
# include <ros/ros.h>
# include <rviz/panel.h>
#endif

#include <acin_reconstruction/ScanList.h>
#include <acin_automation/ExecuteScriptAction.h>
#include <acin_automation/PlanViewsAction.h>
#include <actionlib/client/simple_action_client.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include "geometry_msgs/Transform.h"
#include "geometry_msgs/PoseStamped.h"

#include <QLabel>

class QLineEdit;
class QSpinBox;
class QDoubleSpinBox;
class QComboBox;
class QStringListModel;
class QListView;
class QProgressBar;
class QTextEdit;
class QPushButton;
class QCheckBox;
class MoveGroupInterface;
class QSize;
class QRadioButton;

namespace acin_gui
{

class LabelEllipsis: public QLabel {
Q_OBJECT
public:
    LabelEllipsis(QWidget *parent = 0);
protected:
    void paintEvent(QPaintEvent* event) override;
    QSize minimumSizeHint() const override;
    QSize sizeHint() const override;
};

class ObjectReconstructionPanel: public rviz::Panel {
Q_OBJECT
public:
    ObjectReconstructionPanel(QWidget *parent = 0);
    
    // Override Rviz functions for saving and loading data
    virtual void load(const rviz::Config& config);
    virtual void save(rviz::Config config) const;
    
public Q_SLOTS:
    
    // Internal slots
protected Q_SLOTS:
    void callTransform(bool forward);
    void callLocalRegistration();
    void updateScanListROS(const acin_reconstruction::ScanList::ConstPtr& msg);
    void importRGBD(const QStringList& colorImages, const QStringList& depthImages);
    void importPointCloud(const QStringList& files);
    void getPoseTarget(geometry_msgs::PoseStamped& pose);
    void updateMarker(const geometry_msgs::PoseStamped& pose, bool solution);
    void updateReference();
    bool setGoal(const geometry_msgs::PoseStamped& pose);
    void scan();
    void activeCb();
    void scriptDoneCb(const actionlib::SimpleClientGoalState& state,const acin_automation::ExecuteScriptResultConstPtr& result);
    void scriptFeedbackCb(const acin_automation::ExecuteScriptFeedbackConstPtr& feedback);
    void viewPlanningDoneCb(const actionlib::SimpleClientGoalState& state,const acin_automation::PlanViewsResultConstPtr& result);
    void viewPlanningFeedbackCb(const acin_automation::PlanViewsFeedbackConstPtr& feedback);

protected:
    ros::ServiceClient transformClient_;
    ros::ServiceClient localRegistrationClient_;
    ros::ServiceClient multiwayRegistrationClient_;
    ros::ServiceClient loadClient_;
    ros::ServiceClient scanClient_;
    ros::ServiceClient modifyClient_;
    ros::ServiceClient displayClient_;
    ros::ServiceClient exportClient_;
    ros::ServiceClient connectClient_;
    ros::ServiceClient boundingBoxClient_;
    ros::ServiceClient setReferenceClient_;
    ros::ServiceClient getIkClient_;
    ros::Subscriber scanListSub_;
    ros::Publisher markerPub_;
    ros::Publisher poseTargetPub_;
    ros::Publisher goalStatePub_;
    
    int fileCounter_;
    
    actionlib::SimpleActionClient<acin_automation::ExecuteScriptAction>* scriptActionClient_;
    actionlib::SimpleActionClient<acin_automation::PlanViewsAction>* viewPlanningActionClient_;
    
    QDoubleSpinBox *translationSpinBox_[3];
    QDoubleSpinBox *rotationSpinBox_[3];
    
    QDoubleSpinBox *lowerBoundSpinBox_[3];
    QDoubleSpinBox *upperBoundSpinBox_[3];
    
    QDoubleSpinBox *thresholdSpinBox_;
    
    QDoubleSpinBox *sphereSpinBox_[4];
    QDoubleSpinBox *poseSpinBox_[3];
    
    QDoubleSpinBox *downsampleSpinBox_;
    
    QPushButton *startViewPlanningBtn_;
    QPushButton *startScriptBtn_;
    QPushButton *stopBtn_;
    QPushButton *chooseBtn_;
    std::string automationMode;
    
    QDoubleSpinBox *viewPlanningDistanceSpinBox_;
    QSpinBox *viewPlanningNumberSpinBox_;
    
    QLabel *scanLabel_;
    QLabel *statusLabel_;
    QLabel *updateLabel_;
    QLabel *exportLabel_;
    
    QProgressBar *progressBar_;
    QTextEdit *statusTextEdit_;
    
    QLineEdit *pathLineEdit_;
    QLineEdit *scriptLineEdit_;
    
    QStringListModel *model_;
    QListView *scanListView_;
    QListView *registerListView_;
    QListView *reconstructListView_;
    
    QComboBox *methodComboBox_;
    
    QLineEdit *targetLineEdit_;
    
    QRadioButton *worldRadioBtn_;
    QRadioButton *markerRadioBtn_;
    
    QCheckBox *saveColorCheckBox_;
    QCheckBox *saveDepthCheckBox_;
    QCheckBox *savePoseCheckBox_;
    
    moveit::planning_interface::MoveGroupInterface *moveGroup_;
    
    // ROS node handle
    ros::NodeHandle nh_;
};

} // end namespace acin_reconstruction_gui

#endif // OBJECT_RECONSTRUCTION_PANEL_H
