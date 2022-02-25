#include "ros/ros.h"

#include "std_msgs/Bool.h"

#include "control_msgs/FollowJointTrajectoryAction.h"
#include <actionlib/server/simple_action_server.h>

#include "ros_acin_robot_support/JointSpaceTraj.h"
#include "ros_acin_robot_support/Joints.h"

#include "trajectory_msgs/JointTrajectoryPoint.h"
#include "trajectory_msgs/JointTrajectory.h"

typedef actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> Server;

// Global variables
ros::Publisher pub;
int trajectoryLength = 0;
int overlap = 0;
int robotDimension = 0;
std::vector<std::string> jointOrder;
bool motionEnabled = false;

void correctPositionOrder(std::vector<trajectory_msgs::JointTrajectoryPoint>& points, std::vector<int> jointMapping)
{
    int length = points.size();
    std::vector<double> reorderedPositions(robotDimension);
    
    for (int i = 0; i < length; ++i) {
        for (int j = 0; j < robotDimension; ++j)
            reorderedPositions[j] = points[i].positions[jointMapping[j]];
        points[i].positions = reorderedPositions;
    }
}

void execute(const control_msgs::FollowJointTrajectoryGoalConstPtr& goal, Server* actionServer)
{
    // Check number of trajectory points
    if (goal->trajectory.points.size() < 2) {
        ROS_ERROR("There are not enough points in received trajectory");
        actionServer->setAborted();
        return;
    }
    
    // Check number of joints in received trajectory
    if (goal->trajectory.joint_names.size() < robotDimension) {
        ROS_ERROR("There are not enough joints in received trajectory");
        actionServer->setAborted();
        return;
    }
    
    // Get joint mapping
    std::vector<int> jointMapping(robotDimension, -1);
    for (int i = 0; i < robotDimension; ++i) {
        for (int j = 0; j < robotDimension; ++j) {
            if (jointOrder[i] == goal->trajectory.joint_names[j]) {
                jointMapping[i] = j;
                break;
            }
        }
        if (jointMapping[i] == -1) {
            ROS_ERROR_STREAM("Joint " << jointOrder[i] << " is not contained in received trajectory");
            actionServer->setPreempted();
            return;
        }
    }
    
    // Correct joint order
    std::vector<trajectory_msgs::JointTrajectoryPoint> reorderedPoints;
    reorderedPoints = goal->trajectory.points;
    correctPositionOrder(reorderedPoints, jointMapping);
    
    // Calculate required number of trajectory messages
    int totalLength = reorderedPoints.size();
    int freeLength = trajectoryLength - overlap;
    int messageNumber;
    if (totalLength > trajectoryLength) {
        messageNumber = totalLength / freeLength;
        int remainder = totalLength % freeLength;
        if (remainder > overlap)
            messageNumber += 1;
    } else {
        messageNumber = 1;
    }
    
    // Append points such that each message has the same length
    int missing = messageNumber * freeLength + overlap - totalLength;
    for (int j = missing; j > 0; --j)
        reorderedPoints.push_back(reorderedPoints.back());
    
    // Prepare ACIN's trajectory message
    ros_acin_robot_support::JointSpaceTraj trajectory;
    trajectory.t_k = std::vector<ros::Time>(trajectoryLength);
    trajectory.JointsArray = std::vector<ros_acin_robot_support::Joints>(trajectoryLength);
    
    ros::Duration duration;
    ros::Time t0(0);
    
    // Send trajectory messages
    ros::Time start = ros::Time::now();
    ros::Duration dur1(0);
    ros::Duration dur2(0);
    for (int i = 0; i < messageNumber; ++i) {
        if (actionServer->isPreemptRequested()) {
            actionServer->setPreempted();
            return;
        } else if (!motionEnabled) {
            actionServer->setPreempted();
            ROS_ERROR("Motion disabled");
            return;
        } else {
            // Fill in data
            trajectory.header.stamp = ros::Time::now();
            for (int k = 0; k < trajectoryLength; ++k) {
                trajectory.JointsArray[k].Joints = reorderedPoints[i*freeLength+k].positions;
                trajectory.t_k[k] = t0 + reorderedPoints[i*freeLength+k].time_from_start;
            }
            
            dur2 = ros::Time::now() - start;
            if (dur1 > dur2) {
                duration = dur1 - dur2;
                duration.sleep();
            }
            
            pub.publish(trajectory);
            dur1 = trajectory.t_k[freeLength] - t0;
        }
    }
    
    // Wait until end of trajectory
    dur2 = ros::Time::now() - start;
    dur1 = trajectory.t_k.back() - t0;
    if (dur1 > dur2) {
        duration = dur1 - dur2;
        duration.sleep();
    }
    
    actionServer->setSucceeded();
}

void motionEnable(const std_msgs::Bool::ConstPtr& msg)
{
    motionEnabled = msg->data;
}

int main(int argc, char** argv)
{
    // Initialize ROS
    ros::init(argc, argv, "controller_gateway");
    ros::NodeHandle nh;
    
    // Get parameters
    bool failure;
    
    if (!nh.getParam("gateway/trajectory_length", trajectoryLength)) {
        ROS_ERROR("Parameter 'trajectory_length' not found");
        failure = true;
    } else if (trajectoryLength < 1) {
        ROS_ERROR("Parameter 'trajectory_length' must be positive");
        failure = true;
    }
    
    if (!nh.getParam("gateway/overlap", overlap)) {
        ROS_ERROR("Parameter 'overlap' not found");
        failure = true;
    } else if (overlap < 1) {
        ROS_ERROR("Parameter 'overlap' must be positive");
        failure = true;
    } else if (trajectoryLength > 0 && overlap >= trajectoryLength) {
        ROS_ERROR("Parameter 'overlap' must smaller than 'trajectory_length'");
        failure = true;
    }
    
    if (!nh.getParam("gateway/robot_dimension", robotDimension)) {
        ROS_ERROR("Parameter 'robot_dimension' not found");
        failure = true;
    } else if (robotDimension < 1) {
        ROS_ERROR("Parameter 'robot_dimension' not valid");
        failure = true;
    }
    
    if (!nh.getParam("gateway/joint_order", jointOrder)) {
        ROS_ERROR("Parameter 'joint_order' not found");
        failure = true;
    } else if (robotDimension > 0 && jointOrder.size() != robotDimension) {
        ROS_ERROR("Length of parameter 'joint_order' does not match 'robot_dimension'");
        failure = true;
    }
    
    if (failure)
        return 0;
    
    // Start publisher and FollowJointTrajectoryAction server
    pub = nh.advertise<ros_acin_robot_support::JointSpaceTraj>("ros_acin_robot_control/desired_JointSpace_traj", 1);
    Server server(nh, "arm_controller/follow_joint_trajectory", boost::bind(&execute, _1, &server), false);
    server.start();
    
    // Start subscriber
    ros::Subscriber sub = nh.subscribe("/ros_acin_robot_control/motion_EN", 1, motionEnable);
    
    ros::spin();
    return 0;
}
