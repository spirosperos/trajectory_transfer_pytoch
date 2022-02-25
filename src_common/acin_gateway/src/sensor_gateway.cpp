#include "ros/ros.h"
#include "sensor_msgs/JointState.h"

// Global variables
ros::Publisher pub;
std::vector<std::string> jointOrder;
std::vector<std::string> fixedJoints;
std::vector<double> fixedValues;

void callback(const sensor_msgs::JointState::ConstPtr& msg)
{
    sensor_msgs::JointState jointState(*msg);
    jointState.name = jointOrder;
    for (int i = 0; i < fixedJoints.size(); ++i) {
        jointState.name.push_back(fixedJoints[i]);
        jointState.position.push_back(fixedValues[i]);
    }
    pub.publish(jointState);
}

int main(int argc, char** argv)
{
    // Initialize ROS
    ros::init(argc, argv, "sensor_gateway");
    ros::NodeHandle nh;
    
    // Get parameters
    if (!nh.getParam("gateway/joint_order", jointOrder)) {
        ROS_ERROR("Parameter 'joint_order' not found");
        return 0;
    }
    
    if (nh.getParam("gateway/fixed_joints", fixedJoints)) {
        if (!nh.getParam("gateway/fixed_values", fixedValues)) {
            ROS_ERROR("Parameter 'fixed_values' not found");
            return 0;
        } else if (fixedValues.size() != fixedJoints.size()) {
            ROS_ERROR("Lengths of 'fixed_joints' and 'fixed_values' do not match");
            return 0;
        }
        ROS_INFO("Publish fixed joints");
    }
    
    // Start publisher and subscriber
    pub = nh.advertise<sensor_msgs::JointState>("measurement/joint_states", 1);
    ros::Subscriber sub = nh.subscribe("ros_acin_robot_control/joint_state", 1, callback);
    
    ros::spin();
    return 0;
}
