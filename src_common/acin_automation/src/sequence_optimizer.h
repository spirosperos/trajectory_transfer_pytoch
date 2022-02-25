#ifndef SEQUENCE_OPTIMIZER_H
#define SEQUENCE_OPTIMIZER_H

#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>
#include <moveit_msgs/RobotState.h>

#include <moveit/move_group_interface/move_group_interface.h>

#include "vertex.h"
#include "node.h"

void loadParameters(sensor_msgs::JointState startState);
void optimizeGraph(std::vector<Vertex*>& vertices);
void optimizeTree(std::vector<Vertex*>& vertices);
void reversePlan(moveit::planning_interface::MoveGroupInterface::Plan& plan);
Node* minimumSpanningTree(std::vector<Vertex*>& vertices);
bool initializeGraph(std::vector<Vertex*>& vertices, std::vector<Edge*>& edges,
                     sensor_msgs::JointState& startState, std::vector<geometry_msgs::PoseStamped>& poses);

#endif
