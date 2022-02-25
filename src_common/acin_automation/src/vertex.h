#ifndef VERTEX_H
#define VERTEX_H

class Edge;
#include <geometry_msgs/PoseStamped.h>
#include <moveit_msgs/GetPositionIK.h>
#include <moveit/move_group_interface/move_group_interface.h>

class Vertex {
public:
    static moveit_msgs::GetPositionIK::Request request;
    std::vector<Edge*> edges;
    geometry_msgs::PoseStamped pose;
    sensor_msgs::JointState jointState;
    double keyValue;
    bool included;
    bool isValid;
    
    Vertex(geometry_msgs::PoseStamped& pose);
    Vertex(geometry_msgs::PoseStamped& pose, const std::vector<double>& seed);
    Vertex(sensor_msgs::JointState& jointState);
    
    void addEdge(Edge *edge);
    bool inverseKinematicsSolution(const std::vector<double>& seed);
    bool inverseKinematicsSolution();
    Vertex* neighbor(const Edge *edge);
    void updateNeighbors();
    void setEdgeCosts(const std::vector<double>& costs);
    void getEdgeCosts(std::vector<double>& costs);
    void updateDistances();
};

#endif
