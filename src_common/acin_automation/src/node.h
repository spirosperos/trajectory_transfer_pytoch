#ifndef NODE_H
#define NODE_H

#include "edge.h"
#include "vertex.h"
#include <sensor_msgs/JointState.h>
#include <moveit_msgs/MotionPlanResponse.h>

class Node {
private:
    void preorderWalk(std::vector<sensor_msgs::JointState>& jointStates,
                      std::vector<moveit_msgs::MotionPlanResponse>& plans,
                      bool connect);
    void preorderCost(double& cost, std::vector<Vertex*>& vertices, bool connect);
public:
    std::vector<Node*> children;
    Vertex *vertex;
    Edge *edge;
    
    Node(Vertex *vertex, Edge *edge);
    ~Node();
    void addChild(Node *node);
    Node* findNode(const Vertex *vertex);
    Edge* findNonExactEdge();
    bool preorderWalk(std::vector<sensor_msgs::JointState>& jointStates,
                      std::vector<moveit_msgs::MotionPlanResponse>& plans);
    double preorderCost();
    int countLeafs();
    double cost();
};

#endif
