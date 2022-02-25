#include "node.h"
#include "sequence_optimizer.h"
#include <moveit_msgs/MotionPlanResponse.h>

extern moveit::planning_interface::MoveGroupInterface *moveGroup;

Node::Node(Vertex *vertex, Edge *edge)
{
    this->vertex = vertex;
    this->edge = edge;
}

Node::~Node()
{
    for (int i = 0; i < children.size(); ++i)
        delete children[i];
}

void Node::addChild(Node *node)
{
    children.push_back(node);
}

Node* Node::findNode(const Vertex *vertex)
{
    if (this->vertex == vertex)
        return this;
    
    for (int i = 0; i < children.size(); ++i) {
        Node *found = children[i]->findNode(vertex);
        if (found != NULL)
            return found;
    }
    
    return NULL;
}

double Node::cost()
{
    double cost = 0;
    if (edge != NULL)
        cost += edge->length;
    
    for (int i = 0; i < children.size(); ++i)
        cost += children[i]->cost();
    return  cost;
}

void Node::preorderWalk(std::vector<sensor_msgs::JointState>& jointStates,
                        std::vector<moveit_msgs::MotionPlanResponse>& plans,
                        bool connect)
{
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    moveit_msgs::MotionPlanResponse planResponse;
    
    if (connect) {
        moveit_msgs::RobotState robotState;
        robotState.joint_state = jointStates.back();
        moveGroup->setStartState(robotState);
        moveGroup->setJointValueTarget(vertex->jointState);
        moveGroup->plan(plan);
        
        // Try to find a solution several times
        bool success = false;
        for (int i = 0; i < 500; ++i) {
            if (moveGroup->plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS) {
                success = true;
                break;
            }
        }
        
        if (success)
            planResponse.error_code.val = planResponse.error_code.SUCCESS;
    } else {
        plan = edge->plan;
        if (edge->start == vertex)
            reversePlan(plan);
        
        if (edge->length < std::numeric_limits<double>::max())
            planResponse.error_code.val = planResponse.error_code.SUCCESS;
    }
    
    planResponse.trajectory_start = plan.start_state_;
    planResponse.trajectory = plan.trajectory_;
    planResponse.planning_time = plan.planning_time_;
    plans.push_back(planResponse);
    
    jointStates.push_back(vertex->jointState);
    
    if (children.size() == 0)
        return;
    
    children[0]->preorderWalk(jointStates, plans, false);
    for (int i = 1; i < children.size(); ++i)
        children[i]->preorderWalk(jointStates, plans, true);
}

bool Node::preorderWalk(std::vector<sensor_msgs::JointState>& jointStates,
                        std::vector<moveit_msgs::MotionPlanResponse>& plans)
{
    if (children.size() == 0)
        return true;
    
    children[0]->preorderWalk(jointStates, plans, false);
    for (int i = 1; i < children.size(); ++i)
        children[i]->preorderWalk(jointStates, plans, true);
    
    for (int i = 0; i < plans.size(); ++i) {
        if (plans[i].error_code.val != plans[i].error_code.SUCCESS)
            return false;
    }
    
    return true;
}

void Node::preorderCost(double& cost, std::vector<Vertex*>& vertices, bool connect)
{
    if (connect) {
        // Find edge which connects the current vertex to the previous one
        for (int i = 0; i < vertex->edges.size(); ++i) {
            if (vertex->neighbor(vertex->edges[i]) == vertices.back()) {
                cost += vertex->edges[i]->length;
                break;
            }
        }
    } else {
        // The connection is already known
        cost += edge->length;
    }
    
    vertices.push_back(vertex);
    
    if (children.size() == 0)
        return;
    
    // Proceed with children
    children[0]->preorderCost(cost, vertices, false);
    for (int i = 1; i < children.size(); ++i)
        children[i]->preorderCost(cost, vertices, true);
}

double Node::preorderCost()
{
    double cost = 0;
    std::vector<Vertex*> visited;
    visited.push_back(vertex);
    
    if (children.size() == 0)
        return 0;
    
    // Proceeed with children
    children[0]->preorderCost(cost, visited, false);
    for (int i = 1; i < children.size(); ++i)
        children[i]->preorderCost(cost, visited, true);
    
    return cost;
}

int Node::countLeafs()
{
    if (children.size() == 0)
        return 1;
    
    int leafs = 0;
    for (int i = 0; i < children.size(); ++i)
        leafs += children[i]->countLeafs();
    
    return leafs;
}

Edge* Node::findNonExactEdge()
{
    if (edge != NULL && !edge->exact)
        return edge;
    
    for (int i = 0; i < children.size(); ++i) {
        Edge *found = children[i]->findNonExactEdge();
        if (found != NULL)
            return found;
    }
    
    return NULL;
}
