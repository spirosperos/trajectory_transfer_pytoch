#include "edge.h"
#include "vertex.h"
#include <cmath>

extern moveit::planning_interface::MoveGroupInterface *moveGroup;
extern std::vector<double> amax;
extern std::vector<double> vmax;

Edge::Edge(Vertex *start, Vertex *end)
{
    this->start = start;
    this->end = end;
    calculateApproxLength();
    start->addEdge(this);
    end->addEdge(this);
}

void Edge::calculateApproxLength()
{
    std::vector<double> tmin;
    for (int i = 0; i < vmax.size(); ++i) {
        double ds = abs(start->jointState.position[i] - end->jointState.position[i]);
        if (ds < pow(vmax[i], 2) / amax[i]) {
            // Velocity limit is not attained
            tmin.push_back(2 * sqrt(ds / amax[i]));
        } else {
            // Velocity limit is attained
            tmin.push_back(ds / vmax[i] + vmax[i] / amax[i]);
        }
    }
    
    length = *std::max_element(tmin.begin(), tmin.end());
    exact = false;
}

void Edge::calculateExactLength()
{
    moveit_msgs::RobotState robotState;
    robotState.joint_state = start->jointState;
    moveGroup->setStartState(robotState);
    moveGroup->setJointValueTarget(end->jointState);
    
    // Try to find a solution several times
    bool success = false;
    for (int i = 0; i < 500; ++i) {
        if (moveGroup->plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS) {
            success = true;
            break;
        }
    }
    
    if (success) {
        // If plan was found
        length = plan.trajectory_.joint_trajectory.points.back().time_from_start.toSec();
    } else {
        // If no valid plan was found
        length = std::numeric_limits<double>::max();
    }
    
    exact = true;
}
