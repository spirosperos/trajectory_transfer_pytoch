#include "vertex.h"
#include "edge.h"

extern ros::ServiceClient ikClient;
extern std::vector<bool> mask;

Vertex::Vertex(geometry_msgs::PoseStamped& pose)
{
    included = false;
    keyValue = std::numeric_limits<double>::max();
    this->pose = pose;
    
    isValid = inverseKinematicsSolution();
}

Vertex::Vertex(geometry_msgs::PoseStamped& pose, const std::vector<double>& seed)
{
    included = false;
    keyValue = std::numeric_limits<double>::max();
    this->pose = pose;
    
    isValid = inverseKinematicsSolution(seed);
}

Vertex::Vertex(sensor_msgs::JointState& jointState)
{
    included = false;
    keyValue = std::numeric_limits<double>::max();
    this->jointState = jointState;
}

void Vertex::addEdge(Edge *edge)
{
    edges.push_back(edge);
}

Vertex* Vertex::neighbor(const Edge *edge)
{
    if (edge->start == this)
        return edge->end;
    else
        return edge->start;
}

void Vertex::setEdgeCosts(const std::vector<double>& costs)
{
    for (int i = 0; i < edges.size(); ++i)
        edges[i]->length = costs[i];
}

void Vertex::getEdgeCosts(std::vector<double>& costs)
{
    costs.reserve(edges.size());
    for (int i = 0; i < edges.size(); ++i)
        costs.push_back(edges[i]->length);
}

void Vertex::updateDistances()
{
    for (int i = 0; i < edges.size(); ++i)
        edges[i]->calculateApproxLength();
}

void Vertex::updateNeighbors()
{
    for (int i = 0; i < edges.size(); ++i) {
        Vertex *neighbor = this->neighbor(edges[i]);
        if (!(neighbor->included) && (neighbor->keyValue > edges[i]->length))
            neighbor->keyValue = edges[i]->length;
    }
}

bool Vertex::inverseKinematicsSolution()
{
    
    std::vector<double> seed = request.ik_request.robot_state.joint_state.position;
    for (int i = 0; i < seed.size(); ++i) {
        if (mask[i])
            seed[i] = 2 * (double)rand() / RAND_MAX - 1;
    }
    return inverseKinematicsSolution(seed);
}

bool Vertex::inverseKinematicsSolution(const std::vector<double>& seed)
{
    request.ik_request.pose_stamped = pose;
    request.ik_request.robot_state.joint_state.position = seed;
    
    moveit_msgs::GetPositionIK::Response response;
    ikClient.call(request, response);
    
    if (response.error_code.val == response.error_code.SUCCESS) {
        jointState = response.solution.joint_state;
        return true;
    } else {
        return false;
    }
}
