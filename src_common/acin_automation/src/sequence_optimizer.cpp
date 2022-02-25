#include "ros/ros.h"
#include <std_msgs/String.h>
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/MotionPlanResponse.h>
#include <moveit_msgs/RobotState.h>
#include <moveit_msgs/GetPositionIK.h>
#include <geometry_msgs/PoseStamped.h>

#include "acin_automation/Optimize.h"
#include "acin_automation/Select.h"
#include "sequence_optimizer.h"
#include "vertex.h"
#include "edge.h"
#include "node.h"

#include <limits>

// Global variables
ros::NodeHandle *nh_;
ros::ServiceClient ikClient;
moveit::planning_interface::MoveGroupInterface *moveGroup;
std::vector<double> amax;
std::vector<double> vmax;
std::vector<bool> mask;

void deleteObjects(std::vector<Vertex*>& vertices, std::vector<Edge*>& edges)
{
    for (int i = 0; i < vertices.size(); ++i)
        delete vertices[i];
    
    for (int i = 0; i < edges.size(); ++i)
        delete edges[i];
}

void reversePlan(moveit::planning_interface::MoveGroupInterface::Plan& plan)
{
    int length = plan.trajectory_.joint_trajectory.points.size();
    int dim = plan.trajectory_.joint_trajectory.joint_names.size();
    std::vector<trajectory_msgs::JointTrajectoryPoint> originalPoints = plan.trajectory_.joint_trajectory.points;
    std::vector<trajectory_msgs::JointTrajectoryPoint>& reversedPoints = plan.trajectory_.joint_trajectory.points;
    ros::Duration end(originalPoints.back().time_from_start);
    
    for (int i = 0; i < length; ++i) {
        for (int j = 0; j < dim; ++j) {
            reversedPoints[i].positions[j] = originalPoints[length-1-i].positions[j];
            reversedPoints[i].velocities[j] = -originalPoints[length-1-i].velocities[j];
            reversedPoints[i].accelerations[j] = -originalPoints[length-1-i].velocities[j];
            reversedPoints[i].time_from_start = end - originalPoints[length-1-i].time_from_start;
        }
    }
}

void loadParameters(sensor_msgs::JointState startState)
{
    // Get parameters
    std::string plannerId;
    if(nh_->getParam("/shared/planner_id", plannerId))
        moveGroup->setPlannerId(plannerId);
    double vs;
    nh_->param("/shared/max_velocity_scaling_factor", vs, 0.1);
    moveGroup->setMaxVelocityScalingFactor(vs);
    double as;
    nh_->param("/shared/max_acceleration_scaling_factor", as, 0.3);
    moveGroup->setMaxAccelerationScalingFactor(as);
    double d;
    nh_->param("/shared/allowed_planning_time", d, 5.0);
    moveGroup->setPlanningTime(d);
    int i;
    nh_->param("/shared/num_planning_attempts", i, 10);
    moveGroup->setNumPlanningAttempts(i);
    
    // Initialize rest of IK request
    std::vector<std::string>& joints = startState.name;
    
    mask = std::vector<bool>(joints.size(), false);
    amax = std::vector<double>();
    vmax = std::vector<double>();
    std::vector<std::string> ikJoints = moveGroup->getJointNames();
    
    for (int i = 0; i < joints.size(); ++i) {
        for (int j = 0; j < ikJoints.size(); ++j) {
            if (ikJoints[j] == joints[i]) {
                mask[i] = true;
                break;
            }
        }
        
        double vl;
        nh_->param("/robot_description_planning/joint_limits/"+joints[i]+"/max_velocity", vl, 1.0);
        vmax.push_back(vs*vl);
        
        double al;
        nh_->param("/robot_description_planning/joint_limits/"+joints[i]+"/max_acceleration", al, 1.0);
        amax.push_back(as*al);
    }
    
    Vertex::request.ik_request.robot_state.joint_state = startState;
}

bool optimizeCallback(acin_automation::OptimizeRequest& req, acin_automation::OptimizeResponse& res)
{
    loadParameters(req.start_state);
    
    std::vector<Vertex*> vertices;
    std::vector<Edge*> edges;
    
    if (!initializeGraph(vertices, edges, req.start_state, req.target_poses)) {
        ROS_ERROR("Initialization of graph failed");
        res.success = false;
        return true;
    }
    
    optimizeGraph(vertices);
    
    int planningRequests = 0;
    
    while (true) {
        Node *root = minimumSpanningTree(vertices);
        double kappa = root->cost();
        while (root->cost() < req.beta*kappa) {
            Edge *found = root->findNonExactEdge();
            if (found != NULL) {
                found->calculateExactLength();
                ++planningRequests;
            } else {
                if (root->preorderWalk(res.joint_states, res.plans)) {
                    res.total_duration = ros::Duration(root->preorderCost());
                    planningRequests += root->countLeafs() - 1;
                    res.num_planning_requests = planningRequests;
                    res.success = true;
                } else {
                    res.success = false;
                }
                
                delete root;
                deleteObjects(vertices, edges);
                
                return true;
            }
        }
        delete root;
    }
}

bool selectCallback(acin_automation::SelectRequest& req, acin_automation::SelectResponse& res)
{
    loadParameters(req.start_state);
    
    // Initialize graph
    std::vector<Vertex*> vertices;
    std::vector<Edge*> edges;
    vertices.reserve(req.target_poses.size() + 1);
    edges.reserve(req.target_poses.size());
    
    // Create vertices
    vertices.push_back(new Vertex(req.start_state));
    for (int i = 0; i < req.target_poses.size(); ++i) {
        Vertex *vertex = new Vertex(req.target_poses[i], vertices[0]->jointState.position);
        vertices.push_back(vertex);
    }
    
    // Check if all vertices are valid
    for (int i = 1; i < vertices.size(); ++i) {
        if (!vertices[i]->isValid) {
            ROS_ERROR("Initialization of graph failed");
            res.success = false;
            return true;
        }
    }
    
    // Connect vertices with edges
    for (int i = 0; i < req.target_poses.size(); ++i)
        edges.push_back(new Edge(vertices[0], vertices[i+1]));
    
    optimizeTree(vertices);
    
    while (true) {
        // Find edge with lowest cost
        Edge *minEdge;
        int index = 0;
        double min = std::numeric_limits<double>::max();
        for (int i = 0; i < edges.size(); ++i) {
            if (edges[i]->length < min) {
                min = edges[i]->length;
                index = i;
                minEdge = edges[i];
            }
        }
        
        if (minEdge->exact) {
            res.index = index;
            
            res.plan.error_code.val = res.plan.error_code.SUCCESS;
            res.plan.trajectory_start = minEdge->plan.start_state_;
            res.plan.trajectory = minEdge->plan.trajectory_;
            res.plan.planning_time = minEdge->plan.planning_time_;
            res.success = min < std::numeric_limits<double>::max();
            
            deleteObjects(vertices, edges);
            
            return true;
        } else {
            minEdge->calculateExactLength();
        }
    }
}

void optimizeGraph(std::vector<Vertex*>& vertices)
{
    Node *root = minimumSpanningTree(vertices);
    double currentCost = root->preorderCost();
    
    // Try to optimize graph until optimization fails too often
    int failureCounter = 0;
    int maxFailures = 5 * (vertices.size()-1);
    while (true) {
        // Iterate over all vertices except the root
        for (int i = 1; i < vertices.size(); ++i) {
            // Save current vertex and its edges
            std::vector<double> edgeCosts;
            vertices[i]->getEdgeCosts(edgeCosts);
            sensor_msgs::JointState jointState = vertices[i]->jointState;
            
            // Generate new robot configuration and update distances
            if (vertices[i]->inverseKinematicsSolution())
                vertices[i]->updateDistances();
            else
                continue;
            
            // Compare the new preorder walk to the previous one
            Node *newRoot = minimumSpanningTree(vertices);
            if (newRoot->preorderCost() < currentCost) {
                // Keep the new graph
                currentCost = newRoot->preorderCost();
                delete root;
                root = newRoot;
                failureCounter = 0;
            } else {
                // Recover the previous graph
                vertices[i]->setEdgeCosts(edgeCosts);
                vertices[i]->jointState = jointState;
                delete newRoot;
                ++failureCounter;
                if (failureCounter > maxFailures) {
                    delete root;
                    return;
                }
            }
        }
    }
}

void optimizeTree(std::vector<Vertex*>& vertices)
{
    // Try to optimize tree until optimization fails too often
    int failureCounter = 0;
    int maxFailures = 5 * (vertices.size()-1);
    while (true) {
        // Iterate over all vertices except the root
        for (int i = 1; i < vertices.size(); ++i) {
            // Save current vertex and its edge
            double edgeCost = vertices[i]->edges[0]->length;
            sensor_msgs::JointState jointState = vertices[i]->jointState;
            
            // Generate new robot configuration and update distance
            if (vertices[i]->inverseKinematicsSolution())
                vertices[i]->updateDistances();
            else
                continue;
            
            // Compare the new edge cost to the previous one
            double newCost = vertices[i]->edges[0]->length;
            if (newCost < edgeCost) {
                // Keep the new tree
                failureCounter = 0;
            } else {
                // Recover the previous tree
                vertices[i]->edges[0]->length = edgeCost;
                vertices[i]->jointState = jointState;
                ++failureCounter;
                if (failureCounter > maxFailures)
                    return;
            }
        }
    }
}

Node* minimumSpanningTree(std::vector<Vertex*>& vertices)
{
    int N = vertices.size();
    
    // Initialize vertices for Prime's algorithm
    vertices[0]->included = true;
    for (int i = 1; i < N; ++i) {
        vertices[i]->included = false;
        vertices[i]->keyValue = std::numeric_limits<double>::max();
    }
    
    // Create root node and update neighbors
    Node *root = new Node(vertices[0], NULL);
    vertices[0]->updateNeighbors();
    
    // Iterate over all remaining vertices
    for (int i = 1; i < N; ++i) {
        // Find vertex with minimum key value which is not included yet
        Vertex *foundVertex;
        double currentMinimum = std::numeric_limits<double>::max();
        for (int j = 1; j < N; ++j) {
            if (!(vertices[j]->included) && !(vertices[j]->keyValue > currentMinimum)) {
                foundVertex = vertices[j];
                currentMinimum = vertices[j]->keyValue;
            }
        }
        
        // Find parent node of the current vertex
        for (int k = 0; k < foundVertex->edges.size(); ++k) {
            Edge *currentEdge = foundVertex->edges[k];
            Vertex *neighbor = foundVertex->neighbor(currentEdge);
            
            if ((currentEdge->length == foundVertex->keyValue) && neighbor->included) {
                // Include current vertex in minimum spanning tree
                root->findNode(neighbor)->addChild(new Node(foundVertex, currentEdge));
                foundVertex->included = true;
                break;
            }
        }
        
        // Update neighbors for next iteration
        foundVertex->updateNeighbors();
    }
    
    return root;
}

bool initializeGraph(std::vector<Vertex*>& vertices, std::vector<Edge*>& edges,
                     sensor_msgs::JointState& startState, std::vector<geometry_msgs::PoseStamped>& poses)
{
    // Reserve memory for vertices
    int numVertices = poses.size() + 1;
    vertices.reserve(numVertices);
    
    // Reserve memory for edges
    edges.reserve((numVertices * poses.size()) / 2);
    
    // Create vertices
    vertices.push_back(new Vertex(startState));
    for (int i = 0; i < poses.size(); ++i) {
        Vertex *vertex = new Vertex(poses[i], vertices[i]->jointState.position);
        vertices.push_back(vertex);
    }
    
    // Check if all vertices are valid
    for (int i = 1; i < numVertices; ++i) {
        if (!vertices[i]->isValid)
            return false;
    }
    
    // Connect vertices with edges
    for (int i = 0; i < poses.size(); ++i) {
        for (int j = i+1; j < numVertices; ++j)
            edges.push_back(new Edge(vertices[i], vertices[j]));
    }
    
    return true;
}

// Initialize static variables
moveit_msgs::GetPositionIK::Request Vertex::request = moveit_msgs::GetPositionIK::Request();

int main(int argc, char **argv)
{
    ros::init(argc, argv, "sequence_optimizer");
    ros::NodeHandle nh;
    nh_ = &nh;
    
    // Async Spinner for MoveIt
    ros::AsyncSpinner spinner(0);
    spinner.start();
    
    // Name of planning group
    std::string groupName = "arm";
    
    // Prepare message for IK request
    Vertex::request.ik_request.group_name = groupName;
    Vertex::request.ik_request.avoid_collisions = true;
    Vertex::request.ik_request.timeout = ros::Duration(0.1);
    
    // Services
    ros::ServiceServer service1 = nh.advertiseService("optimize", optimizeCallback);
    ros::ServiceServer service2 = nh.advertiseService("select", selectCallback);
    
    ikClient = nh.serviceClient<moveit_msgs::GetPositionIK>("compute_ik");
    moveGroup = new moveit::planning_interface::MoveGroupInterface(groupName);
    
    ros::waitForShutdown();
    return 0;
}
