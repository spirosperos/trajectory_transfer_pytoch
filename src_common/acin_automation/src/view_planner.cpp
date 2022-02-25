#include <ros/ros.h>

#include <acin_automation/Select.h>
#include <acin_automation/PlanViewsAction.h>

#include <acin_reconstruction/Scan.h>
#include <acin_reconstruction/Modify.h>
#include <acin_reconstruction/BoundingBox.h>
#include <acin_reconstruction/MultiwayRegister.h>

#include <actionlib/server/simple_action_server.h>

#include <visualization_msgs/Marker.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <moveit_msgs/GetPositionIK.h>

#include <cmath>

class Ellipsoid
{
private:
    /* The private functions of this class are based on the algorithm presented in
     * "Distance from a Point to an Ellipse, an Ellipsoid, or a Hyperellipsoid"
     * by David Eberly (see https://geometrictools.com) and licensed under the
     * Creative Commons Attribution 4.0 International License. */
    
    double robustLength(double v0, double v1) const
    {
        double w0 = abs(v0);
        double w1 = abs(v1);
        
        if (w0 > w1)
            return w0 * sqrt(1 + pow(w1 / w0, 2));
        else
            return w1 * sqrt(1 + pow(w0 / w1, 2));
    }
    
    double robustLength(double v0, double v1, double v2) const
    {
        double w0 = abs(v0);
        double w1 = abs(v1);
        double w2 = abs(v2);
        
        if (w0 > w1 && w0 > w2)
            return w0 * sqrt(1 + pow(w1 / w0, 2) + pow(w2 / w0, 2));
        else if (w1 > w0 && w1 > w2)
            return w1 * sqrt(1 + pow(w0 / w1, 2) + pow(w2 / w1, 2));
        else
            return w2 * sqrt(1 + pow(w0 / w2, 2) + pow(w1 / w2, 2));
    }
    
    double getRootEllipse(double r0, double z0, double z1, double g) const
    {
        double n0 = r0 * z0;
        double s0 = z1 - 1;
        double s1 = (g < 0 ? 0 : robustLength(n0, z1) - 1);
        double s = 0;
        
        for (int i = 0; i < 50; ++i) {
            s = (s0 + s1) / 2;
            if (s == s0 || s == s1)
                break;
            double ratio0 = n0 / (s + r0);
            double ratio1 = z1 / (s + 1);
            g = pow(ratio0, 2) + pow(ratio1, 2) - 1;
            
            if (g > 0)
                s0 = s;
            else if (g < 0)
                s1 = s;
            else
                break;
        }
        
        return s;
    }
    
    double getRootEllipsoid(double r0, double r1, double z0, double z1, double z2, double g) const
    {
        double n0 = r0 * z0;
        double n1 = r1 * z1;
        double s0 = z2 - 1;
        double s1 = ( g < 0 ? 0 : robustLength(n0, n1, z2) - 1);
        double s = 0;
        
        for (int i = 0; i < 50; ++i) {
            s = (s0 + s1) / 2;
            if (s == s0 || s == s1)
                break;
            double ratio0 = n0 / (s + r0);
            double ratio1 = n1 / (s + r1);
            double ratio2 = z2 / (s + 1);
            g = pow(ratio0, 2) + pow(ratio1, 2) + pow(ratio2, 2) - 1;
            
            if (g > 0)
                s0 = s;
            else if (g < 0)
                s1 = s;
            else
                break;
        }
        
        return s;
    }
    
    double distancePointEllipse(double e0, double e1, double y0, double y1, double& x0, double& x1) const
    {
        double distance;
        if (y1 > 0) {
            if (y0 > 0) {
                double z0 = y0 / e0;
                double z1 = y1 / e1;
                double g = pow(z0, 2) + pow(z1, 2) - 1;
                if (g != 0) {
                    double r0 = pow(e0 / e1, 2);
                    double sbar = getRootEllipse(r0, z0, z1, g);
                    x0 = r0 * y0 / (sbar + r0);
                    x1 = y1 / (sbar + 1);
                    distance = sqrt(pow(x0 - y0, 2) + pow(x1 - y1, 2));
                } else {
                    x0 = y0;
                    x1 = y1;
                    distance = 0;
                }
            } else {
                x0 = 0;
                x1 = e1;
                distance = abs(y1- e1);
            }
        } else {
            double numer0 = e0 * y0;
            double denom0 = pow(e0, 2) - pow(e1, 2);
            
            if (numer0 < denom0) {
                double xde0 = numer0 / denom0;
                x0 = e0 * xde0;
                x1 = e1 * sqrt(1 - pow(xde0, 2));
                distance = sqrt(pow(x0 - y0, 2) + pow(x1, 2));
            } else {
                x0 = e0;
                x1 = 0;
                distance = abs(y0 - e0);
            }
        }
        
        return distance;
    }
    
    double distancePointEllipsoid(double y0, double y1, double y2, double& x0, double& x1, double& x2) const
    {
        double distance;
        if (y2 > 0) {
            if (y1 > 0) {
                if (y0 > 0) {
                    double z0 = y0 / e0;
                    double z1 = y1 / e1;
                    double z2 = y2 / e2;
                    double g = pow(z0, 2) + pow(z1, 2) + pow(z2, 2) - 1;
                    if (g != 0) {
                        double r0 = pow(e0 / e2, 2);
                        double r1 = pow(e1 / e2, 2);
                        double sbar = getRootEllipsoid(r0, r1, z0, z1, z2, g);
                        x0 = r0 * y0 / (sbar + r0);
                        x1 = r1 * y1 / (sbar + r1);
                        x2 = y2 / (sbar + 1);
                        distance = sqrt(pow(x0 - y0, 2) + pow(x1 - y1, 2) + pow(x2 - y2, 2));
                    } else {
                        x0 = y0;
                        x1 = y1;
                        x2 = y2;
                        distance = 0;
                    }
                } else {
                    x0 = 0;
                    distance = distancePointEllipse(e1, e2, y1, y2, x1, x2);
                }
            } else {
                if (y0 > 0) {
                    x1 = 0;
                    distance = distancePointEllipse(e0, e2, y0, y2, x0, x2);
                } else {
                    x0 = 0;
                    x1 = 0;
                    x2 = e2;
                    distance = abs(y2- e2);
                }
            }
        } else {
            double denom0 = pow(e0, 2) - pow(e2, 2);
            double denom1 = pow(e1, 2) - pow(e2, 2);
            double numer0 = e0 * y0;
            double numer1 = e1 * y1;
            bool computed = false;
            if (numer0 < denom0 && numer1 < denom1) {
                double xde0 = numer0 / denom0;
                double xde1 = numer1 / denom1;
                double xde0sqr = pow(xde0, 2);
                double xde1sqr = pow(xde1, 2);
                double discr = 1 - xde0sqr - xde1sqr;
                
                if (discr > 0) {
                    x0 = e0 * xde0;
                    x1 = e1 * xde1;
                    x2 = e2 * sqrt(discr);
                    distance = sqrt((x0 - y0) * (x0 - y0) + (x1 - y1) * (x1 - y1) + x2 * x2);
                    computed = true;
                }
            }
            
            if (!computed) {
                x2 = 0;
                distance = distancePointEllipse(e0, e1, y0, y1, x0, x1);
            }
        }
        
        return distance;
    }

public:
    Eigen::Matrix3d rotation;
    Eigen::Vector3d position;
    // Extent of ellipsoid
    double e0;
    double e1;
    double e2;
    
    Ellipsoid(const visualization_msgs::Marker& ellipsoidMsg)
    {
        Eigen::Quaterniond quaternion(ellipsoidMsg.pose.orientation.w,
                                      ellipsoidMsg.pose.orientation.x,
                                      ellipsoidMsg.pose.orientation.y,
                                      ellipsoidMsg.pose.orientation.z);
        rotation = quaternion.toRotationMatrix();
        
        position(0) = ellipsoidMsg.pose.position.x;
        position(1) = ellipsoidMsg.pose.position.y;
        position(2) = ellipsoidMsg.pose.position.z;
        
        e0 = ellipsoidMsg.scale.x / 2;
        e1 = ellipsoidMsg.scale.y / 2;
        e2 = ellipsoidMsg.scale.z / 2;
    }
    
    void calcNormal(const Eigen::Vector3d &point, Eigen::Vector3d &normal) const
    {
        normal(0) = point(0) / pow(e0, 2);
        normal(1) = point(1) / pow(e1, 2);
        normal(2) = point(2) / pow(e2, 2);
    }
    
    void calcProjection(const Eigen::Vector3d& point, Eigen::Vector3d& projection) const
    {
        double xp;
        double yp;
        double zp;
        
        Eigen::Vector3d transformedPoint = rotation.transpose() * (point - position);
        
        distancePointEllipsoid(abs(transformedPoint(0)), abs(transformedPoint(1)), abs(transformedPoint(2)), xp, yp, zp);
        
        projection(0) = transformedPoint(0) > 0 ? xp : -xp;
        projection(1) = transformedPoint(1) > 0 ? yp : -yp;
        projection(2) = transformedPoint(2) > 0 ? zp : -zp;
        
        projection = rotation * projection + position;
    }
    
    void projectViews(const std::vector<geometry_msgs::Point> &distantViews, std::vector<geometry_msgs::Point> &ellipsoidViews) const
    {
        for (int i = 0; i < distantViews.size(); ++i) {
            Eigen::Vector3d distantPoint(distantViews[i].x, distantViews[i].y, distantViews[i].z);
            Eigen::Vector3d projected;
            
            calcProjection(distantPoint, projected);
            
            geometry_msgs::Point ellipsoidPoint;
            ellipsoidPoint.x = projected(0);
            ellipsoidPoint.y = projected(1);
            ellipsoidPoint.z = projected(2);
            
            ellipsoidViews.push_back(ellipsoidPoint);
        }
    }
};

class PlanViewsAction
{
protected:
    ros::NodeHandle nh;
    actionlib::SimpleActionServer<acin_automation::PlanViewsAction> as;
    
    acin_automation::PlanViewsFeedback feedback;
    acin_automation::PlanViewsResult result;
    
    ros::ServiceClient ikClient;
    ros::ServiceClient scanClient;
    ros::ServiceClient selectClient;
    ros::ServiceClient modifyClient;
    ros::ServiceClient registerClient;
    ros::ServiceClient boundingBoxClient;
    
    ros::Publisher boundingBoxPublisher;
    ros::Publisher ellipsoidPublisher;
    ros::Publisher ellipsoidViewPublisher;
    ros::Publisher distantViewPublisher;
    
    acin_reconstruction::Scan scanSrv;
    acin_reconstruction::Modify modifySrv;
    acin_reconstruction::BoundingBox bboxSrv;
    acin_reconstruction::MultiwayRegister registerSrv;
    
    acin_automation::Select selectSrv;
    
    visualization_msgs::Marker bboxMsg;
    visualization_msgs::Marker ellipsoidMsg;
    
    visualization_msgs::Marker activeViewMsg;
    visualization_msgs::Marker plannedViewsMsg;
    visualization_msgs::Marker completedViewsMsg;
    visualization_msgs::Marker infeasibleViewsMsg;
    
    moveit_msgs::GetPositionIK ikService;
    
    moveit::planning_interface::MoveGroupInterface *moveGroup;
    
    moveit_msgs::RobotTrajectory robotTrajectory;

public:
    PlanViewsAction(std::string name) :
        as(nh, name, boost::bind(&PlanViewsAction::execute, this, _1), false)
    {
        moveGroup = new moveit::planning_interface::MoveGroupInterface("arm");
        
        ikClient = nh.serviceClient<moveit_msgs::GetPositionIK>("compute_ik");
        scanClient = nh.serviceClient<acin_reconstruction::Scan>("perform_scan");
        selectClient = nh.serviceClient<acin_automation::Select>("select");
        modifyClient = nh.serviceClient<acin_reconstruction::Modify>("modify");
        registerClient = nh.serviceClient<acin_reconstruction::MultiwayRegister>("multiway_register");
        boundingBoxClient = nh.serviceClient<acin_reconstruction::BoundingBox>("bounding_box");
        
        boundingBoxPublisher = nh.advertise<visualization_msgs::Marker>("bounding_box", 1, true);
        ellipsoidPublisher = nh.advertise<visualization_msgs::Marker>("ellipsoid", 1, true);
        ellipsoidViewPublisher = nh.advertise<visualization_msgs::Marker>("ellipsoid_views", 1, true);
        distantViewPublisher = nh.advertise<visualization_msgs::Marker>("distant_views", 1, true);
        
        // Prepare bounding box request
        bboxSrv.request.operation = bboxSrv.request.CALC;
        
        // Prepare message for bounding box
        bboxMsg.header.frame_id = "marker";
        bboxMsg.ns = "bounding_box";
        bboxMsg.type = bboxMsg.CUBE;
        bboxMsg.action = bboxMsg.ADD;
        bboxMsg.color.r = 1;
        bboxMsg.color.a = 0.4;
        
        // Prepare message for ellipsoid
        ellipsoidMsg.header.frame_id = "marker";
        ellipsoidMsg.ns = "ellipsoid";
        ellipsoidMsg.type = ellipsoidMsg.SPHERE;
        ellipsoidMsg.action = ellipsoidMsg.ADD;
        ellipsoidMsg.color.b = 1;
        ellipsoidMsg.color.a = 0.3;
        
        // Prepare messages for views
        activeViewMsg.header.frame_id = "marker";
        activeViewMsg.ns = "active_view";
        activeViewMsg.type = activeViewMsg.POINTS;
        activeViewMsg.action = activeViewMsg.ADD;
        activeViewMsg.scale.x = 0.01;
        activeViewMsg.scale.y = 0.01;
        activeViewMsg.color.a = 1;
        activeViewMsg.color.r = 1;
        activeViewMsg.color.g = 1;
        
        plannedViewsMsg.header.frame_id = "marker";
        plannedViewsMsg.ns = "planned_views";
        plannedViewsMsg.type = plannedViewsMsg.POINTS;
        plannedViewsMsg.action = plannedViewsMsg.ADD;
        plannedViewsMsg.scale.x = 0.01;
        plannedViewsMsg.scale.y = 0.01;
        plannedViewsMsg.color.a = 1;
        plannedViewsMsg.color.b = 1;
        
        completedViewsMsg.header.frame_id = "marker";
        completedViewsMsg.ns = "completed_views";
        completedViewsMsg.type = completedViewsMsg.POINTS;
        completedViewsMsg.action = completedViewsMsg.ADD;
        completedViewsMsg.scale.x = 0.01;
        completedViewsMsg.scale.y = 0.01;
        completedViewsMsg.color.a = 1;
        completedViewsMsg.color.g = 1;
        
        infeasibleViewsMsg.header.frame_id = "marker";
        infeasibleViewsMsg.ns = "infeasible_views";
        infeasibleViewsMsg.type = infeasibleViewsMsg.POINTS;
        infeasibleViewsMsg.action = infeasibleViewsMsg.ADD;
        infeasibleViewsMsg.scale.x = 0.01;
        infeasibleViewsMsg.scale.y = 0.01;
        infeasibleViewsMsg.color.a = 1;
        infeasibleViewsMsg.color.r = 1;
        
        // Prepare register request
        registerSrv.request.method = registerSrv.request.POINT_TO_PLANE_ICP;
        
        // Initialize IK request
        ikService.request.ik_request.group_name = "iiwa";
        ikService.request.ik_request.robot_state.joint_state.name = moveGroup->getJointNames();
        
        int dimension = ikService.request.ik_request.robot_state.joint_state.name.size();
        ikService.request.ik_request.robot_state.joint_state.position = std::vector<double>(dimension, 0);
        ikService.request.ik_request.avoid_collisions = true;
        ikService.request.ik_request.timeout = ros::Duration(0.1);
        
        as.start();
    }
    
    ~PlanViewsAction()
    {
        delete moveGroup;
    }
    
    void planViews(const Ellipsoid &ellipsoid, int remaining, double distance,
                   std::vector<geometry_msgs::PoseStamped> &plannedPoses,
                   std::vector<geometry_msgs::Point> &plannedDistantViews,
                   std::vector<geometry_msgs::Point> &plannedEllipsoidViews,
                   std::vector<geometry_msgs::Point> &completedEllipsoidViews,
                   std::vector<geometry_msgs::Point> &infeasibleDistantViews,
                   std::vector<geometry_msgs::Point> &infeasibleEllipsoidViews)
    {
        // Find nearest points on surface for completed views
        std::vector<Eigen::Vector3d> points;
        for (int i = 0; i < completedEllipsoidViews.size(); ++i) {
            Eigen::Vector3d point(completedEllipsoidViews[i].x,
                                  completedEllipsoidViews[i].y,
                                  completedEllipsoidViews[i].z);
            point = ellipsoid.rotation.transpose() * (point - ellipsoid.position);
            point(0) = point(0) / ellipsoid.e0;
            point(1) = point(1) / ellipsoid.e1;
            point(2) = point(2) / ellipsoid.e2;
            
            points.push_back(point);
        }
        
        // Find nearest points on surface for infeasible views
        for (int i = 0; i < infeasibleEllipsoidViews.size(); ++i) {
            Eigen::Vector3d point(infeasibleEllipsoidViews[i].x,
                                  infeasibleEllipsoidViews[i].y,
                                  infeasibleEllipsoidViews[i].z);
            point = ellipsoid.rotation.transpose() * (point - ellipsoid.position);
            point(0) = point(0) / ellipsoid.e0;
            point(1) = point(1) / ellipsoid.e1;
            point(2) = point(2) / ellipsoid.e2;
            
            points.push_back(point);
        }
        
        // Plan enough views
        while (plannedPoses.size() < remaining) {
            // Generate preliminary samples
            int size = 200;
            std::vector<Eigen::Vector3d> samples(size);
            std::vector<double> costs(size);
            
            for (int i = 0; i < size; ++i) {
                samples[i] = Eigen::Vector3d::Random();
                samples[i].normalize();
                
                // Calculate cost of point
                costs[i] = 0;
                for (int j = 0; j < points.size(); ++j)
                    // Cost function with regularization term
                    costs[i] += 1 / (pow((points[j]-samples[i]).norm(), 2) + 0.00001);
            }
            
            // Select preliminary sample with smallest cost
            int idx = std::min_element(costs.begin(), costs.end()) - costs.begin();
            
            Eigen::Vector3d spherePoint = samples[idx];
            points.push_back(samples[idx]);
            
            Eigen::Vector3d surfacePosition(ellipsoid.e0 * spherePoint(0),
                                            ellipsoid.e1 * spherePoint(1),
                                            ellipsoid.e2 * spherePoint(2));
            
            Eigen::Vector3d normal;
            ellipsoid.calcNormal(surfacePosition, normal);
            normal.normalize();
            
            Eigen::Vector3d cameraPosition = surfacePosition + distance * normal;
            
            // Calculate quaternion of camera rotation
            cameraPosition = ellipsoid.rotation * cameraPosition + ellipsoid.position;
            surfacePosition = ellipsoid.rotation * surfacePosition + ellipsoid.position;
            Eigen::Vector3d direction = - ellipsoid.rotation * normal;
            Eigen::Vector3d down(0, 0, -1);
            
            Eigen::Vector3d xaxis = down.cross(direction);
            xaxis.normalize();
            
            Eigen::Vector3d yaxis = direction.cross(xaxis);
            yaxis.normalize();
            
            Eigen::Matrix3d cameraRotation;
            cameraRotation.row(0) = xaxis;
            cameraRotation.row(1) = yaxis;
            cameraRotation.row(2) = direction;
            
            Eigen::Quaterniond cameraQuaternion(cameraRotation.transpose());
            
            // Prepare camera pose
            geometry_msgs::PoseStamped cameraPose;
            cameraPose.header.frame_id = "marker";
            cameraPose.pose.position.x = cameraPosition(0);
            cameraPose.pose.position.y = cameraPosition(1);
            cameraPose.pose.position.z = cameraPosition(2);
            cameraPose.pose.orientation.w = cameraQuaternion.w();
            cameraPose.pose.orientation.x = cameraQuaternion.x();
            cameraPose.pose.orientation.y = cameraQuaternion.y();
            cameraPose.pose.orientation.z = cameraQuaternion.z();
            
            // Check if camera is above marker board
            if (cameraPose.pose.position.z > 0) {
                // Compute IK solution
                ikService.request.ik_request.pose_stamped = cameraPose;
                ikClient.call(ikService);
            } else {
                ikService.response.error_code.val = ikService.response.error_code.FAILURE;
            }
            
            if (ikService.response.error_code.val == ikService.response.error_code.SUCCESS) {
                plannedPoses.push_back(cameraPose);
                plannedDistantViews.push_back(cameraPose.pose.position);
                geometry_msgs::Point point;
                point.x = surfacePosition(0);
                point.y = surfacePosition(1);
                point.z = surfacePosition(2);
                plannedEllipsoidViews.push_back(point);
            } else {
                infeasibleDistantViews.push_back(cameraPose.pose.position);
                geometry_msgs::Point point;
                point.x = surfacePosition(0);
                point.y = surfacePosition(1);
                point.z = surfacePosition(2);
                infeasibleEllipsoidViews.push_back(point);
            }
        }
    }
    
    void execute(const acin_automation::PlanViewsGoalConstPtr &goal)
    {
        std::vector<geometry_msgs::Point> completedDistantViews;
        std::vector<geometry_msgs::Point> infeasibleDistantViews;
        
        std::vector<std::string> fragments;
        fragments.push_back("init");
        registerSrv.request.fragments = fragments;
        
        // Delete all existing scans
        modifySrv.request.operation = modifySrv.request.DELETE_ALL;
        modifyClient.call(modifySrv);
        
        // Get parameters
        std::string plannerId;
        if(nh.getParam("/shared/planner_id", plannerId))
            moveGroup->setPlannerId(plannerId);
        double vel;
        nh.param("/shared/max_velocity_scaling_factor", vel, 0.1);
        moveGroup->setMaxVelocityScalingFactor(vel);
        double acc;
        nh.param("/shared/max_acceleration_scaling_factor", acc, 0.3);
        moveGroup->setMaxAccelerationScalingFactor(acc);
        double d;
        nh.param("/shared/allowed_planning_time", d, 5.0);
        moveGroup->setPlanningTime(d);
        int i;
        nh.param("/shared/num_planning_attempts", i, 10);
        moveGroup->setNumPlanningAttempts(i);
        
        // Save current state
        moveit::core::RobotStatePtr startState = moveGroup->getCurrentState();
        
        // Calculate progress step
        double progressStep = 1.0 / (4 * goal->scans + 5);
        
        // Send feedback
        feedback.progress = progressStep;
        feedback.message = "Performing initial scan...";
        as.publishFeedback(feedback);
        
        // Make initial scan
        scanSrv.request.name = "init";
        if (!scanClient.call(scanSrv) || !scanSrv.response.success) {
            as.setPreempted();
            return;
        }

        modifySrv.request.operation = modifySrv.request.COLOR;
        modifySrv.request.fragment = "init";
        modifySrv.request.color.r = 1;
        modifySrv.request.color.g = 1;
        modifySrv.request.color.b = 1;
        modifySrv.request.color.a = 1;
        modifySrv.request.custom_color = true;
        modifyClient.call(modifySrv);
        modifySrv.request.operation = modifySrv.request.SHOW;
        modifyClient.call(modifySrv);
        
        // Make all other scans
        for (int i = 0; i < goal->scans; ++i) {
            // Send feedback
            feedback.progress += progressStep;
            feedback.message = "Planning views...";
            as.publishFeedback(feedback);
            
            // Get bounding box
            if (i == 0)
                bboxSrv.request.fragment = scanSrv.request.name;
            else
                bboxSrv.request.fragment = "reconstruction";
            
            if (!boundingBoxClient.call(bboxSrv) || !bboxSrv.response.success) {
                feedback.message = "Obtaining bounding box failed";
                as.publishFeedback(feedback);
                as.setPreempted();
                return;
            }
            
            // Visualize bounding box
            bboxMsg.pose = bboxSrv.response.pose;
            bboxMsg.scale = bboxSrv.response.extent;
            boundingBoxPublisher.publish(bboxMsg);
            
            // Visualize ellipsoid
            ellipsoidMsg.pose = bboxSrv.response.pose;
            ellipsoidMsg.scale.x = std::sqrt(3) * bboxSrv.response.extent.x;
            ellipsoidMsg.scale.y = std::sqrt(3) * bboxSrv.response.extent.y;
            ellipsoidMsg.scale.z = std::sqrt(3) * bboxSrv.response.extent.z;
            ellipsoidPublisher.publish(ellipsoidMsg);
            
            // Create ellipsoid object
            Ellipsoid ellipsoid(ellipsoidMsg);
            
            // Project completed and infeasible views onto new ellipsoid
            std::vector<geometry_msgs::Point> completedEllipsoidViews;
            std::vector<geometry_msgs::Point> infeasibleEllipsoidViews;
            ellipsoid.projectViews(completedDistantViews, completedEllipsoidViews);
            ellipsoid.projectViews(infeasibleDistantViews, infeasibleEllipsoidViews);
            
            // Visualize completed and infeasible views
            completedViewsMsg.points = completedDistantViews;
            distantViewPublisher.publish(completedViewsMsg);
            infeasibleViewsMsg.points = infeasibleDistantViews;
            distantViewPublisher.publish(infeasibleViewsMsg);
            
            completedViewsMsg.points = completedEllipsoidViews;
            ellipsoidViewPublisher.publish(completedViewsMsg);
            infeasibleViewsMsg.points = infeasibleEllipsoidViews;
            ellipsoidViewPublisher.publish(infeasibleViewsMsg);
            
            // Plan views
            std::vector<geometry_msgs::Point> plannedDistantViews;
            std::vector<geometry_msgs::Point> plannedEllipsoidViews;
            std::vector<geometry_msgs::PoseStamped> plannedPoses;
            planViews(ellipsoid,  goal->scans - i, goal->distance, plannedPoses,
                      plannedDistantViews, plannedEllipsoidViews,
                      completedEllipsoidViews,
                      infeasibleDistantViews, infeasibleEllipsoidViews);
            
            // Visualize planned and infeasible views
            plannedViewsMsg.points = plannedDistantViews;
            distantViewPublisher.publish(plannedViewsMsg);
            infeasibleViewsMsg.points = infeasibleDistantViews;
            distantViewPublisher.publish(infeasibleViewsMsg);
            
            plannedViewsMsg.points = plannedEllipsoidViews;
            ellipsoidViewPublisher.publish(plannedViewsMsg);
            infeasibleViewsMsg.points = infeasibleEllipsoidViews;
            ellipsoidViewPublisher.publish(infeasibleViewsMsg);
            
            // Check if preempt requested
            if (as.isPreemptRequested() || !ros::ok()) {
                as.setPreempted();
                return;
            }
            
            // Send feedback
            feedback.progress += progressStep;
            feedback.message = "Selecting view...";
            as.publishFeedback(feedback);
            
            // Select a view and plan motion
            selectSrv.request.start_state.name = moveGroup->getJointNames();
            selectSrv.request.start_state.position = moveGroup->getCurrentJointValues();
            selectSrv.request.target_poses = plannedPoses;
            if (!selectClient.call(selectSrv) || !selectSrv.response.success) {
                feedback.message = "Selection failed";
                as.publishFeedback(feedback);
                as.setPreempted();
                return;
            }
            
            // Check if preempt requested
            if (as.isPreemptRequested() || !ros::ok()) {
                as.setPreempted();
                return;
            }
            
            // Set corresponding view active
            geometry_msgs::Point activeDistantView = plannedDistantViews[selectSrv.response.index];
            geometry_msgs::Point activeEllipsoidView = plannedEllipsoidViews[selectSrv.response.index];
            
            // Visualize planned views without active view
            if (i < goal->scans -1) {
                plannedDistantViews.erase(plannedDistantViews.begin() + selectSrv.response.index);
                plannedViewsMsg.points = plannedDistantViews;
                distantViewPublisher.publish(plannedViewsMsg);
                
                plannedEllipsoidViews.erase(plannedEllipsoidViews.begin() + selectSrv.response.index);
                plannedViewsMsg.points = plannedEllipsoidViews;
                ellipsoidViewPublisher.publish(plannedViewsMsg);
            } else {
                plannedViewsMsg.action = plannedViewsMsg.DELETE;
                distantViewPublisher.publish(plannedViewsMsg);
                ellipsoidViewPublisher.publish(plannedViewsMsg);
                plannedViewsMsg.action = plannedViewsMsg.ADD;
            }
            
            // Visualize active view
            activeViewMsg.points.clear();
            activeViewMsg.points.push_back(activeDistantView);
            distantViewPublisher.publish(activeViewMsg);
            
            activeViewMsg.points.clear();
            activeViewMsg.points.push_back(activeEllipsoidView);
            ellipsoidViewPublisher.publish(activeViewMsg);
            
            // Send feedback
            feedback.progress += progressStep;
            feedback.message = "Moving...";
            as.publishFeedback(feedback);
            
            // Move camera
            robotTrajectory.joint_trajectory = selectSrv.response.plan.trajectory.joint_trajectory;
            moveGroup->execute(robotTrajectory);
            
            // Check if preempt requested
            if (as.isPreemptRequested() || !ros::ok()) {
                as.setPreempted();
                return;
            }
            
            // Wait 2 seconds
            ros::Duration duration(2);
            duration.sleep();
            
            // Send feedback
            feedback.progress += progressStep;
            feedback.message = "Scanning...";
            as.publishFeedback(feedback);
            
            // Scan object
            scanSrv.request.name = "scan" + std::to_string(i);
            if (!scanClient.call(scanSrv) || !scanSrv.response.success) {
                feedback.message = "Scanning failed";
                as.publishFeedback(feedback);
                as.setPreempted();
                return;
            } else {
                // Remove active view
                activeViewMsg.action = activeViewMsg.DELETE;
                distantViewPublisher.publish(activeViewMsg);
                ellipsoidViewPublisher.publish(activeViewMsg);
                activeViewMsg.action = activeViewMsg.ADD;
                
                // Visualize completed views
                completedDistantViews.push_back(activeDistantView);
                completedViewsMsg.points = completedDistantViews;
                distantViewPublisher.publish(completedViewsMsg);
                
                completedEllipsoidViews.push_back(activeEllipsoidView);
                completedViewsMsg.points = completedEllipsoidViews;
                ellipsoidViewPublisher.publish(completedViewsMsg);
            }
            
            if (i == 0) {
                modifySrv.request.operation = modifySrv.request.HIDE;
                modifySrv.request.fragment = "init";
                modifyClient.call(modifySrv);
            }
            
            // Add scan to reconstruction
            modifySrv.request.operation = modifySrv.request.ADD;
            modifySrv.request.fragment = scanSrv.request.name;
            modifySrv.request.target = "reconstruction";
            modifyClient.call(modifySrv);
            modifySrv.request.operation = modifySrv.request.COLOR;
            modifySrv.request.fragment = "reconstruction";
            modifySrv.request.color.r = 1;
            modifySrv.request.color.g = 1;
            modifySrv.request.color.b = 1;
            modifySrv.request.color.a = 1;
            modifySrv.request.custom_color = true;
            modifyClient.call(modifySrv);
            modifySrv.request.operation = modifySrv.request.SHOW;
            modifyClient.call(modifySrv);
            
            // Add scan to list for registration
            registerSrv.request.fragments.push_back(scanSrv.request.name);
        }
        
        // Send feedback
        feedback.progress += progressStep;
        feedback.message = "Moving to start...";
        as.publishFeedback(feedback);
        
        // Move robot to start state
        moveGroup->setJointValueTarget(*startState);
        
        // Try to find a solution several times
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success = false;
        for (int i = 0; i < 100; ++i) {
            if (moveGroup->plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS) {
                success = true;
                break;
            }
        }
        
        if (success) {
            moveGroup->execute(plan);
        } else {
            feedback.message = "No motion plan found";
            as.publishFeedback(feedback);
        }
        
        // Send feedback
        feedback.progress += progressStep;
        feedback.message = "Registering...";
        as.publishFeedback(feedback);
        
        // Register scans (iteration 1)
        registerSrv.request.threshold = 0.003;
        if (!registerClient.call(registerSrv) || !registerSrv.response.success) {
            feedback.message = "Registration failed";
            as.publishFeedback(feedback);
            as.setPreempted();
            return;
        }
        
        // Register scans (iteration 2)
        registerSrv.request.threshold = 0.0003;
        if (!registerClient.call(registerSrv) || !registerSrv.response.success) {
            feedback.message = "Registration failed";
            as.publishFeedback(feedback);
            as.setPreempted();
            return;
        }
        
        // Send feedback
        feedback.progress += progressStep;
        feedback.message = "Merging scans...";
        as.publishFeedback(feedback);
        
        // Delete old reconstruction
        modifySrv.request.operation = modifySrv.request.HIDE;
        modifySrv.request.fragment = "reconstruction";
        modifyClient.call(modifySrv);
        modifySrv.request.operation = modifySrv.request.DELETE;
        modifySrv.request.fragment = "reconstruction";
        modifyClient.call(modifySrv);
        
        // Add all scans to reconstructed object (except initial scan)
        for (int j = 1; j < registerSrv.request.fragments.size(); ++j) {
            modifySrv.request.operation = modifySrv.request.ADD;
            modifySrv.request.fragment = registerSrv.request.fragments[j];
            modifySrv.request.target = "reconstruction";
            modifyClient.call(modifySrv);
        }
        
        // Downsample reconstructed object
        modifySrv.request.operation = modifySrv.request.DOWNSAMPLE;
        modifySrv.request.voxel_size = 0.0005;
        modifySrv.request.fragment = "reconstruction";
        modifyClient.call(modifySrv);
        modifySrv.request.operation = modifySrv.request.COLOR;
        modifySrv.request.fragment = "reconstruction";
        modifySrv.request.color.r = 1;
        modifySrv.request.color.g = 1;
        modifySrv.request.color.b = 1;
        modifySrv.request.color.a = 1;
        modifySrv.request.custom_color = true;
        modifyClient.call(modifySrv);
        modifySrv.request.operation = modifySrv.request.SHOW;
        modifySrv.request.fragment = "reconstruction";
        modifyClient.call(modifySrv);
        
        // Send feedback
        feedback.progress = 1;
        feedback.message = "Process finished";
        as.publishFeedback(feedback);
        
        // Finish
        as.setSucceeded(result);
    }
};

int main(int argc, char **argv)
{
    // Set up ROS
    ros::init(argc, argv, "view_planner");
    PlanViewsAction viewPlanner("plan_views");
    ros::spin();
    
    return 0;
}
