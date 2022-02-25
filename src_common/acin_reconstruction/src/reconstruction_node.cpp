#include "ros/ros.h"
#include <open3d/Open3D.h>
#include <open3d/pipelines/registration/GlobalOptimization.h>
#define OPENCV_TRAITS_ENABLE_DEPRECATED
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/aruco/charuco.hpp>

#include "std_msgs/String.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/point_cloud2_iterator.h"
#include "std_srvs/Trigger.h"
#include "geometry_msgs/Transform.h"
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/Pose.h"

#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <acin_reconstruction/Connect.h>
#include <acin_reconstruction/Load.h>
#include <acin_reconstruction/Scan.h>
#include <acin_reconstruction/Modify.h>
#include <acin_reconstruction/Export.h>
#include <acin_reconstruction/ScanList.h>
#include <acin_reconstruction/Register.h>
#include <acin_reconstruction/Transform.h>
#include <acin_reconstruction/MultiwayRegister.h>
#include <acin_reconstruction/BoundingBox.h>
#include <acin_reconstruction/SetReference.h>

#include "tf2/transform_datatypes.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include "parameters.h"

#include <Eigen/Geometry>
#include <algorithm>

#include <list>
#include <vector>
#include <stdlib.h>
#include <iostream>
#include <fstream>


#define PHOXI_OPENCV_SUPPORT
#include "PhoXi.h"

typedef open3d::geometry::PointCloud PointCloud_t;
typedef open3d::geometry::RGBDImage RGBDImage_t;
class Scan;

// Global variables
sensor_msgs::ImageConstPtr tempColor;
sensor_msgs::ImageConstPtr tempDepth;
std::list<Scan> scanList;
pho::api::PPhoXi phoXiDevice;
ros::Publisher pointCloudPub;
ros::Publisher scanListPub;
ros::NodeHandle *ptr_nh;
bool connected;
geometry_msgs::TransformStamped markerFrame;
tf2_ros::Buffer tfBuffer;

bool (*scan)(cv::Mat& color, cv::Mat& depth);
bool (*connectROS)(acin_reconstruction::Connect::Request& req, acin_reconstruction::Connect::Response& res);

class Scan;
bool publish_pointcloud(const PointCloud_t&);
Scan *findScan(std::string);
void insertScan(Scan& scan);
bool multiwayRegister(std::vector<std::string> names, int method, double threshold);

bool estimatePose(const cv::Mat& image, const CameraParameters& params,
                  const cv::Ptr<cv::aruco::CharucoBoard> board, geometry_msgs::Pose& pose);
bool estimatePose(const cv::Mat& image, const CameraParameters& params,
                  const cv::Ptr<cv::aruco::CharucoBoard> board, cv::Vec3d& rvec, cv::Vec3d& tvec);
bool estimatePose(const cv::Mat& image, const CameraParameters& params,
                  const cv::Ptr<cv::aruco::CharucoBoard> board, Eigen::Matrix3d& rmat, Eigen::Vector3d& tvec);

class Scan {
    public:
        static CameraParameters cameraParameters;
        static ProcessingParameters processingParameters;
        static cv::Ptr<cv::aruco::CharucoBoard> board;
        static open3d::geometry::AxisAlignedBoundingBox boundingBox;
    
    private:
        std::shared_ptr<PointCloud_t> sptr_pointCloud;
        Eigen::Vector3d color;
        std::string name;
        bool customColor;
        bool show;
        
    public:
        Scan(const cv::Mat& cvColorImage, const cv::Mat& cvDepthImage, const std::string& name,
             const geometry_msgs::Transform transform, std::string pathPose) {
            this->name = name;
            show = false;
            customColor = false;
            
            // Initialize color randomly
            color(0) = (rand() % 256) / 255.0;
            color(1) = (rand() % 256) / 255.0;
            color(2) = (rand() % 256) / 255.0;
            
            // Create RGBD image
            open3d::geometry::Image imageColor;
            open3d::geometry::Image imageDepth;
            cv::Size size;
            int channels;
            
            cv::Mat cvColorImageConverted;
            channels = cvColorImage.channels();
            if (channels > 1) {
                cv::cvtColor(cvColorImage, cvColorImageConverted, cv::COLOR_BGR2RGB);
            } else {
                cv::cvtColor(cvColorImage, cvColorImageConverted, cv::COLOR_GRAY2RGB);
                channels = 3;
            }
            size = cvColorImageConverted.size();
            
            imageColor.Prepare(size.width, size.height, channels, cvColorImageConverted.elemSize1());
            memcpy(imageColor.data_.data(), &cvColorImageConverted.data[0], imageColor.data_.size());
            
            channels = cvDepthImage.channels();
            imageDepth.Prepare(size.width, size.height, channels, cvDepthImage.elemSize1());
            memcpy(imageDepth.data_.data(), &cvDepthImage.data[0], imageDepth.data_.size());
            
            std::shared_ptr<RGBDImage_t> rgbdImage =
                open3d::geometry::RGBDImage::CreateFromColorAndDepth(imageColor, imageDepth, 1000.0, 3.0, false);
            
            open3d::camera::PinholeCameraIntrinsic intrinsic;
            cameraParameters.getIntrinsic(size.width, size.height, intrinsic);
            sptr_pointCloud = PointCloud_t::CreateFromRGBDImage(*rgbdImage, intrinsic);
            
            // Downsample point cloud
            sptr_pointCloud = sptr_pointCloud->VoxelDownSample(0.001);
            
            // Set default transformation of point cloud
            Eigen::Quaterniond quaternion(transform.rotation.w,
                                          transform.rotation.x,
                                          transform.rotation.y,
                                          transform.rotation.z);
            Eigen::Matrix3d rotation = quaternion.toRotationMatrix();
            Eigen::Vector3d translation(transform.translation.x,
                                        transform.translation.y,
                                        transform.translation.z);
            
            // Detect markers and estimate pose
            bool success = estimatePose(cvColorImageConverted, cameraParameters, board, rotation, translation);
            
            // Transform point cloud based on estimated pose
            // (or default transformation in case estimation failed)
            sptr_pointCloud->Translate(-translation);
            Eigen::Vector3d center(0, 0, 0);
            sptr_pointCloud->Rotate(rotation.transpose(), center);
            
            // Crop point cloud
            if (!boundingBox.IsEmpty())
                sptr_pointCloud = sptr_pointCloud->Crop(boundingBox);
            
            // Remove outliers
            std::tuple<std::shared_ptr<PointCloud_t>, std::vector<size_t>> result;
            
            if (Scan::processingParameters.radiusOutlierRemoval.enable) {
                result = sptr_pointCloud->RemoveRadiusOutliers(Scan::processingParameters.radiusOutlierRemoval.neighbors,
                                                               Scan::processingParameters.radiusOutlierRemoval.searchRadius);
                sptr_pointCloud = std::get<0>(result);
            }
            
            if (Scan::processingParameters.statisticalOutlierRemoval.enable) {
                result = sptr_pointCloud->RemoveStatisticalOutliers(Scan::processingParameters.statisticalOutlierRemoval.neighbors,
                                                                    Scan::processingParameters.statisticalOutlierRemoval.stdRatio);
                sptr_pointCloud = std::get<0>(result);
            }
            
            // Estimate normals
            sptr_pointCloud->EstimateNormals();
            
            // Save pose to file
            if (pathPose != "") {
                std::ofstream out(pathPose);
                out << "measured_pose:" << std::endl;
                out << "  translation:" << std::endl;
                out << "    x: " << transform.translation.x << std::endl;
                out << "    y: " << transform.translation.y << std::endl;
                out << "    z: " << transform.translation.z << std::endl;
                out << "  rotation:" << std::endl;
                out << "    x: " << transform.rotation.x << std::endl;
                out << "    y: " << transform.rotation.y << std::endl;
                out << "    z: " << transform.rotation.z << std::endl;
                out << "    w: " << transform.rotation.w << std::endl;
                out << "estimated_pose:" << std::endl;
                if (success) {
                    Eigen::Quaterniond quaternion(rotation);
                    out << "  translation:" << std::endl;
                    out << "    x: " << translation[0] << std::endl;
                    out << "    y: " << translation[1] << std::endl;
                    out << "    z: " << translation[2] << std::endl;
                    out << "  rotation:" << std::endl;
                    out << "    x: " << quaternion.x() << std::endl;
                    out << "    y: " << quaternion.y() << std::endl;
                    out << "    z: " << quaternion.z() << std::endl;
                    out << "    w: " << quaternion.w() << std::endl;
                }
                out.close();
            }
        }
        
        Scan(const std::string& name, std::shared_ptr<PointCloud_t> sptr_pcd) {
            this->name = name;
            show = false;
            customColor = false;
            
            // Initialize color randomly
            color(0) = (rand() % 256) / 255.0;
            color(1) = (rand() % 256) / 255.0;
            color(2) = (rand() % 256) / 255.0;
            
            // Store point cloud
            sptr_pointCloud = sptr_pcd;
            
            // Estimate normals
            sptr_pointCloud->EstimateNormals();
        }
        
        ~Scan() {
        }
        
        void setColor(const Eigen::Vector3d& color) {
            this->color(0) = color(0);
            this->color(1) = color(1);
            this->color(2) = color(2);
        }
        
        std::shared_ptr<PointCloud_t> getPointCloud() {
            return sptr_pointCloud;
        }
        
        std::shared_ptr<PointCloud_t> getColoredPointCloud() {
            if (customColor) {
                auto sptr_pcd = std::make_shared<PointCloud_t>(sptr_pointCloud->points_);
                sptr_pcd->PaintUniformColor(color);
                return sptr_pcd;
            } else {
                return sptr_pointCloud;
            }
        }
        
        std::string getName() {
            return name;
        }
        
        bool isShown() {
            return show;
        }
        
        void setShow(bool value) {
            show = value;
        }
        
        void setCustom(bool value) {
            customColor = value;
        }
        
        void setPointCloud(std::shared_ptr<PointCloud_t> sptr_pointCloud) {
            this->sptr_pointCloud = sptr_pointCloud;
        }
        
        static void addBoundingBox(Eigen::Vector3d lowerBound, Eigen::Vector3d upperBound) {
            open3d::geometry::AxisAlignedBoundingBox bBox(lowerBound, upperBound);
            boundingBox = bBox;
        }
        
        static void removeBoundingBox() {
            boundingBox.Clear();
        }
        
        int operator ==(Scan scan) {
            if (scan.getName() == name)
                return 1;
            else
                return 0;
        }
};

// Initialize static variables
CameraParameters Scan::cameraParameters = CameraParameters();
ProcessingParameters Scan::processingParameters = ProcessingParameters();
cv::Ptr<cv::aruco::CharucoBoard> Scan::board = cv::Ptr<cv::aruco::CharucoBoard>();
open3d::geometry::AxisAlignedBoundingBox Scan::boundingBox = open3d::geometry::AxisAlignedBoundingBox();


bool estimatePose(const cv::Mat& image, const CameraParameters& params,
                  const cv::Ptr<cv::aruco::CharucoBoard> board, geometry_msgs::Pose& pose)
{
    Eigen::Matrix3d rmat;
    Eigen::Vector3d tvec;
    
    if (estimatePose(image, params, board, rmat, tvec)) {
        Eigen::Quaterniond quaternion(rmat);
        pose.orientation.w = quaternion.w();
        pose.orientation.x = quaternion.x();
        pose.orientation.y = quaternion.y();
        pose.orientation.z = quaternion.z();
        pose.position.x = tvec[0];
        pose.position.y = tvec[1];
        pose.position.z = tvec[2];
        return true;
    } else {
        return false;
    }
}

bool estimatePose(const cv::Mat& image, const CameraParameters& params,
                  const cv::Ptr<cv::aruco::CharucoBoard> board, cv::Vec3d& rvec, cv::Vec3d& tvec)
{
    // Detect AruCo markers
    std::vector<int> markerIds;
    std::vector<std::vector<cv::Point2f>> markerCorners;
    std::vector<std::vector<cv::Point2f>> rejectedCorners;
    cv::Ptr<cv::aruco::DetectorParameters> detectionParameters = cv::aruco::DetectorParameters::create();
    cv::aruco::detectMarkers(image, board->dictionary, markerCorners, markerIds, detectionParameters, rejectedCorners);
    
    if (markerIds.size() > 0) {
        // Try to refind rejected corners
        cv::aruco::refineDetectedMarkers(image, board, markerCorners, markerIds,
                                         rejectedCorners, params.cameraMatrix, params.distCoeffs);
        
        // Get ChArUco corners
        std::vector<int> charucoIds;
        std::vector<cv::Point2f> charucoCorners;
        cv::aruco::interpolateCornersCharuco(markerCorners, markerIds, image,
                                             board, charucoCorners, charucoIds,
                                             params.cameraMatrix, params.distCoeffs);
        if (charucoIds.size() > 0) {
            // Estimate pose
            bool success = cv::aruco::estimatePoseCharucoBoard(charucoCorners, charucoIds, board,
                                                               params.cameraMatrix, params.distCoeffs, rvec, tvec);
            if (success) {
                ROS_INFO("Pose estimation successful");
                return true;
            } else {
                ROS_ERROR("No pose estimated");
            }
        } else {
            ROS_ERROR("No corners detected");
        }
    } else {
        ROS_ERROR("No markers detected");
    }
    
    return false;
}

bool estimatePose(const cv::Mat& image, const CameraParameters& params,
                  const cv::Ptr<cv::aruco::CharucoBoard> board, Eigen::Matrix3d& rmat, Eigen::Vector3d& tvec)
{
    cv::Vec3d rvecCv;
    cv::Vec3d tvecCv;
    
    if (estimatePose(image, params, board, rvecCv, tvecCv)) {
        cv::Mat rmatCv;
        cv::Rodrigues(rvecCv, rmatCv);
        cv::cv2eigen(rmatCv, rmat);
        cv::cv2eigen(tvecCv, tvec);
        return true;
    } else {
        return false;
    }
}

Scan *findScan(std::string name)
{
    std::list<Scan>::iterator it;
    for (it = scanList.begin(); it != scanList.end(); ++it) {
        if (it->getName() == name)
            return &(*it);
    }
    return NULL;
}

void insertScan(Scan& scan)
{
    // Delete all scans with equal name first
    Scan* found = findScan(scan.getName());
    if (found != NULL)
        scanList.remove(*found);
    
    // Add scan to list
    scanList.push_back(scan);
}

void publishScanList() {
    acin_reconstruction::ScanList nameList;
    
    // Iterate over list
    std::list<Scan>::iterator it;
    for (it = scanList.begin(); it != scanList.end(); ++it)
        nameList.scans.push_back(it->getName());
    
    // Send name list to ROS
    scanListPub.publish(nameList);
}

void publishPointClouds()
{
    PointCloud_t pcd;
    
    // Iterate over list
    std::list<Scan>::iterator it;
    for (it = scanList.begin(); it != scanList.end(); ++it) {
        if (it->isShown())
            pcd += *(it->getColoredPointCloud());
    }
    
    // Read data
    std::vector<Eigen::Vector3d> points = pcd.points_;
    std::vector<Eigen::Vector3d> colors = pcd.colors_;
    size_t len = points.size();
    
    // Create message
    sensor_msgs::PointCloud2 pointCloudMsg;
    pointCloudMsg.header.stamp = ros::Time::now();
    pointCloudMsg.header.frame_id = "marker";
    pointCloudMsg.is_bigendian = false;
    pointCloudMsg.is_dense = true;
    pointCloudMsg.height = 1;
    pointCloudMsg.width = len;
    pointCloudMsg.point_step = 16;
    pointCloudMsg.row_step = pointCloudMsg.point_step*pointCloudMsg.width;
    
    // Set fields of point cloud
    sensor_msgs::PointCloud2Modifier modifier(pointCloudMsg);
    modifier.setPointCloud2Fields(4, "x", 1, sensor_msgs::PointField::FLOAT32,
                                     "y", 1, sensor_msgs::PointField::FLOAT32,
                                     "z", 1, sensor_msgs::PointField::FLOAT32,
                                     "rgb", 1, sensor_msgs::PointField::FLOAT32);
    modifier.resize(pointCloudMsg.height*pointCloudMsg.width);
    
    // Create iterators for position and color
    sensor_msgs::PointCloud2Iterator<float> iter_x(pointCloudMsg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(pointCloudMsg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(pointCloudMsg, "z");
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(pointCloudMsg, "r");
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(pointCloudMsg, "g");
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(pointCloudMsg, "b");
    
    // Fill in data
    for (size_t i = 0; i < len; ++i) {
        *iter_x = points[i](0);
        *iter_y = points[i](1);
        *iter_z = points[i](2);
        *iter_r = colors[i](0)*255;
        *iter_g = colors[i](1)*255;
        *iter_b = colors[i](2)*255;
        ++iter_x;
        ++iter_y;
        ++iter_z;
        ++iter_r;
        ++iter_g;
        ++iter_b;
    }
    pointCloudPub.publish(pointCloudMsg);
}

void convertImage(cv::Mat& cvImage, const sensor_msgs::ImageConstPtr& rosImage, bool isDepth)
{
    if (isDepth) {
        int rows = rosImage->height;
        int cols = rosImage->width;
        void *data = (void*) &(rosImage->data[0]);
        cv::Mat cvImg(rows, cols, CV_32F, data);
        cvImg.convertTo(cvImage, CV_32F, 1000.0);
    } else {
        cvImage = cv_bridge::toCvShare(rosImage)->image;
    }
}

void syncCallback(const sensor_msgs::ImageConstPtr& colorImage, const sensor_msgs::ImageConstPtr& depthImage)
{
    tempColor = colorImage;
    tempDepth = depthImage;
}

bool displayScansROS(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
{
    publishPointClouds();
}

bool loadScanROS(acin_reconstruction::Load::Request& req, acin_reconstruction::Load::Response& res)
{
    if (req.type == req.RGBD_IMAGE) {
        cv::Mat cvColorImage = cv::imread(req.path_color, cv::IMREAD_UNCHANGED);
        cv::Mat cvDepthImg = cv::imread(req.path_depth, cv::IMREAD_UNCHANGED);
        cv::Mat cvDepthImage;
        cvDepthImg.convertTo(cvDepthImage, CV_32F, 1/20.0);
        if (!cvColorImage.empty() && !cvDepthImage.empty()) {
            Scan scan = Scan(cvColorImage, cvDepthImage, req.name, req.transform, "");
            insertScan(scan);
            res.success = true;
            res.message = "Loading RGBD image successful";
        } else if (cvColorImage.empty() && !cvDepthImage.empty()) {
            res.success = false;
            res.message = "Error while loading color image";
        } else if (!cvColorImage.empty() && cvDepthImage.empty()) {
            res.success = false;
            res.message = "Error while loading depth image";
        } else {
            res.success = false;
            res.message = "Error while loading color and depth image";
        }
    } else if (req.type == req.POINT_CLOUD) {
        open3d::io::ReadPointCloudOption params;
        std::shared_ptr<PointCloud_t> sptr_pcd = std::make_shared<PointCloud_t>(open3d::geometry::PointCloud());
        bool success = open3d::io::ReadPointCloudFromPLY(req.path_point_cloud, *sptr_pcd, params);
        if (success) {
            Scan scan = Scan(req.name, sptr_pcd);
            insertScan(scan);
            res.success = true;
            res.message = "Loading point cloud successful";
        } else {
            res.success = false;
            res.message = "Error while loading point cloud";
        }
    } else {
        res.success = false;
        res.message = "Unknown type";
    }
    publishScanList();
    return true;
}

bool exportROS(acin_reconstruction::Export::Request& req, acin_reconstruction::Export::Response& res)
{
    Scan* scan = findScan(req.fragment);
    if (scan == NULL) {
        res.success = false;
        res.message = "Fragment not found";
    } else {
        std::shared_ptr<PointCloud_t> sptr_pcd = scan->getPointCloud();
        if (req.type == req.MESH) {
            sptr_pcd->EstimateNormals();
            sptr_pcd->OrientNormalsConsistentTangentPlane(Scan::processingParameters.normalOrientation.neighborPoints);
            std::tuple<std::shared_ptr<open3d::geometry::TriangleMesh>, std::vector<double>> result;
            result = open3d::geometry::TriangleMesh::CreateFromPointCloudPoisson(*sptr_pcd, Scan::processingParameters.meshGeneration.depth);
            std::shared_ptr<open3d::geometry::TriangleMesh> mesh = std::get<0>(result);
            
            // Remove all clusters except the largest one
            std::tuple<std::vector<int>, std::vector<size_t>, std::vector<double>> clusterResult;
            clusterResult = mesh->ClusterConnectedTriangles();
            std::vector<int> clusterIndices = std::get<0>(clusterResult);
            std::vector<size_t> clusterSizes = std::get<1>(clusterResult);
            int objectClusterIndex = std::max_element(clusterSizes.begin(), clusterSizes.end()) - clusterSizes.begin();
            std::vector<bool> mask(clusterIndices.size(), false);
            for (int i = 0; i < mask.size(); ++i) {
                if (clusterIndices[i] != objectClusterIndex)
                    mask[i] = true;
            }
            mesh->RemoveTrianglesByMask(mask);
            
            // Crop mesh
            mesh = mesh->Crop(Scan::boundingBox);
            
            // Write mesh to file
            res.success = open3d::io::WriteTriangleMesh(req.path, *mesh);
            
            if (res.success)
                res.message = "Export of mesh completed";
            else
                res.message = "Error while writing mesh to file";
        } else if (req.type == req.POINT_CLOUD) {
            open3d::io::WritePointCloudOption params;
            res.success = open3d::io::WritePointCloudToPLY(req.path, *sptr_pcd, params);
            if (res.success)
                res.message = "Export of point cloud completed";
            else
                res.message = "Error while writing point cloud to file";
        } else {
            res.success = false;
            res.message = "Unknown type";
        }
    }
    return true;
}

bool connectGazebo(acin_reconstruction::Connect::Request& req, acin_reconstruction::Connect::Response& res)
{
    res.success = false;
    res.message = "Not available in simulation mode";
    return true;
}

bool connectPhoXi(acin_reconstruction::Connect::Request& req, acin_reconstruction::Connect::Response& res)
{
    if (req.operation == req.CONNECT) {
        res.success = false;
        
        // Check if PhoXi Control is running
        pho::api::PhoXiFactory factory;
        if (!factory.isPhoXiControlRunning()) {
            res.message = "PhoXi Control not running";
            return true;
        }
        
        // Get available devices
        std::vector <pho::api::PhoXiDeviceInformation> deviceList = factory.GetDeviceList();
        if (deviceList.empty()) {
            res.message = "No device found";
            return true;
        }
        
        // Connect to device
        phoXiDevice = factory.CreateAndConnectFirstAttached();
        if (phoXiDevice) {
            connected = true;
            res.success = true;
            res.message = "Connected";
        } else {
            res.message = "No device opened";
            return true;
        }
        
        // Obtain calibration parameters
        Scan::cameraParameters.load(phoXiDevice->CalibrationSettings);
        ROS_ERROR_STREAM("cx: " << Scan::cameraParameters.cx);
        ROS_ERROR_STREAM("cy: " << Scan::cameraParameters.cy);
        ROS_ERROR_STREAM("fx: " << Scan::cameraParameters.fx);
        ROS_ERROR_STREAM("fy: " << Scan::cameraParameters.fy);
        ROS_ERROR_STREAM("Distortion coefficients:");
        for (int i = 0; i < phoXiDevice->CalibrationSettings->DistortionCoefficients.size(); ++i)
            ROS_ERROR_STREAM(phoXiDevice->CalibrationSettings->DistortionCoefficients[i]);
    } else {
        phoXiDevice->Disconnect();
        connected = false;
        res.success = true;
        res.message = "Disconnected";
    }
    return true;
}

bool scanGazebo(cv::Mat& color, cv::Mat& depth)
{
    if (tempColor == NULL || tempDepth == NULL)
        return false;
    
    convertImage(color, tempColor, false);
    convertImage(depth, tempDepth, true);
    
    return true;
}

bool scanPhoXi(cv::Mat& color, cv::Mat& depth)
{
    if (!connected) {
        ROS_ERROR("No connection to PhoXi");
        return false;
    }
    
    if (phoXiDevice->TriggerMode != pho::api::PhoXiTriggerMode::Software) {
        if (phoXiDevice->isAcquiring() && !phoXiDevice->StopAcquisition()) {
            ROS_ERROR("Change of trigger mode failed");
            return false;
        }
        
        phoXiDevice->TriggerMode = pho::api::PhoXiTriggerMode::Software;
    }
    
    if (!phoXiDevice->isAcquiring() && !phoXiDevice->StartAcquisition()) {
        ROS_ERROR("Acquisition failed");
        return false;
    }
    
    phoXiDevice->ClearBuffer();
    
    int frameId = phoXiDevice->TriggerFrame();
    if (frameId < 0) {
        ROS_ERROR("Trigger failed");
        return false;
    }
    
    pho::api::PFrame frame = phoXiDevice->GetSpecificFrame(frameId, pho::api::PhoXiTimeout::Infinity);
    
    if (frame) {
        // Get texture
        if (!frame->Texture.Empty()) {
            cv::Mat cvColorImageFloat;
            frame->Texture.ConvertTo(cvColorImageFloat);
            double min;
            double max;
            cv::minMaxIdx(cvColorImageFloat, &min, &max);
            if (max != min) {
                double scaling = 255 / (max - min);
                cvColorImageFloat.convertTo(color, CV_8UC1, scaling, -min*scaling);
            } else {
                ROS_ERROR("Texture contains no information");
                return false;
            }
        } else {
            ROS_ERROR("Retrieving texture failed");
            return false;
        }
        
        // Get depth
        if (!frame->DepthMap.Empty()) {
            frame->DepthMap.ConvertTo(depth);
        } else {
            ROS_ERROR("Retrieving depth map failed");
            return false;
        }
        
        return true;
    } else {
        ROS_ERROR("Retrieving frame failed");
        return false;
    }
}

bool scanROS(acin_reconstruction::Scan::Request& req, acin_reconstruction::Scan::Response& res)
{
    cv::Mat color;
    cv::Mat depth;
    if (scan(color, depth)) {
        geometry_msgs::TransformStamped transformStamped = tfBuffer.lookupTransform("sensor_link", "marker", ros::Time(0));
        
        Scan scan = Scan(color, depth, req.name, transformStamped.transform, req.path_pose);
        insertScan(scan);
        publishScanList();
        
        bool successColor = true;
        bool successDepth = true;
        if (req.path_color != "")
            successColor = cv::imwrite(req.path_color, color);
        if (req.path_depth != "") {
            cv::Mat img;
            depth.convertTo(img, CV_16U, 20.0);
            successDepth = cv::imwrite(req.path_depth, img);
        }
        res.success = successColor && successDepth;
        
        if (res.success)
            res.message = "Scan successful";
        else if (!successColor && successDepth)
            res.message = "Writing color image failed";
        else if (successColor && !successDepth)
            res.message = "Writing depth image failed";
        else
            res.message = "Writing color and depth image failed";
    } else {
        res.message = "Retrieving data failed";
        res.success = false;
    }
    
    return true;
}

bool transformROS(acin_reconstruction::Transform::Request& req, acin_reconstruction::Transform::Response& res)
{
    // Get point cloud
    Scan* scan = findScan(req.fragment);
    if (scan == NULL) {
        res.success = false;
        res.message = "Fragment not found";
    } else {
        std::shared_ptr<PointCloud_t> pcd = scan->getPointCloud();
        
        // Translation
        Eigen::Vector3d translation(req.transform.translation.x,
                                    req.transform.translation.y,
                                    req.transform.translation.z);
        
        // Rotation
        Eigen::Quaterniond quaternion(req.transform.rotation.w,
                                      req.transform.rotation.x,
                                      req.transform.rotation.y,
                                      req.transform.rotation.z);
        
        Eigen::Matrix3d R = quaternion.toRotationMatrix();
        Eigen::Vector3d center(0,0,0);
        
        // Transform point cloud
        if (req.forward) {
            pcd->Rotate(R, center);
            pcd->Translate(translation);
        } else {
            pcd->Translate(-translation);
            pcd->Rotate(R.transpose(), center);
        }
        
        res.success = true;
        res.message = "Transformation successful";
    }
    
    return true;
}

bool registerROS(acin_reconstruction::Register::Request& req, acin_reconstruction::Register::Response& res)
{
    // Get source point cloud
    Scan* sourceScan = findScan(req.source);
    if (sourceScan == NULL) {
        res.success = false;
        res.message = "Source not found";
        return true;
    }
    std::shared_ptr<PointCloud_t> sourcePcd = sourceScan->getPointCloud();
    
    // Get target point cloud
    Scan* targetScan = findScan(req.target);
    if (targetScan == NULL) {
        res.success = false;
        res.message = "Target not found";
        return true;
    }
    std::shared_ptr<PointCloud_t> targetPcd = targetScan->getPointCloud();
    
    // Calculate transformation
    open3d::pipelines::registration::RegistrationResult result;
    Eigen::Matrix4d init = Eigen::Matrix4d::Identity();
    if (req.method == req.POINT_TO_POINT_ICP) {
        open3d::pipelines::registration::TransformationEstimationPointToPoint estimation;
        result = open3d::pipelines::registration::RegistrationICP(*sourcePcd, *targetPcd, req.threshold, init, estimation);
    } else if (req.method == req.POINT_TO_PLANE_ICP) {
        open3d::pipelines::registration::TransformationEstimationPointToPlane estimation;
        result = open3d::pipelines::registration::RegistrationICP(*sourcePcd, *targetPcd, req.threshold, init, estimation);
    } else {
        res.success = false;
        res.message = "Unknown method";
        return true;
    }
    
    // Send calculated transformation
    Eigen::Matrix4d_u& transformation = result.transformation_;
    
    // Insert translation
    res.transform.translation.x = transformation(0,3);
    res.transform.translation.y = transformation(1,3);
    res.transform.translation.z = transformation(2,3);
    
    // Insert rotation
    Eigen::Matrix3d rotationMatrix = transformation.topLeftCorner(3,3);
    Eigen::Quaterniond quaternion(rotationMatrix);
    res.transform.rotation.x = quaternion.x();
    res.transform.rotation.y = quaternion.y();
    res.transform.rotation.z = quaternion.z();
    res.transform.rotation.w = quaternion.w();
    
    res.success = true;
    res.message = "Registration successful";
    return true;
}

bool multiwayRegisterROS(acin_reconstruction::MultiwayRegister::Request& req,
                             acin_reconstruction::MultiwayRegister::Response& res) {
    std::vector<std::string> names;
    if (req.fragments.size() > 0) {
        names = req.fragments;
    } else {
        std::list<Scan>::iterator it;
        for (it = scanList.begin(); it != scanList.end(); ++it)
            names.push_back(it->getName());
    }
    
    // Choose method
    if (req.method == req.POINT_TO_POINT_ICP)
        res.success = multiwayRegister(names, 0, req.threshold);
    else if (req.method == req.POINT_TO_PLANE_ICP)
        res.success = multiwayRegister(names, 1, req.threshold);
    else
        res.success = false;
    
    return true;
}

void maximumSpanningTree(open3d::pipelines::registration::PoseGraph &poseGraph, const std::vector<int> &overlap)
{
    int N = poseGraph.nodes_.size();
    int M = poseGraph.edges_.size();
    
    // Initialize vertices for Prime's algorithm
    std::vector<bool> included(N, false);
    included[0] = true;
    std::vector<int> keyValues(N, 0);
    
    // Update neighbors of vertex 0
    for (int j = 0; j < M; ++j) {
        int neighbor;
        if (poseGraph.edges_[j].source_node_id_ == 0)
            neighbor = poseGraph.edges_[j].target_node_id_;
        else if (poseGraph.edges_[j].target_node_id_ == 0)
            neighbor = poseGraph.edges_[j].source_node_id_;
        else
            continue;
        
        keyValues[neighbor] = overlap[j];
    }
    
    // Iterate over all remaining vertices
    for (int i = 1; i < N; ++i) {
        // Find vertex with maximum key value which is not included yet
        int foundVertex = i;
        double currentMaximum = 0;
        for (int k = 1; k < N; ++k) {
            if (!included[k] && (keyValues[k] > currentMaximum)) {
                foundVertex = k;
                currentMaximum = keyValues[k];
            }
        }
        
        // Include found vertex
        included[foundVertex] = true;
        
        // Update neighbors for next iteration and mark edge as certain
        for (int j = 0; j < M; ++j) {
            int neighbor;
            if (poseGraph.edges_[j].source_node_id_ == foundVertex)
                neighbor = poseGraph.edges_[j].target_node_id_;
            else if (poseGraph.edges_[j].target_node_id_ == foundVertex)
                neighbor = poseGraph.edges_[j].source_node_id_;
            else
                continue;
            
            if (included[neighbor] && (overlap[j] == keyValues[foundVertex]))
                poseGraph.edges_[j].uncertain_ = false;
            
            if (!included[neighbor] && (overlap[j] > keyValues[neighbor]))
                keyValues[neighbor] = overlap[j];
        }
    }
}

bool multiwayRegister(std::vector<std::string> names, int method, double threshold)
{
    // Get point clouds
    int length = names.size();
    std::vector<std::shared_ptr<PointCloud_t>> pcds;
    for (int i = 0; i < length; ++i) {
        Scan *found = findScan(names[i]);
        if (found != NULL)
            pcds.push_back(found->getPointCloud());
    }
    
    ROS_INFO("Building pose graph...");
    open3d::pipelines::registration::PoseGraph poseGraph;
    
    // Create nodes
    poseGraph.nodes_ = std::vector<open3d::pipelines::registration::PoseGraphNode>(length);
    poseGraph.nodes_[0].pose_ = Eigen::Matrix4d::Identity();
    
    // Create edges
    std::vector<int> overlap;
    overlap.reserve(length * (length-1));
    poseGraph.edges_.reserve(length * (length-1));
    
    for (int i = 0; i < length-1; ++i) {
        for (int j = i+1; j < length; ++j) {
            open3d::pipelines::registration::RegistrationResult result =
                open3d::pipelines::registration::EvaluateRegistration(*pcds[i], *pcds[j], threshold);
            open3d::pipelines::registration::PoseGraphEdge edge(i, j, Eigen::Matrix4d::Identity(), Eigen::Matrix6d::Identity(), true);
            poseGraph.edges_.push_back(edge);
            overlap.push_back(result.correspondence_set_.size());
        }
    }
    
    // Find maximum spanning tree
    maximumSpanningTree(poseGraph, overlap);
    
    // Traverse maximum spanning tree
    std::vector<Eigen::Matrix4d> odometries(length, Eigen::Matrix4d::Identity());
    int k = 0;
    for (int i = 0; i < length-1; ++i) {
        for (int j = i+1; j < length; ++j) {
            if (poseGraph.edges_[k].uncertain_) {
                // Case of loop closure
                poseGraph.edges_[k].information_ = open3d::pipelines::registration::GetInformationMatrixFromPointClouds(
                    *pcds[i], *pcds[j], threshold, Eigen::Matrix4d::Identity());
            } else {
                // Case of odometry
                
                // Calculate transformation
                open3d::pipelines::registration::RegistrationResult result;
                
                if (method == 0) {
                    open3d::pipelines::registration::TransformationEstimationPointToPoint estimation;
                    result = open3d::pipelines::registration::RegistrationICP(*pcds[i], *pcds[j], threshold,
                                                                              Eigen::Matrix4d::Identity(), estimation);
                } else if (method == 1) {
                    open3d::pipelines::registration::TransformationEstimationPointToPlane estimation;
                    result = open3d::pipelines::registration::RegistrationICP(*pcds[i], *pcds[j], threshold,
                                                                              Eigen::Matrix4d::Identity(), estimation);
                } else {
                    return false;
                }
                
                poseGraph.edges_[k].transformation_ = result.transformation_;
                poseGraph.edges_[k].information_ = open3d::pipelines::registration::GetInformationMatrixFromPointClouds(
                    *pcds[i], *pcds[j], threshold, result.transformation_);
                
                odometries[j] = result.transformation_ * odometries[i];
                poseGraph.nodes_[j].pose_ = odometries[j].inverse();
            }
            
            ++k;
        }
    }
    
    ROS_INFO("Optimizing pose graph...");
    open3d::pipelines::registration::GlobalOptimizationOption option(threshold, 0.25, 1.0, 0);
    open3d::pipelines::registration::GlobalOptimizationLevenbergMarquardt optimizationMethod;
    open3d::pipelines::registration::GlobalOptimizationConvergenceCriteria criteria;
    open3d::pipelines::registration::GlobalOptimization(poseGraph, optimizationMethod, criteria, option);
    
    ROS_INFO("Transforming point clouds...");
    for (int i = 0; i < length; ++i)
        pcds[i]->Transform(poseGraph.nodes_[i].pose_);
    
    publishPointClouds();
    return true;
}

bool modifyROS(acin_reconstruction::Modify::Request& req, acin_reconstruction::Modify::Response& res)
{
    // If all scans should be deleted
    if (req.operation == req.DELETE_ALL) {
        scanList.erase(scanList.begin(), scanList.end());
        
        res.success = true;
        res.message = "All scans deleted";
        publishScanList();
        publishPointClouds();
        return true;
    }
    
    // Get scan to be modified
    Scan *scan = findScan(req.fragment);
    if (scan == NULL) {
        res.success = false;
        res.message = "Fragment not found";
    } else {
        if (req.operation == req.SHOW) {
            scan->setShow(true);
        } else if (req.operation == req.HIDE) {
            scan->setShow(false);
        } else if (req.operation == req.ADD) {
            Scan *target = findScan(req.target);
            if(target != NULL) {
                *(target->getPointCloud()) += *(scan->getPointCloud());
            } else {
                // Make a copy of the point cloud
                std::shared_ptr<PointCloud_t> sptr_pcd;
                std::vector<size_t> indices;
                sptr_pcd = scan->getPointCloud()->SelectByIndex(indices, true);
                
                Scan *target = new Scan(req.target,sptr_pcd);
                insertScan(*target);
                publishScanList();
            }
        } else if (req.operation == req.DOWNSAMPLE) {
            if (req.voxel_size != 0)
                scan->setPointCloud(scan->getPointCloud()->VoxelDownSample(req.voxel_size));
            else if (req.every_k_points != 0)
                scan->setPointCloud(scan->getPointCloud()->UniformDownSample(req.every_k_points));
        } else if (req.operation == req.DELETE) {
            scanList.remove(*scan);
            publishScanList();
        } else if (req.operation == req.COLOR) {
            scan->setCustom(req.custom_color);
            if(req.custom_color) {
                Eigen::Vector3d color(req.color.r, req.color.g, req.color.b);
                scan->setColor(color);
            }
        }
        
        res.success = true;
        res.message = "Modification successful";
        publishPointClouds();
    }
    return true;
}

bool boundingBoxROS(acin_reconstruction::BoundingBox::Request& req, acin_reconstruction::BoundingBox::Response& res)
{
    if (req.operation == req.ADD) {
        if (req.extent.x > 0 && req.extent.y > 0 && req.extent.z > 0) {
            Eigen::Vector3d lowerBound(req.center.x-req.extent.x/2,
                                       req.center.y-req.extent.y/2,
                                       req.center.z-req.extent.z/2);
            Eigen::Vector3d upperBound(req.center.x+req.extent.x/2,
                                       req.center.y+req.extent.y/2,
                                       req.center.z+req.extent.z/2);
            Scan::addBoundingBox(lowerBound, upperBound);
            res.success = true;
            res.message = "Added bounding box";
        } else {
            res.success = false;
            res.message = "Extent of bounding box has to be positive";
        }
    } else if (req.operation == req.REMOVE) {
        Scan::removeBoundingBox();
        res.success = true;
        res.message = "Removed bounding box";
    } else {
        Scan* scan = findScan(req.fragment);
        if (scan == NULL) {
            res.success = false;
            res.message = "Fragment not found";
        } else {
            open3d::geometry::OrientedBoundingBox bbox = scan->getPointCloud()->GetOrientedBoundingBox();
            
            // Orthogonalize matrix, because Open3D's roation matrix sometimes has determinant -1
            Eigen::Matrix3d rot(bbox.R_);
            rot.col(0).normalize();
            rot.col(1).normalize();
            rot.col(2) = rot.col(0).cross(rot.col(1));
            rot.col(2).normalize();
            rot.col(0) = rot.col(1).cross(rot.col(2));
            rot.col(0).normalize();
            
            Eigen::Quaterniond quaternion(rot);
            res.pose.orientation.w = quaternion.w();
            res.pose.orientation.x = quaternion.x();
            res.pose.orientation.y = quaternion.y();
            res.pose.orientation.z = quaternion.z();
            
            res.pose.position.x = bbox.center_[0];
            res.pose.position.y = bbox.center_[1];
            res.pose.position.z = bbox.center_[2];
            
            res.extent.x = bbox.extent_[0];
            res.extent.y = bbox.extent_[1];
            res.extent.z = bbox.extent_[2];
            
            res.success = true;
            res.message = "Bounding box calculated";
        }
    }
    
    return true;
}

bool setReferenceROS(acin_reconstruction::SetReference::Request& req, acin_reconstruction::SetReference::Response& res)
{
    cv::Mat color;
    cv::Mat depth;
    res.success = false;
    
    if (scan(color, depth)) {
        geometry_msgs::Pose estimatedPose;
        if (estimatePose(color, Scan::cameraParameters, Scan::board, estimatedPose)) {
            tf2::Transform cameraToMarker;
            tf2::fromMsg(estimatedPose, cameraToMarker);
            
            tf2::Transform worldToCamera;
            tf2::fromMsg(req.absolute_pose, worldToCamera);
            
            tf2::Transform worldToMarker = worldToCamera*cameraToMarker;
            
            markerFrame.transform.translation.x = worldToMarker.getOrigin().x();
            markerFrame.transform.translation.y = worldToMarker.getOrigin().y();
            markerFrame.transform.translation.z = worldToMarker.getOrigin().z();
            markerFrame.transform.rotation.w = worldToMarker.getRotation().w();
            markerFrame.transform.rotation.x = worldToMarker.getRotation().x();
            markerFrame.transform.rotation.y = worldToMarker.getRotation().y();
            markerFrame.transform.rotation.z = worldToMarker.getRotation().z();
            
            res.success = true;
        }
    }
    
    return true;
}

int main(int argc, char **argv)
{
    // Set up ROS
    ros::init(argc, argv, "reconstruction_node");
    ros::NodeHandle nh;
    ptr_nh = &nh;
    
    bool simulation;
    if (!nh.getParam("simulation", simulation)) {
        ROS_ERROR("Parameter for simulation undefined");
        return 0;
    }
    
    if (simulation) {
        // Get parameters for camera
        if (!Scan::cameraParameters.load(nh))
            return 0;
        
        scan = &scanGazebo;
        connectROS = &connectGazebo;
    } else {
        scan = &scanPhoXi;
        connectROS = &connectPhoXi;
        connected = false;
    }
    
    // Get parameters for marker
    MarkerParameters markerParameters;
    if (!markerParameters.load(nh))
        return 0;
    Scan::board = markerParameters.board;
    
    // Get parameters for processing
    if (!Scan::processingParameters.load(nh))
        return 0;
    
    pointCloudPub = nh.advertise<sensor_msgs::PointCloud2>("point_cloud", 1, true);
    scanListPub = nh.advertise<acin_reconstruction::ScanList>("scan_list", 1, true);
    
    ros::ServiceServer service1 = nh.advertiseService("perform_scan", scanROS);
    ros::ServiceServer service2 = nh.advertiseService("connect", connectROS);
    ros::ServiceServer service3 = nh.advertiseService("display_scans", displayScansROS);
    ros::ServiceServer service4 = nh.advertiseService("load", loadScanROS);
    ros::ServiceServer service5 = nh.advertiseService("transform", transformROS);
    ros::ServiceServer service6 = nh.advertiseService("register", registerROS);
    ros::ServiceServer service7 = nh.advertiseService("modify", modifyROS);
    ros::ServiceServer service8 = nh.advertiseService("export", exportROS);
    ros::ServiceServer service9 = nh.advertiseService("multiway_register", multiwayRegisterROS);
    ros::ServiceServer service10 = nh.advertiseService("bounding_box", boundingBoxROS);
    ros::ServiceServer service11 = nh.advertiseService("set_reference", setReferenceROS);
    
    message_filters::Subscriber<sensor_msgs::Image> colorSub(nh, "/camera/image_raw", 1);
    message_filters::Subscriber<sensor_msgs::Image> depthSub(nh, "/camera/depth/image_raw", 1);
    message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image> sync(colorSub, depthSub, 1);
    sync.registerCallback(boost::bind(&syncCallback, _1, _2));
    
    // Initialize marker frame
    tf2_ros::TransformBroadcaster br;
    markerFrame.header.frame_id = "world";
    markerFrame.child_frame_id = "marker";
    markerFrame.transform.rotation.w = 1;
    
    // Tf listener
    tf2_ros::TransformListener tfListener(tfBuffer);
    
    // Broadcast marker frame
    ros::Rate rate(10);
    while (nh.ok()) {
        markerFrame.header.stamp = ros::Time::now();
        br.sendTransform(markerFrame);
        ros::spinOnce();
        rate.sleep();
    }
    
    return 0;
}
