#ifndef PARAMETERS_H
#define PARAMETERS_H

#include "ros/ros.h"
#include <open3d/Open3D.h>
#define OPENCV_TRAITS_ENABLE_DEPRECATED
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/aruco/charuco.hpp>

#define PHOXI_OPENCV_SUPPORT
#include "PhoXi.h"

struct MarkerParameters
{
    int squaresX;
    int squaresY;
    double squareLength;
    double markerLength;
    cv::Ptr<cv::aruco::Dictionary> dictionary;
    cv::Ptr<cv::aruco::CharucoBoard> board;
    
    bool load(const ros::NodeHandle& nh);
};

struct CameraParameters
{
    double fx;
    double fy;
    double cx;
    double cy;
    cv::Mat cameraMatrix;
    cv::Mat distCoeffs;
    
    bool load(const ros::NodeHandle& nh);
    void load(const pho::api::PhoXiCalibrationSettings& calibrationSettings);
    void getIntrinsic(int width, int height, open3d::camera::PinholeCameraIntrinsic& intrinsic);
};

struct ProcessingParameters
{
    struct {
        bool enable;
        int neighbors;
        double searchRadius;
    } radiusOutlierRemoval;
    
    struct {
        bool enable;
        int neighbors;
        double stdRatio;
    } statisticalOutlierRemoval;
    
    struct {
        int neighborPoints;
    } normalOrientation;
    
    struct {
        int depth;
    } meshGeneration;
    
    bool load(const ros::NodeHandle& nh);
};

#endif
