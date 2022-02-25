#include "ros/ros.h"
#include "parameters.h"
#include <open3d/Open3D.h>
#include <Eigen/Eigen>
#define OPENCV_TRAITS_ENABLE_DEPRECATED
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>

#define PHOXI_OPENCV_SUPPORT
#include "PhoXi.h"

bool MarkerParameters::load(const ros::NodeHandle& nh)
{
    bool failure = false;
    std::string parameterName;
    std::string ns = "marker/";
    
    // Get parameters for ChArUco board
    parameterName = ns + "squares_x";
    if (!nh.getParam(parameterName, squaresX)) {
        ROS_ERROR_STREAM("Parameter '" + parameterName + "' not found");
        failure = true;
    }
    
    parameterName = ns + "squares_y";
    if (!nh.getParam(parameterName, squaresY)) {
        ROS_ERROR_STREAM("Parameter '" + parameterName + "' not found");
        failure = true;
    }
    
    parameterName = ns + "square_length";
    if (!nh.getParam(parameterName, squareLength)) {
        ROS_ERROR_STREAM("Parameter '" + parameterName + "' not found");
        failure = true;
    }
    
    parameterName = ns + "marker_length";
    if (!nh.getParam(parameterName, markerLength)) {
        ROS_ERROR_STREAM("Parameter '" + parameterName + "' not found");
        failure = true;
    }
    
    std::string dictString;
    parameterName = ns + "dictionary";
    if (!nh.getParam(parameterName, dictString)) {
        ROS_ERROR_STREAM("Parameter '" + parameterName + "' not found");
        failure = true;
    }
    
    // Return if a parameter could not be loaded
    if (failure)
        return false;
    
    // Select ChArUco dictionary
    std::unordered_map<std::string,cv::aruco::PREDEFINED_DICTIONARY_NAME> table = {
        {"DICT_4X4_50", cv::aruco::DICT_4X4_50},
        {"DICT_4X4_100", cv::aruco::DICT_4X4_100},
        {"DICT_4X4_250", cv::aruco::DICT_4X4_250},
        {"DICT_4X4_1000", cv::aruco::DICT_4X4_1000},
        {"DICT_5X5_50", cv::aruco::DICT_5X5_50},
        {"DICT_5X5_100", cv::aruco::DICT_5X5_100},
        {"DICT_5X5_250", cv::aruco::DICT_5X5_250},
        {"DICT_5X5_1000", cv::aruco::DICT_5X5_1000},
        {"DICT_6X6_50", cv::aruco::DICT_6X6_50},
        {"DICT_6X6_100", cv::aruco::DICT_6X6_100},
        {"DICT_6X6_250", cv::aruco::DICT_6X6_250},
        {"DICT_6X6_1000", cv::aruco::DICT_6X6_1000},
        {"DICT_7X7_50", cv::aruco::DICT_7X7_50},
        {"DICT_7X7_100", cv::aruco::DICT_7X7_100},
        {"DICT_7X7_250", cv::aruco::DICT_7X7_250},
        {"DICT_7X7_1000", cv::aruco::DICT_7X7_1000},
        {"DICT_ARUCO_ORIGINAL", cv::aruco::DICT_ARUCO_ORIGINAL}
    };
    auto it = table.find(dictString);
    if (it == table.end()) {
        ROS_ERROR("Given ChArUco dictionary is unknown!");
        return false;
    }
    
    // Create ChArUco board
    dictionary = cv::aruco::getPredefinedDictionary(it->second);
    board = cv::aruco::CharucoBoard::create(squaresX, squaresY, squareLength, markerLength, dictionary);
    
    return true;
}

bool CameraParameters::load(const ros::NodeHandle& nh)
{
    bool failure = false;
    std::string parameterName;
    std::string ns = "camera/";
    
    // Get values of camera matrix
    parameterName = ns + "camera_matrix/fx";
    if (!nh.getParam(parameterName, fx)) {
        ROS_ERROR_STREAM("Parameter '" + parameterName + "' not found");
        failure = true;
    }
    
    parameterName = ns + "camera_matrix/fy";
    if (!nh.getParam(parameterName, fy)) {
        ROS_ERROR_STREAM("Parameter '" + parameterName + "' not found");
        failure = true;
    }
    
    parameterName = ns + "camera_matrix/cx";
    if (!nh.getParam(parameterName, cx)) {
        ROS_ERROR_STREAM("Parameter '" + parameterName + "' not found");
        failure = true;
    }
    
    parameterName = ns + "camera_matrix/cy";
    if (!nh.getParam(parameterName, cy)) {
        ROS_ERROR_STREAM("Parameter '" + parameterName + "' not found");
        failure = true;
    }
    
    // Get distortion coefficients
    std::vector<double> distCoeffsStd;
    parameterName = ns + "dist_coeffs";
    if(!nh.getParam(parameterName, distCoeffsStd)) {
        ROS_ERROR_STREAM("Parameter '" + parameterName + "' not found");
        failure = true;
    }
    
    // Return if a parameter could not be loaded
    if (failure)
        return 0;
    
    // Create camera matrix
    Eigen::Matrix3d cameraMatrixEigen = Eigen::Matrix3d::Identity();
    cameraMatrixEigen(0,0) = fx;
    cameraMatrixEigen(0,2) = cx;
    cameraMatrixEigen(1,1) = fy;
    cameraMatrixEigen(1,2) = cy;
    cv::eigen2cv(cameraMatrixEigen, cameraMatrix);
    
    // Convert distortion coefficients
    distCoeffs = cv::Mat(distCoeffsStd, true);
    
    return true;
}

void CameraParameters::load(const pho::api::PhoXiCalibrationSettings& calibrationSettings)
{
    // Get values of camera matrix
    fx = calibrationSettings.CameraMatrix[0][0];
    fy = calibrationSettings.CameraMatrix[1][1];
    cx = calibrationSettings.CameraMatrix[0][2];
    cy = calibrationSettings.CameraMatrix[1][2];
    
    // Create camera matrix
    Eigen::Matrix3d cameraMatrixEigen = Eigen::Matrix3d::Identity();
    cameraMatrixEigen(0, 0) = fx;
    cameraMatrixEigen(0, 2) = cx;
    cameraMatrixEigen(1, 1) = fy;
    cameraMatrixEigen(1, 2) = cy;
    cv::eigen2cv(cameraMatrixEigen, cameraMatrix);
    
    // Get distortion coefficients
    distCoeffs = cv::Mat(calibrationSettings.DistortionCoefficients, true);
}

void CameraParameters::getIntrinsic(int width, int height, open3d::camera::PinholeCameraIntrinsic& intrinsic)
{
    intrinsic = open3d::camera::PinholeCameraIntrinsic(width, height, fx, fy, cx, cy);
}

bool ProcessingParameters::load(const ros::NodeHandle& nh)
{
    bool success = true;
    std::string parameterName;
    std::string ns = "processing/";
    
    // Get parameters for radius outlier removal
    parameterName = ns + "radius_outlier_removal/enable";
    if (!nh.getParam(parameterName, radiusOutlierRemoval.enable)) {
        ROS_ERROR_STREAM("Parameter '" + parameterName + "' not found");
        success = false;
    }
    
    parameterName = ns + "radius_outlier_removal/neighbors";
    if (!nh.getParam(parameterName, radiusOutlierRemoval.neighbors)) {
        ROS_ERROR_STREAM("Parameter '" + parameterName + "' not found");
        success = false;
    } else if (radiusOutlierRemoval.neighbors <= 0) {
        ROS_ERROR_STREAM("Parameter '" + parameterName + "' must be positive");
        success = false;
    }
    
    parameterName = ns + "radius_outlier_removal/search_radius";
    if (!nh.getParam(parameterName, radiusOutlierRemoval.searchRadius)) {
        ROS_ERROR_STREAM("Parameter '" + parameterName + "' not found");
        success = false;
    } else if (radiusOutlierRemoval.searchRadius <= 0) {
        ROS_ERROR_STREAM("Parameter '" + parameterName + "' must be positive");
        success = false;
    }
    
    // Get parameters for statistical outlier removal
    parameterName = ns + "statistical_outlier_removal/enable";
    if (!nh.getParam(parameterName, statisticalOutlierRemoval.enable)) {
        ROS_ERROR_STREAM("Parameter '" + parameterName + "' not found");
        success = false;
    }
    
    parameterName = ns + "statistical_outlier_removal/neighbors";
    if (!nh.getParam(parameterName, statisticalOutlierRemoval.neighbors)) {
        ROS_ERROR_STREAM("Parameter '" + parameterName + "' not found");
        success = false;
    } else if (statisticalOutlierRemoval.neighbors <= 0) {
        ROS_ERROR_STREAM("Parameter '" + parameterName + "' must be positive");
        success = false;
    }
    
    parameterName = ns + "statistical_outlier_removal/std_ratio";
    if (!nh.getParam(parameterName, statisticalOutlierRemoval.stdRatio)) {
        ROS_ERROR_STREAM("Parameter '" + parameterName + "' not found");
        success = false;
    } else if (statisticalOutlierRemoval.stdRatio <= 0) {
        ROS_ERROR_STREAM("Parameter '" + parameterName + "' must be positive");
        success = false;
    }
    
    // Get parameters for normal orientation
    parameterName = ns + "normal_orientation/neighbor_points";
    if (!nh.getParam(parameterName, normalOrientation.neighborPoints)) {
        ROS_ERROR_STREAM("Parameter '" + parameterName + "' not found");
        success = false;
    } else if (normalOrientation.neighborPoints <= 0) {
        ROS_ERROR_STREAM("Parameter '" + parameterName + "' must be positive");
        success = false;
    }
    
    // Get parameters for mesh generation
    parameterName = ns + "mesh_generation/depth";
    if (!nh.getParam(parameterName, meshGeneration.depth)) {
        ROS_ERROR_STREAM("Parameter '" + parameterName + "' not found");
        success = false;
    } else if (meshGeneration.depth <= 0) {
        ROS_ERROR_STREAM("Parameter '" + parameterName + "' must be positive");
        success = false;
    }
    
    return success;
}
