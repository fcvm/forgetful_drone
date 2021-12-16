#pragma once



#include <Eigen/Dense>
#include <rapid_trajectories/Vec3.h>
#include <nav_msgs/Odometry.h>
#include <quadrotor_common/trajectory_point.h>
#include <kindr/minimal/quat-transformation.h>
#include <quadrotor_common/math_common.h>
#include <chrono>
#include <filesystem>
#include <ros/ros.h>
#include "dirent.h"
#include <fstream>
#include <visualization_msgs/Marker.h>

#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_datatypes.h>






namespace forgetful_drone
{



class Pose
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::Vector3f position{0.0, 0.0, 0.0};
    Eigen::Quaternionf orientation{1.0, 0.0, 0.0, 0.0};

    geometry_msgs::Point position_as_geometry_msg() const;
    geometry_msgs::Quaternion orientation_as_geometry_msg() const;
    geometry_msgs::Pose as_geometry_msg() const;
    double yaw() const;
};


Eigen::Vector3d 
EigenVector3d_From_Vec3
( const Vec3& IN );


Vec3 
Vec3_From_EigenVector3d
( const Eigen::Vector3d& IN );


geometry_msgs::Pose 
GeometryMsgsPose_From_NavMsgsOdometry
( const nav_msgs::Odometry& In );


quadrotor_common::TrajectoryPoint 
QCTrajectoryPoint_From_KMQuatTransformation
( const kindr::minimal::QuatTransformation& In );


geometry_msgs::Point
GeometryMsgsPoint_From_EigenVector3d
( const Eigen::Vector3d& IN );


Eigen::Vector3d
EigenVector3d_From_GeometryMsgsPoint
( const geometry_msgs::Point& IN );

Eigen::Quaterniond
EigenQuaterniond_From_GeometryMsgsQuaternion
( const geometry_msgs::Quaternion& IN );

geometry_msgs::Quaternion
GeometryMsgsQuaternion_From_EigenQuaterniond
( const Eigen::Quaterniond& IN );


// 

std::string 
getCurrUTCDateTimeAsString
();


double 
Saturation
( const double& InputVal, const double& LowerLimit, const double& UpperLimit );


// FILESYSTEM RELATED

std::vector<std::string> 
getFileNamesInDir
( const std::string& IN_DirPath );


void
writeStringToFile
( const std::string& Cont, const std::string& Dest );


void 
deleteDirectoryContents
( const std::string& dir_path );









/// Contains all possible colors that can be used in visualization:
/// 1) RED
/// 2) GREEN
/// 3) BLUE
/// 4) YELLOW
/// 5) PURPLE
/// 6) WHITE
/// 7) BLACK
/// 8) GREY
enum class VisualizationColors
{
    RED,
    GREEN,
    BLUE,
    YELLOW,
    PURPLE,
    WHITE,
    BLACK,
    GREY
};





visualization_msgs::Marker
getTrajMarker
();


void 
setRGBOfVisMarker
( const VisualizationColors& IN_Color, visualization_msgs::Marker& OUT_VisMarker );



enum class VisPosTypes
{
    REFERENCE,
    HORIZON,
    EXPERT,
    CURRGATE,
    LT_END
};


void
rvizPosition
(
    const Eigen::Vector3d& Pos,
    const VisPosTypes& Type,
    const ros::Publisher& ROSPub
);



bool
fetchROSParameters
(
    const ros::NodeHandle& ROSNH,
    const std::vector<std::pair<const char*, const bool*>>& KeysAndBoolOutputs,
    const std::vector<std::pair<const char*, const int*>>& KeysAndIntOutputs,
    const std::vector<std::pair<const char*, const double*>>& KeysAndDoubleOutputs,
    const std::vector<std::pair<const char*, const std::string*>>& KeysAndStringOutputs
);



void 
runMultiThreadedSpinner();

}