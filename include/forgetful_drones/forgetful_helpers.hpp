#pragma once

#include <Eigen/Dense>
#include <rapid_trajectories/Vec3.h>
#include <nav_msgs/Odometry.h>
#include <quadrotor_common/trajectory_point.h>
#include <kindr/minimal/quat-transformation.h>
#include <quadrotor_common/math_common.h>
#include <chrono>
#include <experimental/filesystem>
#include <ros/ros.h>
#include "dirent.h"
#include <fstream>
#include <visualization_msgs/Marker.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgcodecs/imgcodecs.hpp>

#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_datatypes.h>

#include <algorithm>
#include <random>

#include <ros/xmlrpc_manager.h>


#define ROS_PACKAGE_PATH ros::package::getPath("forgetful_drones")
#define ROS_NODE_NAME ros::this_node::getName()//.c_str()
#define ROS_LOG_PREFIX std::string("[") + ROS_NODE_NAME + std::string("]    ")

#define STREAM_PRECISION 4
#define ROSDEBUG(args) ROS_DEBUG_STREAM(ROS_LOG_PREFIX << std::setprecision(STREAM_PRECISION) << args)
#define ROSINFO(args) ROS_INFO_STREAM(ROS_LOG_PREFIX << std::setprecision(STREAM_PRECISION) << args)
#define ROSWARN(args) ROS_WARN_STREAM(ROS_LOG_PREFIX << std::setprecision(STREAM_PRECISION) << args)
#define ROSERROR(args) ROS_ERROR_STREAM(ROS_LOG_PREFIX << std::setprecision(STREAM_PRECISION) << args)
#define ROSFATAL(args) ROS_FATAL_STREAM(ROS_LOG_PREFIX << std::setprecision(STREAM_PRECISION) << args)
#define ROSDEBUG_FUNCTION_ENTERED() ROSDEBUG(__FUNCTION__ << " ()")


#define GET_VAR_NAME(Var) (#Var)
#define GET_VAR_REP(Var) GET_VAR_NAME(Var) << ": " << Var




#include "forgetful_drones/BuildSimulation.h"
#include "forgetful_drones/StartSimulation.h"
#include "forgetful_drones/StopSimulation.h"
#include "forgetful_drones/LoadRacetrack.h"





namespace forgetful_drone {

// using ...

using fdBS = forgetful_drones::BuildSimulation;
using fdBSReq = forgetful_drones::BuildSimulation::Request;
using fdBSRes = forgetful_drones::BuildSimulation::Response;

using fdStartS = forgetful_drones::StartSimulation;
using fdStartSReq = forgetful_drones::StartSimulation::Request;
using fdStartSRes = forgetful_drones::StartSimulation::Response;

using fdStopS = forgetful_drones::StopSimulation;
using fdStopSReq = forgetful_drones::StopSimulation::Request;
using fdStopSRes = forgetful_drones::StopSimulation::Response;

using fdLR = forgetful_drones::LoadRacetrack;
using fdLRReq = forgetful_drones::LoadRacetrack::Request;
using fdLRRes = forgetful_drones::LoadRacetrack::Response;



// classes

class Pose {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::Vector3f position{0.0, 0.0, 0.0};
    Eigen::Quaternionf orientation{1.0, 0.0, 0.0, 0.0};

    Pose();
    Pose(const Eigen::Vector3f pos, const Eigen::Quaternionf ori);
    Pose(const geometry_msgs::Pose& pose);

    geometry_msgs::Point position_as_geometry_msg() const;
    geometry_msgs::Quaternion orientation_as_geometry_msg() const;
    geometry_msgs::Pose as_geometry_msg() const;
    double yaw() const;

    friend auto operator<< (std::ostream& os, Pose const& pose) -> std::ostream& { 
        return os << std::setprecision (3)
            << "position: [" 
                << pose.position.x() << ", " 
                << pose.position.y() << ", " 
                << pose.position.z() << "], "
            << "orientation: ["
                << pose.orientation.w() << ", " 
                << pose.orientation.x() << ", " 
                << pose.orientation.y() << ", " 
                << pose.orientation.z() << "]";
    }
};


Eigen::Vector3d 
EV3d___Vec3
( const Vec3& IN );

Eigen::Quaternionf EQf_from_Yaw(const double& IN, const bool& InDegree);


double Yaw_From_EQd (const Eigen::Quaterniond& IN);
double Yaw_From_EQf (const Eigen::Quaternionf& IN);


Vec3 
Vec3_from_EV3d
( const Eigen::Vector3d& IN );


geometry_msgs::Pose 
GMPose_From_NMO
( const nav_msgs::Odometry& In );


geometry_msgs::Pose GMPose___EV3d (const Eigen::Vector3d& ev3d);

geometry_msgs::Pose GMPose___EV3d_EQd (
    const Eigen::Vector3d& ev3d,
    const Eigen::Quaterniond& eqd
);



quadrotor_common::TrajectoryPoint 
QCTrajectoryPoint_From_KMQuatTransformation
( const kindr::minimal::QuatTransformation& In );


geometry_msgs::Point
GMPoint__from__EV3d
( const Eigen::Vector3d& IN );


Eigen::Vector3d EV3d___GMP (const geometry_msgs::Point& p);
Eigen::Vector3f EV3f___GMP (const geometry_msgs::Point& p);

Eigen::Vector3d
EV3d_From_GMV3
( const geometry_msgs::Vector3& IN );

Eigen::Quaternionf EQf___GMQ (const geometry_msgs::Quaternion& q);
Eigen::Quaterniond EQd___GMQ (const geometry_msgs::Quaternion& q);

geometry_msgs::Quaternion
GMQ_From_EQd
( const Eigen::Quaterniond& IN );


std::vector<double>
StdVector_From_EigenVector
( const Eigen::VectorXd& IN );

Eigen::VectorXd
EigenVector_From_StdVector
( const std::vector<double>& IN );

// 

std::string 
UTCDateTime
();




// FILESYSTEM RELATED

std::vector<std::string> 
getFileNamesInDir
( const std::string& IN_DirPath );


void
writeStringToFile
( const std::string& Cont, const std::string& Dest );


void 
delDirContents
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

void
rvizState
(
    const Eigen::Vector3d& Pos,
    const Eigen::Vector3d& Vel,
    const VisPosTypes& Type,
    const ros::Publisher& ROSPub
);



bool fetchROSParameters (
    const ros::NodeHandle& rosNH,
    const std::vector <std::pair<const char*, const bool*>>& keys_bools,
    const std::vector <std::pair<const char*, const int*>>& keys_ints,
    const std::vector <std::pair<const char*, const double*>>& keys_doubles,
    const std::vector <std::pair<const char*, const std::string*>>& keys_strings,
    const bool& log_enabled = true
);


template<typename Type> bool fetchROSParameter (
    const ros::NodeHandle& rosNH,
    const char* key,
    Type& dest,
    const bool& log_enabled = true
){
    XmlRpc::XmlRpcValue Output;

    try {
        if (!rosNH.getParam(key, Output)) {
            ROS_ERROR_STREAM(ROS_LOG_PREFIX 
                << "Fetching ROS parameter \"" << key << "\" failed.");
            return false;
        }
    } catch (XmlRpc::XmlRpcException e) {
        ROS_ERROR_STREAM(ROS_LOG_PREFIX 
            << "Fetching ROS parameter \"" << key << "\" failed with: "
            << e.getMessage());
            return false;
    }
    
    
    dest = static_cast<Type>(Output);
    if (log_enabled) ROS_DEBUG_STREAM(ROS_LOG_PREFIX 
        << "Fetched " << key << ": " << dest);
    return true;
}

template<typename Type> bool fetchROSArrayParameter (
    const char* key,
    const std::vector<Type>& dest,
    const ros::NodeHandle& rosNH,
    const bool& log_enabled = true
){
    XmlRpc::XmlRpcValue Output;

    try {
        if (!rosNH.getParam(key, Output)) {
            ROSERROR("Failed to fetch ROS parameter " << key);
            return false;
        }
    } catch (XmlRpc::XmlRpcException e) {
        ROSERROR("Failed to fetch ROS parameter " << key << ": " << e.getMessage());
        return false;
    }

    ROS_ASSERT(Output.getType() == XmlRpc::XmlRpcValue::TypeArray);
    std::string info = "[";

    //ROSDEBUG(GET_VAR_REP(Output.size()));
    
    const_cast<std::vector<Type>&>(dest).clear();
    for (int i = 0; i < Output.size(); ++i) {
        const_cast<std::vector<Type>&>(dest).push_back(static_cast<Type>(Output[i]));
        info += std::to_string(dest.back()) + ", ";
    }
    info.pop_back(); info.pop_back(); info += "]";
    if (log_enabled) ROSDEBUG("Fetched " << key << ": " << info);
    return true;
}


template<typename T> T capMinMax (T val, const T& min, const T& max) {
    return std::max( std::min (val, max), min);
}


void runMultiThreadedSpinner();
void runForgetfulSimulator();



void checkTimerPeriod (
    const std::string& tag,
    const ros::TimerEvent& te,
    const double& period
);

void playAudioFile (const std::string fpath);
void playAudio (const std::string txt);


template<typename T> bool callRosSrv (
    const std::string& tag,
    ros::ServiceClient& srv_cl,
    T& srv
) {
    if (!srv_cl.call(srv)) {
        ROS_ERROR_STREAM(tag << "Failed to call ROS service \"" << srv_cl.getService().c_str() << "\"");
        return false;
    }
    return true;
}


bool isDir (const std::string& dir_path);

bool isFile (const std::string& dir_path);

bool createDir (
    const std::string& tag,
    const std::string& dir_path
);

bool copyFile (
    const std::string& tag,
    const std::string& src_path,
    const std::string& dst_path
);

void saveCVMat (
    const std::string& tag,
    const std::string& file_path,
    const cv::Mat& cv_mat
);

bool isEmpty (
    const std::string& tag,
    const cv::Mat& cv_mat
);



template<typename T, std::size_t N> T getRandomElement(
    const std::array<T, N>& arr,
    const std::default_random_engine& dre
) {
    std::uniform_int_distribution<size_t> uid{0, N};
    return arr[uid(dre)];
}





std::string asSeqNo (const int& width, const int& no);
std::string asFixedFloat (const int& width, const int& prec, const double& no);

}