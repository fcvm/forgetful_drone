#pragma once

// forgetful_drone
#include "forgetful_drones/forgetful_helpers.hpp"
#include "forgetful_drones_msgs/BuildDroneRacingSimulation.h"
#include <minkindr_conversions/kindr_msg.h>
#include <minkindr_conversions/kindr_tf.h>

// ros
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/Empty.h>

// gazebo_ros
#include <gazebo_msgs/SpawnModel.h>
#include <gazebo_msgs/ModelStates.h>
#include <gazebo_msgs/DeleteModel.h>

// standard libraries
#include <assert.h>
#include <Eigen/Dense>
#include <chrono>
#include <cmath>
#include <cstring>
#include <fstream>
#include <iostream>
#include <iterator>
#include <sstream>
#include <thread>
#include <vector>
#include <memory>
#include <random>

// flightlib
#include "flightlib/bridges/unity_bridge.hpp"
#include "flightlib/bridges/unity_message_types.hpp"
#include "flightlib/common/quad_state.hpp"
#include "flightlib/common/types.hpp"
#include "flightlib/objects/quadrotor.hpp"
#include "flightlib/objects/static_gate.hpp"
#include "flightlib/sensors/rgb_camera.hpp"

// trajectory
#include <polynomial_trajectories/minimum_snap_trajectories.h>
#include <polynomial_trajectories/polynomial_trajectories_common.h>
#include <polynomial_trajectories/polynomial_trajectory.h>
#include <polynomial_trajectories/polynomial_trajectory_settings.h>
#include <quadrotor_common/trajectory_point.h>

// rpg quadrotor
#include <autopilot/autopilot_helper.h>
#include <autopilot/autopilot_states.h>
#include <quadrotor_common/parameter_helper.h>
#include <quadrotor_msgs/AutopilotFeedback.h>

#include "forgetful_drones/matplotlibcpp.h"









namespace forgetful_drone{ 
class ForgetfulSimulator{


public:

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    ForgetfulSimulator() : ForgetfulSimulator (ros::NodeHandle(), ros::NodeHandle("~")) {}
    ForgetfulSimulator (const ros::NodeHandle &nh, const ros::NodeHandle &pnh);
    virtual ~ForgetfulSimulator();


private: // ROS related

    // node handle
    ros::NodeHandle m_ROSRNH; // resolved to the node's namespace.
    ros::NodeHandle m_ROSPNH; // resolved to: <node's namespace>/<node's name>.
    const char* m_ROSNodeName;
    std::string m_ROSNodePath;

    // publisher
    image_transport::Publisher m_ITPub_RGBImg;
    image_transport::Publisher m_ITPub_DepthImg;
    image_transport::Publisher m_ITPub_SegmentationImg;
    image_transport::Publisher m_ITPub_OpticalFlowImg;
    unsigned int m_CamFrameCount;

    // subscriber
    ros::Subscriber m_ROSSub_StateEstimate;
        void ROSCB_StateEstimate(const nav_msgs::Odometry::ConstPtr& msg);

        void ROSCB_SimulatorStart(const std_msgs::Empty::ConstPtr& msg);
        void startSimulation();
        void ROSCB_SimulatorStop(const std_msgs::Empty::ConstPtr& msg);
        void stopSimulation();
        

    // main loop timer
    ros::Timer m_ROSTimer_MainLoop;
        void ROSCB_MainLoop(const ros::TimerEvent& event);

    // service servers
        bool ROSCB_buildDroneRacingSimulation(
            forgetful_drones_msgs::BuildDroneRacingSimulation::Request& req,
            forgetful_drones_msgs::BuildDroneRacingSimulation::Response& res
            );
        bool buildDroneRacingSimulation(
            const forgetful_drones_msgs::BuildDroneRacingSimulation::Request::_UnityScene_type UnityScene,
            const forgetful_drones_msgs::BuildDroneRacingSimulation::Request::_RacetrackSite_type& SceneSite,
            const forgetful_drones_msgs::BuildDroneRacingSimulation::Request::_RacetrackType_type& RacetrackType,
            const forgetful_drones_msgs::BuildDroneRacingSimulation::Request::_RaceGateType_type& RaceGateType
            );
        void computeRaceTrack_Figure8(const forgetful_drones_msgs::BuildDroneRacingSimulation::Request::_RacetrackType_type& RacetrackType);
        //void computeRaceTrack_Fig8Rand();
        //std::vector<std::shared_ptr<flightlib::StaticGate>> m_UnityGates;
        std::vector<std::shared_ptr<flightlib::StaticObject>> m_UnityGates;
        void spawnGatesInUnity();
        void spawnGatesInGazebo();
        void spawnGatesInRViz();
        bool m_DroneSpawned;
        const std::string m_DroneModelName;
        const std::string m_DroneModelDescription;
        void spawnDroneInGazebo();
        void spawnDroneInUnity();
        const double m_DroneCamFOV;
        const int m_DroneCamWidth;
        const int m_DroneCamHeight;

    // service clients
    ros::ServiceClient m_ROSSrvCl_GazeboSpawnURDFModel;


private: // Flightmare related

    // unity quadcopter
    std::shared_ptr<flightlib::Quadrotor> m_FM_DronePtr;
    std::shared_ptr<flightlib::RGBCamera> m_FM_RGBCam;
    flightlib::QuadState m_FM_DroneState;

    // Flightmare(Unity3D)
    std::shared_ptr<flightlib::UnityBridge> m_UnityBridgePtr;
    flightlib::SceneID m_UnitySceneID;
    bool m_UnityReady;
    const bool m_UnityRender_ON;
    flightlib::RenderMessage_t m_FM_UnityOutput;
    const std::string m_FM_PointCloudFileName;

    void setUnity(const bool RenderOn);
    void connectUnity();


private: // auxiliary variables

    const double m_MainLoopFreq;


private: // parameters

    const bool m_Debug_ON;
    const bool m_RacetrackDataGeneration_ON;

    std::vector<std::pair<const char*, const bool*>> m_KeysBoolPtrs
    {
        {"unity_render_on", &m_UnityRender_ON},
        {"debug_on", &m_Debug_ON},
        {"racetrack_data_generation_on", &m_RacetrackDataGeneration_ON},
    };
    
    std::vector<std::pair<const char*, const int*>> m_KeysIntPtrs
    {
        {"drone_cam_width" , &m_DroneCamWidth},
        {"drone_cam_height" , &m_DroneCamHeight},
        //{"randomized_sprint_number_of_gates" , &m_RandSprint_GateN},
        //{"randomized_sprint_min_number_of_gates", &m_RandSprint_GateMinN},
        //{"randomized_sprint_max_number_of_gates", &m_RandSprint_GateMaxN},
        //{"randomized_sprint_min_number_of_gates_in_section", &m_RandSprint_SectionGateMinN},
        //{"randomized_sprint_max_number_of_gates_in_section", &m_RandSprint_SectionGateMaxN},
        {"drone_marker_number_of_rotors", &m_RVizDroneRotorN},
        {"unity_scene_index", &m_UnitySceneIdx},
        {"racetrack_site_index", &m_UnitySiteIdx},
        {"racetrack_type_index", &m_RacetrackTypeIdx},
        {"race_gate_type_index", &m_RaceGateTypeIdx},
        
        
        
    };

    std::vector<std::pair<const char*, const double*>> m_KeysDoublePtrs
    {
        {"main_loop_freq", &m_MainLoopFreq},
        {"drone_cam_field_of_view", &m_DroneCamFOV},
        {"randomized_figure8_max_axial_shift", &m_RandFig8_MaxAxShift},
        {"randomized_figure8_max_yaw_twist", &m_RandFig8_MaxYawTwist},
        {"randomized_figure8_min_scale", &m_RandFig8_MinScale},
        {"randomized_figure8_max_scale", &m_RandFig8_MaxScale},
        //{"randomized_sprint_min_gate_spacing", &m_RandSprint_MinSpacing},
        //{"randomized_sprint_unidirectional_standard_deviation_of_gate_spacing", &m_RandSprint_UnidirStdDevSpacing},
        //{"randomized_sprint_left_right_mean_yaw_between_gates", &m_RandSprint_LRMeanYaw},
        //{"randomized_sprint_left_right_standard_deviation_yaw_between_gates", &m_RandSprint_LRStdDevYaw},
        //{"randomized_sprint_mean_pitch_between_gates", &m_RandSprint_MeanPitch},
        //{"randomized_sprint_standard_deviation_pitch_between_gates", &m_RandSprint_StdDevPitch},
        //{"dynamic_gates_max_axial_amplitude", &m_DynGates_MaxAxAmp},
        //{"dynamic_gates_max_axial_frequency", &m_DynGates_MaxAxFreq},
        {"drone_marker_arm_length", &m_RVizDroneArmLength},
        {"drone_marker_body_width", &m_RVizDroneBodyWidth},
        {"drone_marker_body_height", &m_RVizDroneBodyHeight},
        {"drone_marker_scale", &m_RVizDroneScale},
        //{"simulator_loop_nominal_period_time", &m_SimulatorLoopTime}         
    };

    std::vector<std::pair<const char*, const std::string*>> m_KeysStringPtrs
    {
        {"drone_model_description", &m_DroneModelDescription}
    };






private: // old ROS

    // ----
    //ros::Subscriber m_ROSSub_DynamicGatesSwitch;
    //ros::Subscriber m_ROSSub_GazeboModelStates;    // Subscribes to ROS topic "/gazebo/model_states"
    //ros::Publisher m_ROSPub_GazeboSetModelState;        // gazebo_msgs::ModelState -> "/gazebo/set_model_state". Used in: respawnGazeboGateModelsWithRandomAxialShifts(), moveGazeboGateModels()
    ros::Publisher m_ROSPub_RVizGates;
    ros::Publisher m_ROSPub_RVizDrone;

    ros::ServiceClient m_ROSSrvCl_GazeboDeleteModel; // gazebo_msgs::DeleteModel -> "/gazebo/delete_model"
    //ros::ServiceClient m_ROSSrvCl_GazeboResetSimulation; // std_srvs::Empty -> "/gazebo/reset_simulation"
    ros::ServiceClient m_ROSSrvCl_GazeboSpawnSDFModel; // gazebo_msgs::SpawnModel -> "/gazebo/spawn_sdf_model"
    

    ros::ServiceServer m_ROSSrvSv_BuildDroneRacingSimulation; // "sim_rand_sprint_race" <- forgetful_drones_msgs::SimRace
    
    //ros::Timer m_ROSTimer_SimulatorLoop;
    ros::Subscriber m_ROSSub_SimulatorStart;
    ros::Subscriber m_ROSSub_SimulatorStop;
    

    
    
    bool m_SimulationReady;
    const std::string m_GroundPlaneID; // Name for ground plane model in Gazebo.

    
    int m_UnitySceneIdx;
    int m_UnitySiteIdx;
    const int m_RacetrackTypeIdx;
    const int m_RaceGateTypeIdx;
    std::string m_GatePrefabID;
    

private:

    void debugNode();

    // Rand Figure 8 Race Specific
    const double m_RandFig8_MaxAxShift;
    const double m_RandFig8_MaxYawTwist;
    const double m_RandFig8_MinScale;
    const double m_RandFig8_MaxScale;

    // Rand Sprint Race Specific
    //const int m_RandSprint_GateN;
    //const int m_RandSprint_GateMinN;
    //const int m_RandSprint_GateMaxN;    
    //const int m_RandSprint_SectionGateMinN;
    //const int m_RandSprint_SectionGateMaxN;
    //const double m_RandSprint_MinSpacing;
    //const double m_RandSprint_UnidirStdDevSpacing;
    //const double m_RandSprint_LRMeanYaw;
    //const double m_RandSprint_LRStdDevYaw;
    //const double m_RandSprint_MeanPitch;
    //const double m_RandSprint_StdDevPitch;

    // Dynamic Gate Specific
    //const double m_DynGates_MaxAxAmp;
    //const double m_DynGates_MaxAxFreq;
    

    // Drone RVIZ
    const int m_RVizDroneRotorN;
    const double m_RVizDroneArmLength;
    const double m_RVizDroneBodyWidth;
    const double m_RVizDroneBodyHeight;
    const double m_RVizDroneScale;
    const std::string m_RVizDroneFrameID;



    //const double m_SimulatorLoopTime;
    std::default_random_engine m_RandEngine;

    

private:
    void spawnDroneInRViz();

    forgetful_drone::Pose m_RacetrackPose_Flightmare;
    forgetful_drone::Pose m_RacetrackPose_Gazebo;
    float m_GateWaypointHeight;
    std::vector<forgetful_drone::Pose> m_GatesInitPose_Flightmare;
    std::vector<forgetful_drone::Pose> m_GatesInitPose_Gazebo;
    forgetful_drone::Pose m_DroneInitPose_Flightmare;
    forgetful_drone::Pose m_DroneInitPose_Gazebo;
    Eigen::Vector3f m_GroundPlaneOrigin_Gazebo;
    Eigen::Vector2f m_GroundPlaneSize_Gazebo;
    


////////////////////////////////////////////////////////
private:// ROS CALLBACKS & Service & Timer Functions //
//////////////////////////////////////////////////////


    // /// \brief Sets "m_GazeboModelStates" to latest message on ROS topic "/gazebo/model_states".
    //void ROS_CB_GazeboModelStates( const gazebo_msgs::ModelStates::ConstPtr& msg );
    // void ROSCB_DynamicGatesSwitch( const std_msgs::Bool::ConstPtr& msg );
    // bool ROSSrvFunc_buildDroneRacingSimulation(
    //     forgetful_drones_msgs::BuildDroneRacingSimulation::Request& req,
    //     forgetful_drones_msgs::BuildDroneRacingSimulation::Response& res
    //     );
        void deleteAllGazeboModelsExceptDrone();
    //     void computeGatePoses_Fig8Rand();
    //     void computeGatePoses_SprintRand();
    //     void setGatesID();
    //     void initRandParamsOfDynGates();
         void computeGazeboGroundPlane();
    //     void spawnRandGatesInGazeboAndRVIZ();


    //     void spawnDroneInGazebo();
    //     void resetDroneModelState();
    // void ROSTimerFunc_SimulatorLoop( const ros::TimerEvent& MainLoop_TimerEvent );
    // void visualizeGatesInRVIZ();
    // std::string m_Gate_DAEFilePath;
    std::string m_GateSTLFilePath;
    std::string m_GateSDFFilePath;



private: // members

    visualization_msgs::Marker m_RVizGate;
    visualization_msgs::MarkerArray m_RVizGates;
    //std::vector< std::string > m_GatesID;
    //std::vector< Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > m_DynGates_AxAmps;
    //std::vector< Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > m_DynGates_AxFreqs;
    //std::vector< Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > m_DynGates_InitAxPhases;

    



    std::string getGroundPlaneSDF(const Eigen::Vector2f& GroundPlaneSize);
    void spawnGroundPlaneInGazebo();


    void setRacetrackPose();


private: // friends

    friend bool fetchROSParameters
    (
        const ros::NodeHandle& ROSNH,
        const std::vector<std::pair<const char*, const bool*>>& KeysAndBoolOutputs,
        const std::vector<std::pair<const char*, const int*>>& KeysAndIntOutputs,
        const std::vector<std::pair<const char*, const double*>>& KeysAndDoubleOutputs,
        const std::vector<std::pair<const char*, const std::string*>>& KeysAndStringOutputs
    );


}; // class ForgetfulSimulator 
} // namespace forgetful_drone