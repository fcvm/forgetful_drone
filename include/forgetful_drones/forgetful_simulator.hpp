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

// gazebo_ros
#include <gazebo_msgs/SpawnModel.h>

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

// flightlib
#include "flightlib/bridges/unity_message_types.hpp"

#include "flightlib/bridges/unity_bridge.hpp"

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


/*
class manual_timer {
  std::chrono::high_resolution_clock::time_point t0;
  double timestamp{0.0};

 public:
  void start() { t0 = std::chrono::high_resolution_clock::now(); }
  void stop() {
    timestamp = std::chrono::duration<double>(
                  std::chrono::high_resolution_clock::now() - t0)
                  .count() *
                1000.0;
  }
  const double &get() { return timestamp; }
};

// void setupQuad();
bool setUnity(const bool render);
bool connectUnity(void);

// unity quadrotor
std::shared_ptr<flightlib::Quadrotor> quad_ptr_;
std::shared_ptr<flightlib::RGBCamera> rgb_camera_;
flightlib::QuadState quad_state_;

// Flightmare(Unity3D)
std::shared_ptr<flightlib::UnityBridge> unity_bridge_ptr_;
flightlib::SceneID scene_id_{flightlib::UnityScene::WAREHOUSE};
bool unity_ready_{false};
bool unity_render_{true};
flightlib::RenderMessage_t unity_output_;
uint16_t receive_id_{0};
*/









namespace forgetful_drone{ class ForgetfulSimulator{


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

    // publisher
    image_transport::Publisher m_ITPub_RGBImg;
    image_transport::Publisher m_ITPub_DepthImg;
    image_transport::Publisher m_ITPub_SegmentationImg;
    image_transport::Publisher m_ITPub_OpticalFlowImg;
    unsigned int m_CamFrameCount;

    // subscriber
    ros::Subscriber m_ROSSub_StateEstimate;
        void ROSCB_StateEstimate(const nav_msgs::Odometry::ConstPtr& msg);

    // main loop timer
    ros::Timer m_ROSTimer_MainLoop;
        void ROSCB_MainLoop(const ros::TimerEvent& event);

    // service servers
        bool ROSCB_buildDroneRacingSimulation(
            forgetful_drones_msgs::BuildDroneRacingSimulation::Request& req,
            forgetful_drones_msgs::BuildDroneRacingSimulation::Response& res
            );
        bool ForgetfulSimulator::buildDroneRacingSimulation(
            const geometry_msgs::Pose& RaceTrackPose_WorldRF,
            const forgetful_drones_msgs::BuildDroneRacingSimulation::Request::_RaceType_type& RaceTrackType,
            const double& GateWaypointHeight, //-> GateType
            const forgetful_drones_msgs::BuildDroneRacingSimulation::Request::_DynGatesActivated_type& GatesDynamic
            );
        void compGatesAndDroneInitPose_Fig8Det( 
            const geometry_msgs::Pose& in_RaceTrackPose_WorldRF, 
            const double& in_GatesWaypointHeight,
            std::vector<forgetful_drone::Pose>& out_GatesInitPose_WorldRF,
            forgetful_drone::Pose& out_DroneInitPose_WorldRF
            );
        std::vector<std::shared_ptr<flightlib::StaticGate>> m_UnityGates;
        void spawnGatesInUnity(const std::vector<forgetful_drone::Pose> GatesInitPose_WorldRF);
        bool m_DroneSpawned;
        const std::string m_DroneModelName;
        const std::string m_DroneModelDescription;
        void spawnDroneInGazebo(const forgetful_drone::Pose& DroneInitPose_WorldRF);
        void spawnDroneInUnity(const forgetful_drone::Pose& DroneInitPose_WorldRF);
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
    std::shared_ptr<flightlib::UnityBridge> m_FM_UnityBridgePtr;
    flightlib::SceneID m_FM_SceneID;
    bool m_FM_UnityReady;
    const bool m_FM_UnityRenderOn;
    flightlib::RenderMessage_t m_FM_UnityOutput;
    uint16_t m_FM_receive_id_;

    void setUnity(const bool RenderOn);
    void connectUnity();


private: // auxiliary variables

    double m_MainLoopFreq;


private: // parameters

    std::vector<std::pair<const char*, const bool*>> m_KeysBoolPtrs
    {
        {"unity_render", &m_FM_UnityRenderOn},
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
        //{"drone_marker_number_of_rotors", &m_Drone_RotorN},
    };

    std::vector<std::pair<const char*, const double*>> m_KeysDoublePtrs
    {
        {"main_loop_freq", &m_MainLoopFreq},
        {"drone_cam_field_of_view", &m_DroneCamFOV},
        //{"gate_waypoint_height_above_base", &m_GateWaypointHeight},
        //{"gate_base_min_altitude", &m_GateBaseMinAltitude},
        //{"randomized_figure8_max_axial_shift", &m_RandFig8_MaxAxShift},
        //{"randomized_figure8_min_scale", &m_RandFig8_MinScale},
        //{"randomized_figure8_max_scale", &m_RandFig8_MaxScale},
        //{"randomized_sprint_min_gate_spacing", &m_RandSprint_MinSpacing},
        //{"randomized_sprint_unidirectional_standard_deviation_of_gate_spacing", &m_RandSprint_UnidirStdDevSpacing},
        //{"randomized_sprint_left_right_mean_yaw_between_gates", &m_RandSprint_LRMeanYaw},
        //{"randomized_sprint_left_right_standard_deviation_yaw_between_gates", &m_RandSprint_LRStdDevYaw},
        //{"randomized_sprint_mean_pitch_between_gates", &m_RandSprint_MeanPitch},
        //{"randomized_sprint_standard_deviation_pitch_between_gates", &m_RandSprint_StdDevPitch},
        //{"dynamic_gates_max_axial_amplitude", &m_DynGates_MaxAxAmp},
        //{"dynamic_gates_max_axial_frequency", &m_DynGates_MaxAxFreq},
        //{"wall_buffer_distance_to_gate", &m_WallBufferDistance},
        //{"wall_height", &m_WallHeight},
        //{"wall_thickness", &m_WallThickness},
        //{"drone_initial_position_x", &m_Drone_InitX},            
        //{"drone_initial_position_y", &m_Drone_InitY},
        //{"drone_initial_position_z", &m_Drone_InitZ},
        //{"drone_initial_position_yaw", &m_Drone_InitYaw},
        //{"drone_marker_arm_length", &m_Drone_ArmLength},
        //{"drone_marker_body_width", &m_Drone_BodyWidth},
        //{"drone_marker_body_height", &m_Drone_BodyHeight},
        //{"drone_marker_scale", &m_Drone_Scale},
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
    //ros::Publisher m_ROSPub_RvizGates;
    //ros::Publisher m_ROSPub_RvizDrone;

    //ros::ServiceClient m_ROSSrvCl_GazeboDeleteModel; // gazebo_msgs::DeleteModel -> "/gazebo/delete_model"
    //ros::ServiceClient m_ROSSrvCl_GazeboResetSimulation; // std_srvs::Empty -> "/gazebo/reset_simulation"
    //ros::ServiceClient m_ROSSrvCl_GazeboSpawnSDFModel; // gazebo_msgs::SpawnModel -> "/gazebo/spawn_sdf_model"
    

    //ros::ServiceServer m_ROSSrvSv_BuildDroneRacingSimulation; // "sim_rand_sprint_race" <- forgetful_drones_msgs::SimRace
    
    //ros::Timer m_ROSTimer_SimulatorLoop;


    
    //std::uniform_real_distribution< double > m_UniRealDistri_0To1;
    //bool m_SimulationReady;
    //const std::string m_Models_DirPath; // Path to directory of this ROS package (forgetful_drones).
    //const std::string m_Gate_NamePrefix; // Prefix for names of gate models in Gazebo.
    //const std::string m_Gate_TexDirPath;
    //const std::vector< std::string > m_Gate_TexFileNames; // Path to directory of all texture resources
    //const std::string m_Gate_DAEDirPath;
    //const std::vector< std::string > m_Gate_DAEFileNames; // Path to directory of all .dae resources
    //const std::string m_Gate_STLDirPath;
    //const std::vector< std::string > m_Gate_STLFileNames; // Path to directory of all .sdf resources
    //const std::string m_Gate_RandIllumScriptFilePath; // Path to python script that sets the tags ´abient´ and ´emission´ to command line args.
    //const std::string m_Gate_TmpDirPath;
    //const std::string m_Ground_Name; // Name for ground plane model in Gazebo.
    //const std::string m_Ground_TexDirPath;
    //const std::vector< std::string > m_Ground_TexFileNames; // Path to directory of all texture resources
    //const std::string m_Ground_TmpDirPath;
    //const std::string m_Wall_NamePrefix; // Prefix for names of wall models in Gazebo.
    //const std::string m_Wall_TexDirPath;
    //const std::vector< std::string > m_Wall_TexFileNames; // Path to directory of all texture resources
    //const std::string m_Wall_TmpDirPath;
    
    
    

private:

    // Race track generic
    //const double m_GateWaypointHeight;
    //const double m_GateBaseMinAltitude;

    // Rand Figure 8 Race Specific
    //const double m_RandFig8_MaxAxShift;
    //const double m_RandFig8_MinScale;
    //const double m_RandFig8_MaxScale;

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
    
    // Environment Specific (until now only ground and walls)
    //const double m_WallBufferDistance;
    //const double m_WallHeight;
    //const double m_WallThickness;

    // Drone RVIZ
    //const int m_Drone_RotorN;
    //const double m_Drone_ArmLength;
    //const double m_Drone_BodyWidth;
    //const double m_Drone_BodyHeight;
    //const double m_Drone_Scale;
    //const std::string m_Drone_FrameID;
    //const Eigen::Vector3d m_Drone_InitPosition;
    //const double m_Drone_InitX;
    //const double m_Drone_InitY;
    //const double m_Drone_InitZ;
    //const double m_Drone_InitYaw;
    




    //const double m_SimulatorLoopTime;
    //std::default_random_engine m_RandEngine;

    










private:
    //void rvizDrone();



////////////////////////////////////////////////////////
private:// ROS CALLBACKS & Service & Timer Functions //
//////////////////////////////////////////////////////


    // /// \brief Sets "m_GazeboModelStates" to latest message on ROS topic "/gazebo/model_states".
    // void ROS_CB_GazeboModelStates( const gazebo_msgs::ModelStates::ConstPtr& msg );
    // void ROSCB_DynamicGatesSwitch( const std_msgs::Bool::ConstPtr& msg );
    // bool ROSSrvFunc_buildDroneRacingSimulation(
    //     forgetful_drones_msgs::BuildDroneRacingSimulation::Request& req,
    //     forgetful_drones_msgs::BuildDroneRacingSimulation::Response& res
    //     );
    //     void deleteAllGazeboModelsExceptDrone();
    //     void compGatesAndDroneInitPose_Fig8Det();
    //     void computeGatePoses_Fig8Rand();
    //     void computeGatePoses_SprintRand();
    //     void setGatesID();
    //     void initRandParamsOfDynGates();
    //     void computePositionAndSizeOfGroundAndWalls();
    //     void spawnRandGatesInGazeboAndRVIZ();


    //     void spawnDroneInGazebo();
    //     void resetDroneModelState();
    // void ROSTimerFunc_SimulatorLoop( const ros::TimerEvent& MainLoop_TimerEvent );
    // void visualizeGatesInRVIZ();
    // std::string m_Gate_DAEFilePath;



private: // members

    //visualization_msgs::Marker m_GateMarker;
    //visualization_msgs::MarkerArray m_GateMarkerArray;
    //gazebo_msgs::ModelStates m_GazeboModelStates;
    //std::vector< Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > m_GatesInitPos_WorldRF;
    //std::vector< double > m_GatesInitYaw_WorldRF;
    //std::vector< std::string > m_GatesID;
    //std::vector< Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > m_DynGates_AxAmps;
    //std::vector< Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > m_DynGates_AxFreqs;
    //std::vector< Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > m_DynGates_InitAxPhases;
    //Eigen::Vector2d m_Ground_BasePoint;
    //Eigen::Vector2d m_Ground_Size;
    //std::array< Eigen::Vector3d, 4 > m_Walls_BasePoint;
    //std::array< Eigen::Vector3d, 4 > m_Walls_Size;
    //bool m_DroneSpawned;
    



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