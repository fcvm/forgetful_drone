/* Notes



*/

/* Abbreviations
AOV: Angle of View
IRF: Image Reference Frame
GRF: Goal Reference Frame
GT: Global Trajectory
LT: Local Trajectory


Cam: Camera
Deg: Degree
Rad: Radian
Dist: Distance
Max: Maximum
Min: Minimum
Pos: Position
Vel: Velocity
Acc: Acceleration
Vis: Visualization
Traj: Trajectory
Freq: Frequency
Nav: Navigation
Temp: Temporary
Sim: Simulation
Env: Environment
Dir: Directory
Amp: Amplitude
Curr: Current
Proj: Projection
Ref: Reference
Idx: Index
Img: Image
Mtx: Mutex

*/





#pragma once



// C++ standard libraries
#include <cstring> // provides functions (e.g. strlen, strcpy) for dealing with C-style strings (null-terminated arrays of characters). It's the C++ version of the classic string.h header from C.
#include <fstream> // Input/output stream class to operate on files.
#include <mutex>   // The mutex class is a synchronization primitive that can be used to protect shared data from being simultaneously accessed by multiple threads.
#include <array>
#include <functional>

// Eigen: C++ template library for linear algebra: matrices, vectors, numerical solvers, and related algorithms
#include <Eigen/Dense>
#include <Eigen/StdVector>

// OpenCV: library of programming functions mainly aimed at real-time computer vision.
#include <opencv2/imgcodecs/imgcodecs.hpp>

// ROS
#include <ros/ros.h> // Client library to quickly interface with ROS Topics, Services, and Parameters.
// ROS Packages
#include <autopilot/autopilot_states.h> // enum class autopilot::States
#include <cv_bridge/cv_bridge.h>        // Convert between ROS Image messages and OpenCV images.
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <kindr/minimal/quat-transformation.h> // This class is a frame transformation built from a quaternion and a point that takes points from frame B to frame A.
#include <minkindr_conversions/kindr_msg.h>
#include <minkindr_conversions/kindr_tf.h>
#include <polynomial_trajectories/minimum_snap_trajectories.h>
#include <polynomial_trajectories/polynomial_trajectories_common.h>
#include <quadrotor_common/geometry_eigen_conversions.h> // Functions to convert between Eigen and geometry_msgs
#include <quadrotor_common/math_common.h>                // various functions
#include <quadrotor_common/parameter_helper.h>           // Functions to fetch parameters from ROS parameter server
#include <quadrotor_common/trajectory_point.h>           // struct TrajectoryPoint
#include <rapid_trajectories/RapidTrajectoryGenerator.h>
#include <rapid_trajectories/Vec3.h>  // class Vec3
#include <tf/transform_broadcaster.h> // This class provides an easy way to publish coordinate frame transform information
#include <tf/transform_listener.h>    // This class automatically subscribes to ROS transform messages.
                                      // ROS Messages
// gazebo_msgs/*
#include <gazebo_msgs/ModelStates.h>
#include <gazebo_msgs/SetModelState.h>
// geometry_msgs
#include <geometry_msgs/PoseStamped.h>
// nav_msgs
#include <nav_msgs/Odometry.h>
// quadrotor_msgs
#include <quadrotor_msgs/AutopilotFeedback.h>
#include <quadrotor_msgs/TrajectoryPoint.h>
// sensor_msgs
#include <sensor_msgs/Image.h>
// std_msgs/*
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int16.h>
#include <std_msgs/String.h>
// visualization_msgs
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>










#include <boost/range/adaptor/indexed.hpp>
#include <limits> 
#include <random>

#include <initializer_list>
#include "geometry_msgs/Point.h"

#include "trajectory_generation_helper/polynomial_trajectory_helper.h"


#include "dirent.h"
#include <ros/package.h>
#include <stdlib.h>


#include <chrono>
#include <gazebo_msgs/SpawnModel.h>
#include <gazebo_msgs/DeleteModel.h>
#include <std_srvs/Empty.h>
#include <tf/transform_datatypes.h>



// sudo apt-get install libcgal-dev
//#include <CGAL/Cartesian.h>
//#include <CGAL/Random.h>
//#include <CGAL/Exact_rational.h>
//#include <CGAL/Min_sphere_of_spheres_d.h>





//#include "forgetful_drones_msgs/SimRace.h"
#include "forgetful_drones_msgs/BuildDroneRacingSimulation.h"

#include <sys/stat.h>

#include <filesystem>






namespace forgetful_drone
{
    class ForgetfulSimulator
    {

    //////////////////////////////////
    public:// CON- AND DESTRUCTORS //
    ////////////////////////////////


        ForgetfulSimulator() : ForgetfulSimulator( 
            ros::NodeHandle(), 
            ros::NodeHandle("~") 
            ) {}

        ForgetfulSimulator(
            const ros::NodeHandle &nh, 
            const ros::NodeHandle &pnh
            );

        virtual ~ForgetfulSimulator();



    /////////////////////////////////////
    private:// CONST MEMBER VARIABLES //
    ///////////////////////////////////
        

        ros::NodeHandle m_ROSRNH; // ROS node handle that is resolved to the node's namespace.
        ros::NodeHandle m_ROSPNH; // ROS node handle that is resolved to the namespace: <node's namespace>/<node's name>.
        
        std::uniform_real_distribution< double > m_UniRealDistri_0To1;
        bool m_SimulationReady;

        
        
        const std::string m_Models_DirPath; // Path to directory of this ROS package (forgetful_drones).
        
        const std::string m_Gate_NamePrefix; // Prefix for names of gate models in Gazebo.
        const std::string m_Gate_TexDirPath;
        const std::vector< std::string > m_Gate_TexFileNames; // Path to directory of all texture resources
        const std::string m_Gate_DAEDirPath;
        const std::vector< std::string > m_Gate_DAEFileNames; // Path to directory of all .dae resources
        const std::string m_Gate_STLDirPath;
        const std::vector< std::string > m_Gate_STLFileNames; // Path to directory of all .sdf resources
        const std::string m_Gate_RandIllumScriptFilePath; // Path to python script that sets the tags ´abient´ and ´emission´ to command line args.
        const std::string m_Gate_TmpDirPath;
        
        const std::string m_Ground_Name; // Name for ground plane model in Gazebo.
        const std::string m_Ground_TexDirPath;
        const std::vector< std::string > m_Ground_TexFileNames; // Path to directory of all texture resources
        const std::string m_Ground_TmpDirPath;

        const std::string m_Wall_NamePrefix; // Prefix for names of wall models in Gazebo.
        const std::string m_Wall_TexDirPath;
        const std::vector< std::string > m_Wall_TexFileNames; // Path to directory of all texture resources
        const std::string m_Wall_TmpDirPath;

        const std::string m_Drone_Name;
        
        
        
        
        
        ros::Subscriber m_ROSSub_DynamicGatesSwitch;
        ros::Subscriber m_ROSSub_GazeboModelStates;    // Subscribes to ROS topic "/gazebo/model_states"
        
        ros::Publisher m_ROSPub_GazeboSetModelState;        // gazebo_msgs::ModelState -> "/gazebo/set_model_state". Used in: respawnGazeboGateModelsWithRandomAxialShifts(), moveGazeboGateModels()
        ros::Publisher m_ROSPub_RvizGates;
        ros::Publisher m_ROSPub_RvizDrone;

        ros::ServiceClient m_ROSSrvCl_GazeboDeleteModel; // gazebo_msgs::DeleteModel -> "/gazebo/delete_model"
        ros::ServiceClient m_ROSSrvCl_GazeboResetSimulation; // std_srvs::Empty -> "/gazebo/reset_simulation"
        ros::ServiceClient m_ROSSrvCl_GazeboSpawnSDFModel; // gazebo_msgs::SpawnModel -> "/gazebo/spawn_sdf_model"
        ros::ServiceClient m_ROSSrvCl_GazeboSpawnURDFModel;

        ros::ServiceServer m_ROSSrvSv_BuildDroneRacingSimulation; // "sim_rand_sprint_race" <- forgetful_drones_msgs::SimRace
        
        ros::Timer m_ROSTimer_SimulatorLoop;






        
    
        // Race track generic
        const double m_GateWaypointHeight;
        const double m_GateBaseMinAltitude;

        // Rand Figure 8 Race Specific
        const double m_RandFig8_MaxAxShift;
        const double m_RandFig8_MinScale;
        const double m_RandFig8_MaxScale;
    
        // Rand Sprint Race Specific
        const int m_RandSprint_GateN;
        const int m_RandSprint_GateMinN;
        const int m_RandSprint_GateMaxN;    
        const int m_RandSprint_SectionGateMinN;
        const int m_RandSprint_SectionGateMaxN;
        const double m_RandSprint_MinSpacing;
        const double m_RandSprint_UnidirStdDevSpacing;
        const double m_RandSprint_LRMeanYaw;
        const double m_RandSprint_LRStdDevYaw;
        const double m_RandSprint_MeanPitch;
        const double m_RandSprint_StdDevPitch;

        // Dynamic Gate Specific
        const double m_DynGates_MaxAxAmp;
        const double m_DynGates_MaxAxFreq;
        
        // Environment Specific (until now only ground and walls)
        const double m_WallBufferDistance;
        const double m_WallHeight;
        const double m_WallThickness;

        // Drone RVIZ
        const int m_Drone_RotorN;
        const double m_Drone_ArmLength;
        const double m_Drone_BodyWidth;
        const double m_Drone_BodyHeight;
        const double m_Drone_Scale;
        const std::string m_Drone_FrameID;
        const Eigen::Vector3d m_Drone_InitPosition;
        const double m_Drone_InitX;
        const double m_Drone_InitY;
        const double m_Drone_InitZ;
        const double m_Drone_InitYaw;
        const std::string m_Drone_URDF;




        const double m_SimulatorLoopTime;



        // Drone
        
        

        
        std::default_random_engine m_RandEngine;

        
        
        // DEBUG

        //ros::Timer m_ROS_Timer_DEBUG;
        //ros::ServiceClient SrvCl_DEBUG;
        //void runDebugLoop( const ros::TimerEvent& MainLoop_TimerEvent );

        //DEBUG


    





    ////////////////////////////////////
    private:// INITIALIZER FUNCTIONS //
    //////////////////////////////////

    //bool initROSParams();
    bool AllPathsExists();
    bool AllResourcesValid();
    bool generateGateTmpFiles();

    void generateGroundTmpFiles();
    std::string getGroundMatScript( const int& Texture_i );
    void generateWallTmpFiles();
    std::string getWallMatScript( const int& Texture_i );

    void rvizDrone();



    ////////////////////////////////////////////////////////
    private:// ROS CALLBACKS & Service & Timer Functions //
    //////////////////////////////////////////////////////
    
    
        /// \brief Sets "m_GazeboModelStates" to latest message on ROS topic "/gazebo/model_states".
        void ROS_CB_GazeboModelStates( const gazebo_msgs::ModelStates::ConstPtr& msg );
        

        void ROSCB_DynamicGatesSwitch( const std_msgs::Bool::ConstPtr& msg );


        bool ROSSrvFunc_buildDroneRacingSimulation(
            forgetful_drones_msgs::BuildDroneRacingSimulation::Request& req,
            forgetful_drones_msgs::BuildDroneRacingSimulation::Response& res
            );

            void deleteAllGazeboModelsExceptDrone();

            void computeGatePoses_Fig8Det();
            void computeGatePoses_Fig8Rand();
            void computeGatePoses_SprintRand();

            void setGateModelNames();

            void initRandParamsOfDynGates();

            void computePositionAndSizeOfGroundAndWalls();

            void spawnRandGatesInGazeboAndRVIZ();
                //bool generateRandGateFiles();
                std::string getRandGateSDF( size_t& OUT_TmpGate_Idx );
                std::string getGateSDF( const size_t& TexRes_i );
                //std::string getMovingGateSDF( const size_t& Gate_i );

            void spawnRandGroundInGazebo();
                std::string getGroundSDF( const size_t& Texture_i );
                std::string getRandGroundSDF();
        
            void spawnRandWallsInGazebo();
                std::string getWallSDF( const size_t& Wall_i, const size_t& Texture_i );
                std::string getRandWallSDF( const size_t& Wall_i );

            void spawnDroneInGazebo();

            void resetDroneModelState();

        void ROSTimerFunc_SimulatorLoop( const ros::TimerEvent& MainLoop_TimerEvent );

        void visualizeGatesInRVIZ();
        std::string m_Gate_DAEFilePath;


    /////////////////////////////////////////
    private:// NON-CONST MEMBER VARIABLES //
    ///////////////////////////////////////

    visualization_msgs::Marker m_GateMarker;
    //visualization_msgs::MarkerArray m_DroneMarkerArray;
    visualization_msgs::MarkerArray m_GateMarkerArray;
    gazebo_msgs::ModelStates m_GazeboModelStates;

    std::vector< Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > m_GatesInitPositions;
    std::vector< double > m_Gates_Yaw;
    std::vector< std::string > m_Gates_ModelName;
    std::vector< Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > m_DynGates_AxAmps;
    std::vector< Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > m_DynGates_AxFreqs;
    std::vector< Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > m_DynGates_InitAxPhases;
    
    Eigen::Vector2d m_Ground_BasePoint;
    Eigen::Vector2d m_Ground_Size;

    std::array< Eigen::Vector3d, 4 > m_Walls_BasePoint;
    std::array< Eigen::Vector3d, 4 > m_Walls_Size;


    bool m_DroneSpawned;



    //////////////////////
    private:// FRIENDS //
    ////////////////////

    friend
    bool
    fetchROSParameters
    (
        const ros::NodeHandle& ROSNH,
        const std::vector<std::pair<const char*, const bool*>>& KeysAndBoolOutputs,
        const std::vector<std::pair<const char*, const int*>>& KeysAndIntOutputs,
        const std::vector<std::pair<const char*, const double*>>& KeysAndDoubleOutputs,
        const std::vector<std::pair<const char*, const std::string*>>& KeysAndStringOutputs
    );

};
} 