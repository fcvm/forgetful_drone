/* Notes

???*??? open questions
TODO<...>
LSTM-CNN: notice that gate was passed, also predict velocity andacceleration

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


// ??? Pos Vel Acc, Velocity -> Speed, In Out -> IN OUT, mark methods as const???
// ??? WRF, ARF, DRF, GRF: World, Autopilot, Drone, Goal Reference Frame ???
// Struct for states and poses in WRF and ARF.


#pragma once

//UNUSED INCLUDES
//#include <string.h>

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

namespace RQTG = RapidQuadrocopterTrajectoryGenerator;


namespace forgetful_drone
{
    class ForgetfulDrones
    {

    /////////////////////////////////////////////////////////////////////////////////////////
    public:// CON- AND DESTRUCTORS /////////////////////////////////////////////////////////


        /// The default constructor calls its own overload:
        /// -> ForgetfulDrones(ros::NodeHandle(), ros::NodeHandle("~"))
        ForgetfulDrones() : ForgetfulDrones(ros::NodeHandle(), ros::NodeHandle("~")) {}

        
        /// Constructor that
        /// 1) hfkjhfk
        /// 2) ljhljfskj
        /// @param nh ROS node handle that is relatively resolved
        /// @param pnh ROS node handle that is privately resolved
        ForgetfulDrones(const ros::NodeHandle &nh, const ros::NodeHandle &pnh);

        virtual ~ForgetfulDrones();


    /////////////////////////////////////////////////////////////////////////////////////////
    private:// ENUM CLASSES ////////////////////////////////////////////////////////////////


        /// Contains all possible states to which the autopilot can be set:
        /// 1) OFF
        /// 2) HOVER
        /// 3) RACING
        enum class AutopilotStates
        {
            OFF,
            HOVER,
            RACING
        };

        /// Contains all possible colors that can be used in visualization:
        /// 1) RED
        /// 2) GREEN
        /// 3) BLUE
        /// 4) YELLOW
        /// 5) PURPLE
        /// 6) WHITE
        /// 7) BLACK
        enum class VisualizationColors
        {
            RED,
            GREEN,
            BLUE,
            YELLOW,
            PURPLE,
            WHITE,
            BLACK
        };

        /// Contains all possible constraint types on the end state of a local trajectory to be generated:
        /// 1) POSITION
        /// 2) POSITION_VELOCITY
        /// 3) POSITION_VELOCITY_ACCELERATION
        enum class TrajectoryEndStateConstraintTypes
        {
            POSITION,
            POSITION_VELOCITY,
            POSITION_VELOCITY_ACCELERATION
        };

        /// Contains all possible modes of navigation
        /// 1) DATACOLLECTION
        /// 2) TESTING
        enum class FlightMissions
        {
            DATACOLLECTION,
            TESTING
        };

        /// Contains all possible types of trajectories
        /// 1) GLOBAL
        /// 2) LOCAL
        enum class TrajectoryTypes
        {
            GLOBAL, LOCAL
        };


    /////////////////////////////////////////////////////////////////////////////////////////
    private:// STRUCTS /////////////////////////////////////////////////////////////////////


        /// Contains the following parameters used when computing a ???what type of trajectory??? trajectory:
        /// 1) double index_rightwards
        /// 2) double index_upwards
        /// 3) double current_velocity
        /// 4) double max_velocity
        struct TrajectoryParameters
        {
            double index_rightwards;
            double index_upwards;
            double current_velocity;
            double max_velocity;
        };


    /////////////////////////////////////////////////////////////////////////////////////////
    private:// ROS MEMBER VARIABLES ////////////////////////////////////////////////////////
        
        
        ros::NodeHandle m_ROSRNH; // ROS node handle that is resolved to the node's namespace.
        ros::NodeHandle ROS_PNH; // ROS node handle that is resolved to the namespace: <node's namespace>/<node's name>.

        ros::Subscriber ROS_Sub_StateEstimate;        // Subscribes to ROS topic "state_estimate"
        ros::Subscriber ROS_Sub_StartNavigation;      // Subscribes to ROS topic "start_navigation"
        ros::Subscriber ROS_Sub_HardStop;             // Subscribes to ROS topic "hard_stop"
        ros::Subscriber ROS_Sub_CNNOutTraj;           // Subscribes to ROS topic "/cnn_out/traj"
        ros::Subscriber ROS_Sub_OnlyNetwork;          // Subscribes to ROS topic "only_network"
        ros::Subscriber ROS_Sub_RunIdx;               // Subscribes to ROS topic "run_idx"
        ros::Subscriber ROS_Sub_SetupEnvironment;     // Subscribes to ROS topic "setup_environment"
        ros::Subscriber ROS_Sub_ImageRGB;             // Subscribes to ROS topic "image_rgb"
        ros::Subscriber ROS_Sub_GazeboModelStates;    // Subscribes to ROS topic "/gazebo/model_states"
        ros::Subscriber ROS_Sub_ReplaceGates;         // Subscribes to ROS topic "/replace_gates"
        
        ros::Publisher ROS_Pub_Crashed;                    // std_msgs::Empty -> "/crashed"     
        ros::Publisher ROS_Pub_PassedGate;                 // std_msgs::Empty -> "/passed_gate"
        ros::Publisher ROS_Pub_CopilotFeedthrough;         // std_msgs::Bool -> "copilot/feedthrough"
        ros::Publisher ROS_Pub_AutopilotReferenceState;    // quadrotor_msgs::TrajectoryPoint> -> "autopilot/reference_state"
        ros::Publisher ROS_Pub_AutopilotOff;               // std_msgs::Empty -> "autopilot/off"
        ros::Publisher m_ROSPub_AutopilotStart;             // std_msgs::Empty -> "autopilot/start"
        ros::Publisher m_ROSPub_BridgeArm;                  // std_msgs::Bool -> "bridge/arm"
        ros::Publisher ROS_Pub_Divergence;                 // std_msgs::Float64 -> "divergence"
        ros::Publisher ROS_Pub_ImageWithPrediction;        // sensor_msgs::Image -> "image_with_prediction"
        ros::Publisher ROS_Pub_GoalMarker;                 // visualization_msgs::Marker -> "goal_marker"
        ros::Publisher ROS_Pub_Debug;                      // visualization_msgs::Marker -> "debug"
        ros::Publisher ROS_Pub_VehicleMarker;              // visualization_msgs::MarkerArray> -> "vehicle_marker"
        ros::Publisher ROS_Pub_TrajectoriesCNN;            // visualization_msgs::Marker -> "trajectories_cnn"              
        ros::Publisher ROS_Pub_GlobalTrajectory;           // visualization_msgs::Marker -> "global_trajectory"
        ros::Publisher ROS_Pub_GazeboSetModelState;        // gazebo_msgs::ModelState -> "/gazebo/set_model_state". Used in: respawnGazeboGateModelsWithRandomAxialShifts(), moveGazeboGateModels()                 
        ros::Publisher ROS_Pub_RvizGates;                  // visualization_msgs::MarkerArray -> "rviz_gates". Used in: visualizeGazeboGateModelsInRVIZ()

        ros::ServiceClient ROS_SrvClient_GazeboSetModelState;   // gazebo_msgs::ModelState -> "/gazebo/set_model_state"
        ros::ServiceClient ROS_SrvClient_GazeboSpawnGazeboModel;      // gazebo_msgs::SpawnModel -> "/gazebo/spawn_gazebo_model"

        ros::Timer ROS_Timer_runMainLoop; // ROS timer that calls runMainLoop() every 20 ms.
        
        // tf::TransformListener tf_listener_; // ???Can I turn this mofo off???


    /////////////////////////////////////////////////////////////////////////////////////////
    private:// ROS CALLBACKS ///////////////////////////////////////////////////////////////

        /// \brief Incoming message on ROS topic "start_navigation" triggers:
        /// Visualizes drone position in world reference frame as green cube in RVIZ. 
        /// Scans sampled global trajectory for minimum/maximum speed and 
        /// index of state with minimum distance to drone position and 
        /// sets relating member variables. 
        /// Sets autopilot state to RACING.
        /// Sets "m_Run_StartTime" to current time.
        /// setReferenceStateInWRFToDronePosYawInWRF()
        /// Publish the current state estimation to ROS topic "autopilot/reference_state"
        /// in order that the drone hovers.
        /// Publish true to ROS topic "copilot/feedthrough"
        void ROSCallback_StartNavigation( const std_msgs::EmptyConstPtr& msg );
        
        /// \brief Incoming message on ROS topic "hard_stop" triggers:
        /// m_Run_CameraFrame_Count = 0;
        /// Switchs off data recording. 
        /// Sets reference state in world/autopilot reference frame 
        /// to current drone position and yaw world/autopilot reference frame. 
        /// Makes drone hover by publishing current drone position and 
        /// yaw in autopilot reference state to ROS topic "autopilot/reference_state". 
        /// Sets autopilot state to HOVER.
        void ROSCallback_HardStop( const std_msgs::EmptyConstPtr& msg );

        /// \brief calls updateTransformationsBetweenReferenceFrames( 
        /// latest message on ROS topic "state_estimate" )
        void ROSCallback_StateEstimate( const nav_msgs::OdometryConstPtr& msg );

        /// \brief Sets "m_CNN_Output" to latest message on ROS topic "/cnn_out/traj".
        void ROSCallback_CNNOutTraj( const geometry_msgs::TwistStampedConstPtr& msg );

        /// \brief Incoming message on ROS topic "only_network" triggers:
        /// Sets "m_ORCFlightMission" to TESTING/DATACOLLECTION if message true/false.
        void ROSCallback_OnlyNetwork( const std_msgs::BoolConstPtr& msg );

        /// \brief Sets "m_Run_Count" to latest message on ROS topic "run_idx".
        /// Sets "m_Run_DirectoryExists" to false.
        void ROSCallback_RunIdx( const std_msgs::Int16ConstPtr& msg );

        /// !!! Not optimized !!!
        /// \brief Incoming message on ROS topic "setup_environment" triggers:
        /// Sets "m_Run_SimEnvIsReady" currently to false.
        /// Sets "p_DataRecording_On" to false.
        /// Sets "m_Gates_Curr_i = p_Gates_Start_i".
        /// Calls respawnGazeboGateModelsWithRandomAxialShifts().
        /// Calls visualizeGazeboGateModelsInRVIZ().
        /// Calls initializeGlobalTrajectory()
        /// Displays green cube at drone position in RVIZ.
        /// If "m_ORCFlightMission" in DATACOLLECTION :
        /// setMinAndMaxSpeedAfterScanningGlobalTrajectory(), 
        /// setIndexOfGTProjStateToGTStateWithMinDist2( DronePos_WRF ).
        /// Sets "m_Run_SimEnvIsReady" to true.
        /// Visualizes drone position in world reference frame as green cube in RVIZ. 
        /// Scans sampled global trajectory for minimum/maximum speed 
        /// and index of state with minimum distance to drone position and 
        /// sets relating member variables. 
        /// Sets autopilot state to RACING.
        /// Sets "m_Run_StartTime" to current time.
        /// setReferenceStateInWRFToDronePosYawInWRF()
        /// Make drone hover
        /// Publish true to ROS topic "copilot/feedthrough".
        void ROSCallback_SetupEnvironment( const std_msgs::EmptyConstPtr& msg );

        /// \brief Sets m_Drone_Cam_Frame to latest message on ROS topic "image_rgb".
        /// Publish a processed (shrinked, grid lines, visualization of Expert/CNN output) 
        /// version of m_Drone_Cam_Frame to ROS topic "image_with_prediction".
        void ROSCallback_ImageRGB( const sensor_msgs::ImageConstPtr& msg );

        /// \brief Sets "m_Gazebo_ModelStates" to latest message on ROS topic "/gazebo/model_states".
        void ROSCallback_GazeboModelStates( const gazebo_msgs::ModelStates::ConstPtr& msg );

        /// \brief Incoming message on ROS topic "/replace_gates" triggers:
        /// Publishs a ModelState (gazebo_msgs::ModelState) 
        /// to ROS topic "/gazebo/set_model_state" for each gate modelled in Gazebo. 
        /// The position of the published ModelState is subject to uniform real distribution.
        void ROSCallback_ReplaceGates( const std_msgs::EmptyConstPtr& msg );


    /////////////////////////////////////////////////////////////////////////////////////////
    private:// MEMBER FUNCTIONS ////////////////////////////////////////////////////////////
    



        /// \brief Sets transformations "m_T_ARF_DRF" and "m_T_WRF_DRF" from
        /// parameter.
        /// \param DronePos_ARF Drone position in autopilot reference frame (state estimate).
        /// \param m_T_ARF_DRF Adjusted.
        /// \param m_T_WRF_DRF Adjusted.
        void updateTransformationsBetweenReferenceFrames( const nav_msgs::Odometry& DronePos_ARF );


        /// \brief Main loop with period time of "m_ROS_Param_MainLoop_NominalDuration".
        /// If last iteration took more time. log ROS_ERROR.
        /// Visualizes drone by publishing "m_RVIZ_Drone" to ROS topic "vehicle_marker".
        /// If "m_Run_SimEnvIsReady":
        /// if "p_Gates_Dynamic_On" moveGazeboGateModels(),
        /// visualizeGazeboGateModelsInRVIZ()
        /// if autopilot is in state OFF or HOVER, visualizeGateWaypointsInRVIZ()
        /// if autopilot is in state RACING, 
        /// performNavigation(), setReferenceStateFromLocalTrajectory()
        /// and publish reference state in autopilot reference frame to ROS topic "autopilot/reference_state".
        /// Increment "m_MainLoop_Iterations_Count".
        void runMainLoop( const ros::TimerEvent &time );

            /// \brief Sets reference state in world and autopilot reference frame 
            /// to current state of local trajectory.
            /// \param m_ReferenceState_WRF Adjusted.
            /// \param m_ReferenceState_ARF Adjusted.
            void setReferenceStateFromLocalTrajectory();


        /// \brief Checks divergence from drone to reference state and aborts run if necessary.
        /// Every "m_ROS_Param_MainLoop_Its2LTCompsRatio"-th iteration of main loop,
        /// collect data or do testing according to current "m_ORCFlightMission".
        void performNavigation();

            /// \brief Computes and publishs distance 
            /// (drone in world reference frame -> reference state in world reference frame)
            /// to ROS topic "divergence".
            /// Returns true if distance greater than "p_MainLoop_DivergenceThreshold2Abort".
            bool checkDivergence_Drone2RefState();

            /// \brief Switchs off data recording. 
            /// Sets reference state in world/autopilot reference frame to current drone position 
            /// and yaw world/autopilot reference frame.
            /// Makes drone hover by publishing current drone position and yaw in autopilot reference state 
            /// to ROS topic "autopilot/reference_state".
            /// Sets autopilot state to HOVER.
            /// Adds line "m_Run_Count" to file fails.txt in root directory.
            /// Publish empty message to ROS topic "/crashed"
            void abortRunDueToDivergence_Drone2RefState();


            



            /// \brief !!! Not ready !!!
            /// Sets index to GT projection state.
            /// Displays [red/blue/black] cube 
            /// for [reference state/projection state/waypoint of current gate] in RVIZ. ??? Put after update of current gate waypoint index ???
            /// If distance (reference state -> waypoint of current gate) 
            /// shorter than "p_Gates_DistThreshold2UpdateIdx", 
            /// increment index of current gate and publish empty msg to ROS topic "/passed_gate".
            ///
            /// Fills "m_Expert_Output" with expert goal position in image reference frame and expert normalized speed.
            /// If "p_Expert_Perturbation_On", add random perturbation to expert output used in this function.
            ///
            /// Computes goal position in world reference frame and speed to goal position from output of expert.
            /// Computes local trajectory
            /// ( reference state in world reference frame -> goal position in world reference frame )
            /// with duration according to speed to goal position.
            ///
            /// If "p_DataRecording_On" and "RunElapsedTime > p_Run_DataRecording_StartTime",
            /// save training data (image file and label).
            ///
            /// Increment "m_Run_CameraFrame_Count"
            ///
            /// If trajectory was succesfully computed, 
            /// visualize the local trajectory in RVIZ,
            /// set m_LT_SubsequentlyFailedComputations_Count = 0
            /// set m_LT_StartTime to current time.
            /// If LT computation failed and the max allowed count of subsequent fails was reached,
            /// go to hover state and publish empty message to ROS topic "/crashed".
            /// \param WHICHGET Adjusted?
            void executeDataCollection();





                /// \brief Returns vector with random elements in [ +/- 0.1, +/- 0.1, +/- 0.05 ].
                Eigen::Vector3d getRandomPerturbation4ExpertOutput();

                /// \brief Writes camera frame to image file and adds label as line to file
                /// "/labels.txt" located in sub-directory of current run.
                void saveTrainingData();
                
                /// \brief Returns string representation of "m_Run_Count" with width of 4 digits.
                std::string getStringRepresentation4RunCount();

                /// \brief Returns string representation of "m_Run_CameraFrame_Count" with width of 5 digits.
                std::string getStringRepresentation4CameraFrameCount();
                
                /// \brief Creates sub-directory of current run in "p_Run_RootDirPath".
                /// \param RunCountString String representation of "m_Run_Count" with width of 4 digits
                /// used to identify the sub-directory of current run.
                void createDirectory4TrainingDataOfRun( const std::string& RunCountString );
                
                /// \brief Writes color converted version of "m_Drone_Cam_Frame" to image file.
                /// The file is located in sub-directory of current run and 
                /// the filename contains the count of current run's camera frame.
                /// \param RunCountString String representation of "m_Run_Count" with width of 4 digits
                /// used to identify the sub-directory of current run.
                /// \param CameraFrameCountString String representation of "m_Run_CameraFrame_Count" 
                /// with width of 5 digits used to give saved image an unique filename.
                /// \param m_CloneImgMtx Locked and unlocked.
                void saveTrainingData_Image( const std::string& RunCountString, const std::string& CameraFrameCountString );
            
                /// \brief Append the line
                ///"<m_Expert_Output.x>;<.y>;<.z>;<m_GT_MaxSpeed>;<m_Run_CameraFrame_Count == 0? 1 : 0>"
                /// to file "/labels.txt" located in sub-directory of current run.
                /// \param RunCountString String representation of "m_Run_Count" with width of 4 digits
                /// used to identify the "/labels.txt" pertaining to current run.
                void saveTrainingData_Label( const std::string& RunCountString );
            



            
            /// \brief !!! Not ready !!!
            /// Sets index to GT projection state. ??? Remove ???
            /// Displays [red/blue/black] cube 
            /// for [reference state/projection state/waypoint of current gate] in RVIZ. ??? Remove ???
            /// If distance (reference state -> waypoint of current gate) 
            /// shorter than "p_Gates_DistThreshold2UpdateIdx", 
            /// increment index of current gate and publish empty msg to ROS topic "/passed_gate". ??? Remove ???
            /// ??? The following is based on assumption stated in ??? comments in cpp file.
            /// Computes goal position in drone reference frame and speed to goal position from output of CNN.
            /// Computes local trajectory
            /// ( reference state in drone reference frame -> goal position in drone reference frame )
            /// with duration according to speed to goal position.
            /// If trajectory was succesfully computed, 
            /// set m_Run_CameraFrame_Count = 0 ??? Remove ???,
            /// visualize the local trajectory in RVIZ,
            /// set m_LT_SubsequentlyFailedComputations_Count = 0
            /// set m_LT_StartTime to current time.
            /// If LT computation failed and the max allowed count of subsequent fails was reached,
            /// go to hover state and publish empty message to ROS topic "/crashed".
            /// \param WHICHGET Adjusted?
            void executeTesting();
                
                






        /// \brief Sets index to GT projection state
        /// by iterating through states of GT :
        /// First, projects 1: direction from state to reference state
        /// onto 2: the direction of GT at state.
        /// Found projection state, if the projection level drops below
        /// speed like value at projection state.
        /// After iteration, if reference state is farer than 1 m 
        /// away from projection state, set index to state on GT that
        /// has min distance to reference state.
        /// \param m_GT_ProjectionState_i Adjusted.
        void setIndexOfGlobalTrajectoryProjectionState();


        /// \brief Increment index of current gate and publish empty msg to ROS topic "/passed_gate".
        /// \param m_Gates_Curr_i Adjusted.
        /// \param m_Gates_Last_i Adjusted.
        void updateIdxOfCurrGate();


        /// \brief Set reference state in world reference frame to 
        /// drone position and yaw in world reference frame.
        /// \param m_ReferenceState_WRF Adjusted.
        void setReferenceStateInWRFToDronePosYawInWRF();


        /// \brief Fills expert output with expert goal position in image reference frame
        /// and expert normalized speed.
        /// \param m_Expert_Output Adjusted.
        void setExpertOutput();

            /// \brief Returns expert goal position in image reference frame
            /// by transforming goal position in world reference which is  
            /// either just the waypoint of the current gate (if p_Gates_Dynamic_On or p_Gates_Respawn_MaxAxialShift > 0.0)
            /// or (else) the state of global trajectory with a horizon that is computed as: 
            /// the shorter of the distances from the reference state to the last/current gate
            /// but at least p_Expert_MinHorizon.
            /// Display yellow cube for at expert goal position in RVIZ.
            Eigen::Vector2d getExpertGoalPosInIRF();

                /// \brief Returns expert goal position in world reference frame which is  
                /// either the waypoint of the current gate (if p_Gates_Dynamic_On or p_Gates_Respawn_MaxAxialShift > 0.0)
                /// or a state of global trajectory with a horizon that is computed as: 
                /// the shorter of the distances from the reference state to the last/current gate
                /// but at least p_Expert_MinHorizon.
                Eigen::Vector3d getExpertGoalPosInWRF();

                    /// \brief Computes horizon
                    /// (=distance to projection state of global trajectory)
                    /// and returns position of global trajectory state with that horizon.
                    Eigen::Vector3d getExpertGoalPosInWRFFromGlobalTrajectory();

                /// \brief Transforms a position in world reference frame to
                /// position in drone reference frame.
                /// \param IN_Pos_DRF Position (XYZ) in world reference frame.
                /// \return Position (XYZ) in world reference frame.
                Eigen::Vector3d transformPos_WRF2DRF( const Eigen::Vector3d& IN_Pos_WRF );

                /// \brief Transforms a position in drone reference frame to
                /// position in image reference frame.
                /// \param IN_Pos_DRF Position (XYZ) in drone reference frame.
                /// \return Position (XY in [-1,1]²) in image reference frame.
                Eigen::Vector2d transformPos_DRF2IRF( const Eigen::Vector3d& IN_Pos_DRF );

            /// \brief Return expert normalized speed 
            /// which is the speed of GT state with horizon (=m_Expert_Speed_Horizon)
            /// normalized with maximum speed of global trajectory (=m_GT_MaxSpeed).
            double getExpertNormalizedSpeed();
               
                    
        /// \brief Returns const reference to horizon state (upcoming state of global trajectory that has
        /// specified distance to projection state of global trajectory).
        /// \param Horizon Distance from projection state to horizon state.
        const quadrotor_common::TrajectoryPoint& 
        getGTStateAtHorizonOf( const double& Horizon ) const;


        /// \brief Calls exitAutopilotState*() according to current autopilot state
        /// and enterAutopilotState*() according to parameter if not equal.
        /// \param NextAutopilotState State the autopilot should be updated to.
        /// \param m_AutopilotState Adjusted.
        /// \param p_DataRecording_On Adjusted if updated to HOVER.
        /// \param m_ReferenceState_WRF Adjusted if updated to HOVER.
        /// \param m_ReferenceState_ARF Adjusted if updated to HOVER.
        /// \param m_GT_MaxSpeed Adjusted if updated to RACING.
        /// \param m_GT_MinSpeed Adjusted if updated to RACING.
        /// \param m_GT_ProjectionState_i Adjusted if updated to RACING.
        void updateAutopilotStateTo( const AutopilotStates& NextAutopilotState );
            
            /// \brief No implementation.
            void updateAutopilotStateToCurrentState( const std::string& NameOfLastAutopilotState );
            
            /// \brief No implementation.
            void exitAutopilotStateOFF();
            
            /// \brief No implementation.
            void exitAutopilotStateHOVER();
            
            /// \brief No implementation.
            void exitAutopilotStateRACING();

            /// \brief /// Sets autopilot state to OFF.
            /// \param m_AutopilotState Adjusted.
            void enterAutopilotStateOFF( const std::string& NameOfLastAutopilotState );

            /// \brief Switchs off data recording. 
            /// Sets reference state in world/autopilot reference frame to current drone position 
            /// and yaw world/autopilot reference frame.
            /// Makes drone hover by publishing current drone position and yaw in autopilot reference state 
            /// to ROS topic "autopilot/reference_state".
            /// Sets autopilot state to HOVER.
            /// \param p_DataRecording_On Adjusted.
            /// \param m_ReferenceState_WRF Adjusted.
            /// \param m_ReferenceState_ARF Adjusted.
            /// \param m_AutopilotState Adjusted.
            void enterAutopilotStateHOVER( const std::string& NameOfLastAutopilotState );
            
            /// \brief Visualizes drone position in world reference frame as green cube in RVIZ.
            /// Scans sampled global trajectory for minimum/maximum speed and index of state
            /// with minimum distance to drone position and sets relating member variables.
            /// Sets autopilot state to RACING. 
            /// \param m_GT_MaxSpeed Adjusted.
            /// \param m_GT_MinSpeed Adjusted.
            /// \param m_GT_ProjectionState_i Adjusted.
            /// \param m_AutopilotState Adjusted.
            void enterAutopilotStateRACING( const std::string& NameOfLastAutopilotState );


   

        
    //////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////
    private:// MEMBER VARIABLES ////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////

    
        // Not sorted yet
        FlightMissions m_ORCFlightMission; /// \brief ???
        std::mutex m_CloneImgMtx;
        // Constantly updated state in reference frame of the world fetched from LT. ???
        quadrotor_common::TrajectoryPoint m_ReferenceState_WRF;
        quadrotor_common::TrajectoryPoint m_ReferenceState_ARF;
        bool p_DataRecording_On; // Set true to create a directory and save images into it. >>> Set to false in ROSCallback_SetupEnvironment(...), ROSCallback_HardStop(...), performNavigation() if checkStateEstDesStateDivergence()==true and in executeDataCollection() if m_MainLoop_IterationsInRacing_Count % m_ROS_Param_MainLoop_Its2LTCompsRatio==0, if computeTrajectory(...) returned false and m_LT_SubsequentlyFailedComputations_Count >= p_LT_SubsequentlyFailedComputations_MaxCount. Is checked in executeDataCollection() if m_MainLoop_IterationsInRacing_Count % m_ROS_Param_MainLoop_Its2LTCompsRatio == 0 to save image and if additionally elapsed time > p_Run_DataRecording_StartTime to create a directory. save images and corresponding trajectories to disk


        /// MainLoop
        ///////////////////
        unsigned int m_MainLoop_Iterations_Count;
        const double m_ROS_Param_MainLoop_NominalDuration;
        double p_MainLoop_DivergenceThreshold2Abort; // If this is exceeded, going to hover state.
        int m_ROS_Param_MainLoop_Its2LTCompsRatio; // ratio of publishing desired state to autopilot to replanning trajectory. The mainloop runs at 50Hz, replan the trajectory every n-th iteration of the mainloop


        // Drone
        ////////

        /// Current state of the autopilot.
        /// =OFF: Constructor
        /// =HOVER: ROSCallback_HardStop(...)
        /// =RACING: ROSCallback_SetupEnvironment(...), ROSCallback_StartNavigation(...)
        AutopilotStates m_AutopilotState; 

        /// Current raw image from onboard camera of drone.
        /// =msg: ROSCallback_ImageRGB(...)
        /// ROSCallback_ImageRGB(...): 1) imageRaw = m_Drone_Cam_Frame.clone() 2) edit imageRaw into image 3) publish image to ROS topic "image_with_prediction".
        /// executeDataCollection() if m_MainLoop_IterationsInRacing_Count%m_ROS_Param_MainLoop_Its2LTCompsRatio==0, p_DataRecording_On==true: 1) image_save = m_Drone_Cam_Frame.clone() 2) cv::imwrite(filename_img, image_save)
        cv::Mat m_Drone_Cam_Frame;
        double p_Drone_Cam_HalfYawAOV_Deg; // Does not have to equal actual FOV of camera. Used in ROSCallback_ImageRGB(..) to get dimension of image and to draw grid lines in image.
        double p_Drone_Cam_HalfPitchAOV_Deg; // Does not have to equal actual FOV of camera. Used in ROSCallback_ImageRGB(..) to get dimension of image and to draw grid lines in image.
        double p_Drone_Cam_HalfYawAOV_Rad; // Does not have to equal actual FOV of camera. Used in executeDataCollection() to calculate traj_coor_right as part of CNN selection. Used in computeTrajectory(...) to denormalize network_selection.x().
        double p_Drone_Cam_HalfPitchAOV_Rad; // Does not have to equal actual FOV of camera. Used in executeDataCollection() to calculate traj_coor_up as part of CNN selection. Used in computeTrajectory(...) to denormalize network_selection.y().
        unsigned int p_Drone_Rotors_n = 4;
        double p_Drone_ArmLength = 0.2;
        double p_Drone_BodyWidth = 0.15;
        double p_Drone_BodyHeight = 0.1;

        // CNN
        Eigen::Vector3d m_CNN_Output;


        // Expert
            /// Current Prediction of CNN: .x() and .y(): coordinates in reference frame of camera image, .z(): normalized velocity
            /// =zero vector: Constructor
            /// =msg data: ROSCallback_CNNOutTraj(...)
            /// ROSCallback_ImageRGB(...): 1) write .z() into image 2) draw circle at (.x(), .y()) into image
            /// executeTesting() if m_MainLoop_IterationsInRacing_Count%m_ROS_Param_MainLoop_Its2LTCompsRatio==0: parameter of computeTrajectory(...)
        Eigen::Vector3d m_Expert_Output;
        double p_Expert_MinHorizon; // minimum length to plan trajectories when collecting data. Used in getEndState() as min value for horizon which is the distance of end_state (return value) found on global trajectory.
        bool p_Expert_Perturbation_On; // Set true to add random perturbation to network selection in executeDataCollection() when calling computeTrajectory(...). add some noise to executed actions for data augmentation
        const double m_Expert_Speed_Horizon = 0.5; // Distance of GT state to GT projection state from which speed prediction is derived.  ???why 0.5???

        // Transformations State estimation
        kindr::minimal::QuatTransformation m_T_WRF_DRF;
        kindr::minimal::QuatTransformation m_T_WRF_ARF;
        kindr::minimal::QuatTransformation m_T_ARF_DRF;


        // Gazebo
        gazebo_msgs::ModelStates m_Gazebo_ModelStates;


        // Run
        ros::WallTime m_Run_StartTime; // Wall-clock time at start of ???what???. >>> Set to current wall-clock time in ROSCallback_SetupEnvironment(...) and ROSCallback_StartNavigation(...). Used to compare elapsed wall-clock time with p_Run_DataRecording_StartTime in runMainLoop(...) to call moveGazeboGateModels() and in executeDataCollection() to set p_RecordingDataModeIsACTIVATED=true. Used to set t to elapsed time in moveGazeboGateModels().
        int m_Run_Count; // =5000: constructor. ########## =msg->data: ROSCallback_RunIdx(...). ########### In performNavigation() if diverged write line to file. ############ In executeDataCollection() used for name of created folder and name of saved image.
        unsigned int m_Run_CameraFrame_Count; // =0: constructor, ROSCallback_HardStop(...) and executeTesting() if success_nw ##########. ++: executeDataCollection(). ########## If ==0: In executeDataCollection() if p_DataRecording_On set new_dagger_batch = 1.
        bool m_Run_DirectoryExists; /// Bool that is true if at current run a directory to save camera frames exists.
        bool m_Run_SimEnvIsReady; // runMainLoop(...): If ==false do nothing except publishing vehicle marker.
        std::string p_Run_RootDirPath;          // Not in *_main.yaml! Used in performNavigation() if checkStateEstDesStateDivergence()==true write to file p_Run_RootDirPath+"/fails.txt". Used in executeDataCollection() if m_MainLoop_IterationsInRacing_Count % m_ROS_Param_MainLoop_Its2LTCompsRatio == 0, ( ros::WallTime::now() - m_Run_StartTime".
        double p_Run_DataRecording_StartTime; // don't record data during initial transient. Used in runMainLoop(..) as condition to call moveGazeboGateModels(). Used in executeDataCollection() as condition to create directory.

        // RVIZ
        ////////////////
        visualization_msgs::MarkerArray m_RVIZ_Drone;
        /// Used in initializeVisMarker4Drone(...)
        constexpr static double p_RVIZ_Drone_Scale = 0.5;
        int m_RVIZ_LT_Count; // =0: constructor. ########## In visualizeTrajectory(...) if p_RVIZ_AllLTs_On msg.id = ++m_RVIZ_LT_Count else msg.id = 1.0.
        std::string p_RVIZ_Drone_Childs_FrameID;
        double p_RVIZ_LT_SamplingFreq; // trajectory sampling frequency to visualize curve. In constructor used reciprocal as lower limit for p_RVIZ_LT_Duration. In visualizeTrajectory() set trajectory_dt to reciprocal.
        double p_RVIZ_LT_Duration; // [s] show planned trajectory up to this horizon, only possible if p_RVIZ_CompleteLT_On is false. In visualizeTrajectory() if p_RVIZ_CompleteLT_On==false sample trajectory wit this duration.
        bool p_RVIZ_AllLTs_On; // Set (true/false) to set (msg.id = ++m_RVIZ_LT_Count/msg.id = 1.0) in visualizeTrajectory(...) ???Whats da real deal??? show only newest trajectory if set to true
        bool p_RVIZ_CompleteLT_On; // Set [true/false] to call sampleTrajectory(...) with [m_LocTraj.GetEndTime()/std::min(m_LocTraj.GetEndTime())] in visualizeTrajectory(). show full trajectory for each planning step


        // Gates
        std::string m_Gates_COLLADA_Filename;
        unsigned int m_Gates_n;
        std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> 
            m_Gates_WaypointPos_Temp; /// \brief Contains current waypoint positions, one through each gate. Used for navigation.
        std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>> 
            m_Gates_OriginPosYaw_Temp; /// \brief Contains gates origin positions and yaws. Used to spawn gate COLLADA files in Gazebo.
        std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>> 
            p_Gates_OriginPosYaw; /// \brief  ??? do not remove Waypoints???
        std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> 
            m_Gates_Dynamic_RandomPhases;
        size_t m_Gates_Last_i; // =0: constructor. ########## Set in ROSCallback_SetupEnvironment(...) and updateIdxOfCurrGate() if dist_to_next_gate.norm() < p_Gates_DistThreshold2UpdateIdx. ########## Used in getEndState() to get position of last gate.
        size_t m_Gates_Curr_i;  // Index of the gate that is currently targeted, i.e., the drone aims to fly through this gate.
        double p_Gates_Dynamic_AxialAmp; // max amplitude for moving at test time. Used in moveGazeboGateModels() as axial amplitudes of gate positions
        double p_Gates_Dynamic_RadFreq; // max speed moving gates. Used in moveGazeboGateModels() as angular frequency of gate positions
        double p_Gates_HalfHeight; // waypoint z with respect to bottom of gate. Used in getGoalPositions() to set goal_positions (return value) = m_Gates_XYZ_Current but added this to z component. 
        const double p_Gates_Respawn_MinZ;
        double p_Gates_Respawn_MaxAxialShift; // max amplitude for statically replacing the gates at new runs. In getEndState() if p_Gates_Respawn_MaxAxialShift > 0.0 then end_state (return value) = position of current gate. In replaceGates() used to manipulate gate positions.
        double p_Gates_DistThreshold2UpdateIdx; // In updateIdxOfCurrGate(), if distance falls below increase m_Gates_Curr_i and m_Gates_Last_i. when within this distance to waypoint, plan to next waypoint
        int p_Gates_Start_i; // Value of m_Gates_Curr_i at start of each ???run??? when ROSCallback_SetupEnvironment(...) is called. we start in front of the last gate :13
        bool p_Gates_Dynamic_On;

        // Global Trajectory
        std::vector<quadrotor_common::TrajectoryPoint> m_GT_States; /// \brief Contains all states of global trajectory.
        double m_GT_MaxSpeed; // =0: constructor, begin of setIndexOfGTProjStateToGTStateWithMinDist2(...). ############ ???
        double m_GT_MinSpeed; // =MAXDOUBLE: constructor, begin of setIndexOfGTProjStateToGTStateWithMinDist2(...). ############ ???
        size_t m_GT_ProjectionState_i; // Index of state of global trajectory that is Projection of drone on GT considering position and velocity direction.
        polynomial_trajectories::PolynomialTrajectorySettings m_GT_Settings;
        const double m_GT_MaxThrust = 18.0;
        const double m_GT_MaxRollPitchRate = 1.5;
        double m_GT_SamplingTime;
        std::string p_GT_DirPath;          // Not in *_main.yaml! Used in saveGlobalTrajectory() and loadGlobalTrajectory().
        double p_GT_MaxVel; // max velocity of precomputed global trajectory. Used in initializeGlobalTrajectory(...) as input arg for polynomial_trajectories::minimum_snap_trajectories::generateMinimumSnapRingTrajectoryWithSegmentRefinement(...).
        bool p_GT_UseExisting_On; // Set [true/false] to [load/generate] a global trajectory in initializeGlobalTrajectory(...). load saved global trajectory

        // Local Trajectory (LT)
        RQTG::RapidTrajectoryGenerator m_LocTraj;
        constexpr static double m_LT_MaxVelocityIncrement = 0.5;
        const Vec3 m_LT_GravityVector = { 0.0, 0.0, -9.81 };
        ros::Time m_LT_StartTime;
        unsigned int m_LT_SubsequentlyFailedComputations_Count; // =0: constructor, executeDataCollection() if success, executeTesting() if success_nw. ########## Used in executeDataCollection() and executeTesting() if success==false: if m_LT_SubsequentlyFailedComputations_Count < p_LT_SubsequentlyFailedComputations_MaxCount m_LT_SubsequentlyFailedComputations_Count++ else updateAutopilotStateTo( AutopilotStates::HOVER ).
        double p_LT_MaxAltitude; // maximum altitude of trajectory. In computeTrajectory(#5) condition for: m_LocTraj<-LocalTrajectory and return true
        double p_LT_MinAltitude; // minimum altitude of trajectory. In computeTrajectory(#5) condition for: m_LocTraj<-LocalTrajectory and return true
        double p_LT_MinThrust; // min allowed normalized thrust. In computeTrajectory(#5) condition for: m_LocTraj<-LocalTrajectory and return true
        double p_LT_MaxThrust; // max allowed normalized thrust. In computeTrajectory(#5) condition for: m_LocTraj<-LocalTrajectory and return true
        double p_LT_MaxBodyRates; // max allowed roll/pitch rate. In computeTrajectory(#5) condition for: m_LocTraj<-LocalTrajectory and return true
        double p_LT_InputFeasibilityCheck_MinSamplingTime; // trajectory is tested for feasibility in these time steps. determines precision of this test. Used in In computeTrajectory(#5).
        double p_LT_Duration; // In computeTrajectory(...) rescaling factor for desired_velocity to calculate planning_length of end_state
        double p_LT_MinDist; // length of trajectories len at test time to be increased. In computeTrajectory(...) lower limit for p_FactorScalingDesiredVelocityToDistanceBetweenDroneAndEndStateOfLocalTrajectoryrescaled
        double p_LT_MaxDist; // length of trajectories len at test time to be increased. In computeTrajectory(...) upper limit for p_FactorScalingDesiredVelocityToDistanceBetweenDroneAndEndStateOfLocalTrajectoryrescaled
        double p_LT_MaxSpeed; // Used in computeTrajectory(...) to ???denormailze velocity prediction by CNN???
        double p_LT_MinSpeed; // Used in computeTrajectory(...) as lower limit for desired_velocity.
        int p_LT_SubsequentlyFailedComputations_MaxCount; // In executeDataCollection() and executeTesting(), if m_LT_SubsequentlyFailedComputations_Count = p_LT_SubsequentlyFailedComputations_MaxCount go to hover state. after max_failed_trials the quadrotor stops
        
        
        
    //////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////       
    private: // METHODS (??? THAT ARE NOT PART OF ORIGINAL CLASS ???) //////
    ///////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////


        /// \brief Initializes a marker that can be used to visualize drone in RVIZ.
        /// \param m_RVIZ_Drone Initialized.
        void initializeVisMarker4Drone();

        /// \brief Visualize a Cube in RVIZ for debug purpose
        /// by publishing a visualization marker to ROS topic "debug".
        /// \param RVIZ_Cube_X x-coordinate of cube to be visualized
        /// \param RVIZ_Cube_Y y-coordinate of cube to be visualized
        /// \param RVIZ_Cube_Z z-coordinate of cube to be visualized
        /// \param RVIZ_Cube_ID .id of cube to be visualized
        /// \param RVIZ_Cube_VisColor Color of cube to be visualized
        void displayCubeInRVIZ(
            const double& RVIZ_Cube_X,
            const double& RVIZ_Cube_Y,
            const double& RVIZ_Cube_Z,
            const int& RVIZ_Cube_ID,
            const VisualizationColors& RVIZ_Cube_VisColor
            );


        /// \brief Visualize all current gate waypoints in RVIZ
        /// by publishing a yellow visualization marker for each gate waypoint 
        /// to ROS topic "goal_marker".
        void visualizeGateWaypointsInRVIZ();


        /// \brief Visualize either global or current local trajectory according to parameter,
        /// i.e., publish purple marker to "global_trajectory" or green marker to "trajectories_cnn"
        /// \param TrajType Specifies if global or current local trajectory should be visualized.
        void visualizeTrajectory( const TrajectoryTypes& TrajType );

            /// \brief Publishs a visualization marker of purple color for the global trajectory
            /// to the ROS topic "global_trajectory".
            /// The marker's .id = 0.
            /// \param Vis_GT_Marker Visualization marker for global trajectory
            void visualizeGlobalTrajectory( visualization_msgs::Marker& Vis_GT_Marker );
            
            /// \brief Publishs a visualization marker of green color for the local trajectory
            /// to the ROS topic "trajectories_cnn".
            /// If "p_RVIZ_AllLTs_On" the marker has an invidual .id = 1,2,... for every call of this function.
            /// If not "p_RVIZ_CompleteLT_On" only states until max "p_RVIZ_LT_Duration" are visualized.
            /// \param Vis_LT_Marker Visualization marker for local trajectory
            void visualizeLocalTrajectory( visualization_msgs::Marker& Vis_LT_Marker );
                
                /// \brief Sets .color.r/g/b of a visualization marker according to specified color.
                /// \param IN_VisColor Color of visualization marker
                /// \param OUT_VisMarker Visualization marker
                void setColorRGBOfVisMarker(
                    const VisualizationColors& IN_VisColor,
                    visualization_msgs::Marker& OUT_VisMarker
                    );

                /// \brief Resample local trajectory with specified sampling time and duration.
                /// \param IN_Duration Duration of resampled local trajectory
                /// \param IN_SamplingTime Sampling time of resampled local trajectory
                /// \param OUT_LT_Resampled Resampled local trajectory
                void resampleLocalTrajectory(
                    const double& IN_Duration,
                    const double& IN_SamplingTime,
                    std::vector<quadrotor_common::TrajectoryPoint>& OUT_LT_Resampled
                    );
            
                    /// \brief Returns state of local trajectory at specified time point.
                    /// \param IN_TimePoint Specified time point
                    /// \return State of local trajectory at specified time point
                    quadrotor_common::TrajectoryPoint getStateOfLocalTrajectoryAtTimePoint( const double& IN_TimePoint );

                /// \brief Sets .points of a visualization marker for a trajectory.
                /// \param IN_Trajectory To be visualized trajectory
                /// \param OUT_TrajectoryVisMarker Visualization marker for to be visualized trajectory
                void setPointsOfTrajectoryVisMarker(
                    const std::vector<quadrotor_common::TrajectoryPoint>& IN_Trajectory,
                    visualization_msgs::Marker& OUT_TrajectoryVisMarker
                    );
        

        /// \brief Computes goal position in world reference frame and speed to goal position from output of CNN.
        /// \param IN_GoalX_IRF CNN output: x-coordinate ([-1, 1]) of goal position in image reference frame
        /// \param IN_GoalY_IRF CNN output: y-coordinate ([-1, 1]) of goal position in image reference frame
        /// \param IN_Speed2Goal_Normalized CNN output: normalized speed ([0, 1]) on way to goal position
        /// \param OUT_GoalPos_WRF To be computed goal position in world reference frame
        /// \param OUT_Speed2Goal To be computed speed to goal position
        void processCNNOutput(
            const double& IN_GoalX_IRF, 
            const double& IN_GoalY_IRF,
            const double& IN_Speed2Goal_Normalized,
            Eigen::Vector3d& OUT_GoalPos_WRF,
            double& OUT_Speed2Goal
            );

            /// \brief Transforms a point (x, y) in image reference frame with distance information
            /// to point (x, y, z) in drone reference frame.
            /// \param IN_PointX_IRF x-coordinate ([-1, 1]) of point in image reference frame
            /// \param IN_PointY_IRF y-coordinate ([-1, 1]) of point in image reference frame
            /// \param IN_Dist2Point Distance from camera position to point
            /// \return Position of point in drone reference frame.
            geometry_msgs::Point transform_IRF2DRF(
                const double& IN_PointX_IRF, 
                const double& IN_PointY_IRF, 
                const double& IN_Dist2Point
                );


        /// \brief Computes a minimum jerk trajectory 
        /// connecting parameter IN_StartState with parameter IN_EndPosition.
        /// The duration of the trajectory depends on parameter IN_Velocity
        /// but capped by velocity of parameter IN_StartState and constant max value.
        /// The computed trajectory is checked for input and position feasibility.
        /// If passed the trajectory will be not discarded and the function will return true.
        /// \param IN_StartState Const reference to instance of "quadrotor_common::TrajectoryPoint"
        /// \param IN_EndPosition Const reference to instance of "Eigen::Vector3d"
        /// \param IN_Velocity Const reference to instance of "double"
        /// \param m_LocTraj Adjusted if trajectory passed feasibility test
        /// \return true if trajectory passed feasibility test, false else.
        bool computeLocalTrajectory(
            const quadrotor_common::TrajectoryPoint& IN_StartState,
            const Eigen::Vector3d& IN_EndPosition,
            const double& IN_Velocity
            );

            /// \brief Checks if parameter LocalTrajectory is input feasible and position (floor and ceiling) feasibile.
            /// \param LocalTrajectory Reference to instance of "RapidQuadrocopterTrajectoryGenerator::RapidTrajectoryGenerator"
            /// \return true if trajectory passed all feasibility tests, false else.
            bool LocalTrajectoryIsInputAndPositionFeasible( RQTG::RapidTrajectoryGenerator& LocalTrajectory );


        /// \brief Either loads global trajectory from file
        /// or computes global trajectory and saves it to file.
        /// Publishs a Marker (visualization_msgs::Marker) for global trajectory
        /// to ROS topic "global_trajectory".
        /// \param m_GT_States Adjusted.
        void initializeGlobalTrajectory();
    
            /// \brief Loads global trajectory from file "global_trajectory.txt".
            /// \param m_GT_States Adjusted.
            void loadGlobalTrajectory();

            /// \brief Computes a minimum snap ring trajectory
            /// through modified gate center positions.
            /// \param m_GT_States Adjusted.
            void computeGloTrajForExpert();
            
            /// \brief Saves global trajectory to file "global_trajectory.txt".
            void saveGlobalTrajectory();


        /// \brief Scans sampled global trajectory for index of state with minimum distance to parameter and sets relating member variables.
        /// \param Position Const reference to Eigen::Vector3d.
        /// \param m_GT_ProjectionState_i Adjusted.
        void setIndexOfGTProjStateToGTStateWithMinDist2( const Eigen::Vector3d& Position );


        /// \brief Scans sampled global trajectory for minimum and maximum velocity and set relating member variables.
        /// \param m_GT_MaxSpeed Adjusted.
        /// \param m_GT_MinSpeed Adjusted.
        void setMinAndMaxSpeedAfterScanningGlobalTrajectory();


        /// \brief Reads in x, y, z and yaw of gate origins from ROS parameter server.
        /// \param p_Gates_OriginPosYaw Adjusted.
        /// \param m_Gates_OriginPosYaw_Temp Adjusted.
        /// \param m_Gates_n Adjusted.
        void loadGatesXYZYawFromROSParameterServer(); //TODO<Eliminate the need for filtering out waypoints>


        /// \brief Publishs a ModelState (gazebo_msgs::ModelState)
        /// to ROS topic "/gazebo/set_model_state"
        /// for each gate modelled in Gazebo.
        /// The position of the published ModelState is subject to uniform real distribution.
        /// \param m_Gates_WaypointPos_Temp Adjusted.
        /// \param m_Gates_OriginPosYaw_Temp Adjusted.
        void respawnGazeboGateModelsWithRandomAxialShifts(); // TODO<Try to not search through gazebo models every time>


        /// \brief Publishs a ModelState (gazebo_msgs::ModelState) 
        /// to ROS topic "/gazebo/set_model_state"
        /// for each gate modelled in Gazebo.
        /// The position of the published ModelState is subject to a time dependent harmonic oscillation.
        /// \param m_Gates_OriginPosYaw_Temp Adjusted.
        void moveGazeboGateModels(); // TODO<Try to not search through gazebo models every time>


        /// \brief Publishs a MarkerArray (visualization_msgs::MarkerArray) 
        /// to the ROS topic "rviz_gates".
        /// The MarkerArray contains a Marker (visualization_msgs::Marker) 
        /// for each gate that is currently modelled in Gazebo.
        void visualizeGazeboGateModelsInRVIZ(); // TODO<Now its designed to Gazebo->RVIZ. Try to design This->Rviz.>





    

    //////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////
    private: // Python Stuff          //////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////    
    //////////////////////////////////////////////////////////////////////



        void prepareRandomizedGazeboSimulation();
        void startDataCollection();
};

} // namespace forgetful_drone