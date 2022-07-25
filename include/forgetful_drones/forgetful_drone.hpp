#pragma once

//#include <filesystem>
#include <experimental/filesystem>

#include <ros/ros.h>
#include "forgetful_drones/BuildDroneRacingSimulation.h"
#include "forgetful_drones/StartDroneRacingSimulation.h"
#include "forgetful_drones/StopDroneRacingSimulation.h"
#include "forgetful_drones/TrainBrain.h"
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float64.h>
#include <std_srvs/Empty.h>
#include <Eigen/Dense>
#include <ros/package.h>
#include "forgetful_drones/FlyDroneThroughRaceTrack.h"
#include "forgetful_drones/FlyDroneProvidingNetworkOutput.h"
#include "forgetful_drones/NavigatorState.h"
#include "forgetful_drones/ComputeGlobalTrajectory.h"
#include <quadrotor_msgs/AutopilotFeedback.h>
#include <ros/console.h>
#include <sensor_msgs/Image.h>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PoseStamped.h>
#include <quadrotor_msgs/ControlCommand.h>
#include <sensor_msgs/Imu.h>






#include <trajectory_generation_helper/polynomial_trajectory_helper.h>
#include <minkindr_conversions/kindr_msg.h>

#include <rapid_trajectories/RapidTrajectoryGenerator.h>
#include <rapid_trajectories/Vec3.h>
#include <thread>
#include <future>
#include <functional>

//#include <rviz/SendFilePath.h>


#include <torch/script.h>

#include "flightlib/json/json.hpp" //#include <nlohmann/json.hpp> version conflict


#include "forgetful_drones/forgetful_simulator.hpp"





namespace forgetful_drone {
namespace RQTG = RapidQuadrocopterTrajectoryGenerator;
using BDRS = forgetful_drones::BuildDroneRacingSimulation;
using BDRSR = forgetful_drones::BuildDroneRacingSimulationRequest;

class ForgetfulDrone{


////////////////////
private:// ENUMS //
//////////////////
        
    /// Contains all implemented flight missions selectable by the user through ROS parameter.
    /// 1) GENERATE_TRAINING_DATA
    /// 2) NAVIGATE_WITH_ANN
    /// 3) NAVIGATE_WITH_EXPERT
    /// 4) TRACK_GLOBAL_TRAJECTORY
    enum FlightMissions
    {
        _NOT_SET,
        GENERATE_TRAINING_DATA,
        NAVIGATE_WITH_ANN,
        NAVIGATE_WITH_EXPERT,
        TRACK_GLOBAL_TRAJECTORY,
    };

//////////////////////////////////
public:// CON- AND DESTRUCTORS //
////////////////////////////////


    ForgetfulDrone() 
        : ForgetfulDrone( ros::NodeHandle(), ros::NodeHandle("~") ) 
        {}

    ForgetfulDrone(
        const ros::NodeHandle& RNH, 
        const ros::NodeHandle& PNH
        );

    virtual ~ForgetfulDrone();


/////////////////////
private:// Expert //
///////////////////





/////////////////////////////////////
private:// CONST MEMBER VARIABLES //
///////////////////////////////////
    

    ros::NodeHandle m_rosRNH; // ROS node handle that is resolved to the node's namespace.
    ros::NodeHandle m_rosPNH; // ROS node handle that is resolved to the namespace: <node's namespace>/<node's name>.

    ros::Subscriber m_rosSUB_GROUND_TRUTH_ODOMETRY;
    ros::Subscriber m_rosSUB_GROUND_TRUTH_IMU;
    ros::Subscriber m_rosSUB_AUTOPILOT_FEEDBACK;
    ros::Subscriber m_rosSUB_CONTROL_COMMAND;
    ros::Subscriber m_rosSUB_BRAIN_OUTPUT;
    ros::Subscriber m_rosSUB_FLIGHTMARE_RGB;

    ros::Publisher m_rosPUB_RVIZ_NAVIGATION_POINTS;
    ros::Publisher m_rosPUB_RVIZ_RGB_LABELED;
    ros::Publisher m_rosPUB_RVIZ_GLOBAL_TRAJECTORY;
    //ros::Publisher m_ROSPUB_SIMULATOR_DYNAMIC_GATES_SWITCH;
    ros::Publisher m_rosPUB_AUTOPILOT_OFF;
    ros::Publisher m_rosPUB_AUTOPILOT_START;
    ros::Publisher m_rosPUB_AUTOPILOT_LAND;
    ros::Publisher m_rosPUB_BRIDGE_ARM;
    //ros::Publisher m_ROSPUB_NAVIGATOR_STATE_SWITCH;
    ros::Publisher m_rosPUB_AUTOPILOT_REFERENCE_STATE;
    ros::Publisher m_rosPUB_RVIZ_LOCAL_TRAJECTORY;
    ros::Publisher m_rosPUB_AUTOPILOT_POSE_COMMAND;
    ros::Publisher m_rosPUB_BRAIN_ENABLEINFERENCE;
    ros::Publisher m_rosPUB_BRAIN_TRIGGERINFERENCE;
    ros::Publisher m_rosPUB_BRAIN_LOADCHECKPOINT;

    ros::ServiceClient m_rosSVC_BRAIN_TRAIN_ANN;
    ros::ServiceClient m_rosSVC_SIMULATOR_BUILD;
    ros::ServiceClient m_rosSVC_SIMULATOR_START;
    ros::ServiceClient m_rosSVC_SIMULATOR_STOP;
    //ros::ServiceClient m_rosSVC_RVIZ_LOAD_CONFIG;







    
    void ROSCB_GROUND_TRUTH_ODOMETRY (const nav_msgs::OdometryConstPtr& msg);
    void ROSCB_GROUND_TRUTH_IMU (const sensor_msgs::ImuConstPtr& msg);
    void ROSCB_AUTOPILOT_FEEDBACK (const quadrotor_msgs::AutopilotFeedback::ConstPtr& msg);
    void ROSCB_CONTROL_COMMAND (const quadrotor_msgs::ControlCommand::ConstPtr& msg);
    void ROSCB_BRAIN_OUTPUT (const geometry_msgs::PointConstPtr& msg);
    void ROSCB_FLIGHTMARE_RGB (const sensor_msgs::ImageConstPtr& msg);


    ForgetfulSimulator* m_SimulatorPtr {nullptr};
    uint8_t m_AutopilotState;
    Eigen::Vector3d m_ExpertOutput;
    Eigen::Vector3d m_BrainOutput;
    Eigen::Vector3d m_NavigatorInput;
    std::vector< quadrotor_common::TrajectoryPoint > m_GloTraj;
    std::vector< Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > m_TrackWaypoints;
    size_t m_GloTrajExpertStateIdx;
    double m_Expert_MaxSpeed;
    double m_Expert_MinSpeed;
    kindr::minimal::QuatTransformation m_T_WRF_DRF;
    kindr::minimal::QuatTransformation m_T_ARF_DRF;
    Eigen::Vector3d m_RefStatePos_WRF;
    size_t m_CurrWaypointIdx;
    size_t m_LastWaypointIdx;
    double m_Dist2CurrWaypoint;
    double m_Dist2LastWaypoint;
    bool m_NavigatorENABLED;
    bool m_DataSavingENABLED;
    int m_LocTrajSubseqInfeasibleCnt;
    ros::Time m_LocTrajStartTime;
    RQTG::RapidTrajectoryGenerator m_LocTraj;
    unsigned int m_MainLoopIterCnt;
    unsigned int m_LocTrajFeasibleCnt;
    int m_TotalRunCnt;
    int m_DaggerRepCnt;
    unsigned int m_RGBCnt;
    int m_RunLapCnt;
    int m_RunNumLaps;
    int m_ExpertInterventionsCnt;
    int m_BrainDecisionsCnt;
    int m_ConfigCnt;
    int m_ConfigMarginCnt;
    float m_DaggerMargin;


    uint8_t m_UnitySceneIdx;
    uint8_t m_SceneSiteIdx;
    uint8_t m_TrackTypeIdx;
    uint8_t m_TrackGenerationIdx;
    uint8_t m_TrackDirectionIdx;
    uint8_t m_GateTypeIdx;
    double m_LocTrajMaxSpeed;





    sensor_msgs::ImuConstPtr m_IMUPtr;
    std::array<double, 6> m_IMUData;
    double m_IMULastStampTime;
    double m_IMUTimeIncrement;
    cv_bridge::CvImageConstPtr /*cv::Mat*/ m_RGBPtr;
    cv::Mat m_RGBData;
    double m_RGBTimeIncrement;
    double m_RGBLastStampTime;
    void rvizLabeledCamFrame();
    //quadrotor_msgs::AutopilotFeedback m_AutopilotFeedback;
    quadrotor_msgs::ControlCommand::ConstPtr /*quadrotor_msgs::ControlCommand*/ m_CtrlCmdPtr;
    std::array<double, 7> m_CtrlCmdData;

    
            


    void setRGBData ();
    void setIMUData ();
    void setCtrlCmdData ();


    

    ros::Timer m_rosTMR_MAINLOOP;
    void ROSCB_NAVIGATION_BY_EXPERT (const ros::TimerEvent& te);
    void ROSCB_TRAINING_DATA_GENERATION (const ros::TimerEvent& te);
    void ROSCB_NAVIGATIONBYANN_CPP (const ros::TimerEvent& te);
    void ROSCB_NAVIGATION_BY_ANN_PYTHON (const ros::TimerEvent& te);
    void ROSCB_DAGGER_PYTHON (const ros::TimerEvent& te);
    void ROSCB_DAGGER_FIRST_RUN (const ros::TimerEvent& te);


            void runWaypointUpdater();
            void runExpert();
            void triggerBrain();
            void runDataSaver (const bool& expert_intervened);
            void runBrain();
            void runNavigator(bool& intervened, void (forgetful_drone::ForgetfulDrone::*intervention_fct)(Eigen::Vector3d& _0, double& _1, bool& _2));

            void runWaypointUpdater_updateWaypointIndices();
            bool computeGlobalTrajAsCircuit();
            void findExpertStateWithProjection();
                void findMinDistanceStateIdx(
                    double& min_dist,
                    size_t& min_dist_state_idx
                );
                void findDistance2GlobalTrajectory (
                    const Eigen::Vector3d position,
                    double& min_dist,
                    size_t& min_dist_state_idx
                );
            Eigen::Vector3d PosDRF_From_PosWRF( const Eigen::Vector3d& PosWRF );
            Eigen::Vector2d PosIRF_From_PosDRF( const Eigen::Vector3d& PosDRF );
            void findSpeedState();
            void findHorizonState( const double& Horizon );
            void rvizExpertAndHorizonState();
            void runWaypointUpdater_rvizCurrWaypoint();
            void rvizReferenceState();

            void processNavigatorInput (Eigen::Vector3d& OUT_GoalPos_DRF, double& OUT_Speed2Goal);
            void interveneBrainDecisionWithExpert (Eigen::Vector3d& OUT_GoalPos_DRF, double& OUT_Speed2Goal, bool& exp_intervened);
            void interveneBrainDecisionWithExpert_firstRun (Eigen::Vector3d& OUT_GoalPos_DRF, double& OUT_Speed2Goal, bool& exp_intervened);
            void doNothing (Eigen::Vector3d& OUT_GoalPos_DRF, double& OUT_Speed2Goal, bool& exp_intervened);
            Eigen::Vector3d XYZ_DRF_From_XYDist_IRF( 
                const double& X_IRF, 
                const double& Y_IRF, 
                const double& Dist_IRF 
                );
            Eigen::Vector3d XYZ_ARF_From_XYZ_DRF( const Eigen::Vector3d& XYZ_DRF );
            bool computeLocalTrajectory(
            const Eigen::Vector3d& GoalPos_DRF,
            const double& Speed2Goal
            );
            bool checkFeasibility (RQTG::RapidTrajectoryGenerator& lt);

            void updateFailedRunsFile ();
            void saveTrainSample (const bool& expert_intervened);
    

        void rvizLocTraj();

        void publishReferenceState2Autopilot();

    torch::jit::script::Module m_TorchScriptModule;
    std::vector<double> m_GRUHiddenState;




/////////////////////////////
private:// ROS PARAMETERS //
///////////////////////////

    //const bool m_DYNAMIC_GATES_ON;
    const bool p_RVIZ_LABELEDRGB_SAVED {};
    const bool p_RVIZ_LABELEDRGB_ENABLED {};


    const int p_CONFIG_NUMRUNS {};
    
    const int p_GLOBAL_TRAJECTORY_POLYNOMIAL_ORDER {};
    const int p_GLOBAL_TRAJECTORY_CONTINUITY_ORDER {};
    const int p_RUN_NUMLAPS {};
    

    const double p_DRONE_CAMERA_HALF_YAW_AOV {};
    const double p_DRONE_CAMERA_HALF_PITCH_AOV {};
    
    const int p_WPARR_MAXDUR {};

    //const double m_GTMinWeightVel;
    //const double m_GTMinWeightAcc;
    //const double m_GTMinWeightJerk;
    //const double m_GTMinWeightSnap;
    const double p_GLOBAL_TRAJECTORY_MAX_THRUST {};
    const double p_GLOBAL_TRAJECTORY_MAX_ROLL_PITCH_RATE {};
    const double p_GLOBAL_TRAJECTORY_MAX_SPEED {};
    const double p_GLOBAL_TRAJECTORY_NON_DIMENSIONAL_TEMPORAL_RANGE {};
    const double p_GLOBAL_TRAJECTORY_NON_DIMENSIONAL_SPATIAL_RANGE {};
    const double p_WAYPOINT_ARRIVAL_THRESHOLD_DISTANCE {};
    const double p_EXPERT_MIN_HORIZON {};
    //const double m_DRONE_CAMERA_HALF_YAW_AOV;
    //const double m_DRONE_CAMERA_HALF_PITCH_AOV;
    const double p_EXPERT_SPEED_HORIZON {};
    //const double m_ROSParam_NominalExpertLoopTime;
    const double p_NAV_REFSTATE_MAXDIVERGENCE {};
    const double p_EXPERT_PROJECTION_MAX_DIVERGENCE_FROM_GLOBAL_TRAJECTORY {};



    /////
    const int p_NAV_REPLANNING_MAINLOOPITERSCNT {};
    const int p_NAV_LOCTRAJ_FEASIBILITY_MAXSUCCESSIVEFAILSCNT {};

    const int p_REPEATED_SETUP_RUN_N {};

    const double p_NAV_INPUTPERTURBATION_ELEMENTWISEAMP {};
    const double p_MAIN_LOOP_FREQUENCY {};
    
    const double p_LOCAL_TRAJECTORY_MIN_SPEED {};
    const double p_LOCAL_TRAJECTORY_MAX_SPEED_INCREMENT {};
    const double p_LOCAL_TRAJECTORY_DURATION {};
    const double p_LOCAL_TRAJECTORY_MIN_DISTANCE {};
    const double p_LOCAL_TRAJECTORY_MAX_DISTANCE {};
    const double p_LOCAL_TRAJECTORY_MAX_ALTITUDE {};
    const double p_LOCAL_TRAJECTORY_MIN_ALTITUDE {};
    const double p_LOCAL_TRAJECTORY_MIN_THRUST {};
    const double p_LOCAL_TRAJECTORY_MAX_THRUST {};
    const double p_LOCAL_TRAJECTORY_MAX_BODY_RATES {};
    const double p_LOCAL_TRAJECTORY_INPUT_FEASIBILITY_CHECK_MIN_SAMPLING_TIME {};


    const bool p_RVIZ_LOCAL_TRAJECTORY_DISPLAY_ELAPSED_ENABLED {};
    const double p_RVIZ_LOCAL_TRAJECTORY_DURATION {};
    const double p_RVIZ_LOCAL_TRAJECTORY_SAMPLING_FREQUENCY {};

    
    const int p_BRAIN_TORCHDEVICE {};
    const int p_TORCHINTERFACE_IDX {};


    const int p_DAGGER_FIRSTRUN_NUMLAPS {};
    const int p_DAGGER_NUM_EPOCHS {};
    const double p_DAGGER_EXPERT_INTERVENTION_SHARE_THRESHOLD {};
    


void performFlightMission_TrainingDataGeneration();
void performFlightMission_NavigationByBrain();
void performFlightMission_NavigationByExpert();
void performFlightMission_GlobalTrajectoryTracking();
void performFlightMission_DAGGER();

void waitForAutopilotState(
    const uint8_t& ap_state,
    const bool& exit_state = false) const;
void launchDroneOffGround();

void flyDroneToPose(const geometry_msgs::Pose& target_pose) const;
void flyDroneToInitPose();
void flyDroneAboveTrack();
//void flyDroneBetweenLastAndSecLastGate();
void landDroneOnGround();
bool computeGlobalTrajectory();
void precomputeGloTrajForExpert();
void rvizGloTraj();

void initMainLoopTimer (void (forgetful_drone::ForgetfulDrone::*callback)(const ros::TimerEvent&));

/////////////////////////////
private:// ANN PARAMETERS //
///////////////////////////

const int p_ANN_GRU_HIDDENSIZE {};
const int p_ANN_GRU_NUMLAYERS {};
const int p_DATA_PROCESSED_RGB_HEIGHT {};
const int p_DATA_PROCESSED_RGB_WIDTH {};

std::vector<float> m_ANNGRUHiddenState;



/////////////////////////////////////////
private:// NON-CONST MEMBER VARIABLES //
///////////////////////////////////////

std::vector<geometry_msgs::Pose, std::allocator<geometry_msgs::Pose>> m_GateInitPoses;
geometry_msgs::Pose m_DroneInitPose;


double m_Expert_MaxHorizon;





const kindr::minimal::QuatTransformation p_T_WRF_ARF {};





std::mutex m_RGBMtx;
std::mutex m_IMUMtx;
std::mutex m_CtrlCmdMtx;

std::string m_RunConfigDpath;

std::string m_RunID;
std::string m_RunDpath;
std::string m_RunRGBDpath;
std::string m_RunDataFpath;
std::string m_RunDaggerLabeledFramesDpath;

//std::string m_RunCountString;






const bool p_EXPERIMENT_NEW {};
const std::string m_EXPERIMENT_DPATH {};
const std::string p_RAW_DPATH {};
const std::string p_CONFIG_DPATH {}, p_OUTPUT_DPATH {};

const int p_FLIGHTMISSION {};
const std::string p_EXPERIMENT_ID {};
const std::vector<int> p_FLIGHTMISSION_UNITYSCENES {};
const std::vector<int> p_FLIGHTMISSION_SCENESITES {};
const std::vector<int> p_FLIGHTMISSION_TRACKTYPES {};
const std::vector<int> p_FLIGHTMISSION_TRACKGENERATIONS {};
const std::vector<int> p_FLIGHTMISSION_TRACKDIRECTIONS {};
const std::vector<int> p_FLIGHTMISSION_GATETYPES {};
const std::vector<double> p_AUTOPILOT_REFFRAME_WRF {};
const std::vector<double> p_LOCTRAJ_MAXSPEEDS {};
const std::vector<double> p_DAGGERMARGINS {};




const std::string p_RACETRACK_TYPE {};
const std::string p_UNITY_SCENE {};
const std::string p_SCENE_SITE {};
const std::string p_GATE_TYPE {};
const std::string p_INTERMEDIATE_TARGET_LOSS_DIRECTION {};
const std::string p_INTERMEDIATE_TARGET_LOSS_GAP_TYPE {};







bool verifyStringParameters () const;
void logRunInfo (const bool& num_runs_known);
void logRunInfo (
    const int& run_cnt,
    const int& run_n,
    const std::string& unity_scene,
    const std::string& scene_site,
    const std::string& racetrack_type,
    const std::string& gate_type,
    const std::string& gap_type = "",
    const std::string& direction = "",
    const int& rep_i = -1,
    const int& rep_n = -1
) const;
void logFlightMissionInfo () const;
void logFlightMissionResults () const;
void reset4NewRun ();
void buildSimulation (
    const std::string& racetrack_type,
    const std::string& unity_scene,
    const std::string& scene_site,
    const std::string& gate_type,
    const std::string& itl_direction,
    const std::string& itl_gap_type
);

void buildSimulation (ForgetfulSimulator& fs); void buildSimulation ();
void setTrackWaypoints ();
void initWaypointIndices ();
void setExpertMaxHorizon ();
void insertGapWaypoint ();

void insertITLWaypoint ();
void startSimulation (ForgetfulSimulator& fs); void startSimulation ();
bool runNavigation ();
void initExperimentDirectory ();
void createRunDirectory (
    const int& scene_idx,
    const int& site_idx,
    const int& gate_idx,
    const int& rep_idx = -1,
    const int& dir_idx = -1,
    const int& gap_idx = -1
);
void createRunConfigDirectory ();


void initRunDirectory ();

void initRunDaggerDirectory ();
void stopSimulation (ForgetfulSimulator& fs); void stopSimulation ();

bool initROSParameters ();

bool initBrain ();
bool initBrain_Python ();
bool initBrain_Cpp ();

bool checkPaths ();
bool initExperimentID ();
bool initTrafoArf2Wrf ();

std::string ExperimentDpath();
std::string DataDpath();
std::string RawDpath();
std::string ConfigDpath();
std::string OutputDpath();

std::string RunID ();
std::string RunDPath ();
std::string ConfigFpath ();
std::string ScriptModuleFpath ();
std::string ParametersFpath (const bool& src_not_dst);





static constexpr const char* h_EXPERIMENTS_DNAME = "experiments";
static constexpr const char* h_DATA_DNAME = "data";
static constexpr const char* h_RAW_DNAME = "raw";
static constexpr const char* h_CONFIG_DNAME = "config";
static constexpr const char* h_OUTPUT_DNAME = "output";
static constexpr const char* h_PARAMETERS_DNAME = "parameters";
static constexpr const char* h_PARAMETERS_FNAME = "forgetful_drone.yaml";
static constexpr const char* h_RUN_DNAME_PREFIX = "run";
static constexpr const char* h_RUN_IMAGES_DNAME = "rgb";
static constexpr const char* h_CAMFRAME_FEXT = ".jpg";
static constexpr const char* h_DATA_FNAME = "data.txt";
static constexpr const char* h_IMU_FNAME = "imu.txt";
static constexpr const char* h_CONTROLCOMMAND_FNAME = "control_command.txt";
static constexpr const char* h_WAYPOINTSPEED_FNAME = "waypoint_speed.txt";
static constexpr const char* h_RUN_LABELEDIMAGES_DNAME = "labeled_images";
static constexpr const char* h_RUN_DAGGER_DNAME_PREFIX = "dagger";
static constexpr const char* h_RECORDINGS_FNAME = "recordings.json";
static constexpr const char* h_FAILEDRUNS_FNAME = "failed_runs.txt";
static constexpr const char* h_CONFIG_FNAME = "config.json";
static constexpr const char* h_SCRIPTMODULE_FNAME = "model_scripted_with_annotation.pt";


static constexpr int h_BATCHSIZE = 1;
static constexpr int h_SEQUENCE_LENGTH = 1;



static constexpr std::array<const char*, 5> h_FLIGHTMISSIONS {
    "GLOBAL_TRAJECTORY_TRACKING",
    "NAVIGATION_BY_EXPERT",
    "NAVIGATION_BY_ANN",
    "TRAINING_DATA_GENERATION",
    "DAGGER"
};

//&ForgetfulDrone::ROSCB_NAVIGATION_BY_ANN_PYTHON
//void (forgetful_drone::ForgetfulDrone::*callback)(const ros::TimerEvent&)
//static constexpr std::array<const char*, 2> h_BRAIN_TORCHINTERFACES {
//    "PYTHON",
//    "CPP",
//};
static constexpr std::array<std::tuple<const char*, 
    void (forgetful_drone::ForgetfulDrone::*)(const ros::TimerEvent&)>, 2> h_BRAIN_TORCHINTERFACES {
    std::make_tuple("PYTHON", &ForgetfulDrone::ROSCB_NAVIGATION_BY_ANN_PYTHON),
    std::make_tuple("CPP", &ForgetfulDrone::ROSCB_NAVIGATIONBYANN_CPP),
};


static constexpr std::array<std::tuple<const char*, c10::DeviceType>, 2> h_BRAIN_TORCHDEVICES {
    std::make_tuple("CPU", torch::kCPU),
    std::make_tuple("CUDA", torch::kCUDA),
};

const std::vector<std::string> h_UNITY_SCENES {
    "spaceship_interior",
    "destroyed_city",
    "industrial_park",
    "polygon_city",
    "desert_mountain",
};
const std::vector<std::string> h_SCENE_SITES {
    "a",
    "b",
    "c",
};
const std::vector<std::string> h_TRACK_TYPES {
    "figure8_determinstic",
    "figure8_randomized",
    "circuit_randomized",
    "sprint_randomized",
    "intermediate_target_loss_randomized",
};
const std::vector<std::string> h_GATE_TYPES {
    "tub_dai",
    "thu_dme",
};
const std::vector<std::string> m_ITL_GAP_TYPE_STRS {
    "narrow",
    "wide",
};
const std::vector<std::string> m_ITL_DIRECTION_STRS {
    "clockwise",
    "counterclockwise",
};
const std::unordered_map<std::string, uint8_t> m_STR2VAL_MAP {

    {h_TRACK_TYPES[0], BDRSR::FIGURE8_DETERMINISTIC},
    {h_TRACK_TYPES[1], BDRSR::FIGURE8_RANDOMIZED},
    {h_TRACK_TYPES[2], BDRSR::CIRCUIT_RANDOMIZED},
    {h_TRACK_TYPES[3], BDRSR::SPRINT_RANDOMIZED},
    {h_TRACK_TYPES[4], BDRSR::INTERMEDIATE_TARGET_LOSS},

    {h_UNITY_SCENES[0], BDRSR::SPACESHIP_INTERIOR},
    {h_UNITY_SCENES[1], BDRSR::DESTROYED_CITY},
    {h_UNITY_SCENES[2], BDRSR::INDUSTRIAL_PARK},
    {h_UNITY_SCENES[3], BDRSR::POLYGON_CITY},
    {h_UNITY_SCENES[4], BDRSR::DESERT_MOUNTAIN},

    {h_SCENE_SITES[0], BDRSR::SITE_A},
    {h_SCENE_SITES[1], BDRSR::SITE_B},
    {h_SCENE_SITES[2], BDRSR::SITE_C},

    {h_GATE_TYPES[0], BDRSR::TUB_DAI_GATE},
    {h_GATE_TYPES[1], BDRSR::THU_DME_GATE},

    {m_ITL_GAP_TYPE_STRS[0], BDRSR::NARROW_GAP},
    {m_ITL_GAP_TYPE_STRS[1], BDRSR::WIDE_GAP},

    {m_ITL_DIRECTION_STRS[0], true},
    {m_ITL_DIRECTION_STRS[1], false},
};


double m_ActualNormSpeed;



size_t m_GT_HorizonState_i;
size_t m_GT_SpeedState_i;


c10::DeviceType m_TorchDevice;
torch::TensorOptions m_TorchTensorOptions;








quadrotor_common::TrajectoryPoint m_RefState_ARF;


bool m_GT_CompFailed;

ros::Time m_WpArrLastTime;


//////////////////////
private:// METHODS //
////////////////////

//void switchDynamicGates( const bool& SwitchOn );
void switchNavigator( const bool& SwitchOn );

void resetRVIZ();



public:
    
    
private:
    void gGT_computePreliminaryTemporalCoordinates();



//////////////////////
private:// FRIENDS //
////////////////////

friend bool fetchROSParameters (
    const ros::NodeHandle& rosNH,
    const std::vector<std::pair<const char*, const bool*>>& keys_bools,
    const std::vector<std::pair<const char*, const int*>>& keys_ints,
    const std::vector<std::pair<const char*, const double*>>& keys_doubles,
    const std::vector<std::pair<const char*, const std::string*>>& keys_strings,
    const bool& log_enabled
);


};


void generateRacetrackData();
std::vector<quadrotor_common::TrajectoryPoint> computeGlobalTrajectory
(std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> Waypoints);
}