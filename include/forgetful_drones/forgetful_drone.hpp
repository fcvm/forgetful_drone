#pragma once

//#include <filesystem>
#include <experimental/filesystem>

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float64.h>
#include <std_srvs/Empty.h>
#include <Eigen/Dense>
#include <ros/package.h>
#include "forgetful_drones/NavigatorState.h"
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


#include "forgetful_drones/forgetful_ann.hpp"

#include "forgetful_drones/String.h"
#include "forgetful_drones/Int.h"
#include "forgetful_drones/Float.h"


namespace forgetful_drone {
namespace RQTG = RapidQuadrocopterTrajectoryGenerator;


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

    ros::Subscriber m_SUB__gtOdometry;
    ros::Subscriber m_SUB__gtIMU;
    ros::Subscriber m_SUB__apFeedback;
    ros::Subscriber m_SUB__CtrlCmd;
    ros::Subscriber m_SUB__RGB;

    ros::Publisher m_PUB__rvNavPoints;
    ros::Publisher m_PUB__rvLblRGB;
    ros::Publisher m_PUB__rvGloTraj;
    //ros::Publisher m_ROSPUB_SIMULATOR_DYNAMIC_GATES_SWITCH;
    ros::Publisher m_PUB__apOff;
    ros::Publisher m_PUB__apStart;
    ros::Publisher m_PUB__apLand;
    ros::Publisher m_PUB__bridgeArm;
    //ros::Publisher m_ROSPUB_NAVIGATOR_STATE_SWITCH;
    ros::Publisher m_PUB__apRefState;
    ros::Publisher m_PUB__rvLocTraj;
    ros::Publisher m_PUB__apPoseCmd;


    ros::ServiceClient m_SCL__simBuild;
    ros::ServiceClient m_SCL__simStart;
    ros::ServiceClient m_SCL__simStop;
    ros::ServiceClient m_SCL__simTeleport;
    //ros::ServiceClient m_rosSVC_RVIZ_LOAD_CONFIG;




    // Forgetful Brain
    ros::ServiceClient  m_SCL__fbExpment;
    ros::ServiceClient  m_SCL__fbBuild;
    ros::ServiceClient  m_SCL__fbTrain;
    ros::ServiceClient  m_SCL__fbInfer;
    ros::Publisher      m_PUB__fbTrigger;
    ros::Subscriber     m_SUB__fbOutput;









    
    void ROSCB_GROUND_TRUTH_ODOMETRY (const nav_msgs::OdometryConstPtr& msg);
    void ROSCB_GROUND_TRUTH_IMU (const sensor_msgs::ImuConstPtr& msg);
    void ROSCB_AUTOPILOT_FEEDBACK (const quadrotor_msgs::AutopilotFeedback::ConstPtr& msg);
    void ROSCB_CONTROL_COMMAND (const quadrotor_msgs::ControlCommand::ConstPtr& msg);
    void CB__fbOutput (const geometry_msgs::PointConstPtr& msg);
    void ROSCB_FLIGHTMARE_RGB (const sensor_msgs::ImageConstPtr& msg);


    uint8_t m_AutopilotState;
    Eigen::Vector3d m_ExpOut;
    Eigen::Vector3d m_BrainOutput;
    Eigen::Vector3d m_NavInput;
    std::vector< quadrotor_common::TrajectoryPoint > m_GloTraj;
    std::vector< Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > m_GloTrajWayps;
    size_t m_ExpStateIdx;
    double m_Expert_MaxSpeed;
    double m_Expert_MinSpeed;
    kindr::minimal::QuatTransformation m_WRF_LRF;
    kindr::minimal::QuatTransformation m_ARF_LRF;
    Eigen::Vector3d m_RefStatePos_WRF;
    size_t m_CurrGateIdx;
    size_t m_LastGateIdx;
    double m_CurrGateDist;
    double m_LastGateDist;
    bool m_NavEnabled;
    bool m_SaveEnabled;
    int m_LocTraj_SuccInfeasCnt;
    ros::Time m_LocTraj_t0;
    RQTG::RapidTrajectoryGenerator m_LocTraj;
    unsigned int m_MainIterCnt;
    unsigned int m_LocTrajFeasibleCnt;
    int m_RunIdx;
    bool m_DroneLaunched;
    int m_RunRepIdx;
    int m_RunRepSuccCnt;
    unsigned int m_RGBCnt;
    int m_LapIdx;
    int m_RunNumLaps;
    int m_ExpDecCnt;
    int m_ANNDecCnt;
    int m_ConfigCnt;
    int m_ConfigMarginCnt;
    float m_DaggerMargin;
    bool m_NavInputDisturbed;


    uint8_t m_SceneIdx;
    uint8_t m_SiteIdx;
    uint8_t m_TrackTypeIdx;
    uint8_t m_TrackGenIdx;
    uint8_t m_TrackDirecIdx;
    uint8_t m_GateIdx;
    double m_MaxSpeed;





    sensor_msgs::ImuConstPtr m_IMUPtr;
    std::array<double, 6> m_IMU;
    double m_IMULastStampTime;
    double m_IMU_dt;
    cv_bridge::CvImageConstPtr /*cv::Mat*/ m_RGBPtr;
    cv::Mat m_RGB;
    double m_RGB_dt;
    double m_RGBLastStampTime;
    void rvizLblRGB();
    //quadrotor_msgs::AutopilotFeedback m_AutopilotFeedback;
    quadrotor_msgs::ControlCommand::ConstPtr /*quadrotor_msgs::ControlCommand*/ m_CtrlCmdPtr;
    std::array<double, 7> m_CtrlCmd;

    
            


    void fetchRGBData ();
    void fetchIMUData ();
    void fetchCtrlCmdData ();


    
    double readExpIvShare (const int& run_idx);
    bool BuildRecorded (const std::string& run_id);

    
    
    json openJSON (const std::string& path);

    void addRacingRecord (const std::string& run_id, const bool& completed);
    bool RacingRecorded (const std::string& run_id);


    

    ros::Timer m_rosTMR_MAINLOOP;
    void ROSCB_testExp (const ros::TimerEvent& te);
    void ROSCB_TRAINING_DATA_GENERATION (const ros::TimerEvent& te);
    void CB_Racing_Cpp (const ros::TimerEvent& te);
    void CB_Racing_Python (const ros::TimerEvent& te);
    void CB_DAGGER_Python (const ros::TimerEvent& te);


            void runStatusTracker();
            void runExp();
            void triggerBrain();
            void runDataSaver (const bool& expert_intervened);
            void saveOdometry ();
            void runBrain();
            void runNav(bool& intervened, void (forgetful_drone::ForgetfulDrone::*intervention_fct)(Eigen::Vector3d& _0, double& _1, bool& _2));

            void updStatus();
            bool computeGlobalTrajAsCircuit();
            void findExpState();
                void minDist2GloTraj (
                    const Eigen::Vector3d position,
                    double& min_dist,
                    size_t& min_dist_state_idx
                );
            Eigen::Vector3d LRF___WRF( const Eigen::Vector3d& PosWRF );
            Eigen::Vector2d IRF___LRF( const Eigen::Vector3d& PosDRF );
            void findSpeedState();
            void findWaypState( const double& Horizon );
            void rvizExpAndWaypState();
            void rvizCurrGate();
            void rvizRefState();

            void procNavInput (Eigen::Vector3d& OUT_GoalPos_DRF, double& OUT_Speed2Goal);
            void expIntervention (Eigen::Vector3d& OUT_GoalPos_DRF, double& OUT_Speed2Goal, bool& exp_intervened);
            void noIntervention (Eigen::Vector3d& OUT_GoalPos_DRF, double& OUT_Speed2Goal, bool& exp_intervened);
            Eigen::Vector3d LRF____IRF( 
                const double& X_IRF, 
                const double& Y_IRF, 
                const double& Dist_IRF 
                );
            Eigen::Vector3d ARF___LRF( const Eigen::Vector3d& XYZ_DRF );
            bool compLocTraj(
            const Eigen::Vector3d& GoalPos_DRF,
            const double& Speed2Goal
            );
            bool checkFeasibility (RQTG::RapidTrajectoryGenerator& lt);


    

        void rvizLocTraj();

        void pubRefState();






/////////////////////////////
private:// ROS PARAMETERS //
///////////////////////////

    //const bool m_DYNAMIC_GATES_ON;
    const bool p_RVIZ_LBLRGB_SAVE {};
    const bool p_RVIZ_LBLRGB_ENABLED {};


    const int p_NRUNREPS {};
    
    const int p_GLOBAL_TRAJECTORY_POLYNOMIAL_ORDER {};
    const int p_GLOBAL_TRAJECTORY_CONTINUITY_ORDER {};
    const int p_RUN_NUMLAPS {};
    

    const double p_CAM_HALFYAWAOV {};
    const double p_CAM_HALFPITCHAOV {};
    
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
    const double p_GATE_THRESH_DIST {};
    const double p_EXPERT_MIN_HORIZON {};
    //const double m_DRONE_CAMERA_HALF_YAW_AOV;
    //const double m_DRONE_CAMERA_HALF_PITCH_AOV;
    const double p_EXPERT_SPEED_HORIZON {};
    //const double m_ROSParam_NominalExpertLoopTime;
    const double p_NAV_MAX_DIV {};
    const double p_EXP_THRESHOLD_DIV {};



    /////
    const int p_NAV_REPLAN_ITER_CNT {};
    const int p_LOCTRAJ_MAXSUCCINFEASCNT {};

    const int p_REPEATED_SETUP_RUN_N {};

    const double p_NAV_INPUTDISTURBAMP {};
    const double p_MAIN_FREQ {};
    
    const double p_LOCTRAJ_MINSPEED {};
    const double p_LOCAL_TRAJECTORY_MAX_SPEED_INCREMENT {};
    const double p_LOCTRAJ_DUR {};
    const double p_LOCTRAJ_MINDIST {};
    const double p_LOCTRAJ_MAXDIST {};
    const double p_LOCAL_TRAJECTORY_MAX_ALTITUDE {};
    const double p_LOCAL_TRAJECTORY_MIN_ALTITUDE {};
    const double p_LOCAL_TRAJECTORY_MIN_THRUST {};
    const double p_LOCAL_TRAJECTORY_MAX_THRUST {};
    const double p_LOCAL_TRAJECTORY_MAX_BODY_RATES {};
    const double p_LOCAL_TRAJECTORY_INPUT_FEASIBILITY_CHECK_MIN_SAMPLING_TIME {};


    const bool p_RVIZ_LOCAL_TRAJECTORY_DISPLAY_ELAPSED_ENABLED {};
    const double p_RVIZ_LOCAL_TRAJECTORY_DURATION {};
    const double p_RVIZ_LOCAL_TRAJECTORY_SAMPLING_FREQUENCY {};

    
    const int p_TORCH_DEVICE {};
    const int p_TORCH_INTERFACE_IDX {};


    const int p_DAGGER_FIRSTRUN_NUMLAPS {};
    const int p_DAGGER_NEPOCHS {};
    const double p_EIS_THRSH {};
    


void runMission_Racing();
void runMission_testExp();
void runMission_trackGloTraj();
void runMission_DAGGER();

void waitForAutopilotState(
    const uint8_t& ap_state,
    const bool& exit_state = false) const;
bool wait4APState (const uint8_t& ap_state, const bool& enter, const double& dur) const;
bool launchDrone();

bool flyDroneTo(const geometry_msgs::Pose& target_pose, const double& max_dur) const;
bool flyDroneToInitPose (const bool& from_above);
bool flyDroneAboveTrack ();
bool carryDroneBack ();
bool landDrone();
bool compGloTraj ();
void rvizGloTraj ();

void initMainLoopTimer (void (forgetful_drone::ForgetfulDrone::*callback)(const ros::TimerEvent&));

/////////////////////////////
private:// ANN PARAMETERS //
///////////////////////////

ForgetfulANN m_ANN;





/////////////////////////////////////////
private:// NON-CONST MEMBER VARIABLES //
///////////////////////////////////////

std::vector<geometry_msgs::Pose, std::allocator<geometry_msgs::Pose>> m_GateInitPoses;
geometry_msgs::Pose m_DroneInitPose;


double m_ExpMaxHor;





const kindr::minimal::QuatTransformation p_WRF_ARF {};





std::mutex m_RGBMtx;
std::mutex m_IMUMtx;
std::mutex m_CtrlCmdMtx;

std::string m_RunConfigDpath;

std::string m_RunDpath;
std::string m_exp_dat_raw_rid_rgb_;
std::string m_exp_dat_raw_rid_lbl_;
std::string m_exp_dat_raw_rid_TXT;
std::string m_RunDaggerLabeledFramesDpath;

std::string m_exp_rac_RID;

//std::string m_RunCountString;






const std::string m_EXPERIMENT_DPATH {};
const std::string p_RAW_DPATH {};
const std::string p_CONFIG_DPATH {}, p_OUTPUT_DPATH {};

const int p_MISSION {};
const std::string p_EXPERIMENT_ID {};
const std::vector<int> p_SCENES {};
const std::vector<int> p_SITES {};
const std::vector<int> p_TRACK_TYPES {};
const std::vector<int> p_TRACK_GENS {};
const std::vector<int> p_TRACK_DIRECS {};
const std::vector<int> p_GATES {};
const std::vector<double> p_ARF_POSE_WRF {};
const std::vector<double> p_MAXSPEEDS {};
const std::vector<double> p_DAGGERMARGINS {};




const std::string p_RACETRACK_TYPE {};
const std::string p_UNITY_SCENE {};
const std::string p_SCENE_SITE {};
const std::string p_GATE_TYPE {};
const std::string p_INTERMEDIATE_TARGET_LOSS_DIRECTION {};
const std::string p_INTERMEDIATE_TARGET_LOSS_GAP_TYPE {};







bool verifyStringParameters () const;
void logRunInfo ();

void logMissionInfo (const bool& start) const;
void newRunReset ();

void buildSimulation ();
void setTrackWaypoints ();
void initGateIdc ();
void findExpMaxHor ();
void insertGapWaypoint ();



bool startNavigation ();


void initRacingPaths ();
void initRawRunPaths ();

void startSim ();
void stopSimulation ();



bool initANN ();
bool initANN_Python ();
bool initANN_Cpp ();

bool initExperiment ();


















// Init
bool initTrafoArf2Wrf ();
bool initROSParameters ();

static constexpr std::array <const char*, 5> h_MISSIONS {
    "RACING_GLOBAL_TRAJECTORY",
    "RACING_EXPERT",
    "RACING_ANN",
    "DAGGER"
};



// Paths
std::string RunID ();
std::string _exp_ ();
std::string _exp_cnf_ ();
std::string _exp_cnf_YML (const bool& src);
std::string _exp_cnf_CNF ();
std::string _exp_dat_ ();
std::string _exp_dat_raw_ ();
std::string _exp_dat_raw_rid_ ();
std::string _exp_dat_raw_rid_rgb_ ();
std::string _exp_dat_raw_rid_lbl_ ();
std::string _exp_dat_raw_rid_TXT ();
std::string _exp_out_ ();
std::string _exp_out_BRC ();
std::string _exp_out_TRC ();
std::string _exp_out_CPT ();
std::string _exp_out_ANT ();
std::string _exp_out_TRA ();
std::string _exp_rac_ ();
std::string _exp_rac_RRC ();
std::string _exp_rac_RID ();



// Brain
    //static constexpr int h_BATCHSIZE = 1;
    //static constexpr int h_SEQUENCE_LENGTH = 1;

static constexpr std::array <const char*, 9> h_OPTINPUTS {
    "rgb_dt", "imu_dt",
    "imu_linacc_x", "imu_linacc_y", "imu_linacc_z",
    "imu_angvel_x", "imu_angvel_y", "imu_angvel_z",
    "max_speed"
};
static constexpr std::array <std::tuple <const char*, 
    void (forgetful_drone::ForgetfulDrone::*)(const ros::TimerEvent&)>, 2> h_TORCH_INTERFACES {
    std::make_tuple ("PYTHON", &ForgetfulDrone::CB_Racing_Python),
    std::make_tuple ("CPP", &ForgetfulDrone::CB_Racing_Cpp),
};


// RVIZ
double m_ActualNormSpeed;


// Expert
size_t m_WaypStateIdx;
size_t m_SpeedStateIdx;

// Navigator
quadrotor_common::TrajectoryPoint m_RefState_ARF;
void switchNav (const bool& enabled);

// Status Tracker
ros::Time m_LastGateTime;





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
}