#pragma once




#include <ros/ros.h>
#include "forgetful_drones_msgs/BuildDroneRacingSimulation.h"
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float64.h>
#include <Eigen/Dense>
#include <ros/package.h>
#include "forgetful_drones_msgs/FlyDroneThroughRaceTrack.h"
#include "forgetful_drones_msgs/FlyDroneProvidingNetworkOutput.h"
#include "forgetful_drones_msgs/NavigatorState.h"
#include "forgetful_drones_msgs/ComputeGlobalTrajectory.h"
#include <quadrotor_msgs/AutopilotFeedback.h>
#include <ros/console.h>
#include <sensor_msgs/Image.h>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PoseStamped.h>



#include <trajectory_generation_helper/polynomial_trajectory_helper.h>
#include <minkindr_conversions/kindr_msg.h>

#include <rapid_trajectories/RapidTrajectoryGenerator.h>
#include <rapid_trajectories/Vec3.h>
#include <thread>
#include <future>

#include <rviz/SendFilePath.h>

namespace forgetful_drone{
namespace RQTG = RapidQuadrocopterTrajectoryGenerator;

class ForgetfulDrone{


////////////////////
private:// ENUMS //
//////////////////
        
    /// Contains all implemented flight missions selectable by the user through ROS parameter.
    /// 1) generateTrainingData
    /// 2) testNavigatorWithNetwork
    /// 3) testNavigatorWithExpert
    /// 4) testSimulatorWithAutopilot
    enum class FlightMissions
    {
        generateTrainingData,
        testNavigatorWithNetwork,
        testNavigatorWithExpert,
        testSimulatorWithAutopilot
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
    

    ros::NodeHandle m_ROSRNH; // ROS node handle that is resolved to the node's namespace.
    ros::NodeHandle m_ROSPNH; // ROS node handle that is resolved to the namespace: <node's namespace>/<node's name>.

    ros::Subscriber m_ROSSub_Odometry;
        void ROSCB_Odometry( const nav_msgs::OdometryConstPtr& msg );
    

    ros::Subscriber m_ROSSub_AutopilotFeedback;
        void ROSCB_AutopilotFeedback( const quadrotor_msgs::AutopilotFeedback::ConstPtr& msg );


    ros::Subscriber m_ROSSub_NetworkPrediction;
        void ROSCB_NetworkPrediction( const geometry_msgs::PointConstPtr& msg );
            Eigen::Vector3d m_NetworkPrediciton;

    ros::Subscriber m_ROSSub_RGBCameraImageRaw;
        void ROSCB_RGBCameraImageRaw( const sensor_msgs::ImageConstPtr& msg );
            cv::Mat m_CamFrame;
            cv::Mat m_CamFrameWithPredictions;
            void rvizCamFrameWithPredictions();
    ros::Publisher m_ROSPub_RVIZCamFrameWithPredictions;


    ros::Publisher m_ROSPub_DynGates;
    ros::Publisher m_ROSPub_AutopilotOff;
    ros::Publisher m_ROSPub_AutopilotStart;
    ros::Publisher m_ROSPub_AutopilotLand;
    ros::Publisher m_ROSPub_BridgeArm;
    ros::Publisher m_ROSPub_StartNavigation;
    ros::Publisher m_ROSPub_StopNavigation;
    ros::Publisher m_ROSPub_NavigatorState;
    ros::Publisher m_ROSPub_RVIZGlobalTrajectory;

    ros::Publisher m_ROSPub_RVIZPositions;

    ros::Publisher m_ROSPub_AutopilotReferenceState;
    ros::Publisher m_ROSPub_RVIZLocalTrajectory;

    ros::Publisher m_ROSPub_AutopilotPoseCommand;



    ros::ServiceClient m_ROSSrvCl_BuildDroneRacingSimulation;
    ros::ServiceClient m_ROSSrvCl_ComputeGlobalTrajectory;
    
    ros::ServiceClient m_ROSSrvCl_RVIZLoadConfig;

    

    ros::Timer m_ROSTimer_MainLoop;
        void ROSTimerFunc_MainLoop( const ros::TimerEvent& TimerEvent );
        void ROSTimerFunc_testNavigatorWithExpert( const ros::TimerEvent& TimerEvent );
        void ROSTimerFunc_generateTrainingData( const ros::TimerEvent& TimerEvent );
        void ROSTimerFunc_testNavigatorWithNetwork( const ros::TimerEvent& TimerEvent );

            void runWaypointUpdater();
            //void runGloTraj();
            void runExpert();
            void runTrainDataSaver();
            void runNetwork();
            void runNavigator();

            void updateWaypointIdx();
            bool computeGlobalTrajAsCircuit();
            void findExpertStateWithProjection();
                void findExpertStateWithMinDist();
            Eigen::Vector3d PosDRF_From_PosWRF( const Eigen::Vector3d& PosWRF );
            Eigen::Vector2d PosIRF_From_PosDRF( const Eigen::Vector3d& PosDRF );
            void findSpeedState();
            void findHorizonState( const double& Horizon );
            void rvizExpertAndHorizonState();
            void rvizCurrWaypoint();
            void rvizReferenceState();

            void processNavigatorInput( Eigen::Vector3d& OUT_GoalPos_DRF, double& OUT_Speed2Goal );
            Eigen::Vector3d XYZ_DRF_From_XYDist_IRF( 
                const double& X_IRF, 
                const double& Y_IRF, 
                const double& Dist_IRF 
                );
            Eigen::Vector3d XYZ_ARF_From_XYZ_DRF( const Eigen::Vector3d& XYZ_DRF );
            bool computeLocTraj(
            const Eigen::Vector3d& GoalPos_DRF,
            const double& Speed2Goal
            );
            bool LocTrajFeasible( RQTG::RapidTrajectoryGenerator& LocalTrajectory );

            void markTrainingDataOfCurrRunAsFailed();
            void saveTrainData();
    

        void rvizLocTraj();

        void publishRefStateFromLocTrajToAutopilot();




/////////////////////////////
private:// ROS PARAMETERS //
///////////////////////////

    const bool m_SIMDynGatesEnabled;


    const int m_ORCRunMaxCount;
    
    const int m_GTPolyOrder;
    const int m_GTContiOrder;
    const int m_WUInitWaypointIdxForFIG8;
    const int m_ORCLapMaxCount;
    

    const double m_DRONECamHalfYawAOV;
    const double m_DRONECamHalfPitchAOV;
    
    const double m_ORCNoWaypointArrivalMaxDuration;

    const double m_GTMinWeightVel;
    const double m_GTMinWeightAcc;
    const double m_GTMinWeightJerk;
    const double m_GTMinWeightSnap;
    const double m_GTMaxNormThrust;
    const double m_GTMaxRollPitchRate;
    const double m_GTMaxSpeed;
    const double m_WUThresDist2DetectWaypointArrival;
    const double m_EXPMinHorizon;
    //const double m_DRONECamHalfYawAOV;
    //const double m_DRONECamHalfPitchAOV;
    const double m_EXPSpeedHorizon;
    //const double m_ROSParam_NominalExpertLoopTime;
    const double m_NAVMaxDivFromRefState;
    const double m_EXPMaxDivFromGT4Projection;



    /////
    const int m_NAVMainLoopItersPerLTReplanning;
    const int m_NAVSubseqFailedLTReplanningMaxCount;

    const double m_NAVInputPerturbAxAmp;
    const double m_ORCMainLoopNomPeriodDuration;
    const double m_LTMaxSpeed;
    const double m_LTMinSpeed;
    const double m_LTMaxSpeedIncrement;
    const double m_LTDuration;
    const double m_LTMinDistance;
    const double m_LTMaxDistance;
    const double m_LTMaxAltitude;
    const double m_LTMinAltitude;
    const double m_LTMinNormThrust;
    const double m_LTMaxNormThrust;
    const double m_LTMaxBodyRates;
    const double m_LTInputFeasibilityCheckMinSamplingTime;


    const double m_GTSamplingTime;


    const bool   m_RVIZElapsedLTsEnabled;
    const double m_RVIZLTDuration;
    const double m_RVIZLTSamplingFreq;




void generateTrainingData();
void testNavigatorWithNetwork();
void testNavigatorWithExpert();
void testSimulatorWithAutopilot();

void buildDroneRacingSimulation();
void launchDroneOffGround();
void flyDroneToInitPose();
void flyDroneBetweenLastAndSecLastGate();
void landDroneOnGround();
void computeGloTrajForExpert();
void rvizGloTraj();

////////////////////////////////////////////////////////
private:// ROS CALLBACKS & Service & Timer Functions //
//////////////////////////////////////////////////////



/////////////////////////////////////////
private:// NON-CONST MEMBER VARIABLES //
///////////////////////////////////////

std::vector<geometry_msgs::Pose, std::allocator<geometry_msgs::Pose>> m_Gates_WaypointPose;
geometry_msgs::Pose m_Drone_InitPose;

std::vector< Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > m_GloTrajWaypoints;
double m_Expert_MaxHorizon;

std::vector< quadrotor_common::TrajectoryPoint > m_GloTraj;
double m_Expert_MaxSpeed;
double m_Expert_MinSpeed;

kindr::minimal::QuatTransformation m_Trafo_WRF_DRF;
kindr::minimal::QuatTransformation m_Trafo_WRF_ARF;
kindr::minimal::QuatTransformation m_Trafo_ARF_DRF;

kindr::minimal::QuatTransformation m_Trafo_ARF_RRF;

double m_Dist2CurrWaypoint;
double m_Dist2LastWaypoint;


std::mutex m_CloneImgMtx;


std::string m_TrainingDataDirPath;
std::string m_RunDirPath;
std::string m_RunCountString;

uint8_t m_AutopilotState;
uint8_t m_SIMRaceTrackType;
FlightMissions m_ORCFlightMission;


double m_EstimatedNormalizedSpeed;

Eigen::Vector3d m_ReferenceStatePos_WRF;
size_t m_CurrWaypointIdx;
size_t m_LastWaypointIdx;
size_t m_ExpertStateIdx;
size_t m_HorizonStateIdx;
size_t m_SpeedStateIdx;
int m_RunCount;
int m_CamFrameCount;
int m_LapCount;

Eigen::Vector3d m_ExpertPrediction;
Eigen::Vector3d m_NavigatorInput;



bool m_NavigatorEnabled;
bool m_TrainDataSaverEnabled;
unsigned int m_LocTraj_SubseqInfeasibleN;
ros::Time m_LocTraj_StartTime;
RQTG::RapidTrajectoryGenerator m_LocTraj;
size_t m_MainLoopIterCount;
unsigned int m_LocTraj_FeasibleCompCount;

quadrotor_common::TrajectoryPoint m_ReferenceState_ARF;


bool m_GloTrajCompFailed;

ros::Time m_LastWaypointArrivalTime;


//////////////////////
private:// METHODS //
////////////////////

void switchDynamicGates( const bool& SwitchOn );
void switchNavigator( const bool& SwitchOn );

void resetRVIZ();



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