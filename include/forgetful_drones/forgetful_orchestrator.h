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

//#include <memory>


namespace forgetful_drone
{
    class ForgetfulOrchestrator
    {
    ////////////////////
    private:// ENUMS //
    //////////////////
            
        /// Contains all possible modes of navigation
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


        ForgetfulOrchestrator() : ForgetfulOrchestrator( 
            ros::NodeHandle(), 
            ros::NodeHandle("~") 
            ) {}

        ForgetfulOrchestrator(
            const ros::NodeHandle &ROS_RNH, 
            const ros::NodeHandle &ROS_PNH
            );

        virtual ~ForgetfulOrchestrator();


    ////////////////////////////////////
    private:// INITIALIZER FUNCTIONS //
    //////////////////////////////////

        //bool fetchROSParameters();


    /////////////////////////////////////
    private:// CONST MEMBER VARIABLES //
    ///////////////////////////////////
        

        ros::NodeHandle m_ROSRNH; // ROS node handle that is resolved to the node's namespace.
        ros::NodeHandle m_ROSPNH; // ROS node handle that is resolved to the namespace: <node's namespace>/<node's name>.
        
        ros::Subscriber m_ROSSub_DivergedFromReferenceState;
            void ROSCB_DivergedFromReferenceState( const std_msgs::EmptyConstPtr& msg );



        ros::Subscriber m_ROSSub_AutopilotFeedback;
            void ROSCB_AutopilotFeedback( const quadrotor_msgs::AutopilotFeedback::ConstPtr& msg );


        ros::Subscriber m_ROSSub_ExpertPrediction;
            void ROSCB_ExpertPrediction( const geometry_msgs::PointConstPtr& msg );
                Eigen::Vector3d m_ExpertPrediction;
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
        ros::Publisher m_ROSPub_BridgeArm;
        ros::Publisher m_ROSPub_StartNavigation;
        ros::Publisher m_ROSPub_StopNavigation;
        ros::Publisher m_ROSPub_NavigatorState;


        ros::Publisher m_ROSPub_ExpertGenerateLabels;

        ros::Publisher m_ROSPub_GTSamplingTime;
        ros::Publisher m_ROSPub_GateWaypointXYZs;
        //ros::ServiceClient m_ROS_SrvCl_CompGT;


        ros::ServiceClient m_ROSSrvCl_BuildDroneRacingSimulation;
        ros::ServiceClient m_ROSSrvCl_ComputeGlobalTrajectory;
        ros::ServiceClient m_ROSSrvCl_FlyDroneThroughRaceTrack;
        ros::ServiceClient m_ROSSrvCl_ExpertProvidePredictions;

        ros::Publisher m_ROSPub_DEBUG;


        
        ros::WallDuration m_ROSParam_DataCol_Run_MaxDuration = ros::WallDuration( 40.0 );
        const double m_ROSParam_NavigatorLoop_NominalDuration = 0.02;
        const int m_ROSParam_NavigatorLoop_Iters2LTCompsRatio = 3;



        const bool m_SIMDynGatesEnabled;
        const int m_ORCRunMaxCount;
        const std::string m_ORCFlightMission_temp;
        const std::string m_SIMRaceTrackType_temp;

        const double m_DRONECamHalfYawAOV;
        const double m_DRONECamHalfPitchAOV;
        const double m_LTMaxSpeed;
        const double m_TrainingDataGeneration_MaxRunDuration;




    ////////////////////////////////////
    private:// INITIALIZER FUNCTIONS //
    //////////////////////////////////


    void generateTrainingData();
    void testNavigatorWithNetwork();
    void testNavigatorWithExpert();
    void testSimulatorWithAutopilot();

    void buildDroneRacingSimulation();
    void launchDroneOffGround();
    void computeGloTrajForExpert();

    ////////////////////////////////////////////////////////
    private:// ROS CALLBACKS & Service & Timer Functions //
    //////////////////////////////////////////////////////
    
    

    /////////////////////////////////////////
    private:// NON-CONST MEMBER VARIABLES //
    ///////////////////////////////////////

    std::vector<geometry_msgs::Pose, std::allocator<geometry_msgs::Pose>> m_Gates_WaypointPose;
    geometry_msgs::Pose m_Drone_InitPose;


    


    std::string m_TrainingDataDirPath;

    uint8_t m_AutopilotState;
    uint8_t m_SIMRaceTrackType;
    FlightMissions m_ORCFlightMission;
    

    double m_EstimatedNormalizedSpeed;
    


    //////////////////////
    private:// METHODS //
    ////////////////////

    void switchDynamicGates( const bool& SwitchOn );
    void switchOnOff_NavigatorState( const bool& SwitchOn );



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