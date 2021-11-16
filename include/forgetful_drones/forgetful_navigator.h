#pragma once

#include <ros/ros.h>
#include <quadrotor_common/trajectory_point.h>
#include <Eigen/Dense>
#include <rapid_trajectories/RapidTrajectoryGenerator.h>
#include <rapid_trajectories/Vec3.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/TwistStamped.h>
#include <forgetful_drones_msgs/NavigatorState.h>
#include <kindr/minimal/quat-transformation.h>
#include <minkindr_conversions/kindr_msg.h>
#include <minkindr_conversions/kindr_tf.h>
#include <nav_msgs/Odometry.h>

#include <sensor_msgs/Image.h>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>




namespace forgetful_drone
{
    namespace RQTG = RapidQuadrocopterTrajectoryGenerator;

    class ForgetfulNavigator
    {
    

    //////////////////////////////////
    public:// CON- AND DESTRUCTORS //
    ////////////////////////////////


        ForgetfulNavigator() : ForgetfulNavigator( 
            ros::NodeHandle(), 
            ros::NodeHandle("~") 
            ) {}

        ForgetfulNavigator(
            const ros::NodeHandle &ROS_RNH, 
            const ros::NodeHandle &ROS_PNH
            );

        virtual ~ForgetfulNavigator();



    /////////////////////////////////////
    private:// CONST MEMBER VARIABLES //
    ///////////////////////////////////
        

        ros::NodeHandle m_ROSRNH; // ROS node handle that is resolved to the node's namespace.
        ros::NodeHandle m_ROSPNH; // ROS node handle that is resolved to the namespace: <node's namespace>/<node's name>.
        
        ros::Subscriber m_ROSSub_NavigatorState;
        ros::Subscriber m_ROSSub_NavigatorInput;
        ros::Subscriber m_ROSSub_Odometry;

        /*ros::Subscriber m_ROSSub_RGBCameraImageRaw;
            void ROSCB_RGBCameraImageRaw( const sensor_msgs::ImageConstPtr& msg );
                void rvizCamFrameWithNavigatorInput();
        ros::Publisher m_ROSPub_RVIZCamFrameWithNavigatorInput;*/
        



        //ros::Publisher m_ROSPub_CopilotFeedthrough;
        ros::Publisher m_ROSPub_AutopilotReferenceState;
        ros::Publisher m_ROSPub_RVIZLocalTrajectory;
        ros::Publisher m_ROSPub_DivergedFromReferenceState;
        ros::Publisher m_ROSPub_GlobalTrajectorySamplingTime;

        ros::ServiceServer m_ROSSrvSv_GlobalTrajectorySamplingTime;

        ros::Timer m_ROSTimer_NavigatorLoop;

        const int m_NAVMainLoopItersPerLTReplanning = 5;
        const int m_NAVSubseqFailedLTReplanningMaxCount = 5;
        const double m_ORCMainLoopNomPeriodDuration = 0.02;
        
        const double m_DRONECamHalfYawAOV = 0;
        const double m_DRONECamHalfPitchAOV = 0;
        
        
        const double m_LTMinSpeed = 0.5;
        const double m_LTMaxSpeed = 0.5;
        const double m_LTMaxSpeedIncrement = 0.5;
        const double m_LTDuration = 2.0;
        const double m_LTMinDistance = 2.0;
        const double m_LTMaxDistance = 2.5;
        const double m_LTMaxAltitude = 2.5;
        const double m_LTMinAltitude = 2.5;
        const double m_LTMinNormThrust = 2.5;
        const double m_LTMaxNormThrust = 2.5;
        const double m_LTMaxBodyRates = 2.5;
        const double m_LTInputFeasibilityCheckMinSamplingTime = 2.5;
        const double m_NAVInputPerturbAxAmp;

        const double m_ROSParam_MaxDivergence = 1.0;

        const bool m_RVIZElapsedLTsEnabled = true;
        const double m_RVIZLTDuration = 0.0;
        const double m_RVIZLTSamplingFreq = 10.0;

    ///////////////////////////
    private:// ENUM CLASSES //
    /////////////////////////

    /*
    /// Contains all possible states to which the autopilot can be set:
    /// 1) OFF
    /// 2) HOVER
    /// 3) RACING
    enum class NavigatorStates
    {
        OFF,
        HOVER,
        RACING
    };*/


    /*
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
    */




    ////////////////////////////////////
    private:// INITIALIZER FUNCTIONS //
    //////////////////////////////////

    //bool fetchROSParams();

    ////////////////////////////////////////////////////////
    private:// ROS CALLBACKS & Service & Timer Functions //
    //////////////////////////////////////////////////////


    void ROSCB_NavigatorState( const std_msgs::BoolConstPtr& msg );
    void ROSCB_NavInput( const geometry_msgs::PointConstPtr& msg );
    void ROSCB_Odometry( const nav_msgs::OdometryConstPtr& msg );
    

    void ROSTimerFunc_NavLoop( const ros::TimerEvent& TimerEvent );
        
        void processNavigatorInput( Eigen::Vector3d& OUT_GoalPos_DRF, double& OUT_Speed2Goal );
            //Eigen::Vector3d XYZ_DRF_From_XYDist_IRF( 
            //    const double& X_IRF, 
            //    const double& Y_IRF, 
            //    const double& Dist_IRF 
            //    );
        bool computeLocTraj(
            const Eigen::Vector3d& GoalPos_DRF,
            const double& Speed2Goal
            );
            //bool LocalTrajIsFeasible( RQTG::RapidTrajectoryGenerator& LocalTrajectory );
    

        void rvizLocTraj();

        void publishRefStateFromLocTrajToAutopilot();


    /// \brief Calls exitAutopilotState*() according to current autopilot state
    /// and enterAutopilotState*() according to parameter if not equal.
    /// \param NextAutopilotState State the autopilot should be updated to.
    /// \param m_NavigatorState Adjusted.
    /// \param p_DataRecording_On Adjusted if updated to HOVER.
    /// \param m_ReferenceState_WRF Adjusted if updated to HOVER.
    /// \param m_ReferenceState_ARF Adjusted if updated to HOVER.
    /// \param m_GT_MaxSpeed Adjusted if updated to RACING.
    /// \param m_Expert_MinSpeed Adjusted if updated to RACING.
    /// \param m_GT_ProjectionState_i Adjusted if updated to RACING.
    //void updateNavigatorState( const NavigatorStates& NextAutopilotState );
    void switchOnOff_NavigatorState( const bool& SwitchOn );



    /////////////////////////////////////////
    private:// NON-CONST MEMBER VARIABLES //
    ///////////////////////////////////////

    bool m_NavigatorEnabled;
    //NavigatorStates m_NavigatorState;
    quadrotor_common::TrajectoryPoint m_RefState_DRF;
    quadrotor_common::TrajectoryPoint m_ReferenceState_ARF;
    unsigned int m_LocTraj_SubseqInfeasibleN;
    Eigen::Vector3d m_NavigatorInput;
    ros::Time m_LocTraj_StartTime;
    RQTG::RapidTrajectoryGenerator m_LocTraj;
    size_t m_MainLoopIterCount;

    unsigned int m_LocTraj_FeasibleCompCount;

    kindr::minimal::QuatTransformation m_Trafo_ARF_DRF;
    kindr::minimal::QuatTransformation m_Trafo_WRF_DRF;
    kindr::minimal::QuatTransformation m_Trafo_WRF_ARF;

    cv::Mat m_CamFrame;
    cv::Mat m_CamFrameWithNavigatorInput;



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