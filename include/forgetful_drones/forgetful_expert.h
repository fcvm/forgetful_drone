#pragma once

#include <ros/ros.h>
#include <Eigen/Dense>

#include <std_msgs/Float64.h>
#include <std_msgs/Empty.h>
#include <nav_msgs/Odometry.h>

#include <trajectory_generation_helper/polynomial_trajectory_helper.h>

#include <minkindr_conversions/kindr_msg.h>

#include "forgetful_drones_msgs/NavigatorInput.h"
#include "forgetful_drones_msgs/ComputeGlobalTrajectory.h"





namespace forgetful_drone
{
    class ForgetfulExpert
    {

    //////////////////////////////////
    public:// CON- AND DESTRUCTORS //
    ////////////////////////////////


        ForgetfulExpert() : ForgetfulExpert( 
            ros::NodeHandle(), 
            ros::NodeHandle("~") 
            ) {}

        ForgetfulExpert(
            const ros::NodeHandle &RNH, 
            const ros::NodeHandle &PNH
            );

        virtual ~ForgetfulExpert();



    ////////////////////////////////////
    private:// INITIALIZER FUNCTIONS //
    //////////////////////////////////

        //bool fetchROSParameters();



    /////////////////////////////////////////////////////////
    private:// ROS CALLBACKS & SERVICES & TIMER FUNCTIONS //
    ///////////////////////////////////////////////////////

        ros::NodeHandle m_ROSRNH; // ROS node handle that is resolved to the node's namespace.
        ros::NodeHandle m_ROSPNH; // ROS node handle that is resolved to the namespace: <node's namespace>/<node's name>.
        
        ros::Publisher m_ROSPub_ExpertPrediction;
        ros::Publisher m_ROSPub_RVIZGlobalTrajectory;
        ros::Publisher m_ROSPub_ExpertWaypointPassed;
        ros::Publisher m_ROSPub_RVIZPositions;

        ros::Subscriber m_ROSSub_Odometry;
            void ROSCB_Odometry( const nav_msgs::OdometryConstPtr& msg );
        ros::Subscriber m_ROSSub_AutopilotReferenceState;
            void ROSCB_AutopilotReferenceState( const quadrotor_msgs::TrajectoryPointConstPtr& msg );
        ros::Subscriber m_ROSSub_GlobalTrajectorySamplingTime;
            void ROSCB_GlobalTrajectorySamplingTime( const std_msgs::Float64ConstPtr& msg );

        ros::ServiceServer m_ROSSrvSv_ComputeGlobalTrajectory;
            bool ROSSrvFunc_computeGlobalTraj(
                forgetful_drones_msgs::ComputeGlobalTrajectory::Request& req,
                forgetful_drones_msgs::ComputeGlobalTrajectory::Response& res
                );
                void rvizGloTraj();

        ros::Timer m_ROSTimer_ExpLoop;
            void ROSTimerFunc_ExpertLoop( const ros::TimerEvent& ExpertLoop_TimerEvent );
                void updateWaypointIdx();
                bool computeGlobalTrajAsCircuit();
                void findExpertStateWithProjection();
                    void findExpertStateWithMinDist();
                Eigen::Vector3d PosDRF_From_PosWRF( const Eigen::Vector3d& PosWRF );
                Eigen::Vector2d PosIRF_From_PosDRF( const Eigen::Vector3d& PosDRF );
                const quadrotor_common::TrajectoryPoint& getHorizonState( const double& Horizon ) const;



    /////////////////////////////
    private:// ROS PARAMETERS //
    ///////////////////////////
        
        const int m_GTPolyOrder;
        const int m_GTContiOrder;
        const int m_WUInitWaypointIdxForFIG8;

        const double m_GTMinWeightVel;
        const double m_GTMinWeightAcc;
        const double m_GTMinWeightJerk;
        const double m_GTMinWeightSnap;
        const double m_GTMaxNormThrust;
        const double m_GTMaxRollPitchRate;
        const double m_GTMaxSpeed;
        const double m_WUThresDist2DetectWaypointArrival;
        const double m_EXPMinHorizon;
        const double m_DRONECamHalfYawAOV;
        const double m_DRONECamHalfPitchAOV;
        const double m_EXPSpeedHorizon;
        const double m_ROSParam_NominalExpertLoopTime;
        const double m_NAVMaxDivFromRefState;
        const double m_EXPMaxDivFromGT4Projection;

    

    ///////////////////////////////
    private:// MEMBER VARIABLES //
    /////////////////////////////

    std::vector< quadrotor_common::TrajectoryPoint > m_GloTraj;  
    std::vector< Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > m_GloTrajWaypoints;
    double m_GlobalTraj_SamplingTime;
    double m_Expert_MaxSpeed;
    double m_Expert_MinSpeed;
    kindr::minimal::QuatTransformation m_Trafo_WRF_DRF;
    kindr::minimal::QuatTransformation m_Trafo_WRF_ARF;
    kindr::minimal::QuatTransformation m_Trafo_ARF_DRF;

    kindr::minimal::QuatTransformation m_Trafo_ARF_RRF;
    Eigen::Vector3d m_ReferenceStatePos_WRF;
    
    size_t m_ExpertStateIdx;
    size_t m_CurrWaypointIdx;
    size_t m_LastWaypointIdx;

    double m_Dist2CurrWaypoint;
    double m_Dist2LastWaypoint;

    Eigen::Vector3d m_ExpertPrediction;
    Eigen::Vector3d m_NavigatorInput;

    double m_Expert_MaxHorizon;



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