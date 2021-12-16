#include "forgetful_drones/forgetful_expert.h"
#include "forgetful_drones/forgetful_helpers.h"



namespace forgetful_drone 
{

ForgetfulExpert::ForgetfulExpert(
    const ros::NodeHandle& RNH, 
    const ros::NodeHandle& PNH
    )
    :
    m_ROSRNH( RNH ),
    m_ROSPNH( PNH ),

    m_ROSSub_Odometry( m_ROSRNH.subscribe("odometry_sensor1/odometry", 1, &ForgetfulExpert::ROSCB_Odometry, this) ),
    m_ROSSub_AutopilotReferenceState( m_ROSRNH.subscribe("autopilot/reference_state", 1, &ForgetfulExpert::ROSCB_AutopilotReferenceState, this) ),
    m_ROSSub_GlobalTrajectorySamplingTime( m_ROSRNH.subscribe("expert/global_trajectory_sampling_time", 1, &ForgetfulExpert::ROSCB_GlobalTrajectorySamplingTime, this) ),
    
    m_ROSPub_ExpertPrediction( m_ROSRNH.advertise<geometry_msgs::Point>("expert/prediction", 1) ),
    m_ROSPub_RVIZGlobalTrajectory( m_ROSRNH.advertise<visualization_msgs::Marker>("rviz/global_trajectory", 1) ),
    m_ROSPub_ExpertWaypointPassed( m_ROSRNH.advertise<std_msgs::Empty>("expert/waypoint_passed", 1) ),
    m_ROSPub_RVIZPositions( m_ROSRNH.advertise<visualization_msgs::Marker>("rviz/navigation_points", 0) ),

    m_ROSSrvSv_ComputeGlobalTrajectory( m_ROSRNH.advertiseService("expert/compute_global_trajectory", &ForgetfulExpert::ROSSrvFunc_computeGlobalTraj, this) ),
    m_ROSTimer_ExpLoop( m_ROSRNH.createTimer(ros::Duration(m_ROSParam_NominalExpertLoopTime), &ForgetfulExpert::ROSTimerFunc_ExpertLoop, this, false, false) ),

    //
    m_GloTraj(),
    m_GloTrajWaypoints(),
    m_GlobalTraj_SamplingTime( 0 ),
    m_ExpertStateIdx( 0 ),
    m_Expert_MaxSpeed( 0 ),
    m_Expert_MinSpeed( 0 ),
    m_Trafo_WRF_DRF(),
    m_Trafo_WRF_ARF(),
    m_Trafo_ARF_DRF(),
    m_Trafo_ARF_RRF(),
    m_ReferenceStatePos_WRF(),
    m_CurrWaypointIdx( 0 ),
    m_LastWaypointIdx( 0 ),
    m_Dist2CurrWaypoint( 0 ),
    m_Dist2LastWaypoint( 0 ),
    m_ExpertPrediction(),
    m_NavigatorInput(),


    // const ROS parameters need to be initialized
    m_GTPolyOrder(),
    m_GTContiOrder(),
    m_WUInitWaypointIdxForFIG8(),
    m_GTMinWeightVel(),
    m_GTMinWeightAcc(),
    m_GTMinWeightJerk(),
    m_GTMinWeightSnap(),
    m_GTMaxSpeed(),
    m_GTMaxNormThrust(),
    m_GTMaxRollPitchRate(),
    m_WUThresDist2DetectWaypointArrival(),
    m_EXPMinHorizon(),
    m_DRONECamHalfYawAOV(),
    m_DRONECamHalfPitchAOV(),
    m_EXPSpeedHorizon(),
    m_ROSParam_NominalExpertLoopTime(),
    m_NAVMaxDivFromRefState(),
    m_EXPMaxDivFromGT4Projection()
{
    // Transformation: autopilot reference frame - > world reference frame.
    geometry_msgs::Pose ARFPose_WRF; 
        ARFPose_WRF.position.x     = 0.0;
        ARFPose_WRF.position.y     = 0.0;
        ARFPose_WRF.position.z     = 0.0;
        ARFPose_WRF.orientation.w  = 1.0;
        ARFPose_WRF.orientation.x  = 0.0;
        ARFPose_WRF.orientation.y  = 0.0;
        ARFPose_WRF.orientation.z  = 0.0;
    tf::poseMsgToKindr (
        ARFPose_WRF, 
        &m_Trafo_WRF_ARF
        );

    bool InitializationSuccessful = true;

        std::vector< std::pair<const char*, const bool*> > KeysAndBoolOutputs
        {
            //
        };
        std::vector< std::pair<const char*, const int*> > KeysAndIntOutputs
        {
            {"GlobalTraj_PolyOrder" , &m_GTPolyOrder},
            {"GlobalTraj_ContiOrder", &m_GTContiOrder},
            {"WaypointIdx_StartVal", &m_WUInitWaypointIdxForFIG8}
        };
        std::vector< std::pair<const char*, const double*> > KeysAndDoubleOutputs
        {
            {"GlobalTraj_MinimizationWeight_Vel", &m_GTMinWeightVel},
            {"GlobalTraj_MinimizationWeight_Acc", &m_GTMinWeightAcc},
            {"GlobalTraj_MinimizationWeight_Jerk", &m_GTMinWeightJerk},
            {"GlobalTraj_MinimizationWeight_Snap", &m_GTMinWeightSnap},
            {"GlobalTraj_MaxNormThrust", &m_GTMaxNormThrust},
            {"GlobalTraj_MaxRollPitchRate", &m_GTMaxRollPitchRate},
            {"GlobalTraj_MaxSpeed", &m_GTMaxSpeed},
            {"WaypointIdx_DistanceThreshold2Increment", &m_WUThresDist2DetectWaypointArrival},
            {"MinHorizon", &m_EXPMinHorizon},
            {"Cam_HalfYawAOV", &m_DRONECamHalfYawAOV},
            {"Cam_HalfPitchAOV", &m_DRONECamHalfPitchAOV},
            {"SpeedHorizon", &m_EXPSpeedHorizon},
            {"MaxDivergence", &m_NAVMaxDivFromRefState},
            {"NominalLoopTime", &m_ROSParam_NominalExpertLoopTime},
            {"MaxDivergenceAfterProjection", &m_EXPMaxDivFromGT4Projection}
        };
        std::vector< std::pair<const char*, const std::string*> > KeysAndStringOutputs
        {
            //
        };
        
        if ( 
            ! fetchROSParameters(
                m_ROSPNH,
                KeysAndBoolOutputs,
                KeysAndIntOutputs,
                KeysAndDoubleOutputs,
                KeysAndStringOutputs
                ) 
            ) InitializationSuccessful = false;
    
    
    if ( ! InitializationSuccessful )
    {
        ROS_FATAL( "[%s]\n  >> Initialization failed. Shutting down ROS node...", 
            ros::this_node::getName().c_str() );
        ros::shutdown();
    }
    else 
        ROS_INFO( "[%s]\n  >> Initialization successful.", ros::this_node::getName().c_str() );
}


ForgetfulExpert::~ForgetfulExpert() = default;





void ForgetfulExpert::ROSCB_GlobalTrajectorySamplingTime( const std_msgs::Float64ConstPtr& msg )
{
    m_GlobalTraj_SamplingTime = msg->data;
}



void ForgetfulExpert::ROSTimerFunc_ExpertLoop( const ros::TimerEvent& TimerEvent )
{
    if ( TimerEvent.profile.last_duration.toSec() > m_ROSParam_NominalExpertLoopTime )
        ROS_WARN(
            "[%s]\n  >> Last loop iteration took %f s exceeding the nominal duration of %f s.",
            ros::this_node::getName().c_str(), 
            TimerEvent.profile.last_duration.toSec(),
            m_ROSParam_NominalExpertLoopTime
            );
    //___________________

    updateWaypointIdx();



    // Distance from reference state to current and last gate waypoint.
    m_Dist2CurrWaypoint 
        = ( m_ReferenceStatePos_WRF - m_GloTrajWaypoints[m_CurrWaypointIdx] ).norm();
    m_Dist2LastWaypoint 
        = ( m_ReferenceStatePos_WRF - m_GloTrajWaypoints[m_LastWaypointIdx] ).norm();
    // Horizon: Distance 2 current/last gate if current/last gate is closer
    // but at least p_Expert_MinHorizon
    double Horizon 
        = std::min(
            m_Expert_MaxHorizon, std::max( 
                m_EXPMinHorizon, std::min( 
                    m_Dist2CurrWaypoint, m_Dist2LastWaypoint 
                    ) 
                )
            );
        

    
    //---------------


    findExpertStateWithProjection(); 

    double NormalizedSpeed2Goal 
        = getHorizonState( m_EXPSpeedHorizon ).velocity.norm() 
            / m_Expert_MaxSpeed;

    Eigen::Vector3d HorizonStatePos = getHorizonState( Horizon ).position;


    rvizPosition(
        m_GloTraj[ m_ExpertStateIdx ].position,
        VisPosTypes::EXPERT,
        m_ROSPub_RVIZPositions
        );
    rvizPosition(
        m_GloTrajWaypoints[ m_CurrWaypointIdx ],
        VisPosTypes::CURRGATE,
        m_ROSPub_RVIZPositions
        );
    rvizPosition(
        HorizonStatePos,
        VisPosTypes::HORIZON,
        m_ROSPub_RVIZPositions
        );
    rvizPosition(
        m_ReferenceStatePos_WRF,
        VisPosTypes::REFERENCE,
        m_ROSPub_RVIZPositions
        );

    Eigen::Vector2d GoalPos_IRF 
        = PosIRF_From_PosDRF(
            PosDRF_From_PosWRF(
                HorizonStatePos
                )
            );

    


    m_ExpertPrediction = { GoalPos_IRF.x(), GoalPos_IRF.y(), NormalizedSpeed2Goal };


    forgetful_drones_msgs::NavigatorInput msg;
        msg.GoalX_IRF = m_ExpertPrediction.x();
        msg.GoalY_IRF = m_ExpertPrediction.y();
        msg.Speed2Goal_Normalized = m_ExpertPrediction.z();
    m_ROSPub_ExpertPrediction.publish( msg );

    
    


}


void ForgetfulExpert::updateWaypointIdx()
{
    if ( m_Dist2CurrWaypoint > m_WUThresDist2DetectWaypointArrival )
        return;
        
    m_CurrWaypointIdx++;
    m_CurrWaypointIdx %= m_GloTrajWaypoints.size();
    
    m_LastWaypointIdx 
        = (m_CurrWaypointIdx + m_GloTrajWaypoints.size() - 1) 
            % m_GloTrajWaypoints.size();

    m_ROSPub_ExpertWaypointPassed.publish( std_msgs::Empty() );
}




Eigen::Vector3d ForgetfulExpert::PosDRF_From_PosWRF( const Eigen::Vector3d& PosWRF )
{
    // Transformation from reference frame originating in IN_Pos_WRF
    // (with neutral orientation with respect to world reference frame)
    // to world reference frame
    kindr::minimal::QuatTransformation T_WRF_PosRF;
    geometry_msgs::Pose Pose_WRF;
        Pose_WRF.position.x = PosWRF.x();
        Pose_WRF.position.y = PosWRF.y();
        Pose_WRF.position.z = PosWRF.z();
        Pose_WRF.orientation.w = 1.0;
        Pose_WRF.orientation.x = 0.0;
        Pose_WRF.orientation.y = 0.0;
        Pose_WRF.orientation.z = 0.0;
    tf::poseMsgToKindr(
        Pose_WRF, 
        &T_WRF_PosRF
        );

    // Transformation from world to drone reference frame.
    kindr::minimal::QuatTransformation T_DRF_WRF = m_Trafo_WRF_DRF.inverse();

    // Transformation from reference frame originating in IN_Pos_WRF
    // (with neutral orientation with respect to world reference frame)
    // to drone reference frame
    kindr::minimal::QuatTransformation T_DRF_PosRF = T_DRF_WRF * T_WRF_PosRF;

    // Position in drone reference frame.
    return T_DRF_PosRF.getPosition();
}


Eigen::Vector2d ForgetfulExpert::PosIRF_From_PosDRF( const Eigen::Vector3d& PosDRF )
{
    // Yaw and pitch angle from drone facing direction to goal position
    double Yaw2PosDRF = std::atan2( PosDRF.y(), PosDRF.x() );
    double Pitch2PosDRF = std::atan2( PosDRF.z(), PosDRF.norm() );

    // Image coordinates: above angles related to camera angles of view but in [-1, 1].
    double X_IRF = Saturation( -Yaw2PosDRF / m_DRONECamHalfYawAOV, -1.0, 1.0 );
    double Y_IRF = Saturation( Pitch2PosDRF / m_DRONECamHalfPitchAOV, -1.0, 1.0 );

    // Pos_IRF: 
    return { X_IRF, Y_IRF };
}







const quadrotor_common::TrajectoryPoint& 
ForgetfulExpert::getHorizonState( const double& Horizon ) const
{
    // Returns state of global trajectory that follows "expert state" 
    // with specified min distance 

    double Distance_Expert2HorizonState;
    size_t HorizonStateIdx = m_ExpertStateIdx;
    
    do
    {
        HorizonStateIdx++;
        HorizonStateIdx %= m_GloTraj.size();

        
        Distance_Expert2HorizonState 
            = ( m_GloTraj[ HorizonStateIdx ].position 
                - m_GloTraj[ m_ExpertStateIdx ].position ).norm();

    } while ( Distance_Expert2HorizonState < Horizon );

    return m_GloTraj[ HorizonStateIdx ];
}













/// Update `m_ReferenceStatePos_WRF`
void ForgetfulExpert::ROSCB_AutopilotReferenceState( const quadrotor_msgs::TrajectoryPointConstPtr& msg )
{
    // Transformation: reference state reference frame -> drone reference frame.
    tf::poseMsgToKindr(
        msg->pose, 
        &m_Trafo_ARF_RRF
        );
    
    // Position of transformation from reference state reference frame -> world reference frame.
    m_ReferenceStatePos_WRF = (m_Trafo_WRF_ARF * m_Trafo_ARF_RRF).getPosition();
}




/// Update `m_Trafo_ARF_DRF` and `m_Trafo_WRF_DRF`.
void ForgetfulExpert::ROSCB_Odometry( const nav_msgs::OdometryConstPtr& msg )
{
    // Transformation: drone reference frame -> state estimate reference frame.
    tf::poseMsgToKindr(
        GeometryMsgsPose_From_NavMsgsOdometry( *msg ), 
        &m_Trafo_ARF_DRF
        );

    // Transformation: drone reference frame -> world reference frame.
    m_Trafo_WRF_DRF = m_Trafo_WRF_ARF * m_Trafo_ARF_DRF;
}





bool ForgetfulExpert::ROSSrvFunc_computeGlobalTraj(
    forgetful_drones_msgs::ComputeGlobalTrajectory::Request& req,
    forgetful_drones_msgs::ComputeGlobalTrajectory::Response& res
    )
{
    // --- Set up ´m_GloTrajWaypoints´ ---
    m_GloTrajWaypoints.clear();
    m_GloTrajWaypoints.reserve( req.Waypoints_Pos.size() );
    for ( const geometry_msgs::Point& Waypoint : req.Waypoints_Pos )
        m_GloTrajWaypoints.push_back( EigenVector3d_From_GeometryMsgsPoint(Waypoint) );

    // --- setMaxHorizon() ---
    m_Expert_MaxHorizon = 0.0;
    size_t LastWaypointIdx = m_GloTrajWaypoints.size() - 1;
    for ( size_t WaypointIdx = 0; WaypointIdx < m_GloTrajWaypoints.size(); WaypointIdx++ )
    {
        m_Expert_MaxHorizon = std::max(
            m_Expert_MaxHorizon,
            (m_GloTrajWaypoints[ WaypointIdx ] - m_GloTrajWaypoints[ LastWaypointIdx ]).norm()
            );
        LastWaypointIdx = WaypointIdx;
    }

    // --- Compute global trajectory ---
    if ( ! computeGlobalTrajAsCircuit() ) return false;

    // --- Visualize global trajectory in RVIZ ---
    rvizGloTraj();

    // --- Set Waypoint Idx for deterministic 8 RaceTrack 
    m_CurrWaypointIdx = m_WUInitWaypointIdxForFIG8;
    m_LastWaypointIdx 
        = (m_CurrWaypointIdx + m_GloTrajWaypoints.size() - 1) 
            % m_GloTrajWaypoints.size();
    
    // --- Start Expert loop ---
    m_ROSTimer_ExpLoop.start();
    return true;
}






bool ForgetfulExpert::computeGlobalTrajAsCircuit()
{
    if ( m_GlobalTraj_SamplingTime == 0 ) 
    {
        ROS_ERROR(
            "[%s]\n  >> Sampling time of global trajectory not set.",
            ros::this_node::getName().c_str()
            );
        return false;
    }

    ROS_INFO(
        "[%s]\n  >> Computing global trajectory...\n" 
            "\t#waypoints: %d\n"
            "\tpolynomial order: %d\n"
            "\tcontinuity order: %d\n"
            "\tminimization weights on\n"
            "\t\tvelocity: %f\n"
            "\t\tacceleration: %f\n"
            "\t\tjerk: %f\n"
            "\t\tsnap: %f\n"
            "\tmaximum speed: %f\n"
            "\tmaximum normalized thrust: %f\n"
            "\tmaximum roll-pitch rate: %f\n"
            "\tsampling time: %f\n",
        ros::this_node::getName().c_str(),
        static_cast<int>(m_GloTrajWaypoints.size()),
        m_GTPolyOrder,
        m_GTContiOrder,
        m_GTMinWeightVel,
        m_GTMinWeightAcc,
        m_GTMinWeightJerk,
        m_GTMinWeightSnap,
        m_GTMaxSpeed,
        m_GTMaxNormThrust,
        m_GTMaxRollPitchRate,
        m_GlobalTraj_SamplingTime
        );




    // Set up `Settings`
    polynomial_trajectories::PolynomialTrajectorySettings Settings;
    Settings.way_points = {
        std::make_move_iterator( m_GloTrajWaypoints.begin() ),
        std::make_move_iterator( m_GloTrajWaypoints.end() )
        };
    Settings.polynomial_order = m_GTPolyOrder;
    Settings.continuity_order = m_GTContiOrder;
    Settings.minimization_weights = Eigen::Vector4d{
        m_GTMinWeightVel,
        m_GTMinWeightAcc,
        m_GTMinWeightJerk,
        m_GTMinWeightSnap
        };
    

    // Calculate segment times, first element relates to segment from last to first waypoint.
    Eigen::VectorXd InitialSegmentTimes = Eigen::VectorXd::Ones( m_GloTrajWaypoints.size() );
    Eigen::Vector3d SegmentStart = m_GloTrajWaypoints.back();
    for ( size_t WaypointIdx = 0; WaypointIdx < m_GloTrajWaypoints.size(); WaypointIdx++ ) 
    {
        InitialSegmentTimes[ WaypointIdx ] 
            = ( m_GloTrajWaypoints[ WaypointIdx ] - SegmentStart ).norm() 
                / m_GTMaxSpeed;
        
        SegmentStart = m_GloTrajWaypoints[ WaypointIdx ];
    }


    bool GlobalTrajSpeedFeasible = false;
    while ( ! GlobalTrajSpeedFeasible )
    {

        // Compute global trajectory
        quadrotor_common::Trajectory GlobalTraj 
            = trajectory_generation_helper::polynomials::generateMinimumSnapRingTrajectoryWithSegmentRefinement(
                InitialSegmentTimes,
                Settings,
                m_GTMaxSpeed,
                m_GTMaxNormThrust,
                m_GTMaxRollPitchRate,
                1/m_GlobalTraj_SamplingTime
                );

        // Set up `m_GloTraj`
        m_GloTraj.clear();
        m_GloTraj = { 
            std::make_move_iterator( GlobalTraj.points.begin() ),
            std::make_move_iterator( GlobalTraj.points.end() ),
            };


        // Find the maximum and minimum velocity of the global trajectory.
        double Speed;
        m_Expert_MaxSpeed = 0.0;
        m_Expert_MinSpeed = std::numeric_limits< double >::max();
        for ( size_t StateIdx = 0; StateIdx < m_GloTraj.size(); StateIdx++ )
        {
            Speed = ( m_GloTraj[ StateIdx ].velocity ).norm();
            m_Expert_MaxSpeed = std::max( m_Expert_MaxSpeed, Speed );
            m_Expert_MinSpeed = std::min( m_Expert_MinSpeed, Speed );
        }

        if ( m_Expert_MaxSpeed > 1.1 * m_GTMaxSpeed )
            ROS_WARN(
                "[%s]\n  >> Maximum Speed (%f m/s) of computed global trajectory exceeds specified value (%f m/s) by more than 110 %%."
                "\n     Re-computing global trajectory....",
                ros::this_node::getName().c_str(), m_Expert_MaxSpeed, m_GTMaxSpeed
                );
        else
            GlobalTrajSpeedFeasible = true;
    }

    ROS_INFO( 
        "[%s]\n  >> Global trajectory successfully computed.\n"
            "\t#states: %d\n"
            "\tduration: %1.1f s\n"
            "\tmaximum speed: %1.1f m/s\n"
            "\tminimum speed: %1.1f m/s\n",            
        ros::this_node::getName().c_str(),
        static_cast<int>( m_GloTraj.size() ),
        m_GloTraj.size() * m_GlobalTraj_SamplingTime,
        m_Expert_MaxSpeed, m_Expert_MinSpeed
        );
    

    return true;
}




void ForgetfulExpert::rvizGloTraj()
{
    visualization_msgs::Marker Marker = getTrajMarker();
    setRGBOfVisMarker( VisualizationColors::PURPLE, Marker );
    Marker.id = 0;

    for ( const quadrotor_common::TrajectoryPoint& State : m_GloTraj )
    {
        Marker.points.push_back( GeometryMsgsPoint_From_EigenVector3d(State.position) );
        Marker.points.push_back( GeometryMsgsPoint_From_EigenVector3d(State.position + State.velocity/20.0) );  
    }

    m_ROSPub_RVIZGlobalTrajectory.publish( Marker );
}




void ForgetfulExpert::findExpertStateWithProjection()
{

    // Find "expert state" with projection.
    Eigen::Vector3d ExpState_Direction;
    double ExpState_SpeedLvl;
    Eigen::Vector3d Exp2RefState_Direction;
    double ProjLvl; // "Projection level" of Exp2RefState_Direction onto ExpState_Direction



    ExpState_Direction
        = m_GloTraj[ m_ExpertStateIdx     ].position 
        - m_GloTraj[ m_ExpertStateIdx - 1 ].position;

    ExpState_SpeedLvl = ExpState_Direction.norm();

    ExpState_Direction /= ExpState_SpeedLvl;

    Exp2RefState_Direction
        = m_ReferenceStatePos_WRF
        - m_GloTraj [ m_ExpertStateIdx - 1 ].position;

    ProjLvl = Exp2RefState_Direction.dot( ExpState_Direction );

    while ( ProjLvl > ExpState_SpeedLvl )
    {
        m_ExpertStateIdx++;
        m_ExpertStateIdx %= m_GloTraj.size();


        ExpState_Direction
            = m_GloTraj[ m_ExpertStateIdx     ].position 
            - m_GloTraj[ m_ExpertStateIdx - 1 ].position;

        ExpState_SpeedLvl = ExpState_Direction.norm();

        ExpState_Direction /= ExpState_SpeedLvl;

        Exp2RefState_Direction
            = m_ReferenceStatePos_WRF
            - m_GloTraj [ m_ExpertStateIdx - 1 ].position;
        
        ProjLvl = Exp2RefState_Direction.dot( ExpState_Direction );
    }


    // If the distance from the "expert state" found with above projection
    // to the "reference state" of the autopilot is more than 1 m,
    // find "expert state" as state on global trajectory that has min distance to "reference state".
    double DivergenceFromGlobalTraj 
        = (m_ReferenceStatePos_WRF - m_GloTraj[m_ExpertStateIdx].position).norm();
    
    if ( DivergenceFromGlobalTraj > m_EXPMaxDivFromGT4Projection ) 
        findExpertStateWithMinDist();

}

void ForgetfulExpert::findExpertStateWithMinDist()
{
    // Find "expert state" as state on global trajectory that has min distance 
    // to "reference state" (~ position of the drone).
    double MinDistance = std::numeric_limits< double >::max();
    double Distance;

    for ( size_t StateIdx = 0; StateIdx < m_GloTraj.size(); StateIdx++ )
    {
        Distance = ( m_GloTraj[ StateIdx ].position - m_Trafo_WRF_DRF.getPosition() ).norm();
        if ( Distance < MinDistance )
        {
            MinDistance = Distance;
            m_ExpertStateIdx = StateIdx;
        }
    }
}

























//######################################
//NOT FULLY IMPLEMENTED

//#include <polynomial_trajectories/minimum_snap_trajectories.h>
//#include <polynomial_trajectories/polynomial_trajectories_common.h>
//#include <trajectory_generation_helper/heading_trajectory_helper.h>

//#include "forgetful_drones_msgs/CompGT.h"
//#include "forgetful_drones_msgs/FlyDroneThroughRaceTrack.h"
//#include "forgetful_drones_msgs/NavigatorState.h"


//m_ROSPub_AutopilotTrajectory( m_ROSRNH.advertise<quadrotor_msgs::Trajectory>("autopilot/trajectory", 1) ),
//m_ROSPub_AutopilotPoseCommand( m_ROSRNH.advertise<geometry_msgs::PoseStamped >("autopilot/pose_command", 1) ),

//m_ROSPub_NavigatorState( m_ROSRNH.advertise<forgetful_drones_msgs::NavigatorState>("navigator/state", 1) ),

    //m_ROSSrvSv_FlyDroneThroughRaceTrack(
    //    m_ROSRNH.advertiseService(
    //        "expert/fly_drone_through_race_track", &ForgetfulExpert::ROSSrvFunc_flyDroneThroughRaceTrackWithTrajectoryControl, this
    //        )
    //    ),


/*
bool ForgetfulExpert::ROSSrvFunc_flyDroneThroughRaceTrackWithTrajectoryControl(
    forgetful_drones_msgs::FlyDroneThroughRaceTrack::Request &req, 
    forgetful_drones_msgs::FlyDroneThroughRaceTrack::Response &res
    )
{
    m_GlobalTraj_SamplingTime = req.SamplingTime;
    std::vector<geometry_msgs::Pose, std::allocator<geometry_msgs::Pose>> WaypointPoses;

    if ( req.IsRingTrajectory == true )
    {
        geometry_msgs::PoseStamped msg;
        msg.pose = req.WaypointPoses[ 0 ];
        m_ROSPub_AutopilotPoseCommand.publish( msg );
        ros::Duration( 5.0 ).sleep();

        for ( size_t Round_i = 0; Round_i < req.Rounds_N; Round_i++ )
            WaypointPoses.insert( WaypointPoses.end(), req.WaypointPoses.begin(), req.WaypointPoses.end() );

        WaypointPoses.push_back( req.WaypointPoses[ 0 ] );
    }
    else
    {
        WaypointPoses.resize( req.WaypointPoses.size() + 1 );

        WaypointPoses[ 0 ] = req.InitDronePose;
        WaypointPoses[ 0 ].position.z = 1.2;
        for ( size_t Waypoint_i = 0; Waypoint_i < req.WaypointPoses.size(); Waypoint_i++ )
            WaypointPoses[ Waypoint_i + 1 ] = req.WaypointPoses[ Waypoint_i ];
    }




    ROS_INFO(
        "[%s] Computing global trajectory through <%d> waypoints subject to\n"
            "\t\tpolynomial order <%d>,\n"
            "\t\tcontinuity order <%d>,\n"
            "\t\tminimization weights on\n"
            "\t\t\tvelocity <%f>,\n"
            "\t\t\tacceleration <%f>,\n"
            "\t\t\tjerk <%f> and\n"
            "\t\t\tsnap <%f>,\n"
            "\t\tmaximum speed <%f>,\n"
            "\t\tmaximum normalized thrust <%f>,\n"
            "\t\tmaximum roll-pitch rate <%f> and\n"
            "\t\tsampling time <%f>.",
        ros::this_node::getName().c_str(),
        static_cast<int>( WaypointPoses.size() ),
        m_GTPolyOrder,
        m_GTContiOrder,
        m_ROSParam_GT_MinimizationWeight_Vel,
        m_GTMinWeightAcc,
        m_GTMinWeightJerk,
        m_GTMinWeightSnap,
        m_GTMaxSpeed,
        m_ROSParam_GT_MaxNormThrust,
        m_GTMaxRollPitchRate,
        m_GlobalTraj_SamplingTime
        );

        polynomial_trajectories::PolynomialTrajectorySettings Settings;
            for ( size_t Waypoint_i = 1; Waypoint_i < WaypointPoses.size() - 1; Waypoint_i++ )
                Settings.way_points.push_back( 
                    EigenVector3d_From_GeometryMsgsPoint( WaypointPoses[ Waypoint_i ].position ) );
            Settings.polynomial_order = m_GTPolyOrder;
            Settings.continuity_order = m_GTContiOrder;
            Settings.minimization_weights = Eigen::Vector4d{
                m_ROSParam_GT_MinimizationWeight_Vel,
                m_GTMinWeightAcc,
                m_GTMinWeightJerk,
                m_GTMinWeightSnap
                };
        

        // Calculate segment times, first element relates to segment from last to first waypoint.
        Eigen::VectorXd InitialSegmentTimes = Eigen::VectorXd::Ones( WaypointPoses.size() - 1 );
        Eigen::Vector3d SegmentStart = EigenVector3d_From_GeometryMsgsPoint( WaypointPoses[ 0 ].position );
        for ( size_t Waypoint_i = 1; Waypoint_i < WaypointPoses.size(); Waypoint_i++ ) 
        {
            InitialSegmentTimes[ Waypoint_i - 1 ] 
                = ( EigenVector3d_From_GeometryMsgsPoint( WaypointPoses[ Waypoint_i ].position ) 
                - SegmentStart ).norm() / m_GTMaxSpeed;
            
            SegmentStart = EigenVector3d_From_GeometryMsgsPoint( WaypointPoses[ Waypoint_i ].position );
        }

        for (int i= 0; i<WaypointPoses.size(); i++)
            std::cout 
                << WaypointPoses[i].position.x << ", "
                << WaypointPoses[i].position.y << ", "
                << WaypointPoses[i].position.z << "\n";


        quadrotor_common::TrajectoryPoint StartState;
        StartState.position = EigenVector3d_From_GeometryMsgsPoint( WaypointPoses[ 0 ].position );
        StartState.orientation = EigenQuaterniond_From_GeometryMsgsQuaternion( WaypointPoses[ 0 ].orientation );
        quadrotor_common::TrajectoryPoint EndState;
        EndState.position = EigenVector3d_From_GeometryMsgsPoint( WaypointPoses.back().position );
        EndState.orientation = EigenQuaterniond_From_GeometryMsgsQuaternion( WaypointPoses.back().orientation );
        quadrotor_common::Trajectory GT_Sampled =
        trajectory_generation_helper::polynomials::generateMinimumSnapTrajectoryWithSegmentRefinement(
            InitialSegmentTimes,
            StartState,
            EndState,
            Settings,
            m_GTMaxSpeed,
            m_ROSParam_GT_MaxNormThrust,
            m_GTMaxRollPitchRate,
            1/m_GlobalTraj_SamplingTime
            );

        //m_GloTraj.clear();
        //m_GloTraj = { 
        //    std::make_move_iterator( GT_Sampled.points.begin() ),
        //    std::make_move_iterator( GT_Sampled.points.end() ),
        //    };

        //GT_Sampled.getStateAtTime

        // Find the maximum and minimum velocity along the global trajectory.
        //double Speed;
        //m_Expert_MaxSpeed = 0.0;
        //m_Expert_MinSpeed = std::numeric_limits< double >::max();
        //for ( size_t State_i = 0; State_i < m_GloTraj.size(); State_i++ )
        //{
        //    Speed = ( m_GloTraj[ State_i ].velocity ).norm();
        //    m_Expert_MaxSpeed = std::max( m_Expert_MaxSpeed, Speed );
        //    m_Expert_MinSpeed = std::min( m_Expert_MinSpeed, Speed );
        //}
//
        //
        //ROS_INFO( "[%s] Successfully computed global trajectory of #<%d> states with\n"
        //    "\t\tduration <%1.1f> s,\n"
        //    "\t\tmaximum speed <%1.1f> m/s and\n"
        //    "\t\tminimum speed <%1.1f> m/s.",            
        //    ros::this_node::getName().c_str(),
        //    static_cast<int>( m_GloTraj.size() ),
        //    m_GloTraj.size() * m_GlobalTraj_SamplingTime,
        //    m_Expert_MaxSpeed, m_Expert_MinSpeed
        //    );


    ros::Duration(5.0 ).sleep();


    std::list<quadrotor_common::TrajectoryPoint>::iterator it;
    for (it = GT_Sampled.points.begin(); it != GT_Sampled.points.end(); it++) 
    {
        it->heading = std::atan2( it->velocity.y(), it->velocity.x() );
    }


    m_ROSPub_AutopilotTrajectory.publish( GT_Sampled.toRosMessage() );


    return true;
}
*/























//######################################
//TO ORCHESTRATOR



//#include <ros/package.h>





//m_TrainingDataDirPath( ros::package::getPath("forgetful_drones") + "/training_data" ),

//m_ROSSub_TrainingDataCollection( m_ROSRNH.subscribe("expert/generate_labels", 1, &ForgetfulExpert::ROSCB_GenerateLabels, this) ),
    //m_ROSSub_DataCollectionRunIdx( m_ROSRNH.subscribe("data_collection_run_idx", 1, &ForgetfulExpert::ROSCB_DataCollectionRunIdx, this) ),





/*
void ForgetfulExpert::RVIZualizeCamFrameWithPrediction()
{
    cv::Mat CamFrame_WithPrediciton;
    const double HalfYawAOV_Deg = m_DRONECamHalfYawAOV*180.0/M_PI;
    const double HalfPitchAOV_Deg = m_DRONECamHalfPitchAOV*180.0/M_PI;
    
    
    // Get image dimensions
    int TopBottom = (int) ( m_CamFrame.rows/2.0 * (HalfPitchAOV_Deg/35.0 -1.0) );
    int LeftRight = (int) ( m_CamFrame.cols/2.0 * (HalfYawAOV_Deg/50.0 -1.0) );
    

    // Shrink image
    cv::copyMakeBorder(
        m_CamFrame, 
        CamFrame_WithPrediciton, 
        TopBottom, 
        TopBottom, 
        LeftRight, 
        LeftRight,
        cv::BORDER_CONSTANT, 
        cv::Scalar(50.0, 50.0, 50.0)
        );


    int yPx_N = CamFrame_WithPrediciton.rows;
    int xPx_N = CamFrame_WithPrediciton.cols;


    // Add vertical grid lines
    for (int i = 0; i <= HalfYawAOV_Deg; i += 30) 
    {
        int yaw_px 
            = (int) (i/HalfYawAOV_Deg + 1) * xPx_N/2.0;
        cv::line(
            CamFrame_WithPrediciton, 
            cv::Point(yaw_px, 0), 
            cv::Point(yaw_px, yPx_N), 
            cv::Scalar(100, 100, 100, 0)
            );
        int yaw_px 
            = (int) (-i/HalfYawAOV_Deg + 1) * xPx_N/2.0;
        cv::line(
            CamFrame_WithPrediciton, 
            cv::Point(yaw_px, 0), 
            cv::Point(yaw_px, yPx_N), 
            cv::Scalar(100, 100, 100, 0)
            );
    }

    // Add horizontal grid lines
    for (int i = 0; i <= HalfPitchAOV_Deg; i += 30) 
    {
        int pitch_px 
            = (int) (i/HalfPitchAOV_Deg +1) * yPx_N/2.0;
        cv::line(
            CamFrame_WithPrediciton, 
            cv::Point(0, pitch_px), 
            cv::Point(xPx_N, pitch_px), 
            cv::Scalar(100, 100, 100, 0)
            );
        int pitch_px 
            = (int) (-i/HalfPitchAOV_Deg +1) * yPx_N/2.0;
        cv::line(
            CamFrame_WithPrediciton, 
            cv::Point(0, pitch_px), 
            cv::Point(xPx_N, pitch_px), 
            cv::Scalar(100, 100, 100, 0)
            );
    }

    
    int yPx_Prediction = static_cast<int>( yPx_N * (1.0 - m_ExpertPrediction.y()) /2.0 );
    int xPx_Prediction = static_cast<int>( xPx_N * (1.0 + m_ExpertPrediction.x()) /2.0 );

    // draw boxes behind velocity info to make it better readable
    cv::Mat roi = CamFrame_WithPrediciton( cv::Rect(3, yPx_N - 25, 85, 20) );
    cv::Mat color( roi.size(), CV_8UC3, cv::Scalar(125, 125, 125) );
    double alpha = 0.8;
    cv::addWeighted( color, alpha, roi, 1.0 - alpha, 0.0, roi);

    cv::Mat roi2 = CamFrame_WithPrediciton( cv::Rect( xPx_N - 88, yPx_N - 25, 85, 20) );
    cv::addWeighted( color, alpha, roi2, 1.0 - alpha, 0.0, roi2 );

    char v_nw[200];
    sprintf( v_nw, "%f ", m_ExpertPrediction.z());
    cv::putText( CamFrame_WithPrediciton, v_nw, cv::Point2f(5, yPx_N - 10), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0, 255, 0, 255) );

    char v_mb[200];
    sprintf(v_mb, "%f ", m_Expert_Output.z());
    cv::putText(CamFrame_WithPrediciton, v_mb, cv::Point2f(xPx_N - 80, yPx_N - 10), cv::FONT_HERSHEY_PLAIN, 1,
                cv::Scalar(255, 0, 0, 255));

    cv::circle(
        CamFrame_WithPrediciton, 
        cv::Point( xPx_Prediction, yPx_Prediction ), 
        0, 
        cv::Scalar(255, 0, 0), 
        5, 
        8, 
        0
        );

    cv::circle(
        CamFrame_WithPrediciton, 
        cv::Point(right_coor_nw, up_coor_nw), 
        0, 
        cv::Scalar(0, 255, 0), 
        5, 
        8, 
        0
        );

    cv_bridge::CvImage msg;
        msg.header.stamp = ros::Time::now(); // Same timestamp and tf frame as input image
        msg.encoding = "rgb8";
        msg.image = CamFrame_WithPrediciton;

    m_ROSPub_CamFrameWithPrediction.publish( msg.toImageMsg() );
}
*/

/*
void ForgetfulExpert::ROSCB_DataCollectionRunIdx( const std_msgs::Int16ConstPtr& msg )
{
    m_CamFrameIdx = 0;


        std::ostringstream RunIdxStream;
        RunIdxStream << std::setw( 4 ) << std::setfill( '0' ) << msg->data;
    m_RunIdxString = RunIdxStream.str();


    std::string DirPath 
        = m_TrainingDataDirPath + getCurrUTCDateTimeAsString() + "/run_" + m_RunIdxString + "/images";
    if ( ! std::filesystem::create_directory( DirPath ) )
        ROS_ERROR(
            "[%s] Failed to create directory %s.", 
            ros::this_node::getName().c_str(),
            DirPath.c_str()
            );
    else
        ROS_INFO(
            "[%s] Directory %s succesfully created.", 
            ros::this_node::getName().c_str(),
            DirPath.c_str()
            );
}
*/


// --- Save Image ---


    /*
  
        if ( ! m_DataRecordingActive ) return;


    std::ostringstream CamFrameIdxStream;
    CamFrameIdxStream << std::setw( 5 ) << std::setfill( '0' ) << m_CamFrameIdx;
    std::string CamFrameIdxString = CamFrameIdxStream.str();


    ROS_DEBUG(
        "[%s] Saving current camera frame to image...",
        ros::this_node::getName().c_str()
        );

    std::string ImageFilename 
        = m_TrainingDataDirPath + "/Run_" + m_RunIdxString 
        + "/images/frame_center_" + CamFrameIdxString + ".jpg";
    
    m_CloneImgMtx.lock();
        //convert from bgr to rgb
        cv::Mat SavedImage = m_CamFrame.clone();
        cv::cvtColor(
            m_CamFrame, 
            SavedImage, 
            CV_BGR2RGB
            );

        if ( !cv::imwrite( ImageFilename, SavedImage ) ) 
            ROS_FATAL(
                "Failed to save current camera frame to %s.", 
                ImageFilename.c_str()
                );
        else 
            ROS_DEBUG(
            "Current camera frame successfully saved to %s.", 
            ImageFilename.c_str()
            );

    m_CloneImgMtx.unlock();*/


}