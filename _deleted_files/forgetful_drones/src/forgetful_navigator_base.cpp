#include "forgetful_drones/forgetful_navigator.h"
#include "forgetful_drones/forgetful_helpers.h"



namespace forgetful_drone
{

ForgetfulNavigator::ForgetfulNavigator(
    const ros::NodeHandle& ROS_RNH, 
    const ros::NodeHandle& ROS_PNH
    )
    :
    m_ROSRNH( ROS_RNH ),
    m_ROSPNH( ROS_PNH ),

    m_ROSSub_NavigatorState( m_ROSRNH.subscribe("navigator/state_switch", 1, &ForgetfulNavigator::ROSCB_NavigatorState, this) ),
    m_ROSSub_NavigatorInput( m_ROSRNH.subscribe("navigator/input", 1, &ForgetfulNavigator::ROSCB_NavInput, this) ), 
    // previously "/cnn_out/traj"
    m_ROSSub_Odometry( m_ROSRNH.subscribe("odometry_sensor1/odometry", 1, &ForgetfulNavigator::ROSCB_Odometry, this) ),

    
    //m_ROSSub_RGBCameraImageRaw( m_ROSRNH.subscribe("rgb_camera/camera_1/image_raw", 1, &ForgetfulNavigator::ROSCB_RGBCameraImageRaw, this) ),
    //m_ROSPub_RVIZCamFrameWithNavigatorInput( m_ROSRNH.advertise<sensor_msgs::Image>("rviz/camera_frame_with_navigator_input", 1) ),

    m_ROSPub_AutopilotReferenceState( m_ROSRNH.advertise<quadrotor_msgs::TrajectoryPoint>("autopilot/reference_state", 1) ),
    m_ROSPub_RVIZLocalTrajectory( m_ROSRNH.advertise<visualization_msgs::Marker>("rviz/local_trajectory", 1) ),
    m_ROSPub_DivergedFromReferenceState( m_ROSRNH.advertise<std_msgs::Empty>("navigator/diverged_from_reference_state", 1) ),
    m_ROSPub_GlobalTrajectorySamplingTime( m_ROSRNH.advertise<std_msgs::Float64>("expert/global_trajectory_sampling_time", 1, true) ),

    //m_ROSSrvSv_GlobalTrajectorySamplingTime( m_ROSRNH.advertiseService("expert/compute_global_trajectory", &ForgetfulExpert::ROS_SrvFunc_ComputeGlobalTrajectory, this) ),

    m_NavigatorEnabled( false ),
    //m_NavigatorState( NavigatorStates::OFF ),
    //m_RefState_DRF(),
    m_LocTraj_SubseqInfeasibleN( 0 ),
    m_NavigatorInput(),
    m_LocTraj_StartTime(),
    m_LocTraj( Vec3(), Vec3(), Vec3(), Vec3() ),
    m_MainLoopIterCount( 0 ),
    m_LocTraj_FeasibleCompCount( 0 ),



    m_NAVInputPerturbAxAmp()
{
    bool InitializationSuccessful = true;

        std::vector< std::pair<const char*, const bool*> > KeysAndBoolOutputs
        {
            //
        };
        std::vector< std::pair<const char*, const int*> > KeysAndIntOutputs
        {
            {"Nav_LoopIters2LTCompsRatio" , &m_NAVMainLoopItersPerLTReplanning},
            {"Nav_SubseqFailedLTComps_MaxN", &m_NAVSubseqFailedLTReplanningMaxCount}
        };
        std::vector< std::pair<const char*, const double*> > KeysAndDoubleOutputs
        {
            {"Nav_NominalLoopTime", &m_ORCMainLoopNomPeriodDuration},
            {"Cam_HalfYawAOV", &m_DRONECamHalfYawAOV},
            {"Cam_HalfPitchAOV", &m_DRONECamHalfPitchAOV},
            {"LT_MinSpeed", &m_LTMinSpeed},
            {"LT_MaxSpeed", &m_LTMaxSpeed},
            {"LT_MaxSpeedIncrement", &m_LTMaxSpeedIncrement},
            {"LT_Duration", &m_LTDuration},
            {"LT_MinDist", &m_LTMinDistance},
            {"LT_MaxDist", &m_LTMaxDistance},
            {"LT_MaxZ", &m_LTMaxAltitude},
            {"LT_MinZ", &m_LTMinAltitude},
            {"LT_MinNormThrust", &m_LTMinNormThrust},
            {"LT_MaxNormThrust", &m_LTMaxNormThrust},
            {"LT_MaxBodyRates", &m_LTMaxBodyRates},
            {"LT_InputFeasibilityCheck_MinSamplingTime", &m_LTInputFeasibilityCheckMinSamplingTime},
            {"NavigatorInputPerturbationAmplitude", &m_NAVInputPerturbAxAmp}
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

    //if ( ! fetchROSParams() ) InitializationSuccessful = false;
    if ( ! InitializationSuccessful )
    {
        ROS_FATAL( "[%s] Initialization failed. Shutting down this ROS node", 
            ros::this_node::getName().c_str() );
        ros::shutdown();
    }


    std_msgs::Float64 msg;
    msg.data = m_ORCMainLoopNomPeriodDuration * m_NAVMainLoopItersPerLTReplanning;
    //while ( m_ROSPub_GlobalTrajectorySamplingTime.getNumSubscribers() == 0 )
    //    ros::Duration(0.1).sleep();
    m_ROSPub_GlobalTrajectorySamplingTime.publish( msg );



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




    m_ROSTimer_NavigatorLoop = m_ROSRNH.createTimer(
        ros::Duration( m_ORCMainLoopNomPeriodDuration ),
        &ForgetfulNavigator::ROSTimerFunc_NavLoop,
        this,
        false, 
        true
        );
}



ForgetfulNavigator::~ForgetfulNavigator() = default;









void ForgetfulNavigator::ROSCB_NavigatorState( const std_msgs::BoolConstPtr& msg )
{
    /*
    switch ( (*msg).data )
    {
        case forgetful_drones_msgs::NavigatorState::OFF :
            updateNavigatorState( NavigatorStates::OFF );
            break;

        case forgetful_drones_msgs::NavigatorState::HOVER :
            updateNavigatorState( NavigatorStates::HOVER );
            break;

        case forgetful_drones_msgs::NavigatorState::RACING :
            updateNavigatorState( NavigatorStates::RACING );
            break;
        
        default:
            break;
    }*/

    switchOnOff_NavigatorState( (*msg).data );
}



void ForgetfulNavigator::ROSCB_NavInput( const geometry_msgs::PointConstPtr& msg ) 
{
    m_NavigatorInput 
        = Eigen::Vector3d{ msg->x, msg->y, msg->z }
        + Eigen::Vector3d::Ones() 
            * m_NAVInputPerturbAxAmp 
            * std::sin( 0.5*ros::WallTime::now().toSec() );
}



void ForgetfulNavigator::ROSCB_Odometry( const nav_msgs::OdometryConstPtr& msg )
{
    // Transformation: drone reference frame -> state estimate reference frame.
    tf::poseMsgToKindr(
        GeometryMsgsPose_From_NavMsgsOdometry( *msg ), 
        &m_Trafo_ARF_DRF
        );

    // Transformation: drone reference frame -> world reference frame.
    m_Trafo_WRF_DRF = m_Trafo_WRF_ARF * m_Trafo_ARF_DRF;
}





























void ForgetfulNavigator::ROSTimerFunc_NavLoop( const ros::TimerEvent& TimerEvent )
{
    if ( TimerEvent.profile.last_duration.toSec() > m_ORCMainLoopNomPeriodDuration )
        ROS_WARN(
            "[%s] Last loop iteration took <%f> s (nominal duration <%f> s).",
            ros::this_node::getName().c_str(), 
            TimerEvent.profile.last_duration.toSec(),
            m_ORCMainLoopNomPeriodDuration
            );


    if ( ! m_NavigatorEnabled )
    {
        m_ReferenceState_ARF = {};
        m_ReferenceState_ARF.position = m_Trafo_ARF_DRF.getPosition();
        m_ReferenceState_ARF.orientation = m_Trafo_ARF_DRF.getEigenQuaternion();
        //ROS_DEBUG(
        //    "[%s] Reference Position: (%f, %f, %f)",
        //    ros::this_node::getName().c_str(),
        //    m_ReferenceState_ARF.position.x(), m_ReferenceState_ARF.position.y(), m_ReferenceState_ARF.position.z()
        //    );

        return;
    }

    /*
    switch ( m_NavigatorState )
    {
    case NavigatorStates::OFF:
        m_ReferenceState_ARF = {};
        m_ReferenceState_ARF.position = m_Trafo_ARF_DRF.getPosition();
        m_ReferenceState_ARF.orientation = m_Trafo_ARF_DRF.getEigenQuaternion();
        //ROS_INFO(
        //    "[%s] Reference Position: (%f, %f, %f)",
        //    ros::this_node::getName().c_str(),
        //    m_ReferenceState_ARF.position.x(), m_ReferenceState_ARF.position.y(), m_ReferenceState_ARF.position.z()
        //    );
        return;

    case NavigatorStates::HOVER:
        return;
    
    default:
        break;
    }*/


    // Lambda functions
    auto DroneDivergedFromRefState = [ this ](){
        return m_ROSParam_MaxDivergence < ( m_ReferenceState_ARF.position - m_Trafo_ARF_DRF.getPosition() ).norm();
        };


    if ( DroneDivergedFromRefState() )
    {
        ROS_WARN(
            "[%s] Goal position (%f, %f, %f) and position estimate (%f, %f, %f) diverged.", 
            ros::this_node::getName().c_str(),
            m_ReferenceState_ARF.position.x(), m_ReferenceState_ARF.position.y(), m_ReferenceState_ARF.position.z(),
            m_Trafo_ARF_DRF.getPosition().x(), m_Trafo_ARF_DRF.getPosition().y(), m_Trafo_ARF_DRF.getPosition().z()
            );
        
        //updateNavigatorState( NavigatorStates::HOVER );
        switchOnOff_NavigatorState( false );
        m_ROSPub_DivergedFromReferenceState.publish( std_msgs::Empty() );
    }



    // If it is time, replan local trajectory
    if ( m_MainLoopIterCount % m_NAVMainLoopItersPerLTReplanning == 0 )
    {
        // Compute goal position and speed to reach goal position from CNN output
        Eigen::Vector3d GoalPos_ARF;   
        double Speed2Goal;
        processNavigatorInput( GoalPos_ARF, Speed2Goal );
        //ROS_WARN("+++NAVIGATOR+++ Local Trajectory, End State [DRF]: (%F, %f, %f)",
        //    GoalPos_ARF.x(), GoalPos_ARF.y(), GoalPos_ARF.z());


        


        // Compute local trajectory
        if ( computeLocTraj(GoalPos_ARF, Speed2Goal) )
        {
            m_LocTraj_SubseqInfeasibleN = 0;
            m_LocTraj_StartTime = ros::Time::now();
            rvizLocTraj();
        }
        else
        {
            if ( m_LocTraj_SubseqInfeasibleN < m_NAVSubseqFailedLTReplanningMaxCount )
                m_LocTraj_SubseqInfeasibleN ++;
            else
            {
                ROS_ERROR(
                    "[%s] Reached maximum number of " 
                    "subsequently infeasible local trajectories <%d>. "
                    "If not crashed, going to hover.", 
                    ros::this_node::getName().c_str(),  m_NAVSubseqFailedLTReplanningMaxCount
                    );
                //updateNavigatorState( NavigatorStates::HOVER );
                switchOnOff_NavigatorState( false );
            }
        }
    }



    publishRefStateFromLocTrajToAutopilot();


    m_MainLoopIterCount++;
}




void ForgetfulNavigator::processNavigatorInput( Eigen::Vector3d& OUT_GoalPos_ARF, double& OUT_Speed2Goal )
{
    auto XYZ_DRF_From_XYDist_IRF = [ this ]
        ( const double& X_IRF, const double& Y_IRF, const double& Dist_IRF ) {
        // Compute yaw and pitch angle to position in reference frame of drone.
        const double Yaw = -m_DRONECamHalfYawAOV * X_IRF;
        const double Pitch = m_DRONECamHalfPitchAOV * Y_IRF;

        // Compute vector in drone reference frame.
        return Eigen::Vector3d{
            Dist_IRF * cos( Pitch ) * cos( Yaw ),
            Dist_IRF * cos( Pitch ) * sin( Yaw ),
            Dist_IRF * sin( Pitch )
            };
        };

    auto XYZ_ARF_From_XYZ_DRF = [ this ] ( const Eigen::Vector3d XYZ_DRF) {
        geometry_msgs::Pose Pose_DRF;
            Pose_DRF.position = GeometryMsgsPoint_From_EigenVector3d( XYZ_DRF );
            Pose_DRF.orientation.w = 1.0;
            Pose_DRF.orientation.x = 0.0;
            Pose_DRF.orientation.y = 0.0;
            Pose_DRF.orientation.z = 0.0;
        kindr::minimal::QuatTransformation T_DRF_XYZ;
        tf::poseMsgToKindr(
            Pose_DRF, 
            &T_DRF_XYZ
            );
        kindr::minimal::QuatTransformation T_ARF_XYZ = m_Trafo_ARF_DRF * T_DRF_XYZ;
        
        return Eigen::Vector3d{ T_ARF_XYZ.getPosition() };
        };
    
    const double GoalX_IRF = m_NavigatorInput.x();
    const double GoalY_IRF = m_NavigatorInput.y();
    const double NormSpeed2Goal = m_NavigatorInput.z();


    //ROS_INFO(
    //        "[%s] Navigator Input: (%f, %f, %f)",
    //        ros::this_node::getName().c_str(),
    //        m_NavigatorInput.x(), m_NavigatorInput.y(), m_NavigatorInput.z()
    //        );

    

    // Denormalize speed prediction of CNN with p_LT_MaxSpeed. 
    // p_LT_MinSpeed <= OUT_Speed2Goal <= p_LT_MaxSpeed.
    OUT_Speed2Goal = std::max( 
        m_LTMinSpeed, 
        m_LTMaxSpeed * NormSpeed2Goal 
        );

    //ROS_INFO(
    //        "[%s] Speed to Goal: %f",
    //        ros::this_node::getName().c_str(),
    //        OUT_Speed2Goal
    //        );

    // Compute distance from drone to goal position:
    // p_LT_MinDist <= p_LT_Duration * OUT_Speed2Goal <= p_LT_MaxDist
    double LT_Dist = std::max( 
        m_LTMinDistance,
        std::min( 
            m_LTMaxDistance, 
            m_LTDuration * OUT_Speed2Goal 
            ) 
        );


    OUT_GoalPos_ARF 
        = XYZ_ARF_From_XYZ_DRF( 
            XYZ_DRF_From_XYDist_IRF( 
                GoalX_IRF, GoalY_IRF, LT_Dist ));

    //ROS_INFO(
    //        "[%s] Goal Position: (%f, %f, %f) [DRF]",
    //        ros::this_node::getName().c_str(),
    //        OUT_GoalPos_ARF.x(), OUT_GoalPos_ARF.y(), OUT_GoalPos_ARF.z()
    //        );
}


/*
Eigen::Vector3d ForgetfulNavigator::XYZ_DRF_From_XYDist_IRF( 
    const double& X_IRF, 
    const double& Y_IRF, 
    const double& Dist_IRF 
    )
{
    // Compute yaw and pitch angle to position in reference frame of drone.
    const double Yaw = -m_DRONECamHalfYawAOV * X_IRF;
    const double Pitch = m_DRONECamHalfPitchAOV * Y_IRF;

    // Compute vector in drone reference frame.
    return {
        Dist_IRF * cos( Pitch ) * cos( Yaw ),
        Dist_IRF * cos( Pitch ) * sin( Yaw ),
        Dist_IRF * sin( Pitch )
        };
}
*/


bool ForgetfulNavigator::computeLocTraj(
    const Eigen::Vector3d& GoalPos_ARF,
    const double& Speed2Goal
    )
{
    Vec3 StartPos_ARF = Vec3_From_EigenVector3d( m_ReferenceState_ARF.position );
    Vec3 StartVel_ARF = Vec3_From_EigenVector3d( m_ReferenceState_ARF.velocity );
    Vec3 StartAcc_ARF = Vec3_From_EigenVector3d( m_ReferenceState_ARF.acceleration );
    Vec3 EndPos_ARF = Vec3_From_EigenVector3d( GoalPos_ARF );

    Vec3 Gravity{ 0.0, 0.0, -9.81 };


    auto LocalTrajIsFeasible = [ this ]( RQTG::RapidTrajectoryGenerator& LocalTraj ){
        
        RQTG::RapidTrajectoryGenerator::InputFeasibilityResult InputFeasibility 
            = LocalTraj.CheckInputFeasibility(
                m_LTMinNormThrust,
                m_LTMaxNormThrust,
                m_LTMaxBodyRates,
                m_LTInputFeasibilityCheckMinSamplingTime
                );

        Vec3 BoundaryPoint = { 0.0, 0.0, m_LTMinAltitude };  // a point on the floor
        Vec3 BoundaryVector = { 0.0, 0.0, 1.0 };  // we want to be above the point
        RQTG::RapidTrajectoryGenerator::StateFeasibilityResult PositionFeasibility_Floor 
            = LocalTraj.CheckPositionFeasibility(
                BoundaryPoint, 
                BoundaryVector
                );

        BoundaryPoint[2] = m_LTMaxAltitude;  // a point on the ceiling
        BoundaryVector[2] = -1.0;  // we want to be below the point
        RQTG::RapidTrajectoryGenerator::StateFeasibilityResult PositionFeasibility_Ceiling 
            = LocalTraj.CheckPositionFeasibility(
                BoundaryPoint, 
                BoundaryVector
                );

        if ( InputFeasibility != RQTG::RapidTrajectoryGenerator::InputFeasible )
        {
            ROS_WARN(
                "[%s] Local Trajectory not Input Feasible w.r.t."
                "\tRange of Normalized Thrust: [%f, %f]\n"
                "\tMax Body Rates: %f\n"
                "Checked with minimum sampling time of %f s.",
                ros::this_node::getName().c_str(),
                m_LTMinNormThrust, m_LTMaxNormThrust,
                m_LTMaxBodyRates,
                m_LTInputFeasibilityCheckMinSamplingTime
                );
            return false;
        }
        if ( PositionFeasibility_Floor != RQTG::RapidTrajectoryGenerator::StateFeasible )
        {
            ROS_WARN(
                "[%s] Local Trajectory Position Infeasible.\n"
                "\tMin Altitude of %f m Violated.",
                ros::this_node::getName().c_str(),
                m_LTMinAltitude
                );
            return false;
        }
        if ( PositionFeasibility_Ceiling != RQTG::RapidTrajectoryGenerator::StateFeasible )
        {
            ROS_WARN(
                "[%s] Local Trajectory Position Infeasible.\n"
                "\tMax Altitude of %f m Violated.",
                ros::this_node::getName().c_str(),
                m_LTMaxAltitude
                );
            return false;
        }

        //ROS_DEBUG(
        //    "[%s] Local Trajectory Input And Position feasible.",
        //    ros::this_node::getName().c_str()
        //    );
        return true;
    };
    

    const double LT_Dist = (EndPos_ARF - StartPos_ARF).GetNorm2();

    const double LT_Duration = LT_Dist / std::min({
        Speed2Goal,
        m_ReferenceState_ARF.velocity.norm() + m_LTMaxSpeedIncrement,
        m_LTMaxSpeed
        });

    RQTG::RapidTrajectoryGenerator LocalTraj{
        StartPos_ARF, 
        StartVel_ARF,
        StartAcc_ARF,
        Gravity
        };

    LocalTraj.SetGoalPosition( EndPos_ARF );
    LocalTraj.Generate( LT_Duration );

    
    //ROS_INFO(
    //    "[%s] Local Trajectory\n"
    //    "\tStart Position: (%f, %f, %f) [DRF]\n"
    //    "\tEnd Position: (%f, %f, %f) [DRF]\n"
    //    "\tStart Velocity: (%f, %f, %f)\n"
    //    "\tStart Acceleration: (%f, %f, %f)\n"
    //    "\tGravity: (%f, %f, %f)\n"
    //    "\tDuration: %f\n"
    //    "\tDistance: %f\n",
    //    ros::this_node::getName().c_str(),
    //    StartPos_ARF.x, StartPos_ARF.y, StartPos_ARF.z,
    //    GoalPos_ARF.x(), GoalPos_ARF.y(), GoalPos_ARF.z(),
    //    m_RefState_DRF.velocity.x(), m_RefState_DRF.velocity.y(), m_RefState_DRF.velocity.z(),
    //    m_RefState_DRF.acceleration.x(), m_RefState_DRF.acceleration.y(), m_RefState_DRF.acceleration.z(),
    //    Gravity.x, Gravity.y, Gravity.z,
    //    LT_Duration,
    //    LT_Dist
    //    );
    
    if ( LocalTrajIsFeasible( LocalTraj ) )
    {
        m_LocTraj = LocalTraj;
        return true;
    }
    else
    {
        ROS_WARN(
            "[%s] Computed local trajectory failed input and/or position feasibility check and was therefore discarded."
            "\t\tGoal position: (%f, %f, %f) (DRF)\n"
            "\t\tDuration: <%f> s",
            ros::this_node::getName().c_str(),
            GoalPos_ARF.x(), GoalPos_ARF.y(), GoalPos_ARF.z(),
            LT_Duration
            );

        return false;
    }
}








void ForgetfulNavigator::rvizLocTraj()
{
    visualization_msgs::Marker Marker = getTrajMarker();

    setRGBOfVisMarker( VisualizationColors::GREEN, Marker );

    Marker.id = m_RVIZElapsedLTsEnabled? ++m_LocTraj_FeasibleCompCount : 1;

    double VisDuration = m_RVIZLTDuration == 0.0?
        m_LocTraj.GetEndTime() :
        std::min(m_LocTraj.GetEndTime(), m_RVIZLTDuration);

    
    const Eigen::Quaterniond Q = m_Trafo_WRF_ARF.getEigenQuaternion();
    const Eigen::Vector3d T = m_Trafo_WRF_ARF.getPosition();

    std::vector<quadrotor_common::TrajectoryPoint> Vis_LT;
    for ( double t = 0.0; t <= VisDuration; t += 1/m_RVIZLTSamplingFreq )
        {

            Eigen::Vector3d Pos = Q * (EigenVector3d_From_Vec3( m_LocTraj.GetPosition(t) )) +T;
            Eigen::Vector3d Vel = Q * EigenVector3d_From_Vec3( m_LocTraj.GetVelocity(t) );

            Marker.points.push_back( GeometryMsgsPoint_From_EigenVector3d(Pos) );
            Marker.points.push_back( GeometryMsgsPoint_From_EigenVector3d(Pos + Vel/20.0) );  
        }

    m_ROSPub_RVIZLocalTrajectory.publish( Marker );
}








void ForgetfulNavigator::publishRefStateFromLocTrajToAutopilot()
{
    // Get time of current state of current local trajectory
    const double LocTraj_CurrTime
        = (ros::Time::now() - m_LocTraj_StartTime).toSec() + m_ORCMainLoopNomPeriodDuration;

    /*
    // Get current reference state for autopilot in reference frame of the drone
    m_RefState_DRF.position = EigenVector3d_From_Vec3( m_LocTraj.GetPosition(LocTraj_CurrTime) );
    m_RefState_DRF.velocity = EigenVector3d_From_Vec3( m_LocTraj.GetVelocity(LocTraj_CurrTime) );
    m_RefState_DRF.acceleration = EigenVector3d_From_Vec3( m_LocTraj.GetAcceleration(LocTraj_CurrTime) );
    m_RefState_DRF.heading = std::atan2( m_RefState_DRF.velocity.y(), m_RefState_DRF.velocity.x() );

    
    // transform current reference state from drone reference frame to autopilot reference frame
    const Eigen::Quaterniond Q = m_Trafo_ARF_DRF.getEigenQuaternion();
    const Eigen::Vector3d T = m_Trafo_ARF_DRF.getPosition();

    m_ReferenceState_ARF.position        = Q * m_RefState_DRF.position + T;
    m_ReferenceState_ARF.velocity        = Q * m_RefState_DRF.velocity;
    m_ReferenceState_ARF.acceleration    = Q * m_RefState_DRF.acceleration;
    m_ReferenceState_ARF.heading = quadrotor_common::wrapMinusPiToPi(
        m_RefState_DRF.heading + quadrotor_common::quaternionToEulerAnglesZYX(Q).z() 
        );
    */

    m_ReferenceState_ARF.position        = EigenVector3d_From_Vec3( m_LocTraj.GetPosition(LocTraj_CurrTime) );
    m_ReferenceState_ARF.velocity        = EigenVector3d_From_Vec3( m_LocTraj.GetVelocity(LocTraj_CurrTime) );
    m_ReferenceState_ARF.acceleration    = EigenVector3d_From_Vec3( m_LocTraj.GetAcceleration(LocTraj_CurrTime) );
    m_ReferenceState_ARF.heading = std::atan2( m_ReferenceState_ARF.velocity.y(), m_ReferenceState_ARF.velocity.x() );

    // Publish reference state to autopilot
    //ROS_INFO(
    //        "[%s] Reference State\n\tPosition: (%f, %f, %f)\n\tVelocity: (%f, %f, %f)\n\tAcceleration: (%f, %f, %f)\n\tHeading: %f",
    //        ros::this_node::getName().c_str(),
    //        m_ReferenceState_ARF.position.x(), m_ReferenceState_ARF.position.y(), m_ReferenceState_ARF.position.z(),
    //        m_ReferenceState_ARF.velocity.x(), m_ReferenceState_ARF.velocity.y(), m_ReferenceState_ARF.velocity.z(),
    //        m_ReferenceState_ARF.acceleration.x(), m_ReferenceState_ARF.acceleration.y(), m_ReferenceState_ARF.acceleration.z(),
    //        m_ReferenceState_ARF.heading
    //        );
    m_ROSPub_AutopilotReferenceState.publish( m_ReferenceState_ARF.toRosMessage() );
}






/*
bool ForgetfulNavigator::LocalTrajIsFeasible( 
    RQTG::RapidTrajectoryGenerator& LocalTraj )
{
    RQTG::RapidTrajectoryGenerator::InputFeasibilityResult InputFeasibility 
        = LocalTraj.CheckInputFeasibility(
            m_LTMinNormThrust,
            m_LTMaxNormThrust,
            m_LTMaxBodyRates,
            m_LTInputFeasibilityCheckMinSamplingTime
            );

    Vec3 BoundaryPoint = { 0.0, 0.0, m_LTMinAltitude };  // a point on the floor
    Vec3 BoundaryVector = { 0.0, 0.0, 1.0 };  // we want to be above the point
    RQTG::RapidTrajectoryGenerator::StateFeasibilityResult PositionFeasibility_Floor 
        = LocalTraj.CheckPositionFeasibility(
            BoundaryPoint, 
            BoundaryVector
            );

    BoundaryPoint[2] = m_LTMaxAltitude;  // a point on the ceiling
    BoundaryVector[2] = -1.0;  // we want to be below the point
    RQTG::RapidTrajectoryGenerator::StateFeasibilityResult PositionFeasibility_Ceiling 
        = LocalTraj.CheckPositionFeasibility(
            BoundaryPoint, 
            BoundaryVector
            );

    if ( 
        InputFeasibility == RQTG::RapidTrajectoryGenerator::InputFeasible 
        &&
        PositionFeasibility_Ceiling == RQTG::RapidTrajectoryGenerator::StateFeasible 
        &&
        PositionFeasibility_Floor == RQTG::RapidTrajectoryGenerator::StateFeasible
        )
    {
        return true;
    }
    else
    {
        return false;
    }
}

*/










void ForgetfulNavigator::switchOnOff_NavigatorState( const bool& SwitchOn )
{
    std::string LastState = m_NavigatorEnabled? "ON" : "OFF";

    auto enterON = [LastState, this](){
        m_NavigatorEnabled = true;
        
        m_ReferenceState_ARF = {};
        m_ReferenceState_ARF.position = m_Trafo_ARF_DRF.getPosition();
        m_ReferenceState_ARF.orientation = m_Trafo_ARF_DRF.getEigenQuaternion();
        m_ROSPub_AutopilotReferenceState.publish( m_ReferenceState_ARF.toRosMessage() );

        ROS_INFO ( "[%s]\n  >> Navigator state switched from %s to ON.", 
            ros::this_node::getName().c_str(), LastState.c_str() );
        };

    auto enterOFF = [LastState, this](){
        m_NavigatorEnabled = false;
        
        m_ReferenceState_ARF = {};
        m_ReferenceState_ARF.position = m_Trafo_ARF_DRF.getPosition();
        m_ReferenceState_ARF.orientation = m_Trafo_ARF_DRF.getEigenQuaternion();
        m_ROSPub_AutopilotReferenceState.publish( m_ReferenceState_ARF.toRosMessage() );

        ROS_INFO ( "[%s]\n  >> Navigator state switched from %s to OFF.", 
            ros::this_node::getName().c_str(), LastState.c_str() );
        };

    SwitchOn? enterON() : enterOFF();
}







/*
void ForgetfulNavigator::updateNavigatorState( const NavigatorStates& NextState )
{
    std::string LastStateName;
    switch ( m_NavigatorState ) {
        case NavigatorStates::OFF:      LastStateName = "OFF"; break;
        case NavigatorStates::HOVER:    LastStateName = "HOVER"; break;
        case NavigatorStates::RACING:   LastStateName = "RACING"; break; 
        }


    auto updateToCurrState = [ LastStateName ](){
        ROS_WARN ( "[%s]\n\tAttempt to update navigator state to current navigator state.\n\t\t%s -> %s",
            ros::this_node::getName().c_str(), LastStateName.c_str(), LastStateName.c_str() );
        };
    
    auto exitOFF = [](){};
    auto exitHOVER = [](){};
    auto exitRACING = [](){};

    auto enterOFF = [ LastStateName, this ](){
        m_NavigatorState = NavigatorStates::OFF;
        ROS_INFO ( "[%s]\n\tNavigator state updated.\n\t\t%s -> OFF", 
            ros::this_node::getName().c_str(), LastStateName.c_str() );
        };
    auto enterHOVER = [ LastStateName, this ](){
        m_ReferenceState_ARF = {};
        m_ReferenceState_ARF.position = m_Trafo_ARF_DRF.getPosition();
        m_ReferenceState_ARF.orientation = m_Trafo_ARF_DRF.getEigenQuaternion();
        
        m_ROSPub_AutopilotReferenceState.publish( m_ReferenceState_ARF.toRosMessage() );
        m_NavigatorState = NavigatorStates::HOVER;

        ROS_INFO ( "[%s]\n\tNavigator state updated.\n\t\t%s -> HOVER", 
            ros::this_node::getName().c_str(), LastStateName.c_str() );
        };
    auto enterRACING = [ LastStateName, this ](){
            m_ReferenceState_ARF = {};
            m_ReferenceState_ARF.position = m_Trafo_ARF_DRF.getPosition();
            m_ReferenceState_ARF.orientation = m_Trafo_ARF_DRF.getEigenQuaternion();
            
            m_ROSPub_AutopilotReferenceState.publish( m_ReferenceState_ARF.toRosMessage() );
            
            //std_msgs::Bool msg; msg.data = true;
            //m_ROSPub_CopilotFeedthrough.publish( msg );
            
            m_NavigatorState = NavigatorStates::RACING;

            ROS_INFO ( "[%s]\n\tNavigator state updated.\n\t\t%s -> RACING", 
                ros::this_node::getName().c_str(), LastStateName.c_str() );
        };


    if ( m_NavigatorState == NextState) 
        updateToCurrState();
    else
    {
        switch ( m_NavigatorState )
        {
            case NavigatorStates::OFF: exitOFF();
                switch ( NextState )
                {
                    case NavigatorStates::HOVER: {enterHOVER(); break;}
                    case NavigatorStates::RACING: {enterRACING(); break;}
                }
                break;
            case NavigatorStates::HOVER: exitHOVER();
                switch ( NextState )
                {
                    case NavigatorStates::OFF: {enterOFF(); break;}
                    case NavigatorStates::RACING: {enterRACING(); break;}
                }
                break;
            case NavigatorStates::RACING: exitRACING();
                switch ( NextState )
                {
                    case NavigatorStates::OFF: {enterOFF(); break;}
                    case NavigatorStates::HOVER: {enterHOVER(); break;}
                }
                break;
        }
    }
}
*/






}