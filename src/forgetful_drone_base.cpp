#include "forgetful_drones/forgetful_drone.h"
#include "forgetful_drones/forgetful_helpers.hpp"



//#include <geometry_msgs/TwistStamped.h>


namespace forgetful_drone
{

ForgetfulDrone::ForgetfulDrone( const ros::NodeHandle& RNH, const ros::NodeHandle& PNH )
    :
    m_ROSRNH( RNH ),
    m_ROSPNH( PNH ),
    
    m_ROSSub_Odometry( m_ROSRNH.subscribe( "odometry_sensor1/odometry", 1, &ForgetfulDrone::ROSCB_Odometry, this) ),
    m_ROSSub_AutopilotFeedback( m_ROSRNH.subscribe( "autopilot/feedback", 1, &ForgetfulDrone::ROSCB_AutopilotFeedback, this) ),
    m_ROSSub_NetworkPrediction( m_ROSRNH.subscribe("network/prediction", 1, &ForgetfulDrone::ROSCB_NetworkPrediction, this) ),
    m_ROSSub_RGBCameraImageRaw( m_ROSRNH.subscribe("rgb_camera/camera_1/image_raw", 1, &ForgetfulDrone::ROSCB_RGBCameraImageRaw, this) ),
    
    m_ROSPub_RVIZPositions( m_ROSRNH.advertise<visualization_msgs::Marker>("rviz/navigation_points", 0) ),
    m_ROSPub_RVIZCamFrameWithPredictions( m_ROSRNH.advertise<sensor_msgs::Image>("rviz/camera_frame_with_predictions", 1) ),
    m_ROSPub_RVIZGlobalTrajectory( m_ROSRNH.advertise<visualization_msgs::Marker>("rviz/global_trajectory", 1) ),
    m_ROSPub_DynGates( m_ROSRNH.advertise<std_msgs::Bool>("simulator/dynamic_gates_switch", 1) ), 
    m_ROSPub_AutopilotOff( m_ROSRNH.advertise<std_msgs::Empty>("autopilot/off", 1) ),
    m_ROSPub_AutopilotStart( m_ROSRNH.advertise<std_msgs::Empty>("autopilot/start", 1) ),
    m_ROSPub_AutopilotLand( m_ROSRNH.advertise<std_msgs::Empty>("autopilot/land", 1) ),
    m_ROSPub_BridgeArm( m_ROSRNH.advertise<std_msgs::Bool>("bridge/arm", 1) ),
    m_ROSPub_NavigatorState( m_ROSRNH.advertise<std_msgs::Bool>("navigator/state_switch", 1) ),
    m_ROSPub_AutopilotReferenceState( m_ROSRNH.advertise<quadrotor_msgs::TrajectoryPoint>("autopilot/reference_state", 1) ),
    m_ROSPub_RVIZLocalTrajectory( m_ROSRNH.advertise<visualization_msgs::Marker>("rviz/local_trajectory", 1) ),
    m_ROSPub_AutopilotPoseCommand( m_ROSRNH.advertise<geometry_msgs::PoseStamped>("autopilot/pose_command", 1) ),

    m_ROSSrvCl_BuildDroneRacingSimulation( m_ROSRNH.serviceClient<forgetful_drones_msgs::BuildDroneRacingSimulation>("simulator/build_drone_racing_simulation") ),
    m_ROSSrvCl_RVIZLoadConfig( m_ROSRNH.serviceClient<rviz::SendFilePath>("rviz/load_config") ),



    m_AutopilotState( quadrotor_msgs::AutopilotFeedback::OFF ),
    m_ORCFlightMission(),
    m_SIMRaceTrackType(),
    m_ExpertPrediction(),
    m_NetworkPrediciton(),
    m_GloTraj(),
    m_GloTrajWaypoints(),
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
    m_NavigatorInput(),

    m_NavigatorEnabled( false ),
    m_LocTraj_SubseqInfeasibleN( 0 ),
    m_LocTraj_StartTime(),
    m_LocTraj( Vec3(), Vec3(), Vec3(), Vec3() ),
    m_MainLoopIterCount( 0 ),
    m_LocTraj_FeasibleCompCount( 0 ),
    m_RunCount( -1 ),
    m_CamFrameCount( -1 ),
    m_LapCount( -1 ),
    m_TrainDataSaverEnabled( false ),


    // ROS Parameters
    m_SIMDynGatesEnabled(),
    m_ORCRunMaxCount(),
    m_DRONECamHalfYawAOV(),
    m_DRONECamHalfPitchAOV(),
    m_LTMaxSpeed(),
    m_ORCNoWaypointArrivalMaxDuration(),
    m_GTPolyOrder(),
    m_GTContiOrder(),
    m_WUInitWaypointIdxForFIG8(),
    m_GTMinWeightVel(),
    m_GTMinWeightAcc(),
    m_GTMinWeightJerk(),
    m_GTMinWeightSnap(),
    m_GTMaxNormThrust(),
    m_GTMaxRollPitchRate(),
    m_GTMaxSpeed(),
    m_GTSamplingTime(),
    m_WUThresDist2DetectWaypointArrival(),
    m_EXPMinHorizon(),
    m_EXPSpeedHorizon(),
    m_NAVMaxDivFromRefState(),
    m_EXPMaxDivFromGT4Projection(),
    m_NAVMainLoopItersPerLTReplanning(),
    m_NAVSubseqFailedLTReplanningMaxCount(),
    m_NAVInputPerturbAxAmp(),
    m_ORCMainLoopNomPeriodDuration(),
    m_LTMinSpeed(),
    m_LTMaxSpeedIncrement(),
    m_LTDuration(),
    m_LTMinDistance(),
    m_LTMaxDistance(),
    m_LTMaxAltitude(),
    m_LTMinAltitude(),
    m_LTMinNormThrust(),
    m_LTMaxNormThrust(),
    m_LTMaxBodyRates(),
    m_LTInputFeasibilityCheckMinSamplingTime(),
    m_RVIZElapsedLTsEnabled(),
    m_RVIZLTDuration(),
    m_RVIZLTSamplingFreq(),
    m_ORCLapMaxCount()
{
    // --- Set logger level of this node to DEBUG ---
    if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) 
        ros::console::notifyLoggerLevelsChanged();


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
            {"SIM_dynamic_gates_enabled" , &m_SIMDynGatesEnabled},
            {"RVIZ_elapsed_LTs_enabled", &m_RVIZElapsedLTsEnabled},
        };
        std::vector< std::pair<const char*, const int*> > KeysAndIntOutputs
        {
            {"ORC_number_of_runs" , &m_ORCRunMaxCount},
            {"GT_polynomial_order" , &m_GTPolyOrder},
            {"GT_continuous_order", &m_GTContiOrder},
            {"WU_initial_waypoint_index_for_figure8_race_tracks", &m_WUInitWaypointIdxForFIG8},
            {"NAV_number_of_loop_iterations_per_local_trajectory_replanning" , &m_NAVMainLoopItersPerLTReplanning},
            {"NAV_max_number_of_subsequently_failed_local_trajectory_replannings", &m_NAVSubseqFailedLTReplanningMaxCount},
            {"ORC_number_of_laps_per_run", &m_ORCLapMaxCount}
        };
        std::vector< std::pair<const char*, const double*> > KeysAndDoubleOutputs
        {
            {"DRONE_half_yaw_angle_of_view_of_onboard_camera", &m_DRONECamHalfYawAOV},
            {"DRONE_half_pitch_angle_of_view_of_onboard_camera", &m_DRONECamHalfPitchAOV},
            {"LT_max_speed", &m_LTMaxSpeed},
            {"ORC_duration_without_reaching_waypoint_to_abort_run", &m_ORCNoWaypointArrivalMaxDuration},
            {"GT_velocity_minimization_weight", &m_GTMinWeightVel},
            {"GT_acceleration_minimization_weight", &m_GTMinWeightAcc},
            {"GT_jerk_minimization_weight", &m_GTMinWeightJerk},
            {"GT_snap_minimization_weight", &m_GTMinWeightSnap},
            {"GT_max_normalized_thrust", &m_GTMaxNormThrust},
            {"GT_max_roll_pitch_rate", &m_GTMaxRollPitchRate},
            {"GT_max_speed", &m_GTMaxSpeed},
            {"WU_threshold_distance_to_detect_arrival_at_waypoint", &m_WUThresDist2DetectWaypointArrival},
            {"EXP_min_horizon", &m_EXPMinHorizon},
            {"EXP_speed_horizon", &m_EXPSpeedHorizon},
            {"NAV_max_divergence_from_reference_state", &m_NAVMaxDivFromRefState},
            {"EXP_max_divergence_from_GT_to_use_projection", &m_EXPMaxDivFromGT4Projection},
            
            {"ORC_nominal_period_duration_of_main_loop", &m_ORCMainLoopNomPeriodDuration},
            {"LT_min_speed", &m_LTMinSpeed},
            {"LT_max_speed_increment", &m_LTMaxSpeedIncrement},
            {"LT_duration", &m_LTDuration},
            {"LT_min_distance", &m_LTMinDistance},
            {"LT_max_distance", &m_LTMaxDistance},
            {"LT_max_altitude", &m_LTMaxAltitude},
            {"LT_min_altitude", &m_LTMinAltitude},
            {"LT_min_normalized_thrust", &m_LTMinNormThrust},
            {"LT_max_normalized_thrust", &m_LTMaxNormThrust},
            {"LT_max_body_rates", &m_LTMaxBodyRates},
            {"LT_min_sampling_time_at_input_feasibility_check", &m_LTInputFeasibilityCheckMinSamplingTime},
            {"NAV_axial_amplitude_of_input_perturbation", &m_NAVInputPerturbAxAmp},

            {"RVIZ_duration_of_LTs", &m_RVIZLTDuration},
            {"RVIZ_sampling_frequency_of_LTs", &m_RVIZLTSamplingFreq}
        };

        std::string ORCFlightMission;
        std::string SIMRaceTrackType;
        std::vector< std::pair<const char*, const std::string*> > KeysAndStringOutputs
        {
            {"ORC_flight_mission", &ORCFlightMission},
            {"SIM_race_track_type", &SIMRaceTrackType}
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

        if ( ORCFlightMission == std::string("test_navigator_with_expert") )
            m_ORCFlightMission = FlightMissions::testNavigatorWithExpert;
        else if ( ORCFlightMission == std::string("test_navigator_with_brain") )
            m_ORCFlightMission = FlightMissions::testNavigatorWithNetwork;
        else if ( ORCFlightMission == std::string("generate_training_data") )
            m_ORCFlightMission = FlightMissions::generateTrainingData;
        else if ( ORCFlightMission == std::string("test_simulation_with_autopilot") )
            m_ORCFlightMission = FlightMissions::testSimulatorWithAutopilot;
        else
        {
            ROS_ERROR( "[%s]\n  >> Invalid value of ROS parameter with key \"ORC_flight_mission\".", 
                ros::this_node::getName().c_str() );
            InitializationSuccessful = false;
        }

        if ( SIMRaceTrackType == std::string("deterministic_figure8") )
            m_SIMRaceTrackType = forgetful_drones_msgs::BuildDroneRacingSimulationRequest::FIG8_DET;
        else if ( SIMRaceTrackType == std::string("randomized_figure8") )
            m_SIMRaceTrackType = forgetful_drones_msgs::BuildDroneRacingSimulationRequest::FIG8_RAND;
        else if ( SIMRaceTrackType == std::string("randomized_circuit") )
            m_SIMRaceTrackType = forgetful_drones_msgs::BuildDroneRacingSimulationRequest::CIRCUIT_RAND;
        else if ( SIMRaceTrackType == std::string("randomized_sprint") )
            m_SIMRaceTrackType = forgetful_drones_msgs::BuildDroneRacingSimulationRequest::SPRINT_RAND;
        else
        {
            ROS_ERROR( "[%s]\n  >> Invalid value of ROS parameter with key \"SIM_race_track_type\".", 
                ros::this_node::getName().c_str() );
            InitializationSuccessful = false;
        }

        const_cast<double&>(m_GTSamplingTime) = m_ORCMainLoopNomPeriodDuration * m_NAVMainLoopItersPerLTReplanning;

        if ( m_ORCFlightMission == FlightMissions::testNavigatorWithExpert || m_ORCFlightMission == FlightMissions::generateTrainingData )
            if ( m_SIMRaceTrackType == forgetful_drones_msgs::BuildDroneRacingSimulationRequest::SPRINT_RAND )
            {
                ROS_ERROR(
                    "[%s]\n  >> Flight missions \"Navigator-Expert Test\" and \"Training Data Generation\" are incompatible with race track type \"Randomized Sprint\"", 
                    ros::this_node::getName().c_str() );
                InitializationSuccessful = false;
            }
    
    if ( ! InitializationSuccessful )
    {
        ROS_FATAL( "[%s]\n  >> Initialization failed. Shutting down ROS node...", 
            ros::this_node::getName().c_str() );
        ros::shutdown();
    }
    else 
        ROS_INFO( "[%s]\n  >> Initialization successful.", ros::this_node::getName().c_str() );


    // --- Wait for the other nodes ---
    ros::Duration(2.0).sleep();

    
    
    // --- Perform Specified Mission ---
        switch ( m_ORCFlightMission )
        {
        case FlightMissions::generateTrainingData: generateTrainingData(); break;
        
        case FlightMissions::testNavigatorWithNetwork: testNavigatorWithNetwork(); break;

        case FlightMissions::testNavigatorWithExpert: testNavigatorWithExpert(); break;

        case FlightMissions::testSimulatorWithAutopilot: testSimulatorWithAutopilot(); break;
        
        default: ROS_ERROR( "[%s]\n >> Specified flight mission not implemented.", ros::this_node::getName().c_str() ); break;
        }
}



ForgetfulDrone::~ForgetfulDrone(){}


















//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////// ROS CALLBACKS //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


void 
ForgetfulDrone::ROSCB_AutopilotFeedback
( const quadrotor_msgs::AutopilotFeedback::ConstPtr& msg )
{ 
    m_AutopilotState = msg->autopilot_state;
}


/// Update `m_Trafo_ARF_DRF` and `m_Trafo_WRF_DRF`.
void ForgetfulDrone::ROSCB_Odometry( const nav_msgs::OdometryConstPtr& msg )
{
    // Transformation: drone reference frame -> state estimate reference frame.
    tf::poseMsgToKindr(
        GeometryMsgsPose_From_NavMsgsOdometry( *msg ), 
        &m_Trafo_ARF_DRF
        );

    // Transformation: drone reference frame -> world reference frame.
    m_Trafo_WRF_DRF = m_Trafo_WRF_ARF * m_Trafo_ARF_DRF;


    const geometry_msgs::Vector3& EstimatedVelocity = msg->twist.twist.linear;
    m_EstimatedNormalizedSpeed = sqrt( 
        pow(EstimatedVelocity.x, 2) + pow(EstimatedVelocity.y, 2) + pow(EstimatedVelocity.z, 2) 
        ) / m_LTMaxSpeed;
}





void ForgetfulDrone::ROSCB_NetworkPrediction( const geometry_msgs::PointConstPtr& msg ) 
{
    m_NetworkPrediciton = { msg->x, msg->y, msg->z };
}

void ForgetfulDrone::ROSCB_RGBCameraImageRaw( const sensor_msgs::ImageConstPtr& msg )
{
    // Update `CamFrame`
    m_CloneImgMtx.lock();
        m_CamFrame = cv_bridge::toCvShare( 
            msg, sensor_msgs::image_encodings::TYPE_8UC3 
            )->image.clone();
    m_CloneImgMtx.unlock();
    

    rvizCamFrameWithPredictions();
}


void ForgetfulDrone::rvizCamFrameWithPredictions()
{
    constexpr double RVIZImage_HalfYawAOV = 50.0 * M_PI/180.0;
    constexpr double RVIZImage_HalfPitchAOV = 35.0 * M_PI/180.0;
    constexpr double RVIZImage_GridLineAngleIncrement = 30.0 * M_PI/180.0;
    constexpr double alpha = 0.8;
    
    int xPx, yPx;
    int LeftRight, TopBottom;

    cv::Mat roi;
    cv::Mat color;
    cv_bridge::CvImage msg;
    

    // --- Shrink image to fit configurations in RVIZ ---
    
    LeftRight = (int) ( m_CamFrame.cols/2.0 * (m_DRONECamHalfYawAOV/RVIZImage_HalfYawAOV -1.0) );
    TopBottom = (int) ( m_CamFrame.rows/2.0 * (m_DRONECamHalfPitchAOV/RVIZImage_HalfPitchAOV -1.0) );

    cv::copyMakeBorder(
        m_CamFrame, 
        m_CamFrameWithPredictions, 
        TopBottom, 
        TopBottom, 
        LeftRight, 
        LeftRight,
        cv::BORDER_CONSTANT, 
        cv::Scalar(50.0, 50.0, 50.0)
        );

    const int& xPx_N = m_CamFrameWithPredictions.cols;
    const int& yPx_N = m_CamFrameWithPredictions.rows;


    // -- Add grid lines ---

        // Vertical
    for ( double Yaw = 0.0; Yaw <= RVIZImage_HalfYawAOV; Yaw += RVIZImage_GridLineAngleIncrement ) 
    {
        xPx = (int) ( (Yaw/RVIZImage_HalfYawAOV + 1.0) * xPx_N/2.0 );
        cv::line(
            m_CamFrameWithPredictions, 
            cv::Point(xPx, 0), 
            cv::Point(xPx, yPx_N), 
            cv::Scalar(100, 100, 100, 0)
            );

        xPx = (int) ( (-Yaw/RVIZImage_HalfYawAOV + 1.0) * xPx_N/2.0 );
        cv::line(
            m_CamFrameWithPredictions, 
            cv::Point(xPx, 0), 
            cv::Point(xPx, yPx_N), 
            cv::Scalar(100, 100, 100, 0)
            );
    }

        // Horizontal
    for ( double Pitch = 0; Pitch <= RVIZImage_HalfPitchAOV; Pitch += RVIZImage_GridLineAngleIncrement) 
    {
        yPx = (int) ( (Pitch/RVIZImage_HalfPitchAOV + 1.0) * yPx_N/2.0 );
        cv::line(
            m_CamFrameWithPredictions, 
            cv::Point(0, yPx), 
            cv::Point(xPx_N, yPx),
            cv::Scalar(100, 100, 100, 0)
            );
        yPx = (int) ( (-Pitch/RVIZImage_HalfPitchAOV + 1.0) * yPx_N/2.0 );
        cv::line(
            m_CamFrameWithPredictions, 
            cv::Point(0, yPx), 
            cv::Point(xPx_N, yPx), 
            cv::Scalar(100, 100, 100, 0)
            );
    }

    // --- Add expert prediction ---

    xPx = (int) ( xPx_N * (1.0 + m_ExpertPrediction.x()) /2.0 );
    yPx = (int) ( yPx_N * (1.0 - m_ExpertPrediction.y()) /2.0 );

    cv::circle(
        m_CamFrameWithPredictions, 
        cv::Point( xPx, yPx ), 
        0, 
        cv::Scalar(255, 0, 0), 
        5, 
        8, 
        0
        );
    
    roi = m_CamFrameWithPredictions( cv::Rect( xPx_N - 88, yPx_N - 25, 85, 20) );
    color = cv::Mat( roi.size(), CV_8UC3, cv::Scalar(125, 125, 125) );
    cv::addWeighted( color, alpha, roi, 1.0 - alpha, 0.0, roi );

    //sprintf(speed, "%.6f", m_ExpertPrediction.z());
    cv::putText(
        m_CamFrameWithPredictions, 
        std::to_string( m_ExpertPrediction.z() ).c_str(),
        cv::Point2f(xPx_N - 80, yPx_N - 10), 
        cv::FONT_HERSHEY_PLAIN, 1,
        cv::Scalar(255, 0, 0, 255)
        );
    
    cv::putText(
        m_CamFrameWithPredictions, 
        "Expert", 
        cv::Point2f(xPx_N - 80, yPx_N - 28), 
        cv::FONT_HERSHEY_PLAIN, 0.75,
        cv::Scalar(255, 0, 0, 255)
        );


    // --- Add network prediction ---

    xPx = (int) ( xPx_N * (1.0 + m_NetworkPrediciton.x()) /2.0 );
    yPx = (int) ( yPx_N * (1.0 - m_NetworkPrediciton.y()) /2.0 );

    cv::circle(
        m_CamFrameWithPredictions, 
        cv::Point( xPx, yPx ), 
        0, 
        cv::Scalar(0, 255, 0), 
        5, 
        8, 
        0
        );
    
    // draw boxes behind velocity info to make it better readable
    roi = m_CamFrameWithPredictions( cv::Rect(3, yPx_N - 25, 85, 20) );
    color = cv::Mat( roi.size(), CV_8UC3, cv::Scalar(125, 125, 125) );
    cv::addWeighted( color, alpha, roi, 1.0 - alpha, 0.0, roi);

    //sprintf( speed, "%.6f", m_NetworkPrediciton.z());
    cv::putText( 
        m_CamFrameWithPredictions, 
        std::to_string( m_NetworkPrediciton.z() ).c_str(),
        cv::Point2f(5, yPx_N - 10), 
        cv::FONT_HERSHEY_PLAIN, 
        1, 
        cv::Scalar(0, 255, 0, 255) 
        );

    cv::putText( 
        m_CamFrameWithPredictions, 
        "Network", 
        cv::Point2f(5, yPx_N - 28), 
        cv::FONT_HERSHEY_PLAIN, 
        0.75, 
        cv::Scalar(0, 255, 0, 255) 
        );

    
    // --- Add velocity estimate---
    
    // draw boxes behind velocity info to make it better readable
    roi = m_CamFrameWithPredictions( cv::Rect( (int)(xPx_N/2-42), yPx_N - 25, 85, 20) );
    color = cv::Mat( roi.size(), CV_8UC3, cv::Scalar(125, 125, 125) );
    cv::addWeighted( color, alpha, roi, 1.0 - alpha, 0.0, roi);

    cv::putText( 
        m_CamFrameWithPredictions, 
        std::to_string( m_EstimatedNormalizedSpeed ).c_str(),
        cv::Point2f( (int)(xPx_N/2-39), yPx_N - 10), 
        cv::FONT_HERSHEY_PLAIN, 
        1, 
        cv::Scalar(0, 0, 255, 255) 
        );

    cv::putText( 
        m_CamFrameWithPredictions, 
        "Estimate", 
        cv::Point2f( (int)(xPx_N/2-39), yPx_N - 28), 
        cv::FONT_HERSHEY_PLAIN, 
        0.75, 
        cv::Scalar(0, 0, 255, 255) 
        );


    // --- Publish processed image ---

    msg.header.stamp = ros::Time::now();
    msg.encoding = "rgb8";
    msg.image = m_CamFrameWithPredictions;


    m_ROSPub_RVIZCamFrameWithPredictions.publish( msg.toImageMsg() );
}



void ForgetfulDrone::resetRVIZ()
{
    rviz::SendFilePath srv;
    srv.request.path.data = ros::package::getPath( "forgetful_drones" ) + "/rviz/race_track.rviz";

    if ( ! m_ROSSrvCl_RVIZLoadConfig.call(srv) )
        ROS_WARN( "[%s]\n >>  Failed to reset RVIZ for new run.", 
                    ros::this_node::getName().c_str() );
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////// FLIGHT MISSIONS //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void ForgetfulDrone::launchDroneOffGround()
{
    ROS_INFO( "[%s]\n >>  Launching drone off ground.", 
                    ros::this_node::getName().c_str() );

    //ros::Duration( 5.0 ).sleep();

    m_ROSPub_AutopilotOff.publish( std_msgs::Empty() );

    std_msgs::Bool msg; msg.data = true;
    m_ROSPub_BridgeArm.publish( msg );

    m_ROSPub_AutopilotStart.publish( std_msgs::Empty() );

    ros::Rate Rate( 100.0 );
    while ( m_AutopilotState != quadrotor_msgs::AutopilotFeedback::BREAKING )
        { ros::spinOnce(); Rate.sleep(); }
    while ( m_AutopilotState != quadrotor_msgs::AutopilotFeedback::HOVER )
        { ros::spinOnce(); Rate.sleep(); }
    
    ros::Duration( 3.0 ).sleep();
}


void ForgetfulDrone::flyDroneToInitPose()
{
    ROS_INFO( "[%s]\n >>  Flying drone to initial position.", 
                    ros::this_node::getName().c_str() );

    ros::Rate Rate( 100.0 );
    while ( m_AutopilotState != quadrotor_msgs::AutopilotFeedback::HOVER )
        { ros::spinOnce(); Rate.sleep(); }

    geometry_msgs::PoseStamped msg;
        msg.pose = m_Drone_InitPose;
        msg.pose.position.z += 1.0;

    m_ROSPub_AutopilotPoseCommand.publish( msg );


    while ( m_AutopilotState == quadrotor_msgs::AutopilotFeedback::HOVER )
        { ros::spinOnce(); Rate.sleep(); }
    while ( m_AutopilotState != quadrotor_msgs::AutopilotFeedback::HOVER )
        { ros::spinOnce(); Rate.sleep(); }

    ros::Duration( 1.0 ).sleep();
}

void ForgetfulDrone::flyDroneBetweenLastAndSecLastGate()
{
    ROS_INFO( "[%s]\n >>  Flying drone between last and second last gate.", 
                    ros::this_node::getName().c_str() );

    ros::Rate Rate( 100.0 );
    while ( m_AutopilotState != quadrotor_msgs::AutopilotFeedback::HOVER )
        { ros::spinOnce(); Rate.sleep(); }
    

    geometry_msgs::PoseStamped msg;
        msg.pose.position 
            = GeometryMsgsPoint_From_EigenVector3d(
                m_GloTrajWaypoints[ m_GloTrajWaypoints.size() -2 ]
                + ( m_GloTrajWaypoints[ m_GloTrajWaypoints.size() -1 ]
                - m_GloTrajWaypoints[ m_GloTrajWaypoints.size() -2 ] ) /2
                );
        msg.pose.orientation = m_Drone_InitPose.orientation;

    m_ROSPub_AutopilotPoseCommand.publish( msg );


    while ( m_AutopilotState == quadrotor_msgs::AutopilotFeedback::HOVER )
        { ros::spinOnce(); Rate.sleep(); }
    while ( m_AutopilotState != quadrotor_msgs::AutopilotFeedback::HOVER )
        { ros::spinOnce(); Rate.sleep(); }

    ros::Duration( 1.0 ).sleep();
}


void ForgetfulDrone::landDroneOnGround()
{
    ROS_INFO( "[%s]\n >>  Landing drone on ground", 
                    ros::this_node::getName().c_str() );

    m_ROSPub_AutopilotLand.publish( std_msgs::Empty() );
    
    ros::Rate Rate( 100.0 );
    while ( m_AutopilotState != quadrotor_msgs::AutopilotFeedback::OFF )
        { ros::spinOnce(); Rate.sleep(); }

    std_msgs::Bool msg; msg.data = false;
    m_ROSPub_BridgeArm.publish( msg );

    ros::Duration( 3.0 ).sleep();
}






void ForgetfulDrone::testNavigatorWithExpert()
{
    std::cout << "\n\n";
    ROS_INFO( "[%s]\n >> Starting Flight Mission: Navigator-Expert Test.", ros::this_node::getName().c_str() );
    std::cout << "\n";

    m_ROSTimer_MainLoop = m_ROSRNH.createTimer(
        ros::Duration( m_ORCMainLoopNomPeriodDuration ), 
        &ForgetfulDrone::ROSTimerFunc_testNavigatorWithExpert, 
        this, false, false
        );

    for ( m_RunCount = 0; m_RunCount < m_ORCRunMaxCount; m_RunCount++ )
    {
        
        m_LocTraj_FeasibleCompCount = 0;
        m_LapCount = -1;

        std::cout << std::endl;
        ROS_INFO( "[%s]\n >>  Testing Navigator with Expert: Starting Run #%d.\n", 
                    ros::this_node::getName().c_str(), m_RunCount );



        do
        {
            buildDroneRacingSimulation();
            computeGloTrajForExpert();
        } while ( m_GloTrajCompFailed );

        launchDroneOffGround();
        
        m_ROSTimer_MainLoop.start();
        
        switchNavigator( true );

        std::thread t1(runMultiThreadedSpinner);

        m_LapCount = -1;
        ros::Rate Rate( 100.0 );
        do
        {
            Rate.sleep();
        } while ( m_LapCount < m_ORCLapMaxCount && m_NavigatorEnabled );

        
        if ( m_NavigatorEnabled )
        {
            ROS_INFO( "[%s]\n >>  Testing Navigator with Expert: Run #%d successfully completed.", 
                    ros::this_node::getName().c_str(), m_RunCount );
            switchNavigator( false );
        }
        else
            ROS_WARN( "[%s]\n >>  Testing Navigator with Expert: Run #%d aborted ahead of schedule.", 
                    ros::this_node::getName().c_str(), m_RunCount );


        m_ROSTimer_MainLoop.stop();
        t1.detach();
        t1.~thread();


        landDroneOnGround();
    }
}






void ForgetfulDrone::generateTrainingData()
{
    std::cout << "\n\n";
    ROS_INFO( "[%s]\n >> Starting Flight Mission: Training Data Generation.", ros::this_node::getName().c_str() );
    std::cout << "\n";


    m_ROSTimer_MainLoop = m_ROSRNH.createTimer(
        ros::Duration( m_ORCMainLoopNomPeriodDuration ), 
        &ForgetfulDrone::ROSTimerFunc_generateTrainingData,
        this, false, false
        );

    // Create directory where the training data is saved.
    m_TrainingDataDirPath
        = ros::package::getPath( "forgetful_drones" ) 
        + "/training_data/" + getCurrUTCDateTimeAsString();
    std::filesystem::create_directory( m_TrainingDataDirPath );


   std::string FailedRuns = "";
   int FailedRunsN = 0;



    for ( m_RunCount = 0; m_RunCount < m_ORCRunMaxCount; m_RunCount++ )
    {
        m_LocTraj_SubseqInfeasibleN = 0;
        m_LocTraj_FeasibleCompCount = 0;
        m_LapCount = -1;
        m_CamFrameCount = -1;
        m_MainLoopIterCount = 0;
        m_LocTraj = { Vec3(), Vec3(), Vec3(), Vec3() };
        m_LocTraj_StartTime = {};
        

        std::cout << std::endl;
        ROS_INFO( "[%s]\n >>  Training Data Generation: Starting Run #%d.\n", 
                    ros::this_node::getName().c_str(), m_RunCount );

        std::ostringstream RunCountStream;
        RunCountStream << std::setw( 4 ) << std::setfill( '0' ) << m_RunCount;
        m_RunCountString = RunCountStream.str();
        m_RunDirPath = m_TrainingDataDirPath + "/run_" + m_RunCountString;
        std::filesystem::create_directory( m_RunDirPath );
        std::filesystem::create_directory( m_RunDirPath + "/images" );
        m_CamFrameCount = 0;

        resetRVIZ();

        do
        {
            buildDroneRacingSimulation();
            computeGloTrajForExpert();
        } while ( m_GloTrajCompFailed );
        
        


        if ( m_RunCount == 0 ) launchDroneOffGround();
        else 
        {
            flyDroneToInitPose();
            landDroneOnGround();
            launchDroneOffGround();
        }


        
        m_ROSTimer_MainLoop.start();
        
        switchNavigator( true );

        std::thread t1(runMultiThreadedSpinner);


        m_LastWaypointArrivalTime = ros::Time::now();
        ros::Rate Rate( 10.0 );
        do
        {
            Rate.sleep();

            ros::Duration NoWaypointArrivalDuration = ros::Time::now() - m_LastWaypointArrivalTime;
            if ( NoWaypointArrivalDuration.toSec() > m_ORCNoWaypointArrivalMaxDuration )
            {
                ROS_ERROR(
                    "[%s]\n >> Gate has not been reached within %f s. Abort run #%d ahead of schedule.", 
                    ros::this_node::getName().c_str(), m_ORCNoWaypointArrivalMaxDuration, m_RunCount );
                break;
            }
        } while ( m_LapCount < m_ORCLapMaxCount && m_NavigatorEnabled );
       
        m_TrainDataSaverEnabled = false;
        ROS_INFO(
                "[%s]\n  >> Training data saver DISABLED.", 
                ros::this_node::getName().c_str()
                );
        
        if ( m_NavigatorEnabled )
        {
            ROS_INFO( "[%s]\n >>  Training Data Generation: Run #%d completed as scheduled.", 
                    ros::this_node::getName().c_str(), m_RunCount );
            switchNavigator( false );
        }
        else
        {
            ROS_WARN( "[%s]\n >>  Training Data Generation: Run #%d aborted ahead of schedule.", 
                    ros::this_node::getName().c_str(), m_RunCount );
            markTrainingDataOfCurrRunAsFailed();
            
            
            if ( FailedRunsN == 0 )
                FailedRuns = std::to_string( m_RunCount );
            else 
                FailedRuns += ", " + std::to_string( m_RunCount );

            FailedRunsN++;
        }

        ros::Duration( 3.0 ).sleep();
        m_ROSTimer_MainLoop.stop();
        t1.detach();
        t1.~thread();

        
        flyDroneBetweenLastAndSecLastGate();
    }

    landDroneOnGround();

    if ( ! FailedRunsN == 0 )
        ROS_WARN(
            "[%s]\n >>  Finished Training Data Generation:"
            "\n  >> %d of %d runs failed."
            "\n  >> Failed runs: %s", 
            ros::this_node::getName().c_str(),
            FailedRunsN, m_ORCRunMaxCount, FailedRuns.c_str()

            );
}




//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////// ROS TIMER FUNCTIONS //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



void 
ForgetfulDrone::ROSTimerFunc_testNavigatorWithExpert
( const ros::TimerEvent& TimerEvent )
{
    if ( TimerEvent.profile.last_duration.toSec() > m_ORCMainLoopNomPeriodDuration )
        ROS_WARN(
            "[%s]\n  >> Last main loop iteration took %f s exceeding the nominal duration of %f s.",
            ros::this_node::getName().c_str(), 
            TimerEvent.profile.last_duration.toSec(),
            m_ORCMainLoopNomPeriodDuration
            );
    //___________________


    runWaypointUpdater();

    runExpert();

    runNavigator();
}



void 
ForgetfulDrone::ROSTimerFunc_generateTrainingData
( const ros::TimerEvent& TimerEvent )
{
    if ( TimerEvent.profile.last_duration.toSec() > m_ORCMainLoopNomPeriodDuration )
        ROS_WARN(
            "[%s]\n  >> Last main loop iteration took %f s exceeding the nominal duration of %f s.",
            ros::this_node::getName().c_str(), 
            TimerEvent.profile.last_duration.toSec(),
            m_ORCMainLoopNomPeriodDuration
            );
    //___________________


    runWaypointUpdater();

    runExpert();

    runTrainDataSaver();

    runNavigator();
}



//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////// SIMULATOR //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



void ForgetfulDrone::buildDroneRacingSimulation()
{
    forgetful_drones_msgs::BuildDroneRacingSimulation srv;
        srv.request.RaceType = m_SIMRaceTrackType;
        srv.request.DynGatesActivated = m_SIMDynGatesEnabled;




    ROS_DEBUG(
        "[%s]\n  >> Calling service \"%s\"...",
        ros::this_node::getName().c_str(),
        m_ROSSrvCl_BuildDroneRacingSimulation.getService().c_str()
        );
    if ( ! m_ROSSrvCl_BuildDroneRacingSimulation.call( srv ) )
        ROS_ERROR( 
            "[%s]\n  >> Failed to call service \"%s\".", 
            ros::this_node::getName().c_str(),
            m_ROSSrvCl_BuildDroneRacingSimulation.getService().c_str()
            );
    else
    {
        ROS_DEBUG( 
            "[%s]\n  >> Service \"%s\" successfully called.", 
            ros::this_node::getName().c_str(),
            m_ROSSrvCl_BuildDroneRacingSimulation.getService().c_str()
            );

        m_Gates_WaypointPose = srv.response.Gates_WaypointPose;
        m_Drone_InitPose = srv.response.Drone_InitPose;
    }

    ros::Duration(2.0).sleep(); // Needs time to load drone into Gazebo.
}

void ForgetfulDrone::switchDynamicGates( const bool& Enabled )
{
        std_msgs::Bool msg; 
        msg.data = Enabled; 
    m_ROSPub_DynGates.publish( msg );
}



//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////// GLOBAL TRAJECTORY //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


void ForgetfulDrone::computeGloTrajForExpert()
{
    // --- Set up ´m_GloTrajWaypoints´ ---
    m_GloTrajWaypoints.clear();
    m_GloTrajWaypoints.reserve( m_Gates_WaypointPose.size() );
    for ( const geometry_msgs::Pose& Pose : m_Gates_WaypointPose )
        m_GloTrajWaypoints.push_back( EigenVector3d_From_GeometryMsgsPoint(Pose.position) );

    // --- Set ´m_Expert_MaxHorizon´ <- max dist between waypoints ---
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

    ROS_INFO(
        "[%s]\n  >> Computing circuit global trajectory...\n" 
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
        static_cast<int>( m_GloTrajWaypoints.size() ),
        m_GTPolyOrder,
        m_GTContiOrder,
        m_GTMinWeightVel,
        m_GTMinWeightAcc,
        m_GTMinWeightJerk,
        m_GTMinWeightSnap,
        m_GTMaxSpeed,
        m_GTMaxNormThrust,
        m_GTMaxRollPitchRate,
        m_GTSamplingTime
        );


    // --- Set up Settings of GT ---
    polynomial_trajectories::PolynomialTrajectorySettings GTSettings;
    GTSettings.way_points = {
        std::make_move_iterator( m_GloTrajWaypoints.begin() ),
        std::make_move_iterator( m_GloTrajWaypoints.end() )
        };
    GTSettings.polynomial_order = m_GTPolyOrder;
    GTSettings.continuity_order = m_GTContiOrder;
    GTSettings.minimization_weights = Eigen::Vector4d{
        m_GTMinWeightVel,
        m_GTMinWeightAcc,
        m_GTMinWeightJerk,
        m_GTMinWeightSnap
        };
    

    // --- Compute initial segment times (first element relates to segment from last to first waypoint) ---
    Eigen::VectorXd GTInitSegmentTimes = Eigen::VectorXd::Ones( m_GloTrajWaypoints.size() );
    Eigen::Vector3d SegmentStart = m_GloTrajWaypoints.back();
    for ( size_t WaypointIdx = 0; WaypointIdx < m_GloTrajWaypoints.size(); WaypointIdx++ ) 
    {
        GTInitSegmentTimes[ WaypointIdx ] 
            = ( m_GloTrajWaypoints[ WaypointIdx ] - SegmentStart ).norm() 
                / m_GTMaxSpeed;
        
        SegmentStart = m_GloTrajWaypoints[ WaypointIdx ];
    }


    // --- Compute global trajectories until one is speed feasible ---
    bool GloTraj_SpeedFeasible;
    bool GloTraj_ComputationCompleted;
    int GloTraj_SubseqInfeasibleN = -1;
    int GloTraj_SubseqInfeasibleMaxN = 10;
    m_GloTrajCompFailed = false;
    while ( ! GloTraj_SpeedFeasible || ! GloTraj_ComputationCompleted )
    {
        GloTraj_SubseqInfeasibleN++;
        if ( GloTraj_SubseqInfeasibleN == GloTraj_SubseqInfeasibleMaxN )
        {
            m_GloTrajCompFailed = true;
            ROS_WARN( 
                "[%s]\n  >> Maximum number (%d) of failed global trajectory computations reached."
                "\n  >> Re-building drone racing simulation...",
                ros::this_node::getName().c_str(),
                GloTraj_SubseqInfeasibleMaxN
                );
            return;
        } 

        GloTraj_SpeedFeasible = false;
        GloTraj_ComputationCompleted = false;

        std::atomic<bool> ThreadFinished(false);
        quadrotor_common::Trajectory GlobalTraj;
        std::promise< quadrotor_common::Trajectory > Promise;
        auto Future = Promise.get_future();

        auto compGloTraj = [this, &ThreadFinished]( 
                const Eigen::VectorXd& GTInitSegmentTimes, 
                const polynomial_trajectories::PolynomialTrajectorySettings& GTSettings,
                std::promise< quadrotor_common::Trajectory > && Promise
                )
            {
                quadrotor_common::Trajectory GlobalTraj 
                    = trajectory_generation_helper::polynomials::generateMinimumSnapRingTrajectoryWithSegmentRefinement(
                        GTInitSegmentTimes,
                        GTSettings,
                        m_GTMaxSpeed,
                        m_GTMaxNormThrust,
                        m_GTMaxRollPitchRate,
                        1/m_GTSamplingTime
                        );
            
                Promise.set_value( GlobalTraj );
            
                ThreadFinished = true;
                ROS_DEBUG("[%s]\n  >> Thread to compute global trajectory has run through.",
                    ros::this_node::getName().c_str());
            };

        std::thread t1( compGloTraj, std::ref(GTInitSegmentTimes), std::ref(GTSettings),  std::move(Promise) );
        ros::WallTime ThreadStartTime = ros::WallTime::now();
        ros::WallTime ThreadMaxDuration = ros::WallTime( 10.0 );
        ros::WallDuration ThreadDuration;
        ros::WallRate Rate(1.0);
        do
        {
            if ( ThreadFinished ) break;

            Rate.sleep();
            ThreadDuration = ros::WallTime::now() - ThreadStartTime;
            ROS_DEBUG("[%s]\n  >> Thread to compute global trajectory has been running for %f/%f s.",
                    ros::this_node::getName().c_str(), 
                    ThreadDuration.toSec(), ThreadMaxDuration.toSec() );

        } while ( ThreadDuration.toSec() < ThreadMaxDuration.toSec() );
        
        if ( ThreadFinished )
        {
            t1.join();
            t1.~thread();
            GlobalTraj = Future.get();
        }
        else
        {
            ROS_DEBUG("[%s]\n  >> Thread to compute global trajectory has timed out.",
                    ros::this_node::getName().c_str());
            
            t1.detach();
            t1.~thread();

            continue;
        }
        
        m_GloTraj.clear();
        m_GloTraj = { 
            std::make_move_iterator( GlobalTraj.points.begin() ),
            std::make_move_iterator( GlobalTraj.points.end() ),
            };
        


        // Find the maximum and minimum velocity of the global trajectory.
        m_Expert_MaxSpeed = 0.0;
        m_Expert_MinSpeed = std::numeric_limits< double >::max();
        for ( size_t StateIdx = 0; StateIdx < m_GloTraj.size(); StateIdx++ )
        {
            double Speed = ( m_GloTraj[ StateIdx ].velocity ).norm();
            m_Expert_MaxSpeed = std::max( m_Expert_MaxSpeed, Speed );
            m_Expert_MinSpeed = std::min( m_Expert_MinSpeed, Speed );
        }

        if ( m_Expert_MaxSpeed > 1.1 * m_GTMaxSpeed )
        {
            ROS_WARN(
                "[%s]\n  >> Maximum Speed of computed global trajectory exceeds 110 %% of nominal value (%f/%f m/s)."
                "\n     Re-computing global trajectory...",
                ros::this_node::getName().c_str(), m_Expert_MaxSpeed, m_GTMaxSpeed
                );

            continue;
        }
        else
            GloTraj_SpeedFeasible = true;

        if ( GlobalTraj.trajectory_type == quadrotor_common::Trajectory::TrajectoryType::UNDEFINED )
        {
            ROS_WARN(
                "[%s]\n  >> Computation of global trajectory not successful."
                "\n     Re-computing global trajectory....",
                ros::this_node::getName().c_str()
                );

            continue;
        }
        else
            GloTraj_ComputationCompleted = true;
    }

    ROS_INFO( 
        "[%s]\n  >> Global trajectory successfully computed.\n"
            "\t#states: %d\n"
            "\tduration: %1.1f s\n"
            "\tmaximum speed: %1.1f m/s\n"
            "\tminimum speed: %1.1f m/s\n",            
        ros::this_node::getName().c_str(),
        static_cast<int>( m_GloTraj.size() ),
        m_GloTraj.size() * m_GTSamplingTime,
        m_Expert_MaxSpeed, m_Expert_MinSpeed
        );



    // --- Visualize global trajectory in RVIZ ---
    rvizGloTraj();

    // --- Set Waypoint Idx for deterministic 8 RaceTrack 
    m_CurrWaypointIdx = m_WUInitWaypointIdxForFIG8;
    m_LastWaypointIdx 
        = (m_CurrWaypointIdx + m_GloTrajWaypoints.size() - 1) 
            % m_GloTrajWaypoints.size();
}


void ForgetfulDrone::rvizGloTraj()
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


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////// WAYPOINT UPDATER //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


void ForgetfulDrone::runWaypointUpdater()
{
    // Distance from reference state to current and last gate waypoint.
    m_Dist2CurrWaypoint 
        = ( m_ReferenceStatePos_WRF - m_GloTrajWaypoints[m_CurrWaypointIdx] ).norm();
    m_Dist2LastWaypoint 
        = ( m_ReferenceStatePos_WRF - m_GloTrajWaypoints[m_LastWaypointIdx] ).norm();
    
    updateWaypointIdx();
    rvizCurrWaypoint();
}


void ForgetfulDrone::updateWaypointIdx()
{
    if ( m_Dist2CurrWaypoint > m_WUThresDist2DetectWaypointArrival )
        return;

    if ( m_EstimatedNormalizedSpeed < ( m_LTMinSpeed /2) / m_LTMaxSpeed )
    {
        ROS_DEBUG(
            "[%s]\n  >> Assuming that drone is gathering pace in running-in phase. Not updating waypoint index.",
            ros::this_node::getName().c_str()
            );
        return;
    }

    ROS_INFO(
    "[%s]\n  >> Gate#%d/%d passed.",
    ros::this_node::getName().c_str(), static_cast<int>(m_LastWaypointIdx),
    static_cast<int>( m_GloTrajWaypoints.size() - 1 )
    );
        
        
    ++m_CurrWaypointIdx %= m_GloTrajWaypoints.size();
    m_LastWaypointIdx 
        = (m_CurrWaypointIdx + m_GloTrajWaypoints.size() - 1) 
            % m_GloTrajWaypoints.size();
    
    m_LastWaypointArrivalTime = ros::Time::now();

    if ( m_CurrWaypointIdx == 0 )
    {
        m_LapCount++;
        ROS_INFO(
        "[%s]\n  >> Lap#%d/%d entered.",
        ros::this_node::getName().c_str(), m_LapCount,
        m_ORCLapMaxCount -1
        );
        
    }
}

void ForgetfulDrone::rvizCurrWaypoint()
{
    rvizPosition(
        m_GloTrajWaypoints[ m_CurrWaypointIdx ],
        VisPosTypes::CURRGATE,
        m_ROSPub_RVIZPositions
        );
}





//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////// NAVIGATOR //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////




void ForgetfulDrone::runNavigator()
{
    // Perturb input to navigator
    m_NavigatorInput 
        += Eigen::Vector3d::Ones() 
            * m_NAVInputPerturbAxAmp 
            * std::sin( 0.5*ros::WallTime::now().toSec() );


    if ( ! m_NavigatorEnabled )
    {
        m_ReferenceState_ARF = {};
        m_ReferenceState_ARF.position = m_Trafo_ARF_DRF.getPosition();
        m_ReferenceState_ARF.orientation = m_Trafo_ARF_DRF.getEigenQuaternion();
        return;
    }

    bool DroneDivergedFromRefState
        = m_NAVMaxDivFromRefState 
            < ( m_ReferenceState_ARF.position - m_Trafo_ARF_DRF.getPosition() ).norm();

    if ( DroneDivergedFromRefState )
    {
        ROS_WARN(
            "[%s]\n  >> Reference position (%f, %f, %f) and drone position estimate (%f, %f, %f) diverged.", 
            ros::this_node::getName().c_str(),
            m_ReferenceState_ARF.position.x(), m_ReferenceState_ARF.position.y(), m_ReferenceState_ARF.position.z(),
            m_Trafo_ARF_DRF.getPosition().x(), m_Trafo_ARF_DRF.getPosition().y(), m_Trafo_ARF_DRF.getPosition().z()
            );
        
        switchNavigator( false );
        return;
    }


    // If it is time, replan local trajectory
    bool LocTrajReplanningScheduled = m_MainLoopIterCount % m_NAVMainLoopItersPerLTReplanning == 0;
    
    if ( LocTrajReplanningScheduled )
    {
        // Compute goal position and speed to reach goal position from CNN output
        Eigen::Vector3d GoalPos_ARF;   
        double Speed2Goal;
        processNavigatorInput( GoalPos_ARF, Speed2Goal );

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
                switchNavigator( false );
                return;
            }
        }
    }
    

    publishRefStateFromLocTrajToAutopilot();

    rvizReferenceState();


    m_MainLoopIterCount++;
}










void ForgetfulDrone::rvizReferenceState()
{
    rvizPosition(
        m_ReferenceStatePos_WRF,
        VisPosTypes::REFERENCE,
        m_ROSPub_RVIZPositions
        );
}

void ForgetfulDrone::switchNavigator( const bool& Enabled )
{
    std::string LastState = m_NavigatorEnabled? "ENABLED" : "DISABLED";

    auto enterON = [LastState, this](){
        m_ReferenceState_ARF = {};
        m_ReferenceState_ARF.position = m_Trafo_ARF_DRF.getPosition();
        m_ReferenceState_ARF.orientation = m_Trafo_ARF_DRF.getEigenQuaternion();
        m_ROSPub_AutopilotReferenceState.publish( m_ReferenceState_ARF.toRosMessage() );

        m_NavigatorEnabled = true;
        ROS_INFO ( "[%s]\n  >> Navigator switched from %s to ENABLED.", 
            ros::this_node::getName().c_str(), LastState.c_str() );
        };

    auto enterOFF = [LastState, this](){
        m_NavigatorEnabled = false;
        
        m_ReferenceState_ARF = {};
        m_ReferenceState_ARF.position = m_Trafo_ARF_DRF.getPosition();
        m_ReferenceState_ARF.orientation = m_Trafo_ARF_DRF.getEigenQuaternion();
        m_ROSPub_AutopilotReferenceState.publish( m_ReferenceState_ARF.toRosMessage() );

        ROS_INFO ( "[%s]\n  >> Navigator switched from %s to DISABLED.", 
            ros::this_node::getName().c_str(), LastState.c_str() );
        };

    Enabled? enterON() : enterOFF();
}


void ForgetfulDrone::publishRefStateFromLocTrajToAutopilot()
{
    // Get time of current state of current local trajectory
    const double LocTraj_CurrTime
        = (ros::Time::now() - m_LocTraj_StartTime).toSec() + m_ORCMainLoopNomPeriodDuration;

    m_ReferenceState_ARF.position        = EigenVector3d_From_Vec3( m_LocTraj.GetPosition(LocTraj_CurrTime) );
    m_ReferenceState_ARF.velocity        = EigenVector3d_From_Vec3( m_LocTraj.GetVelocity(LocTraj_CurrTime) );
    m_ReferenceState_ARF.acceleration    = EigenVector3d_From_Vec3( m_LocTraj.GetAcceleration(LocTraj_CurrTime) );
    m_ReferenceState_ARF.heading = std::atan2( m_ReferenceState_ARF.velocity.y(), m_ReferenceState_ARF.velocity.x() );

    //m_ReferenceStatePos_WRF
    geometry_msgs::Pose m_ReferencePose_ARF;
        m_ReferencePose_ARF.position = GeometryMsgsPoint_From_EigenVector3d( m_ReferenceState_ARF.position );
        m_ReferencePose_ARF.orientation.w = 1.0;
        m_ReferencePose_ARF.orientation.x = 0.0;
        m_ReferencePose_ARF.orientation.y = 0.0;
        m_ReferencePose_ARF.orientation.z = 0.0;
    kindr::minimal::QuatTransformation Trafo_ARF_RRF;
    tf::poseMsgToKindr(
        m_ReferencePose_ARF,
        &Trafo_ARF_RRF
        );
    m_ReferenceStatePos_WRF = ( m_Trafo_WRF_ARF * Trafo_ARF_RRF ).getPosition();

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


void ForgetfulDrone::rvizLocTraj()
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


bool 
ForgetfulDrone::computeLocTraj
( 
    const Eigen::Vector3d& GoalPos_ARF, 
    const double& Speed2Goal
)
{
    const Vec3 StartPos_ARF = Vec3_From_EigenVector3d( m_ReferenceState_ARF.position );
    const Vec3 StartVel_ARF = Vec3_From_EigenVector3d( m_ReferenceState_ARF.velocity );
    const Vec3 StartAcc_ARF = Vec3_From_EigenVector3d( m_ReferenceState_ARF.acceleration );
    const Vec3 EndPos_ARF = Vec3_From_EigenVector3d( GoalPos_ARF );
    const Vec3 Gravity{ 0.0, 0.0, -9.81 };

    const double LocTraj_Distance = ( EndPos_ARF - StartPos_ARF ).GetNorm2();

    const double LocTraj_Duration = LocTraj_Distance / std::min({
        Speed2Goal,
        m_ReferenceState_ARF.velocity.norm() + m_LTMaxSpeedIncrement,
        m_LTMaxSpeed
        });

    RQTG::RapidTrajectoryGenerator LocTraj{
        StartPos_ARF, 
        StartVel_ARF,
        StartAcc_ARF,
        Gravity
        };

    LocTraj.SetGoalPosition( EndPos_ARF );
    LocTraj.Generate( LocTraj_Duration );

    
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
    //    LocTraj_Duration,
    //    LocTraj_Distance
    //    );
    
    if ( LocTrajFeasible( LocTraj ) )
    {
        m_LocTraj = LocTraj;
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
            LocTraj_Duration
            );

        return false;
    }
}



bool 
ForgetfulDrone::LocTrajFeasible
( 
    RQTG::RapidTrajectoryGenerator& LocTraj 
)
{  
    RQTG::RapidTrajectoryGenerator::InputFeasibilityResult InputFeasibility 
        = LocTraj.CheckInputFeasibility(
            m_LTMinNormThrust,
            m_LTMaxNormThrust,
            m_LTMaxBodyRates,
            m_LTInputFeasibilityCheckMinSamplingTime
            );

    Vec3 BoundaryPoint = { 0.0, 0.0, m_LTMinAltitude };  // a point on the floor
    Vec3 BoundaryVector = { 0.0, 0.0, 1.0 };  // we want to be above the point
    RQTG::RapidTrajectoryGenerator::StateFeasibilityResult PositionFeasibility_Floor 
        = LocTraj.CheckPositionFeasibility(
            BoundaryPoint, 
            BoundaryVector
            );

    BoundaryPoint[2] = m_LTMaxAltitude;  // a point on the ceiling
    BoundaryVector[2] = -1.0;  // we want to be below the point
    RQTG::RapidTrajectoryGenerator::StateFeasibilityResult PositionFeasibility_Ceiling 
        = LocTraj.CheckPositionFeasibility(
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
}



void 
ForgetfulDrone::processNavigatorInput
( 
    Eigen::Vector3d& OUT_GoalPos_ARF, 
    double& OUT_Speed2Goal 
)
{
    const double& GoalX_IRF = m_NavigatorInput.x();
    const double& GoalY_IRF = m_NavigatorInput.y();
    const double& NormSpeed2Goal = m_NavigatorInput.z();


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
    double LocTraj_Distance = std::max( 
        m_LTMinDistance, std::min( 
            m_LTMaxDistance, m_LTDuration * OUT_Speed2Goal 
            ) 
        );


    OUT_GoalPos_ARF 
        = XYZ_ARF_From_XYZ_DRF( 
            XYZ_DRF_From_XYDist_IRF( 
                GoalX_IRF, GoalY_IRF, LocTraj_Distance ));

    //ROS_INFO(
    //        "[%s] Goal Position: (%f, %f, %f) [DRF]",
    //        ros::this_node::getName().c_str(),
    //        OUT_GoalPos_ARF.x(), OUT_GoalPos_ARF.y(), OUT_GoalPos_ARF.z()
    //        );
}



Eigen::Vector3d 
ForgetfulDrone::XYZ_DRF_From_XYDist_IRF
( 
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

Eigen::Vector3d ForgetfulDrone::XYZ_ARF_From_XYZ_DRF( const Eigen::Vector3d& XYZ_DRF)
{
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
    
    return T_ARF_XYZ.getPosition();
}









//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////// EXPERT //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



void 
ForgetfulDrone::runExpert
()
{
    // Horizon: Distance 2 the closer one of current/last gate but within specified range
    double Horizon 
        = std::min(
            m_Expert_MaxHorizon, std::max( 
                m_EXPMinHorizon, std::min( 
                    m_Dist2CurrWaypoint, m_Dist2LastWaypoint 
                    ) 
                )
            );

    findExpertStateWithProjection(); 
    findHorizonState( Horizon );
    rvizExpertAndHorizonState();

    Eigen::Vector2d GoalPos_IRF 
        = PosIRF_From_PosDRF(
            PosDRF_From_PosWRF(
                m_GloTraj[ m_HorizonStateIdx ].position
                )
            );

    findSpeedState();
    double NormalizedSpeed2Goal
        = m_GloTraj[ m_SpeedStateIdx ].velocity.norm() / m_Expert_MaxSpeed;

    m_ExpertPrediction = { GoalPos_IRF.x(), GoalPos_IRF.y(), NormalizedSpeed2Goal };
    m_NavigatorInput = m_ExpertPrediction;
}









void 
ForgetfulDrone::findExpertStateWithProjection
()
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



void 
ForgetfulDrone::findExpertStateWithMinDist
()
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



void 
ForgetfulDrone::findHorizonState
( 
    const double& Horizon 
)
{
    // Returns state of global trajectory that follows "expert state" 
    // with specified min distance 

    double Distance_Expert2HorizonState;
    m_HorizonStateIdx = m_ExpertStateIdx;
    
    do
    {
        ++m_HorizonStateIdx %= m_GloTraj.size();

        
        Distance_Expert2HorizonState 
            = ( m_GloTraj[ m_HorizonStateIdx ].position 
                - m_GloTraj[ m_ExpertStateIdx ].position ).norm();

    } while ( Distance_Expert2HorizonState < Horizon );
}


void 
ForgetfulDrone::findSpeedState
()
{
    double Distance_Expert2SpeedState;
    m_SpeedStateIdx = m_ExpertStateIdx;
    
    do
    {
        ++m_SpeedStateIdx %= m_GloTraj.size();

        
        Distance_Expert2SpeedState 
            = ( m_GloTraj[ m_SpeedStateIdx ].position 
                - m_GloTraj[ m_ExpertStateIdx ].position ).norm();

    } while ( Distance_Expert2SpeedState < m_EXPSpeedHorizon );
}



Eigen::Vector3d 
ForgetfulDrone::PosDRF_From_PosWRF
( 
    const Eigen::Vector3d& PosWRF 
)
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


Eigen::Vector2d 
ForgetfulDrone::PosIRF_From_PosDRF
( 
    const Eigen::Vector3d& PosDRF 
)
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




void 
ForgetfulDrone::rvizExpertAndHorizonState
()
{
    rvizPosition(
        m_GloTraj[ m_ExpertStateIdx ].position,
        VisPosTypes::EXPERT,
        m_ROSPub_RVIZPositions
        );
    rvizPosition(
        m_GloTraj[ m_HorizonStateIdx ].position,
        VisPosTypes::HORIZON,
        m_ROSPub_RVIZPositions
        );
}













//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////// TRAIN DATA SAVER //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


void 
ForgetfulDrone::runTrainDataSaver
()
{
    if ( ! m_TrainDataSaverEnabled )
        if ( m_CurrWaypointIdx == 0 )
        {
            m_TrainDataSaverEnabled = true;
            ROS_INFO(
                "[%s]\n  >> Training data saver ENABLED.", 
                ros::this_node::getName().c_str()
                );
        }
            

    if ( m_TrainDataSaverEnabled )
        saveTrainData();
}



void ForgetfulDrone::saveTrainData()
{
    std::ostringstream CamFrameCount_Stream;
    CamFrameCount_Stream << std::setw(5) << std::setfill('0') << m_CamFrameCount;
    std::string CamFrameCount_String = CamFrameCount_Stream.str();

    // --- Image ---
    std::string ImageFilePath 
            = m_RunDirPath + "/images/camera_frame_" + CamFrameCount_String + ".jpg";

    m_CloneImgMtx.lock();
        // Convert from bgr to rgb
        cv::Mat CamFrame = m_CamFrame.clone();
        cv::cvtColor( m_CamFrame, CamFrame, CV_BGR2RGB );
    m_CloneImgMtx.unlock();

        if ( ! cv::imwrite( ImageFilePath, CamFrame ) ) 
            ROS_ERROR(
                "[%s]\n  >> Failed to save current camera frame to file \"%s\".", 
                ros::this_node::getName().c_str(), ImageFilePath.c_str() );
    
    // --- Label ---
    
    // save label of best trajectory
    std::ofstream Outfile;
    std::string FilePath = m_RunDirPath + "/labels.txt";
    Outfile.open( FilePath, std::ios_base::app );
    Outfile 
        << std::to_string( m_ExpertPrediction.x() ) + ";" 
        << std::to_string( m_ExpertPrediction.y() ) + ";"
        << std::to_string( m_ExpertPrediction.z() ) + ";" 
        << std::to_string( m_Expert_MaxSpeed      ) + ";"
        << std::to_string( m_CamFrameCount == 0? 1 : 0 ) + "\n"; // ???new dagger batch???
    Outfile.close();


    m_CamFrameCount++;
}

void ForgetfulDrone::markTrainingDataOfCurrRunAsFailed()
{
    // If right now the drone is not in a data generation run, do nothing
    // if ( m_RunCount == -1 || m_RunCount == m_ORCRunMaxCount ) return;

    std::ofstream Outfile;
        std::string Filename = m_TrainingDataDirPath + "/fails.txt";
        Outfile.open( Filename, std::ios_base::app );
        Outfile << std::to_string( m_RunCount ) + "\n";
        Outfile.close();
}






//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////// NOT IMPLEMENTED YET //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void ForgetfulDrone::testNavigatorWithNetwork()
{
    std::cout << "\n\n";
    ROS_INFO( "[%s]\n >> Starting Flight Mission: Navigator-Network Test.", ros::this_node::getName().c_str() );
    std::cout << "\n";

    ROS_ERROR( "[%s]\n >> Specified flight mission not implemented yet.", ros::this_node::getName().c_str() );
}

void ForgetfulDrone::testSimulatorWithAutopilot()
{
    std::cout << "\n\n";
    ROS_INFO( "[%s]\n >> Flight Mission: Simulator-Autopilot Test", ros::this_node::getName().c_str() );
    std::cout << "\n";

    ROS_ERROR( "[%s]\n >> Specified flight mission not implemented yet.", ros::this_node::getName().c_str() );

//forgetful_drones_msgs::FlyDroneThroughRaceTrack srv;
    //srv.request.InitDronePose = m_Drone_InitPose;
    ////srv.request.IsRingTrajectory = true;
    //srv.request.IsRingTrajectory = false;
    //srv.request.Rounds_N = 2;
    //srv.request.SamplingTime = std::min(
    //    m_ROSParam_NavigatorLoop_NominalDuration * m_ROSParam_NavigatorLoop_Iters2LTCompsRatio,
    //    0.05
    //    );
    //srv.request.WaypointPoses = m_Gates_WaypointPose;
    //
    //if ( m_ROSSrvCl_FlyDroneThroughRaceTrack.call( srv ) )
    //    ROS_INFO( "[%s] Service successfully called.", 
    //                ros::this_node::getName().c_str() );
    //else
    //    ROS_INFO( "[%s] Failed to call service.", 
    //                ros::this_node::getName().c_str() );

}

}