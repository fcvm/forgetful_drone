#include "forgetful_drones/forgetful_orchestrator.h"
#include "forgetful_drones/forgetful_helpers.h"



//DEBUG !!!
#include <geometry_msgs/TwistStamped.h>


namespace forgetful_drone 
{

ForgetfulOrchestrator::ForgetfulOrchestrator(
    const ros::NodeHandle& RNH, 
    const ros::NodeHandle& PNH
    )
    :
    m_ROSRNH( RNH ),
    m_ROSPNH( PNH ),



    m_ROSSub_DivergedFromReferenceState( m_ROSRNH.subscribe(
        "navigator/diverged_from_reference_state", 1, 
        &ForgetfulOrchestrator::ROSCB_DivergedFromReferenceState, this
        ) ),
    m_ROSSub_AutopilotFeedback( m_ROSRNH.subscribe(
        "autopilot/feedback", 1, &ForgetfulOrchestrator::ROSCB_AutopilotFeedback, this
        ) ),

    
    m_ROSSub_ExpertPrediction( m_ROSRNH.subscribe("expert/prediction", 1, &ForgetfulOrchestrator::ROSCB_ExpertPrediction, this) ),
    m_ROSSub_NetworkPrediction( m_ROSRNH.subscribe("network/prediction", 1, &ForgetfulOrchestrator::ROSCB_NetworkPrediction, this) ),
    m_ROSSub_RGBCameraImageRaw( m_ROSRNH.subscribe("rgb_camera/camera_1/image_raw", 1, &ForgetfulOrchestrator::ROSCB_RGBCameraImageRaw, this) ),
    m_ROSPub_RVIZCamFrameWithPredictions( m_ROSRNH.advertise<sensor_msgs::Image>("rviz/camera_frame_with_predictions", 1) ),
    

    m_ROSPub_DynGates( m_ROSRNH.advertise<std_msgs::Bool>("simulator/dynamic_gates", 1) ), 
    m_ROSPub_AutopilotOff( m_ROSRNH.advertise<std_msgs::Empty>("autopilot/off", 1) ),
    m_ROSPub_AutopilotStart( m_ROSRNH.advertise<std_msgs::Empty>("autopilot/start", 1) ),
    m_ROSPub_BridgeArm( m_ROSRNH.advertise<std_msgs::Bool>("bridge/arm", 1) ),
    m_ROSPub_NavigatorState( 
        m_ROSRNH.advertise<std_msgs::Bool>("navigator/state_switch", 1
        ) ),
   


    m_ROSSrvCl_BuildDroneRacingSimulation( 
        m_ROSRNH.serviceClient<forgetful_drones_msgs::BuildDroneRacingSimulation>(
            "simulator/build_drone_racing_simulation"
            ) 
        ),

    m_ROSSrvCl_ComputeGlobalTrajectory( 
        m_ROSRNH.serviceClient<forgetful_drones_msgs::ComputeGlobalTrajectory>(
            "expert/compute_global_trajectory"
            ) 
        ),





    m_AutopilotState( quadrotor_msgs::AutopilotFeedback::OFF ),
    m_ORCFlightMission(),
    m_SIMRaceTrackType(),
    m_ExpertPrediction(),
    m_NetworkPrediciton(),



    m_SIMDynGatesEnabled(),
    m_ORCRunMaxCount(),
    m_ORCFlightMission_temp(),
    m_SIMRaceTrackType_temp(),
    m_DRONECamHalfYawAOV(),
    m_DRONECamHalfPitchAOV(),
    m_LTMaxSpeed(),
    m_TrainingDataGeneration_MaxRunDuration()
{
    // --- Set logger level of this node to DEBUG ---
    if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) 
        ros::console::notifyLoggerLevelsChanged();


    bool InitializationSuccessful = true;

        std::vector< std::pair<const char*, const bool*> > KeysAndBoolOutputs
        {
            {"dynamic_gates_activated" , &m_SIMDynGatesEnabled}
        };
        std::vector< std::pair<const char*, const int*> > KeysAndIntOutputs
        {
            {"number_of_training_data_generation_runs" , &m_ORCRunMaxCount}
        };
        std::vector< std::pair<const char*, const double*> > KeysAndDoubleOutputs
        {
            {"Cam_HalfYawAOV", &m_DRONECamHalfYawAOV},
            {"Cam_HalfPitchAOV", &m_DRONECamHalfPitchAOV},
            {"LT_MaxSpeed", &m_LTMaxSpeed},
            {"max_duration_of_training_data_generation_run", &m_TrainingDataGeneration_MaxRunDuration}
        };
        std::vector< std::pair<const char*, const std::string*> > KeysAndStringOutputs
        {
            {"flight_mission", &m_ORCFlightMission_temp},
            {"race_track_type", &m_SIMRaceTrackType_temp}
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
        //if ( ! fetchROSParameters() ) InitializationSuccessful = false;

        if ( m_ORCFlightMission_temp == std::string("test_navigator_with_expert") )
            m_ORCFlightMission = FlightMissions::testNavigatorWithExpert;
        else if ( m_ORCFlightMission_temp == std::string("test_navigator_with_brain") )
            m_ORCFlightMission = FlightMissions::testNavigatorWithNetwork;
        else if ( m_ORCFlightMission_temp == std::string("generate_training_data") )
            m_ORCFlightMission = FlightMissions::generateTrainingData;
        else if ( m_ORCFlightMission_temp == std::string("test_simulation_with_autopilot") )
            m_ORCFlightMission = FlightMissions::testSimulatorWithAutopilot;
        else
        {
            ROS_ERROR( "[%s]\n  >> ROS parameter with key \"flight_mission\" has invalid value.", 
                ros::this_node::getName().c_str() );
            InitializationSuccessful = false;
        }

        if ( m_SIMRaceTrackType_temp == std::string("deterministic_figure8") )
            m_SIMRaceTrackType = forgetful_drones_msgs::BuildDroneRacingSimulationRequest::FIG8_DET;
        else if ( m_SIMRaceTrackType_temp == std::string("randomized_figure8") )
            m_SIMRaceTrackType = forgetful_drones_msgs::BuildDroneRacingSimulationRequest::FIG8_RAND;
        else if ( m_SIMRaceTrackType_temp == std::string("randomized_circuit") )
            m_SIMRaceTrackType = forgetful_drones_msgs::BuildDroneRacingSimulationRequest::CIRCUIT_RAND;
        else if ( m_SIMRaceTrackType_temp == std::string("randomized_sprint") )
            m_SIMRaceTrackType = forgetful_drones_msgs::BuildDroneRacingSimulationRequest::SPRINT_RAND;
        else
        {
            ROS_ERROR( "[%s]\n  >> ROS parameter with key \"race_track_type\" has invalid value.", 
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
        case FlightMissions::generateTrainingData:
            ROS_INFO( "[%s]\n >> Flight Mission: Generate Training Data.", ros::this_node::getName().c_str() );
            generateTrainingData();
            break;
        
        case FlightMissions::testNavigatorWithNetwork:
            ROS_INFO( "[%s]\n >> Flight Mission: Test Navigator With Brain.", ros::this_node::getName().c_str() );
            testNavigatorWithNetwork();
            break;

        case FlightMissions::testNavigatorWithExpert:
            ROS_INFO( "[%s]\n >> Flight Mission: Test Navigator With Expert.", ros::this_node::getName().c_str() );
            testNavigatorWithExpert();
            break;

        case FlightMissions::testSimulatorWithAutopilot:
            ROS_INFO( "[%s]\n >> Flight Mission: Test Simulation With Autopilot.", ros::this_node::getName().c_str() );
            testSimulatorWithAutopilot();
            break;
        
        default:
            ROS_ERROR( "[%s]\n >> Specified flight mission unknown.", ros::this_node::getName().c_str() );
            break;
        }
}



ForgetfulOrchestrator::~ForgetfulOrchestrator()
{
    // Delete training data folders if no data was generated.
    if ( std::filesystem::exists( m_TrainingDataDirPath ) )
        if ( std::filesystem::is_empty(m_TrainingDataDirPath) )
            std::filesystem::remove( m_TrainingDataDirPath );
}





void ForgetfulOrchestrator::ROSCB_ExpertPrediction( const geometry_msgs::PointConstPtr& msg ) 
{
    m_ExpertPrediction = { msg->x, msg->y, msg->z };
}

void ForgetfulOrchestrator::ROSCB_NetworkPrediction( const geometry_msgs::PointConstPtr& msg ) 
{
    m_NetworkPrediciton = { msg->x, msg->y, msg->z };
}

void ForgetfulOrchestrator::ROSCB_RGBCameraImageRaw( const sensor_msgs::ImageConstPtr& msg )
{
    // Update `CamFrame`
    //cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare( msg, sensor_msgs::image_encodings::TYPE_8UC3 );
    //m_CloneImgMtx.lock();
    //m_CamFrame = cv_ptr->image.clone();
    //m_CloneImgMtx.unlock();
    
    m_CamFrame = cv_bridge::toCvShare( 
        msg, sensor_msgs::image_encodings::TYPE_8UC3 
        )->image.clone();

    rvizCamFrameWithPredictions();
}


void ForgetfulOrchestrator::rvizCamFrameWithPredictions()
{
    //return;

    
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







void ForgetfulOrchestrator::ROSCB_DivergedFromReferenceState( const std_msgs::EmptyConstPtr& msg )
{
    // Mark collected data as invalid.
    /*std::ofstream Outfile;
    std::string FilePath = p_Run_RootDirPath + "/fails.txt";
    
    Outfile.open( FilePath, std::ios_base::app );
        Outfile << std::to_string( m_Run_Count ) + "\n";
    Outfile.close();*/
}


void ForgetfulOrchestrator::ROSCB_AutopilotFeedback( const quadrotor_msgs::AutopilotFeedback::ConstPtr& msg )
{
    m_AutopilotState = msg->autopilot_state;

    const geometry_msgs::Vector3& EstimatedVelocity = msg->state_estimate.twist.twist.linear;
    
    m_EstimatedNormalizedSpeed = sqrt( 
        pow(EstimatedVelocity.x, 2) + pow(EstimatedVelocity.y, 2) + pow(EstimatedVelocity.z, 2) 
        ) / m_LTMaxSpeed;
}





void ForgetfulOrchestrator::buildDroneRacingSimulation()
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






void ForgetfulOrchestrator::generateTrainingData()
{
    
    // Create directory where the training data is saved.
    m_TrainingDataDirPath
        = ros::package::getPath( "forgetful_drones" ) 
            + "/training_data/" + getCurrUTCDateTimeAsString();
    std::filesystem::create_directory( m_TrainingDataDirPath );






    for ( int RunIdx = 0; RunIdx < m_ORCRunMaxCount; RunIdx++ )
    {
        ROS_INFO( "[%s]\n >>  Training Data Generation: Starting Run #%d.", 
                    ros::this_node::getName().c_str(), RunIdx );


        buildDroneRacingSimulation();
        computeGloTrajForExpert();
        launchDroneOffGround();

        switchOnOff_NavigatorState( true );

        // CONDITION WHEN TO START saving

        ros::WallTime RunStartTime = ros::WallTime::now();
        ros::WallDuration RunDuration;
        do
        {
            // SAVE DATA

            RunDuration = ros::WallTime::now() - RunStartTime;
            ROS_INFO(
                "[%s]\n  >> Scheduled termination of current run in %d s.",
                ros::this_node::getName().c_str(),
                (int) (m_TrainingDataGeneration_MaxRunDuration - RunDuration.toSec())
                );
            ros::Duration(1.0).sleep();
        } while ( RunDuration.toSec() < m_TrainingDataGeneration_MaxRunDuration );

        switchOnOff_NavigatorState( false );

        std_msgs::Bool msg; msg.data = false;
        ros::Duration( 1.0 ).sleep();
        m_ROSPub_BridgeArm.publish( msg );
        ros::Duration( 1.0 ).sleep();
        m_ROSPub_AutopilotOff.publish( std_msgs::Empty() );
        ros::Duration( 1.0 ).sleep();

    

        ROS_INFO( "[%s]\n >>  Training Data Generation: Run #%d successfully conducted.", 
                    ros::this_node::getName().c_str(), RunIdx );
        ros::Duration(5.0).sleep();

        
    }
}





void ForgetfulOrchestrator::testNavigatorWithExpert()
{
    buildDroneRacingSimulation();

    computeGloTrajForExpert();

    launchDroneOffGround();

    switchOnOff_NavigatorState( true );
}








void ForgetfulOrchestrator::switchDynamicGates( const bool& SwitchOn )
{
    std_msgs::Bool msg; msg.data = SwitchOn;
    
    m_ROSPub_DynGates.publish( msg );
}

void ForgetfulOrchestrator::switchOnOff_NavigatorState( const bool& SwitchOn )
{
    std_msgs::Bool msg; msg.data = SwitchOn; 
    m_ROSPub_NavigatorState.publish( msg );
}




void ForgetfulOrchestrator::launchDroneOffGround()
{
    m_ROSPub_AutopilotOff.publish( std_msgs::Empty() );
    ros::Duration( 1.0 ).sleep();

    std_msgs::Bool msg; msg.data = true;
    m_ROSPub_BridgeArm.publish( msg );
    ros::Duration( 1.0 ).sleep();

    m_ROSPub_AutopilotStart.publish( std_msgs::Empty() );

    
    while ( m_AutopilotState != quadrotor_msgs::AutopilotFeedback::BREAKING )
        { ros::spinOnce(); ros::Duration( 0.5 ).sleep(); ROS_DEBUG("."); }
    while ( m_AutopilotState != quadrotor_msgs::AutopilotFeedback::HOVER )
        { ros::spinOnce(); ros::Duration( 0.5 ).sleep(); ROS_DEBUG("."); }
}


void ForgetfulOrchestrator::computeGloTrajForExpert()
{
    forgetful_drones_msgs::ComputeGlobalTrajectory srv;
    srv.request.Waypoints_Pos.reserve( m_Gates_WaypointPose.size() );
    for ( geometry_msgs::Pose& Pose : m_Gates_WaypointPose )
        srv.request.Waypoints_Pos.push_back( Pose.position );

    
    ROS_DEBUG(
        "[%s]\n  >> Calling service \"%s\"...", 
        ros::this_node::getName().c_str(),
        m_ROSSrvCl_ComputeGlobalTrajectory.getService().c_str()
        );
    if ( ! m_ROSSrvCl_ComputeGlobalTrajectory.call(srv) )
    {
        ROS_ERROR( 
            "[%s]\n  >> Failed to call service \"%s\".", 
            ros::this_node::getName().c_str(),
            m_ROSSrvCl_ComputeGlobalTrajectory.getService().c_str()
            );
    }
    else
    {
        ROS_DEBUG( 
            "[%s]\n  >> Service \"%s\" successfully called.", 
            ros::this_node::getName().c_str(),
            m_ROSSrvCl_ComputeGlobalTrajectory.getService().c_str()
            );
    }
}




void ForgetfulOrchestrator::testSimulatorWithAutopilot()
{
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


void ForgetfulOrchestrator::testNavigatorWithNetwork(){}

}