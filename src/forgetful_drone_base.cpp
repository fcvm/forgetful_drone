#include "forgetful_drones/forgetful_drone.h"
#include "forgetful_drones/forgetful_helpers.hpp"

#include "forgetful_drones/forgetful_global_trajectory.hpp"

#define MY_ROS_DEBUG(...) {std::cout <<std::endl << "\033[36;46m" << "[]" << "\033[0m" << " "; ROS_LOG(::ros::console::levels::Debug, ROSCONSOLE_DEFAULT_NAME, __VA_ARGS__);}
#define MY_ROS_INFO(...)  {std::cout <<std::endl << "\033[36;46m" << "[]" << "\033[0m" << " "; ROS_LOG(::ros::console::levels::Info,  ROSCONSOLE_DEFAULT_NAME, __VA_ARGS__);}
#define MY_ROS_WARN(...)  {std::cout <<std::endl << "\033[36;46m" << "[]" << "\033[0m" << " "; ROS_LOG(::ros::console::levels::Warn,  ROSCONSOLE_DEFAULT_NAME, __VA_ARGS__);}
#define MY_ROS_ERROR(...) {std::cout <<std::endl << "\033[36;46m" << "[]" << "\033[0m" << " "; ROS_LOG(::ros::console::levels::Error, ROSCONSOLE_DEFAULT_NAME, __VA_ARGS__);}
#define MY_ROS_FATAL(...) {std::cout <<std::endl << "\033[36;46m" << "[]" << "\033[0m" << " "; ROS_LOG(::ros::console::levels::Fatal, ROSCONSOLE_DEFAULT_NAME, __VA_ARGS__);}

//#include <geometry_msgs/TwistStamped.h>


namespace forgetful_drone
{

ForgetfulDrone::ForgetfulDrone(const ros::NodeHandle& RNH, const ros::NodeHandle& PNH)
    :
    m_ROSRNH(RNH),
    m_ROSPNH(PNH),
    m_ROSNodeName{ros::this_node::getName().c_str()},
    m_ROSNodePath{ros::package::getPath("forgetful_drones")},
    
    m_ROSSub_Odometry(m_ROSRNH.subscribe("odometry_sensor1/odometry", 1, &ForgetfulDrone::ROSCB_Odometry, this)),
    m_ROSSub_AutopilotFeedback(m_ROSRNH.subscribe("autopilot/feedback", 1, &ForgetfulDrone::ROSCB_AutopilotFeedback, this)),
    m_ROSSub_NetworkPrediction(m_ROSRNH.subscribe("network/prediction", 1, &ForgetfulDrone::ROSCB_NetworkPrediction, this)),
    m_ROSSub_FlightmareRGB(m_ROSRNH.subscribe("flightmare/rgb", 1, &ForgetfulDrone::ROSCB_RGBCameraImageRaw, this)),
    
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

    m_ROSPub_SimulatorStart( m_ROSRNH.advertise<std_msgs::Empty>("simulator/start", 1) ),
    m_ROSPub_SimulatorStop( m_ROSRNH.advertise<std_msgs::Empty>("simulator/stop", 1) ),

    m_ROSSrvCl_BuildDroneRacingSimulation( m_ROSRNH.serviceClient<forgetful_drones_msgs::BuildDroneRacingSimulation>("simulator/build_drone_racing_simulation") ),
    m_ROSSrvCl_RVIZLoadConfig( m_ROSRNH.serviceClient<rviz::SendFilePath>("rviz/load_config") ),



    m_AutopilotState(quadrotor_msgs::AutopilotFeedback::OFF),
    m_ORCFlightMission(),
    m_SIMRaceTrackType(),
    m_ExpertPrediction(),
    m_NetworkPrediciton(),
    m_GloTraj(),
    m_GT_WP(),
    m_GT_ExpertState_i( 0 ),
    m_Expert_MaxSpeed( 0 ),
    m_Expert_MinSpeed( 0 ),
    m_Trafo_WRF_DRF(),
    m_Trafo_WRF_ARF(),
    m_Trafo_ARF_DRF(),
    m_Trafo_ARF_RRF(),
    m_RefStatePos_WRF(),
    m_CurrWP_i( 0 ),
    m_LastWP_i( 0 ),
    m_Dist2CurrWP( 0 ),
    m_Dist2LastWP( 0 ),
    m_NavigatorInput(),

    m_NavigatorEnabled( false ),
    m_LocTraj_SubseqInfeasibleN( 0 ),
    m_LocTraj_StartTime(),
    m_LocTraj( Vec3(), Vec3(), Vec3(), Vec3() ),
    m_MainLoopIter_i( 0 ),
    m_LocTraj_FeasibleCompCount( 0 ),
    m_Run_i( -1 ),
    m_CamFrame_i( -1 ),
    m_Lap_i( -1 ),
    m_TrainDataSaverEnabled{false},


    // ROS Parameters
    m_SIMDynGatesEnabled(),
    m_ORCRunMaxCount(),
    m_DRONECamHalfYawAOV(),
    m_DRONECamHalfPitchAOV(),
    m_LTMaxSpeed(),
    m_ORCCurrWP_MaxFlightDuration(),
    m_GTPolyOrder(),
    m_GTContiOrder(),
    m_WUInitWaypointIdxForFIG8(),
    m_GTMinWeightVel(),
    m_GTMinWeightAcc(),
    m_GTMinWeightJerk(),
    m_GTMinWeightSnap(),
    m_GTMaxThrust(),
    m_GTMaxRollPitchRate(),
    m_GTMaxSpeed(),
    m_GTNonDimTemporalRange(),
    m_GTNonDimSpatialRange(),
    m_GTSamplingTime(),
    m_WUThresDist2DetectWaypointArrival(),
    m_EXPMinHorizon(),
    m_EXPSpeedHorizon(),
    m_NAVMaxDist2RefState(),
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
    m_ORCLapMaxCount(),
    m_RacetrackDataGeneration_ON(),
    m_Debug_ON()
{
    // --- Set logger level of this node to DEBUG ---
    if(ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) 
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
    tf::poseMsgToKindr(ARFPose_WRF, &m_Trafo_WRF_ARF);



    bool InitializationSuccessful = true;

        std::vector< std::pair<const char*, const bool*> > KeysAndBoolOutputs
        {
            {"SIM_dynamic_gates_enabled" , &m_SIMDynGatesEnabled},
            {"RVIZ_elapsed_LTs_enabled", &m_RVIZElapsedLTsEnabled},
            {"debug_on", &m_Debug_ON},
            {"racetrack_data_generation_on", &m_RacetrackDataGeneration_ON},
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
            {"ORC_duration_without_reaching_waypoint_to_abort_run", &m_ORCCurrWP_MaxFlightDuration},
            {"GT_velocity_minimization_weight", &m_GTMinWeightVel},
            {"GT_acceleration_minimization_weight", &m_GTMinWeightAcc},
            {"GT_jerk_minimization_weight", &m_GTMinWeightJerk},
            {"GT_snap_minimization_weight", &m_GTMinWeightSnap},
            {"GT_max_normalized_thrust", &m_GTMaxThrust},
            {"GT_max_roll_pitch_rate", &m_GTMaxRollPitchRate},
            {"GT_max_speed", &m_GTMaxSpeed}, 
            {"GT_max_non_dim_temporal_range", &m_GTNonDimTemporalRange},
            {"GT_max_non_dim_spatial_range", &m_GTNonDimSpatialRange},
            {"WU_threshold_distance_to_detect_arrival_at_waypoint", &m_WUThresDist2DetectWaypointArrival},
            {"EXP_min_horizon", &m_EXPMinHorizon},
            {"EXP_speed_horizon", &m_EXPSpeedHorizon},
            {"NAV_max_divergence_from_reference_state", &m_NAVMaxDist2RefState},
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
        
        if (!fetchROSParameters(m_ROSPNH, KeysAndBoolOutputs, KeysAndIntOutputs, KeysAndDoubleOutputs, KeysAndStringOutputs)) 
            InitializationSuccessful = false;

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
            MY_ROS_ERROR("[%s]    Invalid value of ROS parameter with key \"ORC_flight_mission\".", m_ROSNodeName);
            InitializationSuccessful = false;
        }

        if ( SIMRaceTrackType == std::string("deterministic_figure8") )
            m_SIMRaceTrackType = forgetful_drones_msgs::BuildDroneRacingSimulationRequest::FIGURE8_DETERMINISTIC;
        else if ( SIMRaceTrackType == std::string("randomized_figure8") )
            m_SIMRaceTrackType = forgetful_drones_msgs::BuildDroneRacingSimulationRequest::FIGURE8_RANDOMIZED;
        else if ( SIMRaceTrackType == std::string("randomized_circuit") )
            m_SIMRaceTrackType = forgetful_drones_msgs::BuildDroneRacingSimulationRequest::CIRCUIT_RANDOMIZED;
        else if ( SIMRaceTrackType == std::string("randomized_sprint") )
            m_SIMRaceTrackType = forgetful_drones_msgs::BuildDroneRacingSimulationRequest::SPRINT_RANDOMIZED;
        else
        {
            ROS_ERROR( "[%s]\n  >> Invalid value of ROS parameter with key \"SIM_race_track_type\".", 
                ros::this_node::getName().c_str() );
            InitializationSuccessful = false;
        }

        //const_cast<double&>(m_GTSamplingTime) = m_ORCMainLoopNomPeriodDuration * m_NAVMainLoopItersPerLTReplanning;
        const_cast<double&>(m_GTSamplingTime) = m_ORCMainLoopNomPeriodDuration;
        

        if ( m_ORCFlightMission == FlightMissions::testNavigatorWithExpert || m_ORCFlightMission == FlightMissions::generateTrainingData )
            if ( m_SIMRaceTrackType == forgetful_drones_msgs::BuildDroneRacingSimulationRequest::SPRINT_RANDOMIZED )
            {
                ROS_ERROR(
                    "[%s]\n  >> Flight missions \"Navigator-Expert Test\" and \"Training Data Generation\" are incompatible with race track type \"Randomized Sprint\"", 
                    ros::this_node::getName().c_str() );
                InitializationSuccessful = false;
            }
    
    if ( ! InitializationSuccessful )
    {
        MY_ROS_FATAL("[%s]    Initialization failed. Shut down ROS node.", m_ROSNodeName);
        ros::shutdown();
    }
    else 
        ROS_INFO("[%s]    Node initialized.", m_ROSNodeName);


    // --- Wait for the other nodes ---
    ros::Duration(2.0).sleep();

    
    if (m_Debug_ON)
    {
        std::cout << "\n\nDEBUG ON\n\n";
        std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> Waypoints = {
            { -20.45 ,   -8.65   ,   2.0  }, // #01
            //{ -12.55 ,   -11.15  ,   2.1  }, // #02
            //{ -4.15  ,   -5.35   ,   2.2  }, // #03
            { 3.45   ,   4.25    ,   2.3  }, // #04
            //{ 11.95  ,   11.15   ,   2.4  }, // #05
            //{ 21.85  ,   6.85    ,   2.5  }, // #06
            //{ 24.25  ,   -1.75   ,   2.6  }, // #07
            { 19.25  ,   -9.55   ,   2.7  }, // #08
            //{ 10.55  ,   -10.65  ,   2.6  }, // #09
            //{ 2.85   ,   -5.95   ,   2.5  }, // #10
            //{ -4.95  ,   4.65    ,   2.4  }, // #11
            { -12.95 ,   9.65    ,   2.3  }, // #12
            //{ -21.05 ,   6.65    ,   2.2  }, // #13
            //{ -24.25 ,   -2.15   ,   2.1  }  // #14
            };

        Waypoints = {
            { -20.45 ,   -8.65   ,   2.0  }, // #01
            { -12.55 ,   -11.15  ,   2.1  }, // #02
            { -4.15  ,   -5.35   ,   2.2  }, // #03
            { 3.45   ,   4.25    ,   2.3  }, // #04
            { 11.95  ,   11.15   ,   2.4  }, // #05
            { 21.85  ,   6.85    ,   2.5  }, // #06
            { 24.25  ,   -1.75   ,   2.6  }, // #07
            { 19.25  ,   -9.55   ,   2.7  }, // #08
            { 10.55  ,   -10.65  ,   2.6  }, // #09
            { 2.85   ,   -5.95   ,   2.5  }, // #10
            { -4.95  ,   4.65    ,   2.4  }, // #11
            { -12.95 ,   9.65    ,   2.3  }, // #12
            { -21.05 ,   6.65    ,   2.2  }, // #13
            { -24.25 ,   -2.15   ,   2.1  }  // #14
            };

        //std::vector<long double> SpatRanges {1, 2, 4, 8, 16, 32, 64, 128, 256, 512};
        //std::vector<long double> TempRanges {1, 2, 4, 8, 16, 32, 64, 128, 256, 512};
        ////std::vector<long double> TempRanges {1e-5, 1e-4, 1e-3, 1e-2, 1e-1, 1e0, 1e1, 1e2, 1e3, 1e4, 1e-5}; //m_GTNonDimTemporalRange
        ////std::vector<long double> SpatRanges {1e-5, 1e-4, 1e-3, 1e-2, 1e-1, 1e0, 1e1, 1e2, 1e3, 1e4, 1e-5}; //m_GTNonDimSpatialRange
        //for (int T_i = 0; T_i < TempRanges.size(); T_i++)
        //for (int W_j = 0; W_j < SpatRanges.size(); W_j++)
        //{
        //    ForgetfulGlobalTrajectory<long double>(
        //        Waypoints, 
        //        m_GTPolyOrder, m_GTContiOrder, m_GTMaxSpeed, m_GTMaxThrust, m_GTMaxRollPitchRate, m_GTSamplingTime,
        //        3, TempRanges[T_i], SpatRanges[W_j], false );
        //    ros::Duration(1.0).sleep();
        //}
        ForgetfulGlobalTrajectory<long double>(
                Waypoints, 
                m_GTPolyOrder, m_GTContiOrder, m_GTMaxSpeed, m_GTMaxThrust, m_GTMaxRollPitchRate, m_GTSamplingTime,
                3, m_GTNonDimTemporalRange, m_GTNonDimSpatialRange, false );
        ros::shutdown();
    }

    if (m_RacetrackDataGeneration_ON)
    {
        std::cout<<std::endl;
        ROS_INFO("[%s]\n  >> GENERATE RACETRACK DATA: Global Trajectories", ros::this_node::getName().c_str());

        ros::Duration(5.0).sleep();

        int UNITY_SCENES_N = 5;
        int UNITY_SITES_N = 3;
        int GATE_TYPES_N = 2;
        int REPEATED_CONDITIONS_N = 3;

        for (int SceneIdx = 0; SceneIdx < UNITY_SCENES_N; SceneIdx++)
        {
            for (int SiteIdx = 0; SiteIdx < UNITY_SITES_N; SiteIdx++)
            {
                for (int GateTypeIdx = 0; GateTypeIdx < GATE_TYPES_N; GateTypeIdx++)
                {
                    for (int RepCondIdx = 0; RepCondIdx < REPEATED_CONDITIONS_N; RepCondIdx++)
                    {
                        std::cout << "\n\tScene: " << SceneIdx <<", ";
                        std::cout << "Site: " << SiteIdx <<", ";
                        std::cout << "Gate Type: " << GateTypeIdx <<", ";
                        std::cout << "Repeated Conditions: " << RepCondIdx <<"\n";



                        
                        
                        std::string InOutDirPath 
                            = ros::package::getPath("forgetful_drones") + "/racetracks/" 
                            + std::to_string(SceneIdx) + "/"
                            + std::to_string(SiteIdx) + "/"
                            + std::to_string(GateTypeIdx) + "/"
                            + std::to_string(RepCondIdx);

                        std::string InFilePath = InOutDirPath +"/waypoints.txt";



                        m_GT_WP.clear();

                        std::ifstream input_file( InFilePath );
                        std::string line;
                        while (std::getline(input_file, line))
                        {
                            std::istringstream iss(line);
                            double x, y, z;
                            if (!(iss >> x >> y >> z)) { break; }

                            m_GT_WP.push_back({x, y, z});
                        }




                        // COmpute Traj
                        ros::Duration(5.0).sleep();
                        precomputeGloTrajForExpert();
                        

                        std::string OutFilePath = InOutDirPath +"/global_trajectory.txt";


                        std::ofstream OutFile(OutFilePath);
                        for (const quadrotor_common::TrajectoryPoint& TrajPoint : m_GloTraj)
                        {
                            OutFile 
                                << TrajPoint.position.x() << " " 
                                << TrajPoint.position.y() << " " 
                                << TrajPoint.position.z() << " " 
                                << TrajPoint.velocity.x() << " "
                                << TrajPoint.velocity.y() << " "
                                << TrajPoint.velocity.z() << "\n";
                        }
                    }
                }
            }
        }

        ros::shutdown();
    }
    
    
    switch ( m_ORCFlightMission )
    {
        case FlightMissions::generateTrainingData: generateTrainingData(); break;
        case FlightMissions::testNavigatorWithNetwork: testNavigatorWithNetwork(); break;
        case FlightMissions::testNavigatorWithExpert: testNavigatorWithExpert(); break;
        case FlightMissions::testSimulatorWithAutopilot: testSimulatorWithAutopilot(); break;
        default: MY_ROS_ERROR("[%s]    Specified flight mission not implemented.", m_ROSNodeName); break;
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
void ForgetfulDrone::ROSCB_Odometry(const nav_msgs::OdometryConstPtr& msg)
{
    // Transformation: drone reference frame -> state estimate reference frame.
    tf::poseMsgToKindr(
        GeometryMsgsPose_From_NavMsgsOdometry(*msg), 
        &m_Trafo_ARF_DRF
        );

    // Transformation: drone reference frame -> world reference frame.
    m_Trafo_WRF_DRF = m_Trafo_WRF_ARF * m_Trafo_ARF_DRF;


    const geometry_msgs::Vector3& EstimatedVelocity = msg->twist.twist.linear;
    m_EstimatedNormalizedSpeed = sqrt(pow(EstimatedVelocity.x, 2) + pow(EstimatedVelocity.y, 2) + pow(EstimatedVelocity.z, 2)) / m_LTMaxSpeed;
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
    srv.request.path.data = m_ROSNodePath + "/rviz/default.rviz";

    if (!m_ROSSrvCl_RVIZLoadConfig.call(srv))
        MY_ROS_WARN("[%s]    Failed to reset RViz.", m_ROSNodeName);
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////// FLIGHT MISSIONS //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void ForgetfulDrone::launchDroneOffGround()
{
    MY_ROS_INFO("[%s]    Launch drone off ground.", m_ROSNodeName);

    m_ROSPub_AutopilotOff.publish(std_msgs::Empty());

        std_msgs::Bool msg; 
        msg.data = true;
    m_ROSPub_BridgeArm.publish(msg);

    m_ROSPub_AutopilotStart.publish(std_msgs::Empty());

    ros::Rate Rate(100.0);
    while (m_AutopilotState != quadrotor_msgs::AutopilotFeedback::BREAKING)
        { ros::spinOnce(); Rate.sleep(); }
    while ( m_AutopilotState != quadrotor_msgs::AutopilotFeedback::HOVER )
        { ros::spinOnce(); Rate.sleep(); }
    
    for (int i = 0; i < 10; i++)
    {
        ros::Duration(1.0).sleep();
    }
    
}


void ForgetfulDrone::flyDroneToInitPose()
{
    MY_ROS_INFO("[%s]    Fly drone to start position.", m_ROSNodeName);

    ros::Rate Rate(100.0);
    while (m_AutopilotState != quadrotor_msgs::AutopilotFeedback::HOVER)
        { ros::spinOnce(); Rate.sleep(); }


        
        
        geometry_msgs::PoseStamped msg;
        msg.pose.position = GeometryMsgsPoint_From_EigenVector3d(m_Trafo_ARF_DRF.getPosition());
        msg.pose.position.z += 5.0;
        msg.pose.orientation = GeometryMsgsQuaternion_From_EigenQuaterniond(m_Trafo_ARF_DRF.getEigenQuaternion());
    m_ROSPub_AutopilotPoseCommand.publish(msg);

    while (m_AutopilotState == quadrotor_msgs::AutopilotFeedback::HOVER)
        { ros::spinOnce(); Rate.sleep(); }
    while (m_AutopilotState != quadrotor_msgs::AutopilotFeedback::HOVER)
        { ros::spinOnce(); Rate.sleep(); }

    ////////
        msg.pose = m_Drone_InitPose;
        msg.pose.position.z += 5.0;
    m_ROSPub_AutopilotPoseCommand.publish(msg);

    while (m_AutopilotState == quadrotor_msgs::AutopilotFeedback::HOVER)
        { ros::spinOnce(); Rate.sleep(); }
    while (m_AutopilotState != quadrotor_msgs::AutopilotFeedback::HOVER)
        { ros::spinOnce(); Rate.sleep(); }

        msg.pose = m_Drone_InitPose;
        msg.pose.position.z += 4.5;
    m_ROSPub_AutopilotPoseCommand.publish(msg);

    while (m_AutopilotState == quadrotor_msgs::AutopilotFeedback::HOVER)
        { ros::spinOnce(); Rate.sleep(); }
    while (m_AutopilotState != quadrotor_msgs::AutopilotFeedback::HOVER)
        { ros::spinOnce(); Rate.sleep(); }
    ///////

    ros::Duration( 1.0 ).sleep();
}

void ForgetfulDrone::flyDroneBetweenLastAndSecLastGate()
{
    MY_ROS_INFO("[%s] Fly drone between last and second last gate.", m_ROSNodeName);

    ros::Rate Rate(100.0);
    while (m_AutopilotState != quadrotor_msgs::AutopilotFeedback::HOVER)
        { ros::spinOnce(); Rate.sleep(); }
    

        geometry_msgs::PoseStamped msg;
        msg.pose.position = GeometryMsgsPoint_From_EigenVector3d(
                m_GT_WP.end()[-2] + (m_GT_WP.end()[-1] - m_GT_WP.end()[-2]) / 2);
        msg.pose.orientation = m_Drone_InitPose.orientation;
    m_ROSPub_AutopilotPoseCommand.publish(msg);


    while (m_AutopilotState == quadrotor_msgs::AutopilotFeedback::HOVER)
        { ros::spinOnce(); Rate.sleep(); }
    while (m_AutopilotState != quadrotor_msgs::AutopilotFeedback::HOVER)
        { ros::spinOnce(); Rate.sleep(); }

    ros::Duration( 1.0 ).sleep();
}


void ForgetfulDrone::landDroneOnGround()
{
    MY_ROS_INFO("[%s]    Land drone on ground", m_ROSNodeName);

    m_ROSPub_AutopilotLand.publish(std_msgs::Empty());
    
    ros::Rate Rate(100.0);
    while (m_AutopilotState != quadrotor_msgs::AutopilotFeedback::OFF)
        { ros::spinOnce(); Rate.sleep(); }

        std_msgs::Bool msg; 
        msg.data = false;
    m_ROSPub_BridgeArm.publish(msg);

    ros::Duration(1.0).sleep();
}






void ForgetfulDrone::testNavigatorWithExpert()
{
    MY_ROS_INFO("[%s]    Starting Flight Mission: Navigator-Expert Test.", m_ROSNodeName);

    m_ROSTimer_MainLoop = m_ROSRNH.createTimer(
        ros::Duration(m_ORCMainLoopNomPeriodDuration), 
        &ForgetfulDrone::ROSTimerFunc_testNavigatorWithExpert, 
        this, false, false);

    for ( m_Run_i = 0; m_Run_i < m_ORCRunMaxCount; m_Run_i++ )
    { 
        m_LocTraj_FeasibleCompCount = 0;
        m_Lap_i = -1;

        MY_ROS_INFO("[%s]    Testing Navigator with Expert: Starting Run #%d.", m_ROSNodeName, m_Run_i );



        do
        {
            buildDroneRacingSimulation();
            
        } while ( !computeGloTrajForExpert() );

        launchDroneOffGround();
        
        m_ROSTimer_MainLoop.start();
        
        switchNavigator( true );

        std::thread t1(runMultiThreadedSpinner);

        m_Lap_i = -1;
        ros::Rate Rate( 100.0 );
        do
        {
            Rate.sleep();
        } while ( m_Lap_i < m_ORCLapMaxCount && m_NavigatorEnabled );

        
        if ( m_NavigatorEnabled )
        {
            MY_ROS_INFO( "[%s]\n >>  Testing Navigator with Expert: Run #%d successfully completed.", 
                    m_ROSNodeName, m_Run_i );
            switchNavigator( false );
        }
        else
            MY_ROS_WARN( "[%s]\n >>  Testing Navigator with Expert: Run #%d aborted ahead of schedule.", 
                    m_ROSNodeName, m_Run_i );


        m_ROSTimer_MainLoop.stop();
        t1.detach();
        t1.~thread();


        landDroneOnGround();
    }
}






void ForgetfulDrone::generateTrainingData()
{
    MY_ROS_INFO("[%s]    FlIGHT MISSION: Training Data Generation", m_ROSNodeName);

    m_ROSTimer_MainLoop = m_ROSRNH.createTimer(
        ros::Duration(m_ORCMainLoopNomPeriodDuration), 
        &ForgetfulDrone::ROSTimerFunc_generateTrainingData,
        this, false, false);

    // Create directory where generated training data is saved
    m_TrainingDataDirPath = m_ROSNodePath + "/training_data/" + getCurrUTCDateTimeAsString();
    std::experimental::filesystem::create_directory(m_TrainingDataDirPath);

    // Record failed runs
    std::string FailedRuns = "";
    int FailedRuns_N = 0;

    // All variations of simulation environments
    const std::array<forgetful_drones_msgs::BuildDroneRacingSimulation::Request::_UnityScene_type, 5> 
        SCENES = {
            forgetful_drones_msgs::BuildDroneRacingSimulation::Request::SPACESHIP_INTERIOR,
            forgetful_drones_msgs::BuildDroneRacingSimulation::Request::DESTROYED_CITY,
            forgetful_drones_msgs::BuildDroneRacingSimulation::Request::INDUSTRIAL_PARK,
            forgetful_drones_msgs::BuildDroneRacingSimulation::Request::POLYGON_CITY,
            forgetful_drones_msgs::BuildDroneRacingSimulation::Request::DESERT_MOUNTAIN };
    const std::array<forgetful_drones_msgs::BuildDroneRacingSimulation::Request::_RacetrackSite_type, 3> 
        SITES = {
            forgetful_drones_msgs::BuildDroneRacingSimulation::Request::SITE_A,
            forgetful_drones_msgs::BuildDroneRacingSimulation::Request::SITE_B,
            forgetful_drones_msgs::BuildDroneRacingSimulation::Request::SITE_C };
    const std::array<forgetful_drones_msgs::BuildDroneRacingSimulation::Request::_RaceGateType_type, 2> 
        GATE_TYPES = {
            forgetful_drones_msgs::BuildDroneRacingSimulation::Request::TUB_DAI_GATE,
            forgetful_drones_msgs::BuildDroneRacingSimulation::Request::THU_DME_GATE };
    
    const int SCENES_N = SCENES.size();
    const int SITES_N = SITES.size(); 
    const int GATE_TYPES_N = GATE_TYPES.size();
    const int REPEATED_RUNS_N = 3;
    const int RUNS_N = SCENES_N * SITES_N * GATE_TYPES_N * REPEATED_RUNS_N;

    m_Run_i = - 1;

    for (int Scene_i = 1; Scene_i < SCENES_N; Scene_i++)
    {
        for (int Site_i = 0; Site_i < SITES_N; Site_i++)
        {
            for (int GateType_i = 0; GateType_i < GATE_TYPES_N; GateType_i++)
            {
                for (int RepRun_i = 0; RepRun_i < REPEATED_RUNS_N; RepRun_i++)
                {
                    m_Run_i++;

                    m_Lap_i = -1;
                    m_MainLoopIter_i = 0;

                    m_LocTraj_SubseqInfeasibleN = 0;
                    m_LocTraj_FeasibleCompCount = 0;
                    m_LocTraj = { Vec3(), Vec3(), Vec3(), Vec3() };
                    m_LocTraj_StartTime = {};
                    
                    resetRVIZ();

                    MY_ROS_INFO("[%s]    Start Run %d/%d: Scene %d/%d - Site %d/%d - Gate Type %d/%d - Repeated Run %d/%d", 
                        m_ROSNodeName, m_Run_i, RUNS_N, Scene_i, SCENES_N, Site_i, SITES_N, GateType_i, GATE_TYPES_N, RepRun_i, REPEATED_RUNS_N);

                    std::string RunDirName = "run_" 
                        + std::to_string(Scene_i) + "-" 
                        + std::to_string(Site_i) + "-" 
                        + std::to_string(GateType_i) + "-" 
                        + std::to_string(RepRun_i);
                    m_RunDirPath = m_TrainingDataDirPath + "/" + RunDirName;
                    std::experimental::filesystem::create_directory( m_RunDirPath );
                    std::experimental::filesystem::create_directory( m_RunDirPath + "/images" );
                    m_CamFrame_i = 0;

                    


                    do
                    {
                        m_ROSPub_SimulatorStop.publish(std_msgs::Empty());

                        forgetful_drones_msgs::BuildDroneRacingSimulation srv;
                        srv.request.RacetrackType = srv.request.FIGURE8_RANDOMIZED;
                        srv.request.UnityScene = SCENES[Scene_i];
                        srv.request.RacetrackSite = SITES[Site_i];
                        srv.request.RaceGateType = GATE_TYPES[GateType_i];
                        
                        if (!m_ROSSrvCl_BuildDroneRacingSimulation.call(srv))
                            MY_ROS_ERROR("[%s]    Failed to call service [%s].", m_ROSNodeName, m_ROSSrvCl_BuildDroneRacingSimulation.getService().c_str())
                        else
                        {
                            m_Gates_WaypointPose = srv.response.GatesInitPose;
                            m_Drone_InitPose = srv.response.DroneInitPose;
                        }

                        m_ROSPub_SimulatorStart.publish(std_msgs::Empty());
                            ros::Duration(5.0).sleep();

                    } while (computeGloTrajForExpert());
                    
                    


                    //if (m_Run_i != 0)
                    //{
                    //    flyDroneToInitPose();
                    //    landDroneOnGround();
                    //}
                    //launchDroneOffGround();

                    if (m_Run_i == 0) launchDroneOffGround();
                    flyDroneToInitPose();




                    m_ROSTimer_MainLoop.start();
                    std::thread t1(runMultiThreadedSpinner);

                    ros::Duration(2.0).sleep();
                    switchNavigator(true);


                    m_LastWP_ArrivalTime = ros::Time::now();
                    ros::Rate Rate(10.0);
                    do
                    {
                        Rate.sleep();

                        ros::Duration CurrWP_FlightDuration = ros::Time::now() - m_LastWP_ArrivalTime;
                        if (CurrWP_FlightDuration.toSec() > m_ORCCurrWP_MaxFlightDuration)
                        {
                            MY_ROS_WARN("[%s]    No Gate passed within %f s. Abort run #%d ahead of schedule.", m_ROSNodeName, m_ORCCurrWP_MaxFlightDuration, m_Run_i);
                            break;
                        }
                    } while (m_Lap_i < m_ORCLapMaxCount && m_NavigatorEnabled);
                
                    m_TrainDataSaverEnabled = false;
                    if (m_NavigatorEnabled)
                    {
                        MY_ROS_INFO("[%s]    Run #%d completed as scheduled.", m_ROSNodeName, m_Run_i );
                        switchNavigator(false);
                    }
                    else
                    {
                        MY_ROS_WARN("[%s]    Run #%d aborted ahead of schedule.", m_ROSNodeName, m_Run_i);
                        markRunTrainingDataAsFailed();
                        
                        if (FailedRuns_N != 0) FailedRuns += ", ";
                        FailedRuns = std::to_string(m_Run_i);

                        FailedRuns_N++;
                    }

                    ros::Duration( 1.0 ).sleep();
                    m_ROSTimer_MainLoop.stop();
                    t1.detach();
                    t1.~thread();

                    
                    flyDroneBetweenLastAndSecLastGate();
                }
            }
        }
    }

    landDroneOnGround();

    MY_ROS_INFO("[%s]    FlIGHT MISSION COMPLETED.", m_ROSNodeName);

    if (!FailedRuns_N == 0)
        MY_ROS_WARN("[%s]    Failed (%d of %d) runs: %s", 
            m_ROSNodeName, FailedRuns_N, RUNS_N, FailedRuns.c_str())
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
ForgetfulDrone::ROSTimerFunc_generateTrainingData(
    const ros::TimerEvent& TimerEvent
){
    const double LastIterDuration = TimerEvent.profile.last_duration.toSec();
    if (LastIterDuration > m_ORCMainLoopNomPeriodDuration)
        MY_ROS_WARN("[%s]    Last main loop iteration took %f s exceeding the nominal duration of %f s.",
            m_ROSNodeName, LastIterDuration, m_ORCMainLoopNomPeriodDuration);


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
        srv.request.RacetrackType = m_SIMRaceTrackType;


    if ( ! m_ROSSrvCl_BuildDroneRacingSimulation.call( srv ) )
        ROS_ERROR( 
            "[%s]\n  >> Failed to call service \"%s\".", 
            ros::this_node::getName().c_str(),
            m_ROSSrvCl_BuildDroneRacingSimulation.getService().c_str()
            );
    else
    {
        m_Gates_WaypointPose = srv.response.GatesInitPose;
        m_Drone_InitPose = srv.response.DroneInitPose;
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





bool ForgetfulDrone::computeGloTrajForExpert()
{
    // --- Set up ´m_GT_WP´ ---
    m_GT_WP.clear();
    m_GT_WP.reserve( m_Gates_WaypointPose.size() );
    for ( const geometry_msgs::Pose& Pose : m_Gates_WaypointPose )
        m_GT_WP.push_back( EigenVector3d_From_GeometryMsgsPoint(Pose.position) );



    // --- Set ´m_Expert_MaxHorizon´ <- max dist between waypoints ---
    m_Expert_MaxHorizon = 0.0;
    size_t LastWaypointIdx = m_GT_WP.size() - 1;
    for ( size_t WaypointIdx = 0; WaypointIdx < m_GT_WP.size(); WaypointIdx++ )
    {
        m_Expert_MaxHorizon = std::max(
            m_Expert_MaxHorizon,
            (m_GT_WP[ WaypointIdx ] - m_GT_WP[ LastWaypointIdx ]).norm()
            );
        LastWaypointIdx = WaypointIdx;
    }

    MY_ROS_INFO("[%s]    Compute global trajectory:  #waypoints-%d  polynomial order-%d  continuity order-%d  max. speed-%f  max. thrust-%f  max. roll-pitch rate-%f  sampling time-%f",
        m_ROSNodeName, static_cast<int>(m_GT_WP.size()), m_GTPolyOrder, m_GTContiOrder, m_GTMaxSpeed, m_GTMaxThrust, m_GTMaxRollPitchRate, m_GTSamplingTime);

    //ForgetfulGlobalTrajectory<long double> GlobalTrajectory = ForgetfulGlobalTrajectory<long double>(
    //    m_GT_WP, m_GTPolyOrder, m_GTContiOrder, m_GTMaxSpeed, m_GTMaxThrust, m_GTMaxRollPitchRate, m_GTSamplingTime,
    //    3, m_GTNonDimTemporalRange, m_GTNonDimSpatialRange, false);
    std::shared_ptr<ForgetfulGlobalTrajectory<long double>> GT_Ptr
        = std::make_shared<ForgetfulGlobalTrajectory<long double>>(m_GT_WP, m_GTPolyOrder, m_GTContiOrder, m_GTMaxSpeed, m_GTMaxThrust, m_GTMaxRollPitchRate, m_GTSamplingTime,
            3, m_GTNonDimTemporalRange, m_GTNonDimSpatialRange, false);
    

    m_GloTraj.clear();
    //for (int i = 0; i < GlobalTrajectory.m_Traj_T_Dim.size(); i++)
    //{
    //    quadrotor_common::TrajectoryPoint TrajectoryPoint;
    //    TrajectoryPoint.position.x() = static_cast<double>(GlobalTrajectory.m_Traj_PosX_Dim(i));
    //    TrajectoryPoint.position.y() = static_cast<double>(GlobalTrajectory.m_Traj_PosY_Dim(i));
    //    TrajectoryPoint.position.z() = static_cast<double>(GlobalTrajectory.m_Traj_PosZ_Dim(i));
    //
    //    TrajectoryPoint.velocity.x() = static_cast<double>(GlobalTrajectory.m_Traj_VelX_Dim(i));
    //    TrajectoryPoint.velocity.y() = static_cast<double>(GlobalTrajectory.m_Traj_VelY_Dim(i));
    //    TrajectoryPoint.velocity.z() = static_cast<double>(GlobalTrajectory.m_Traj_VelZ_Dim(i));
    //
    //    m_GloTraj.push_back(TrajectoryPoint);
    //}
    for (int i = 0; i < GT_Ptr->m_Traj_T_Dim.size(); i++)
    {
        quadrotor_common::TrajectoryPoint TrajectoryPoint;
        TrajectoryPoint.position.x() = static_cast<double>(GT_Ptr->m_Traj_PosX_Dim(i));
        TrajectoryPoint.position.y() = static_cast<double>(GT_Ptr->m_Traj_PosY_Dim(i));
        TrajectoryPoint.position.z() = static_cast<double>(GT_Ptr->m_Traj_PosZ_Dim(i));

        TrajectoryPoint.velocity.x() = static_cast<double>(GT_Ptr->m_Traj_VelX_Dim(i));
        TrajectoryPoint.velocity.y() = static_cast<double>(GT_Ptr->m_Traj_VelY_Dim(i));
        TrajectoryPoint.velocity.z() = static_cast<double>(GT_Ptr->m_Traj_VelZ_Dim(i));

        m_GloTraj.push_back(TrajectoryPoint);
    }

    // Find the maximum and minimum velocity of the global trajectory.
    m_Expert_MaxSpeed = 0.0;
    m_Expert_MinSpeed = std::numeric_limits< double >::max();
    for ( size_t StateIdx = 0; StateIdx < m_GloTraj.size(); StateIdx++ )
    {
        double Speed = ( m_GloTraj[ StateIdx ].velocity ).norm();
        m_Expert_MaxSpeed = std::max( m_Expert_MaxSpeed, Speed );
        m_Expert_MinSpeed = std::min( m_Expert_MinSpeed, Speed );
    }



    


    // --- Set up Settings of GT ---
    //polynomial_trajectories::PolynomialTrajectorySettings GTSettings;
    //GTSettings.way_points = {
    //    std::make_move_iterator( m_GT_WP.begin() ),
    //    std::make_move_iterator( m_GT_WP.end() )
    //    };
    //GTSettings.polynomial_order = m_GTPolyOrder;
    //GTSettings.continuity_order = m_GTContiOrder;
    //GTSettings.minimization_weights = Eigen::Vector4d{
    //    m_GTMinWeightVel,
    //    m_GTMinWeightAcc,
    //    m_GTMinWeightJerk,
    //    m_GTMinWeightSnap
    //    };
    

    // --- Compute initial segment times (first element relates to segment from last to first waypoint) ---
    //Eigen::VectorXd GTInitSegmentTimes = Eigen::VectorXd::Ones( m_GT_WP.size() );
    //Eigen::Vector3d SegmentStart = m_GT_WP.back();
    //for ( size_t WaypointIdx = 0; WaypointIdx < m_GT_WP.size(); WaypointIdx++ ) 
    //{
    //    GTInitSegmentTimes[ WaypointIdx ] 
    //        = ( m_GT_WP[ WaypointIdx ] - SegmentStart ).norm() 
    //            / m_GTMaxSpeed;
    //    
    //    SegmentStart = m_GT_WP[ WaypointIdx ];
    //}


    // --- Compute global trajectories until one is speed feasible ---
    //bool GloTraj_SpeedFeasible = false;
    //bool GloTraj_ComputationCompleted = false;
    //int GloTraj_SubseqInfeasibleN = -1;
    //int GloTraj_SubseqInfeasibleMaxN = 10;
    //m_GT_CompFailed = false;
    //while ( ! GloTraj_SpeedFeasible || ! GloTraj_ComputationCompleted )
    //{
    //    GloTraj_SubseqInfeasibleN++;
    //    ROS_DEBUG("[%s]\n  >> %d subsequently failed GT comps.", ros::this_node::getName().c_str());
    //    
    //    if ( GloTraj_SubseqInfeasibleN == GloTraj_SubseqInfeasibleMaxN )
    //    {
    //        m_GT_CompFailed = true;
    //        ROS_WARN( "[%s]\n  >> Maximum number (%d) of failed global trajectory computations reached.\n  >> Re-building drone racing simulation...",
    //            ros::this_node::getName().c_str(), GloTraj_SubseqInfeasibleMaxN);
    //        return;
    //    } 
//
    //    GloTraj_SpeedFeasible = false;
    //    GloTraj_ComputationCompleted = false;
//
    //    std::atomic<bool> ThreadFinished(false);
    //    quadrotor_common::Trajectory GlobalTraj;
    //    std::promise< quadrotor_common::Trajectory > Promise;
    //    auto Future = Promise.get_future();
//
    //    auto compGloTraj = [this, &ThreadFinished](
    //        const Eigen::VectorXd& GTInitSegmentTimes, 
    //        const polynomial_trajectories::PolynomialTrajectorySettings& GTSettings,
    //        std::promise< quadrotor_common::Trajectory > && Promise)
    //    {
    //        quadrotor_common::Trajectory GlobalTraj 
    //            = trajectory_generation_helper::polynomials::generateMinimumSnapRingTrajectoryWithSegmentRefinement(
    //                GTInitSegmentTimes,
    //                GTSettings,
    //                m_GTMaxSpeed,
    //                m_GTMaxThrust,
    //                m_GTMaxRollPitchRate,
    //                1/m_GTSamplingTime
    //                );
    //    
    //        Promise.set_value(GlobalTraj);
    //    
    //        ThreadFinished = true;
    //        ROS_DEBUG("[%s]\n  >> Thread finished.", ros::this_node::getName().c_str());
    //    };
//
    //    std::thread t1( compGloTraj, std::ref(GTInitSegmentTimes), std::ref(GTSettings),  std::move(Promise) );
    //    ros::WallTime ThreadStartTime = ros::WallTime::now();
    //    ros::WallTime ThreadMaxDuration = ros::WallTime( 10.0 );
    //    ros::WallDuration ThreadDuration;
    //    ros::WallRate Rate(1.0);
    //    do
    //    {
    //        if (ThreadFinished) break;
//
    //        Rate.sleep();
    //        ThreadDuration = ros::WallTime::now() - ThreadStartTime;
    //        ROS_DEBUG("[%s]\n  >> Thread to compute global trajectory has been running for %f/%f s.",
    //                ros::this_node::getName().c_str(), 
    //                ThreadDuration.toSec(), ThreadMaxDuration.toSec());
//
    //    } while ( ThreadDuration.toSec() < ThreadMaxDuration.toSec() );
    //    
    //    if ( ThreadFinished )
    //    {
    //        t1.join();
    //        t1.~thread();
    //        GlobalTraj = Future.get();
    //    }
    //    else
    //    {
    //        ROS_DEBUG("[%s]\n  >> Thread to compute global trajectory has timed out.",
    //                ros::this_node::getName().c_str());
    //        
    //        t1.detach();
    //        t1.~thread();
//
    //        continue;
    //    }
    //    
    //    m_GloTraj.clear();
    //    m_GloTraj = { 
    //        std::make_move_iterator( GlobalTraj.points.begin() ),
    //        std::make_move_iterator( GlobalTraj.points.end() ),
    //        };
    //    
//
//
    //    // Find the maximum and minimum velocity of the global trajectory.
    //    m_Expert_MaxSpeed = 0.0;
    //    m_Expert_MinSpeed = std::numeric_limits< double >::max();
    //    for ( size_t StateIdx = 0; StateIdx < m_GloTraj.size(); StateIdx++ )
    //    {
    //        double Speed = ( m_GloTraj[ StateIdx ].velocity ).norm();
    //        m_Expert_MaxSpeed = std::max( m_Expert_MaxSpeed, Speed );
    //        m_Expert_MinSpeed = std::min( m_Expert_MinSpeed, Speed );
    //    }
//
    //    if ( m_Expert_MaxSpeed > 1.1 * m_GTMaxSpeed )
    //    {
    //        ROS_WARN(
    //            "[%s]\n  >> Maximum Speed of computed global trajectory exceeds 110 %% of nominal value (%f/%f m/s)."
    //            "\n     Re-computing global trajectory...",
    //            ros::this_node::getName().c_str(), m_Expert_MaxSpeed, m_GTMaxSpeed
    //            );
//
    //        continue;
    //    }
    //    else
    //        GloTraj_SpeedFeasible = true;
//
    //    if ( GlobalTraj.trajectory_type == quadrotor_common::Trajectory::TrajectoryType::UNDEFINED )
    //    {
    //        ROS_WARN(
    //            "[%s]\n  >> Computation of global trajectory not successful."
    //            "\n     Re-computing global trajectory....",
    //            ros::this_node::getName().c_str()
    //            );
//
    //        continue;
    //    }
    //    else
    //        GloTraj_ComputationCompleted = true;
    //}

    //for (Eigen::Vector3d& Waypoint : m_GT_WP)
    //{
    //    Waypoint += MeanWaypoint;
    //}
    //for (quadrotor_common::TrajectoryPoint& TrajPoint : m_GloTraj)
    //{
    //    TrajPoint.position += MeanWaypoint;
    //}

    //MY_ROS_INFO("[%s] Computed global trajectory:  #states-%d  duration-%1.1fs  speed range-[%1.1f, %1.1f]m/s  thrust range-[%1.1f, %1.1f]m/s^2  roll-pitch-rate range-[%1.1f, %1.1f]1/s",            
    //    m_ROSNodeName, static_cast<int>(m_GloTraj.size()), static_cast<int>(m_GloTraj.size()) * m_GTSamplingTime,
    //    static_cast<double>(GlobalTrajectory.m_Traj_MinSpeed), static_cast<double>(GlobalTrajectory.m_Traj_MaxSpeed),
    //    static_cast<double>(GlobalTrajectory.m_Traj_MinThrust), static_cast<double>(GlobalTrajectory.m_Traj_MaxThrust),
    //    static_cast<double>(GlobalTrajectory.m_Traj_MinRollPitchRate), static_cast<double>(GlobalTrajectory.m_Traj_MaxRollPitchRate));
    MY_ROS_INFO("[%s] Computed global trajectory:  #states-%d  duration-%1.1fs  speed range-[%1.1f, %1.1f]m/s  thrust range-[%1.1f, %1.1f]m/s^2  roll-pitch-rate range-[%1.1f, %1.1f]1/s",            
        m_ROSNodeName, static_cast<int>(m_GloTraj.size()), static_cast<int>(m_GloTraj.size()) * m_GTSamplingTime,
        static_cast<double>(GT_Ptr->m_Traj_MinSpeed), static_cast<double>(GT_Ptr->m_Traj_MaxSpeed),
        static_cast<double>(GT_Ptr->m_Traj_MinThrust), static_cast<double>(GT_Ptr->m_Traj_MaxThrust),
        static_cast<double>(GT_Ptr->m_Traj_MinRollPitchRate), static_cast<double>(GT_Ptr->m_Traj_MaxRollPitchRate));


    rvizGloTraj();

    // --- Set Waypoint Idx for deterministic 8 RaceTrack 
    m_CurrWP_i = m_WUInitWaypointIdxForFIG8;
    m_LastWP_i = (m_CurrWP_i + m_GT_WP.size() - 1) % m_GT_WP.size();

    //return GlobalTrajectory.m_Successful;
    return GT_Ptr->m_Successful;
}




void ForgetfulDrone::precomputeGloTrajForExpert()
{

    //ros::Duration(5.0).sleep();



    // --- Normalize waypoints around origin ---
    Eigen::Vector3d MeanWaypoint{0, 0, 0};
    for (const Eigen::Vector3d& Waypoint : m_GT_WP) MeanWaypoint += Waypoint /m_GT_WP.size();
    for (Eigen::Vector3d& Waypoint : m_GT_WP) Waypoint -= MeanWaypoint;

    //Eigen::Vector3d MinAxVals{0, 0, 0};
    //for (const Eigen::Vector3d& Waypoint : m_GT_WP)
    //{
    //    MinAxVals.x() = std::min(MinAxVals.x(), Waypoint.x());
    //    MinAxVals.y() = std::min(MinAxVals.y(), Waypoint.y());
    //    MinAxVals.z() = std::min(MinAxVals.z(), Waypoint.z());
    //}
    //for (Eigen::Vector3d& Waypoint : m_GT_WP) Waypoint -= MinAxVals;




    std::cout<<std::endl;
    for (const Eigen::Vector3d& Waypoint : m_GT_WP)
    {
        std::cout << "\t" << Waypoint.x() << "   \t" << Waypoint.y() << "   \t" << Waypoint.z() << std::endl;
    }
    std::cout<<std::endl;



    //// --- Set ´m_Expert_MaxHorizon´ <- max dist between waypoints ---
    //m_Expert_MaxHorizon = 0.0;
    //size_t LastWaypointIdx = m_GT_WP.size() - 1;
    //for (size_t WaypointIdx = 0; WaypointIdx < m_GT_WP.size(); WaypointIdx++)
    //{
    //    m_Expert_MaxHorizon = std::max( 
    //        m_Expert_MaxHorizon,
    //        (m_GT_WP[ WaypointIdx ] - m_GT_WP[ LastWaypointIdx ]).norm() );
    //    LastWaypointIdx = WaypointIdx;
    //}

    /*    ROS_INFO(
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
        static_cast<int>( m_GT_WP.size() ),
        m_GTPolyOrder,
        m_GTContiOrder,
        m_GTMinWeightVel,
        m_GTMinWeightAcc,
        m_GTMinWeightJerk,
        m_GTMinWeightSnap,
        m_GTMaxSpeed,
        m_GTMaxThrust,
        m_GTMaxRollPitchRate,
        m_GTSamplingTime
        );
    */


    // --- Set up Settings of GT ---
    polynomial_trajectories::PolynomialTrajectorySettings GTSettings;
    GTSettings.way_points = {std::make_move_iterator(m_GT_WP.begin()), std::make_move_iterator(m_GT_WP.end())};
    GTSettings.polynomial_order = m_GTPolyOrder;
    GTSettings.continuity_order = m_GTContiOrder;
    GTSettings.minimization_weights = Eigen::Vector4d{m_GTMinWeightVel, m_GTMinWeightAcc, m_GTMinWeightJerk, m_GTMinWeightSnap};
    

    // --- Compute initial segment times (first element relates to segment from last to first waypoint) ---
    Eigen::VectorXd GTInitSegmentTimes = Eigen::VectorXd::Ones(m_GT_WP.size());
    Eigen::Vector3d SegmentStart = m_GT_WP.back();
    for (size_t WaypointIdx = 0; WaypointIdx < m_GT_WP.size(); WaypointIdx++) 
    {
        GTInitSegmentTimes[WaypointIdx] = (m_GT_WP[ WaypointIdx ] - SegmentStart).norm() /m_GTMaxSpeed;
        SegmentStart = m_GT_WP[WaypointIdx];
    }


    
    quadrotor_common::Trajectory GlobalTraj 
        = trajectory_generation_helper::polynomials::generateMinimumSnapRingTrajectoryWithSegmentRefinement(
            GTInitSegmentTimes,
            GTSettings,
            m_GTMaxSpeed,
            m_GTMaxThrust,
            m_GTMaxRollPitchRate,
            1/m_GTSamplingTime
            );
        
  
    m_GloTraj.clear();
    m_GloTraj = {std::make_move_iterator(GlobalTraj.points.begin()), std::make_move_iterator(GlobalTraj.points.end())};

    
    std::cout<<std::endl;
    for (const quadrotor_common::TrajectoryPoint& TrajPoint : m_GloTraj)
    {
        std::cout << "\t" << TrajPoint.position.x() << "   \t" << TrajPoint.position.y() << "   \t" << TrajPoint.position.z() << std::endl;
    }
    std::cout<<std::endl;


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
        ROS_WARN(
            "[%s]\n  >> Maximum Speed of computed global trajectory exceeds 110 %% of nominal value (%f/%f m/s)."
            "\n     Re-computing global trajectory...",
            ros::this_node::getName().c_str(), m_Expert_MaxSpeed, m_GTMaxSpeed
            );


    if ( GlobalTraj.trajectory_type == quadrotor_common::Trajectory::TrajectoryType::UNDEFINED )
        ROS_WARN(
            "[%s]\n  >> Computation of global trajectory not successful."
            "\n     Re-computing global trajectory....",
            ros::this_node::getName().c_str()
            );


    for (Eigen::Vector3d& Waypoint : m_GT_WP) Waypoint += MeanWaypoint; //+ MinAxVals;
    for (quadrotor_common::TrajectoryPoint& TrajPoint : m_GloTraj) TrajPoint.position += MeanWaypoint;// + MinAxVals;

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
    //rvizGloTraj();

    // --- Set Waypoint Idx for deterministic 8 RaceTrack 
    //m_CurrWP_i = m_WUInitWaypointIdxForFIG8;
    //m_LastWP_i 
    //    = (m_CurrWP_i + m_GT_WP.size() - 1) 
    //        % m_GT_WP.size();
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
    m_Dist2CurrWP = (m_RefStatePos_WRF - m_GT_WP[m_CurrWP_i]).norm();
    m_Dist2LastWP = (m_RefStatePos_WRF - m_GT_WP[m_LastWP_i]).norm();
    
    updateWaypointIdx();
    rvizCurrWaypoint();
}


void ForgetfulDrone::updateWaypointIdx()
{
    if (m_Dist2CurrWP > m_WUThresDist2DetectWaypointArrival)
        return;

    if (m_EstimatedNormalizedSpeed < (m_LTMinSpeed/2) / m_LTMaxSpeed)
    {
        MY_ROS_INFO("[%s]    Assuming that drone is gathering pace in running-in phase. Not updating waypoint index.", m_ROSNodeName);
        return;
    }

    MY_ROS_INFO("[%s] Gate #%d/%d passed.", m_ROSNodeName, static_cast<int>(m_LastWP_i), static_cast<int>(m_GT_WP.size() - 1));
        
        
    ++m_CurrWP_i %= m_GT_WP.size();
    m_LastWP_i = (m_CurrWP_i + m_GT_WP.size() - 1) % m_GT_WP.size();
    
    m_LastWP_ArrivalTime = ros::Time::now();

    if (m_CurrWP_i == 0)
    {
        m_Lap_i++;
        MY_ROS_INFO("[%s]\n  >> Lap #%d/%d entered.", m_ROSNodeName, m_Lap_i, m_ORCLapMaxCount -1);
    }
}

void ForgetfulDrone::rvizCurrWaypoint()
{
    rvizPosition(m_GT_WP[m_CurrWP_i], VisPosTypes::CURRGATE, m_ROSPub_RVIZPositions);
}





//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////// NAVIGATOR //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////




void ForgetfulDrone::runNavigator()
{
    // Perturb input to navigator
    m_NavigatorInput += Eigen::Vector3d::Ones() * m_NAVInputPerturbAxAmp * std::sin(0.5*ros::WallTime::now().toSec());

    // Publish current pose until Navigator is enabled.
    if (!m_NavigatorEnabled)
    {
        //m_RefState_ARF = {};
        //m_RefState_ARF.position = m_Trafo_ARF_DRF.getPosition();
        //m_RefState_ARF.orientation = m_Trafo_ARF_DRF.getEigenQuaternion();

        

        MY_ROS_DEBUG("[%s]    Wait for enabled Navigator.", m_ROSNodeName);
        //MY_ROS_DEBUG("[%s]    Drone pose estimate, position: [%.2f, %.2f, %.2f], orientation: [%.4f, %.4f, %.4f, %.4f].", 
        //    m_ROSNodeName,
        //    m_Trafo_ARF_DRF.getPosition().x(), m_Trafo_ARF_DRF.getPosition().y(), m_Trafo_ARF_DRF.getPosition().z(),
        //    m_Trafo_ARF_DRF.getEigenQuaternion().w(), m_Trafo_ARF_DRF.getEigenQuaternion().x(), m_Trafo_ARF_DRF.getEigenQuaternion().y(), m_Trafo_ARF_DRF.getEigenQuaternion().z());
        MY_ROS_DEBUG("[%s]    Drone pose estimate, position: [%.2f, %.2f, %.2f], yaw: [%.2f].", 
            m_ROSNodeName,
            m_Trafo_ARF_DRF.getPosition().x(), m_Trafo_ARF_DRF.getPosition().y(), m_Trafo_ARF_DRF.getPosition().z(),
            Yaw_From_EigenQuaterniond(m_Trafo_ARF_DRF.getEigenQuaternion()));
        ros::Duration(0.5).sleep();
        return;
    }
    

    // If drone diverged too much from reference state, switch Navigator off.
    const double Dist2RefState = (m_RefState_ARF.position - m_Trafo_ARF_DRF.getPosition()).norm();
    bool DroneDiverged = m_NAVMaxDist2RefState < Dist2RefState;
    if (DroneDiverged)
    {
        MY_ROS_ERROR("[%s]    Drone position estimate [%.2f, %.2f, %.2f] diverged from reference position [%.2f, %.2f, %.2f].", 
            m_ROSNodeName,
            m_Trafo_ARF_DRF.getPosition().x(), m_Trafo_ARF_DRF.getPosition().y(), m_Trafo_ARF_DRF.getPosition().z(),
            m_RefState_ARF.position.x(), m_RefState_ARF.position.y(), m_RefState_ARF.position.z());
        
        switchNavigator(false);
        return;
    }


    // If it is time, replan local trajectory
    bool LT_ReplanScheduled = m_MainLoopIter_i % m_NAVMainLoopItersPerLTReplanning == 0;
    if (!LT_ReplanScheduled) MY_ROS_DEBUG("[%s]    Do not replan local trajectory.", m_ROSNodeName);
    if (LT_ReplanScheduled)
    {
        MY_ROS_DEBUG("[%s]    Replan local trajectory.", m_ROSNodeName);
        
        // Compute goal position and speed to reach goal position from CNN output
        Eigen::Vector3d GoalPos_ARF;   
        double Speed2Goal;
        
        //m_NavigatorInput = {0.05, 0.05, 0.0};
        processNavigatorInput(GoalPos_ARF, Speed2Goal);

        // Compute local trajectory
        if (computeLocTraj(GoalPos_ARF, Speed2Goal))
        {
            m_LocTraj_SubseqInfeasibleN = 0;
            m_LocTraj_StartTime = ros::Time::now();
            rvizLocTraj();
        }
        else
        {
            if (m_LocTraj_SubseqInfeasibleN < m_NAVSubseqFailedLTReplanningMaxCount)
                m_LocTraj_SubseqInfeasibleN ++;
            else
            {
                MY_ROS_ERROR("[%s]    Max. number [%d] of subsequently infeasible local trajectories reached.",
                    m_ROSNodeName, m_NAVSubseqFailedLTReplanningMaxCount);
                switchNavigator(false);
                return;
            }
        }
    }
    

    publishRefStateFromLocTrajToAutopilot();

    rvizRefStatePosVel();


    m_MainLoopIter_i++;
}












void ForgetfulDrone::rvizRefStatePosVel()
{
    //rvizPosition(m_RefStatePos_WRF, VisPosTypes::REFERENCE, m_ROSPub_RVIZPositions);
    rvizState(m_RefState_ARF.position, m_RefState_ARF.acceleration, VisPosTypes::REFERENCE, m_ROSPub_RVIZPositions);
}

void ForgetfulDrone::switchNavigator( const bool& Enabled )
{
    std::string LastState = m_NavigatorEnabled? "ENABLED" : "DISABLED";

    auto enterON = [LastState, this](){
        
        // Autopilot expects that the first reference state is close to current position
        m_RefState_ARF = quadrotor_common::TrajectoryPoint();
        m_RefState_ARF.position = m_Trafo_ARF_DRF.getPosition();
        //m_RefState_ARF.orientation = m_Trafo_ARF_DRF.getEigenQuaternion().normalized();
        m_RefState_ARF.heading = Yaw_From_EigenQuaterniond(m_Trafo_ARF_DRF.getEigenQuaternion().normalized());

        m_ROSPub_AutopilotReferenceState.publish(m_RefState_ARF.toRosMessage());

        ros::Duration(3.0).sleep();
        
        m_NavigatorEnabled = true;
        MY_ROS_INFO("[%s]    Switch Navigator: %s -> ENABLED.", m_ROSNodeName, LastState.c_str());
        ros::Duration(5.0).sleep();
        };

    auto enterOFF = [LastState, this](){
        m_NavigatorEnabled = false;
        
        m_RefState_ARF = quadrotor_common::TrajectoryPoint();
        m_RefState_ARF.position = m_Trafo_ARF_DRF.getPosition();
        //m_RefState_ARF.orientation = m_Trafo_ARF_DRF.getEigenQuaternion().normalized();
        m_RefState_ARF.heading = Yaw_From_EigenQuaterniond(m_Trafo_ARF_DRF.getEigenQuaternion().normalized());
        m_ROSPub_AutopilotReferenceState.publish(m_RefState_ARF.toRosMessage());

        MY_ROS_INFO ("[%s]    Switch Navigator: %s -> DISABLED.", m_ROSNodeName, LastState.c_str());
        };

    Enabled? enterON() : enterOFF();
}


void ForgetfulDrone::publishRefStateFromLocTrajToAutopilot()
{
    // Get time of current state of current local trajectory
    const double LocTraj_T
        = (ros::Time::now() - m_LocTraj_StartTime).toSec() + m_ORCMainLoopNomPeriodDuration;
    m_RefState_ARF = quadrotor_common::TrajectoryPoint();
    m_RefState_ARF.position        = EigenVector3d_From_Vec3(m_LocTraj.GetPosition(LocTraj_T));
    m_RefState_ARF.velocity        = EigenVector3d_From_Vec3(m_LocTraj.GetVelocity(LocTraj_T));
    m_RefState_ARF.acceleration    = EigenVector3d_From_Vec3(m_LocTraj.GetAcceleration(LocTraj_T));
    m_RefState_ARF.jerk            = EigenVector3d_From_Vec3(m_LocTraj.GetJerk(LocTraj_T));
    m_RefState_ARF.snap            = {};
    m_RefState_ARF.heading = std::atan2(m_RefState_ARF.velocity.y(), m_RefState_ARF.velocity.x());


    //m_RefStatePos_WRF
    geometry_msgs::Pose RefPose_ARF;
        RefPose_ARF.position = GeometryMsgsPoint_From_EigenVector3d(m_RefState_ARF.position);
        RefPose_ARF.orientation.w = 1.0;
        RefPose_ARF.orientation.x = 0.0;
        RefPose_ARF.orientation.y = 0.0;
        RefPose_ARF.orientation.z = 0.0;
    kindr::minimal::QuatTransformation Trafo_ARF_RRF;
    tf::poseMsgToKindr(RefPose_ARF, &Trafo_ARF_RRF);
    m_RefStatePos_WRF = (m_Trafo_WRF_ARF * Trafo_ARF_RRF).getPosition();
    MY_ROS_DEBUG("[%s]    Reference State\n\tPosition: [%.3f, %.3f, %.3f] (WRF)",
            m_ROSNodeName, m_RefStatePos_WRF.x(), m_RefStatePos_WRF.y(), m_RefStatePos_WRF.z());


    // Publish reference state to autopilot
    MY_ROS_DEBUG("[%s]    Reference State\n\tPosition: [%.3f, %.3f, %.3f]\n\tVelocity: [%.3f, %.3f, %.3f]\n\tAcceleration: [%.3f, %.3f, %.3f]\n\tYaw: %.3f",
            m_ROSNodeName,
            m_RefState_ARF.position.x(), m_RefState_ARF.position.y(), m_RefState_ARF.position.z(),
            m_RefState_ARF.velocity.x(), m_RefState_ARF.velocity.y(), m_RefState_ARF.velocity.z(),
            m_RefState_ARF.acceleration.x(), m_RefState_ARF.acceleration.y(), m_RefState_ARF.acceleration.z(),
            m_RefState_ARF.heading);

    m_ROSPub_AutopilotReferenceState.publish(m_RefState_ARF.toRosMessage());
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

            Eigen::Vector3d Pos = Q * (EigenVector3d_From_Vec3( m_LocTraj.GetPosition(t) )) + T;
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
    const Vec3 StartPos_ARF = Vec3_From_EigenVector3d(m_RefState_ARF.position);
    const Vec3 StartVel_ARF = Vec3_From_EigenVector3d(m_RefState_ARF.velocity);
    const Vec3 StartAcc_ARF = Vec3_From_EigenVector3d(m_RefState_ARF.acceleration);
    
    const Vec3 EndPos_ARF = Vec3_From_EigenVector3d(GoalPos_ARF);
    const Vec3 Gravity{0.0, 0.0, -9.81};

    const double LT_LinDist = (EndPos_ARF - StartPos_ARF).GetNorm2();

    const double LocTraj_Duration = LT_LinDist / std::min({
        Speed2Goal,
        m_RefState_ARF.velocity.norm() + m_LTMaxSpeedIncrement,
        m_LTMaxSpeed});

    RQTG::RapidTrajectoryGenerator LocTraj{StartPos_ARF, StartVel_ARF, StartAcc_ARF, Gravity};

    LocTraj.SetGoalPosition(EndPos_ARF);
    LocTraj.Generate(LocTraj_Duration);

    
    MY_ROS_INFO(
        "[%s]    Local Trajectory\n"
        "\tStart Position: [%.3f, %.3f, %.3f] (ARF)\n"
        "\tStart Velocity: [%.3f, %.3f, %.3f]\n"
        "\tStart Acceleration: [%.3f, %.3f, %.3f]\n"
        "\tEnd Position: [%.3f, %.3f, %.3f] (ARF)\n"
        "\tGravity: [%.3f, %.3f, %.3f]\n"
        "\tDuration: %.3f\n"
        "\tLinear Distance: %.3f\n",
        m_ROSNodeName,
        StartPos_ARF.x, StartPos_ARF.y, StartPos_ARF.z,
        StartVel_ARF.x, StartVel_ARF.y, StartVel_ARF.z,
        StartAcc_ARF.x, StartAcc_ARF.y, StartAcc_ARF.z,
        EndPos_ARF.x, EndPos_ARF.y, EndPos_ARF.z,
        Gravity.x, Gravity.y, Gravity.z,
        LocTraj_Duration,
        LT_LinDist);
    
    if ( LocTrajFeasible( LocTraj ) )
    {
        m_LocTraj = LocTraj;
        return true;
    }
    else
    {
        MY_ROS_WARN("[%s]    Discard infeasible local trajectory.", m_ROSNodeName);
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
        = LocTraj.CheckInputFeasibility(m_LTMinNormThrust, m_LTMaxNormThrust, m_LTMaxBodyRates, m_LTInputFeasibilityCheckMinSamplingTime);

    Vec3 BoundaryPoint = { 0.0, 0.0, m_LTMinAltitude };  // a point on the floor
    Vec3 BoundaryVector = { 0.0, 0.0, 1.0 };  // we want to be above the point
    RQTG::RapidTrajectoryGenerator::StateFeasibilityResult PositionFeasibility_Floor 
        = LocTraj.CheckPositionFeasibility(BoundaryPoint, BoundaryVector);

    BoundaryPoint[2] = m_LTMaxAltitude;  // a point on the ceiling
    BoundaryVector[2] = -1.0;  // we want to be below the point
    RQTG::RapidTrajectoryGenerator::StateFeasibilityResult PositionFeasibility_Ceiling 
        = LocTraj.CheckPositionFeasibility(BoundaryPoint, BoundaryVector);

    if (InputFeasibility != RQTG::RapidTrajectoryGenerator::InputFeasible)
    {
        MY_ROS_WARN(
            "[%s]    Local trajectory input-infeasible.\n"
            "\tThrust range: [%.3f, %.3f]\n"
            "\tmax. body rates: [%.3f]\n"
            "\t\t(checked with min. sampling time of [%.3f] s.",
            m_ROSNodeName,
            m_LTMinNormThrust, m_LTMaxNormThrust,
            m_LTMaxBodyRates,
            m_LTInputFeasibilityCheckMinSamplingTime);
        return false;
    }
    if (PositionFeasibility_Floor != RQTG::RapidTrajectoryGenerator::StateFeasible)
    {
        MY_ROS_WARN(
            "[%s]    Local trajectory position-infeasible.\n"
            "\tmin. altitude: [%.3f] m",
            m_ROSNodeName, m_LTMinAltitude);
        return false;
    }
    if ( PositionFeasibility_Ceiling != RQTG::RapidTrajectoryGenerator::StateFeasible )
    {
        MY_ROS_WARN(
            "[%s]    Local trajectory position-infeasible.\n"
            "\tmax. altitude: [%.3f] m",
            m_ROSNodeName, m_LTMaxAltitude);
        return false;
    }

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


    MY_ROS_INFO("[%s]    Navigator Input: [%.3f, %.3f, %.3f]", m_ROSNodeName, m_NavigatorInput.x(), m_NavigatorInput.y(), m_NavigatorInput.z());

    

    // Denormalize speed
    OUT_Speed2Goal = std::max(m_LTMinSpeed, m_LTMaxSpeed * NormSpeed2Goal);

    MY_ROS_INFO("[%s]    Speed to Goal: [%.3f]", m_ROSNodeName, OUT_Speed2Goal);

    // Compute distance from drone to goal position:
    double LT_LinDist = std::max(m_LTMinDistance, std::min(m_LTMaxDistance, m_LTDuration * OUT_Speed2Goal));
    MY_ROS_INFO("[%s]    Linear distance to goal position: [%.3f]", m_ROSNodeName, LT_LinDist);

    // Compute goal position
    OUT_GoalPos_ARF 
        = XYZ_ARF_From_XYZ_DRF( 
            XYZ_DRF_From_XYDist_IRF(GoalX_IRF, GoalY_IRF, LT_LinDist));

    MY_ROS_INFO("[%s]    Goal Position: [%.3f, %.3f, %.3f] (ARF)", m_ROSNodeName,
            OUT_GoalPos_ARF.x(), OUT_GoalPos_ARF.y(), OUT_GoalPos_ARF.z());
}



Eigen::Vector3d 
ForgetfulDrone::XYZ_DRF_From_XYDist_IRF( 
    const double& X_IRF, 
    const double& Y_IRF, 
    const double& Dist_IRF 
){
    // Compute yaw and pitch angle to position in reference frame of drone.
    const double Yaw = -m_DRONECamHalfYawAOV * X_IRF;
    const double Pitch = m_DRONECamHalfPitchAOV * Y_IRF;

    MY_ROS_INFO("[%s]    Yaw: [%.3f] (DRF), Pitch: [%.3f] (DRF)", m_ROSNodeName, Yaw, Pitch);

    // Compute vector in drone reference frame.
    return {
        Dist_IRF * cos( Pitch ) * cos( Yaw ),
        Dist_IRF * cos( Pitch ) * sin( Yaw ),
        Dist_IRF * sin( Pitch )
        };
}

Eigen::Vector3d ForgetfulDrone::XYZ_ARF_From_XYZ_DRF( const Eigen::Vector3d& XYZ_DRF)
{
    MY_ROS_INFO("[%s]    Goal Position: [%.3f, %.3f, %.3f] (DRF)", m_ROSNodeName,
            XYZ_DRF.x(), XYZ_DRF.y(), XYZ_DRF.z());

    MY_ROS_INFO("[%s]    Drone pose estimate, position: [%.2f, %.2f, %.2f], yaw: [%.2f].", 
            m_ROSNodeName,
            m_Trafo_ARF_DRF.getPosition().x(), m_Trafo_ARF_DRF.getPosition().y(), m_Trafo_ARF_DRF.getPosition().z(),
            Yaw_From_EigenQuaterniond(m_Trafo_ARF_DRF.getEigenQuaternion()));

    geometry_msgs::Pose Pose_DRF;
        Pose_DRF.position = GeometryMsgsPoint_From_EigenVector3d(XYZ_DRF);
        Pose_DRF.orientation.w = 1.0;
        Pose_DRF.orientation.x = 0.0;
        Pose_DRF.orientation.y = 0.0;
        Pose_DRF.orientation.z = 0.0;
    kindr::minimal::QuatTransformation T_DRF_XYZ;
    tf::poseMsgToKindr(Pose_DRF, &T_DRF_XYZ);
    kindr::minimal::QuatTransformation T_ARF_XYZ = m_Trafo_ARF_DRF * T_DRF_XYZ;
    
    return T_ARF_XYZ.getPosition();

    //return m_Trafo_ARF_DRF.getPosition() + kindr::minimal::Position{+3.0, 0.0, 0.5};
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
    double Horizon = std::min(m_Expert_MaxHorizon, std::max(m_EXPMinHorizon, std::min(m_Dist2CurrWP, m_Dist2LastWP)));

    findExpertStateWithProjection(); 
    findHorizonState(Horizon);
    rvizExpertAndHorizonState();

    Eigen::Vector2d GoalPos_IRF = PosIRF_From_PosDRF(PosDRF_From_PosWRF(m_GloTraj[m_GT_HorizonState_i].position));

    findSpeedState();
    double NormalizedSpeed2Goal = m_GloTraj[m_GT_SpeedState_i].velocity.norm() / m_Expert_MaxSpeed;

    m_ExpertPrediction = {GoalPos_IRF.x(), GoalPos_IRF.y(), NormalizedSpeed2Goal};
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
        = m_GloTraj[ m_GT_ExpertState_i     ].position 
        - m_GloTraj[ m_GT_ExpertState_i - 1 ].position;

    ExpState_SpeedLvl = ExpState_Direction.norm();

    ExpState_Direction /= ExpState_SpeedLvl;

    Exp2RefState_Direction
        = m_RefStatePos_WRF
        - m_GloTraj [ m_GT_ExpertState_i - 1 ].position;

    ProjLvl = Exp2RefState_Direction.dot( ExpState_Direction );

    while ( ProjLvl > ExpState_SpeedLvl )
    {
        m_GT_ExpertState_i++;
        m_GT_ExpertState_i %= m_GloTraj.size();


        ExpState_Direction
            = m_GloTraj[ m_GT_ExpertState_i     ].position 
            - m_GloTraj[ m_GT_ExpertState_i - 1 ].position;

        ExpState_SpeedLvl = ExpState_Direction.norm();

        ExpState_Direction /= ExpState_SpeedLvl;

        Exp2RefState_Direction
            = m_RefStatePos_WRF
            - m_GloTraj [ m_GT_ExpertState_i - 1 ].position;
        
        ProjLvl = Exp2RefState_Direction.dot( ExpState_Direction );
    }


    // If the distance from the "expert state" found with above projection
    // to the "reference state" of the autopilot is more than 1 m,
    // find "expert state" as state on global trajectory that has min distance to "reference state".
    double DivergenceFromGlobalTraj 
        = (m_RefStatePos_WRF - m_GloTraj[m_GT_ExpertState_i].position).norm();
    
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
            m_GT_ExpertState_i = StateIdx;
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
    m_GT_HorizonState_i = m_GT_ExpertState_i;
    
    do
    {
        ++m_GT_HorizonState_i %= m_GloTraj.size();

        
        Distance_Expert2HorizonState 
            = ( m_GloTraj[ m_GT_HorizonState_i ].position 
                - m_GloTraj[ m_GT_ExpertState_i ].position ).norm();

    } while ( Distance_Expert2HorizonState < Horizon );
}


void 
ForgetfulDrone::findSpeedState
()
{
    double Distance_Expert2SpeedState;
    m_GT_SpeedState_i = m_GT_ExpertState_i;
    
    do
    {
        ++m_GT_SpeedState_i %= m_GloTraj.size();

        
        Distance_Expert2SpeedState 
            = ( m_GloTraj[ m_GT_SpeedState_i ].position 
                - m_GloTraj[ m_GT_ExpertState_i ].position ).norm();

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
    rvizPosition(m_GloTraj[m_GT_ExpertState_i].position, VisPosTypes::EXPERT, m_ROSPub_RVIZPositions);
    rvizPosition(m_GloTraj[m_GT_HorizonState_i].position, VisPosTypes::HORIZON, m_ROSPub_RVIZPositions);
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
    if (!m_TrainDataSaverEnabled)
        if (m_CurrWP_i == 0)
        {
            m_TrainDataSaverEnabled = true;
            MY_ROS_INFO("[%s]    Training data saver ENABLED.", m_ROSNodeName);
        }
            

    if (m_TrainDataSaverEnabled)
        saveTrainData();
}



void ForgetfulDrone::saveTrainData()
{
    std::ostringstream CamFrame_i_Stream;
    CamFrame_i_Stream << std::setw(5) << std::setfill('0') << m_CamFrame_i;
    std::string CamFrame_i_String = CamFrame_i_Stream.str();

    // --- Image ---
    std::string ImageFilePath = m_RunDirPath + "/images/camera_frame_" + CamFrame_i_String + ".jpg";

    m_CloneImgMtx.lock();
        // Convert from bgr to rgb
        cv::Mat CamFrame = m_CamFrame.clone();
        cv::cvtColor(m_CamFrame, CamFrame, CV_BGR2RGB);
    m_CloneImgMtx.unlock();

        if (!cv::imwrite(ImageFilePath, CamFrame)) 
            MY_ROS_ERROR("[%s]    Failed to save file [%s].", m_ROSNodeName, ImageFilePath.c_str());
    

    // --- Label ---
    // save label of best trajectory
    std::ofstream Outfile;
    std::string FilePath = m_RunDirPath + "/labels.txt";
    Outfile.open(FilePath, std::ios_base::app);
    Outfile 
        << std::to_string(m_ExpertPrediction.x()) + ";" 
        << std::to_string(m_ExpertPrediction.y()) + ";"
        << std::to_string(m_ExpertPrediction.z()) + ";" 
        << std::to_string(m_Expert_MaxSpeed     ) + ";"
        << std::to_string(m_CamFrame_i == 0? 1 : 0 ) + "\n"; // ???new dagger batch???
    Outfile.close();

    m_CamFrame_i++;
}

void ForgetfulDrone::markRunTrainingDataAsFailed()
{
    std::ofstream Outfile;
    std::string Filename = m_TrainingDataDirPath + "/fails.txt";
    
    Outfile.open(Filename, std::ios_base::app);
    Outfile << std::to_string(m_Run_i) + "\n";
    Outfile.close();
}






//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////// NOT IMPLEMENTED YET //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void ForgetfulDrone::testNavigatorWithNetwork()
{
    MY_ROS_INFO("[%s]    FLIGHT MISSION: Navigator-Network Test", m_ROSNodeName);
    MY_ROS_ERROR("[%s]    Specified flight mission not implemented yet.", m_ROSNodeName);
}

void ForgetfulDrone::testSimulatorWithAutopilot()
{
    MY_ROS_INFO("[%s]    FLIGHT MISSION: Simulator-Autopilot Test", m_ROSNodeName);
}





void generateRacetrackData()
{
    std::cout << "\n\n --- GENERATE RACETRACK DATA: Global Trajectories --- \n\n\n";

    ros::Duration(5.0).sleep();

    int UNITY_SCENES_N = 5;
    int UNITY_SITES_N = 3;
    int GATE_TYPES_N = 2;
    int REPEATED_CONDITIONS_N = 3;

    for (int SceneIdx = 0; SceneIdx < UNITY_SCENES_N; SceneIdx++)
    {
        for (int SiteIdx = 0; SiteIdx < UNITY_SITES_N; SiteIdx++)
        {
            for (int GateTypeIdx = 0; GateTypeIdx < GATE_TYPES_N; GateTypeIdx++)
            {
                for (int RepCondIdx = 0; RepCondIdx < REPEATED_CONDITIONS_N; RepCondIdx++)
                {
                    std::cout << "\n\tScene: " << SceneIdx <<", ";
                    std::cout << "Site: " << SiteIdx <<", ";
                    std::cout << "Gate Type: " << GateTypeIdx <<", ";
                    std::cout << "Repeated Conditions: " << RepCondIdx <<"\n";



                    
                    
                    std::string InOutDirPath 
                        = ros::package::getPath("forgetful_drones") + "/racetracks/" 
                        + std::to_string(SceneIdx) + "/"
                        + std::to_string(SiteIdx) + "/"
                        + std::to_string(GateTypeIdx) + "/"
                        + std::to_string(RepCondIdx);

                    std::string InFilePath = InOutDirPath +"/waypoints.txt";



                    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> Waypoints;

                    std::ifstream input_file( InFilePath );
                    std::string line;
                    while (std::getline(input_file, line))
                    {
                        std::istringstream iss(line);
                        double x, y, z;
                        if (!(iss >> x >> y >> z)) { break; }

                        Waypoints.push_back({x, y, z});
                    }




                    // COmpute Traj
                    //ros::Duration(5.0).sleep();
                    std::vector<quadrotor_common::TrajectoryPoint> 
                    GlobalTrajectory = computeGlobalTrajectory(Waypoints);
                    

                    std::string OutFilePath = InOutDirPath +"/global_trajectory.txt";
                    std::ofstream OutFile(OutFilePath);
                    for (const quadrotor_common::TrajectoryPoint& TrajPoint : GlobalTrajectory)
                    {
                        OutFile 
                            << TrajPoint.position.x() << " " 
                            << TrajPoint.position.y() << " " 
                            << TrajPoint.position.z() << " " 
                            << TrajPoint.velocity.x() << " "
                            << TrajPoint.velocity.y() << " "
                            << TrajPoint.velocity.z() << "\n";
                    }
                }
            }
        }
    }
}


std::vector<quadrotor_common::TrajectoryPoint> computeGlobalTrajectory
(std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> Waypoints)
{
    ros::Duration(10.0).sleep();



    // --- Normalize waypoints around origin ---
    Eigen::Vector3d MeanWaypoint{0, 0, 0};
    for (const Eigen::Vector3d& Waypoint : Waypoints) MeanWaypoint += Waypoint /Waypoints.size();
    for (Eigen::Vector3d& Waypoint : Waypoints) Waypoint -= MeanWaypoint;

    //Eigen::Vector3d MinAxVals{0, 0, 0};
    //for (const Eigen::Vector3d& Waypoint : m_GT_WP)
    //{
    //    MinAxVals.x() = std::min(MinAxVals.x(), Waypoint.x());
    //    MinAxVals.y() = std::min(MinAxVals.y(), Waypoint.y());
    //    MinAxVals.z() = std::min(MinAxVals.z(), Waypoint.z());
    //}
    //for (Eigen::Vector3d& Waypoint : m_GT_WP) Waypoint -= MinAxVals;




    std::cout<<std::endl;
    for (const Eigen::Vector3d& Waypoint : Waypoints)
    {
        std::cout << "\t" << Waypoint.x() << "   \t" << Waypoint.y() << "   \t" << Waypoint.z() << std::endl;
    }
    std::cout<<std::endl;



    //// --- Set ´m_Expert_MaxHorizon´ <- max dist between waypoints ---
    //m_Expert_MaxHorizon = 0.0;
    //size_t LastWaypointIdx = m_GT_WP.size() - 1;
    //for (size_t WaypointIdx = 0; WaypointIdx < m_GT_WP.size(); WaypointIdx++)
    //{
    //    m_Expert_MaxHorizon = std::max( 
    //        m_Expert_MaxHorizon,
    //        (m_GT_WP[ WaypointIdx ] - m_GT_WP[ LastWaypointIdx ]).norm() );
    //    LastWaypointIdx = WaypointIdx;
    //}

    /*    ROS_INFO(
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
        static_cast<int>( m_GT_WP.size() ),
        m_GTPolyOrder,
        m_GTContiOrder,
        m_GTMinWeightVel,
        m_GTMinWeightAcc,
        m_GTMinWeightJerk,
        m_GTMinWeightSnap,
        m_GTMaxSpeed,
        m_GTMaxThrust,
        m_GTMaxRollPitchRate,
        m_GTSamplingTime
        );
    */


    // --- Set up Settings of GT ---
    polynomial_trajectories::PolynomialTrajectorySettings GTSettings;
    GTSettings.way_points = {std::make_move_iterator(Waypoints.begin()), std::make_move_iterator(Waypoints.end())};
    GTSettings.polynomial_order = 11;
    GTSettings.continuity_order = 4;
    GTSettings.minimization_weights = Eigen::Vector4d{1, 10, 100, 1000};
    

    // --- Compute initial segment times (first element relates to segment from last to first waypoint) ---
    Eigen::VectorXd GTInitSegmentTimes = Eigen::VectorXd::Ones(Waypoints.size());
    Eigen::Vector3d SegmentStart = Waypoints.back();
    for (size_t WaypointIdx = 0; WaypointIdx < Waypoints.size(); WaypointIdx++) 
    {
        GTInitSegmentTimes[WaypointIdx] = (Waypoints[ WaypointIdx ] - SegmentStart).norm() /10.0;
        SegmentStart = Waypoints[WaypointIdx];
    }


        
    quadrotor_common::Trajectory GlobalTraj 
        = trajectory_generation_helper::polynomials::generateMinimumSnapRingTrajectoryWithSegmentRefinement(
            GTInitSegmentTimes,
            GTSettings,
            10,
            18,
            1.5,
            50
            );

    GlobalTraj.toRosMessage();
        
  
    std::vector<quadrotor_common::TrajectoryPoint> GlobalTrajectory
        = {std::make_move_iterator(GlobalTraj.points.begin()), std::make_move_iterator(GlobalTraj.points.end())};

    
    std::cout<<std::endl;
    for (const quadrotor_common::TrajectoryPoint& TrajPoint : GlobalTrajectory)
    {
        std::cout << "\t" << TrajPoint.position.x() << "   \t" << TrajPoint.position.y() << "   \t" << TrajPoint.position.z() << std::endl;
    }
    std::cout<<std::endl;


    // Find the maximum and minimum velocity of the global trajectory.
    double m_Expert_MaxSpeed = 0.0;
    double m_Expert_MinSpeed = std::numeric_limits< double >::max();
    for ( size_t StateIdx = 0; StateIdx < GlobalTrajectory.size(); StateIdx++ )
    {
        double Speed = ( GlobalTrajectory[ StateIdx ].velocity ).norm();
        m_Expert_MaxSpeed = std::max( m_Expert_MaxSpeed, Speed );
        m_Expert_MinSpeed = std::min( m_Expert_MinSpeed, Speed );
    }

    if ( m_Expert_MaxSpeed > 1.1 * 10.0 )
        std::cout 
        << "\nMaximum Speed (" << m_Expert_MaxSpeed << " m/s) of computed global trajectory exceeds 110 %% of nominal value (10 m/s).\n";

    if ( GlobalTraj.trajectory_type == quadrotor_common::Trajectory::TrajectoryType::UNDEFINED )
        std::cout
        << "\nTrajectory type is undefined after computation.\n";


    for (Eigen::Vector3d& Waypoint : Waypoints) Waypoint += MeanWaypoint; //+ MinAxVals;
    for (quadrotor_common::TrajectoryPoint& TrajPoint : GlobalTrajectory) TrajPoint.position += MeanWaypoint;// + MinAxVals;

    std::cout
        << "\nGlobal trajectory successfully computed:"
        << "\n\t#states: " << GlobalTrajectory.size()
        << "\n\tduration [s]: " << GlobalTrajectory.size() /50
        << "\n\tmaximum speed [m/s]: " << m_Expert_MaxSpeed
        << "\n\tminimum speed [m/s]: " << m_Expert_MinSpeed;



    // --- Visualize global trajectory in RVIZ ---
    //rvizGloTraj();

    // --- Set Waypoint Idx for deterministic 8 RaceTrack 
    //m_CurrWP_i = m_WUInitWaypointIdxForFIG8;
    //m_LastWP_i 
    //    = (m_CurrWP_i + m_GT_WP.size() - 1) 
    //        % m_GT_WP.size();

    return GlobalTrajectory;
}
}