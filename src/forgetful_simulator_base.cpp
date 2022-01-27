#include "forgetful_drones/forgetful_simulator.hpp"

#define MY_ROS_DEBUG(...) {std::cout <<std::endl << "\033[35;45m" << "[]" << "\033[0m" << " "; ROS_LOG(::ros::console::levels::Debug, ROSCONSOLE_DEFAULT_NAME, __VA_ARGS__);}
#define MY_ROS_INFO(...)  {std::cout <<std::endl << "\033[35;45m" << "[]" << "\033[0m" << " "; ROS_LOG(::ros::console::levels::Info,  ROSCONSOLE_DEFAULT_NAME, __VA_ARGS__);}
#define MY_ROS_WARN(...)  {std::cout <<std::endl << "\033[35;45m" << "[]" << "\033[0m" << " "; ROS_LOG(::ros::console::levels::Warn,  ROSCONSOLE_DEFAULT_NAME, __VA_ARGS__);}
#define MY_ROS_ERROR(...) {std::cout <<std::endl << "\033[35;45m" << "[]" << "\033[0m" << " "; ROS_LOG(::ros::console::levels::Error, ROSCONSOLE_DEFAULT_NAME, __VA_ARGS__);}
#define MY_ROS_FATAL(...) {std::cout <<std::endl << "\033[35;45m" << "[]" << "\033[0m" << " "; ROS_LOG(::ros::console::levels::Fatal, ROSCONSOLE_DEFAULT_NAME, __VA_ARGS__);}





namespace forgetful_drone{


ForgetfulSimulator::ForgetfulSimulator (const ros::NodeHandle& RNH, const ros::NodeHandle& PNH)
    :
    m_ROSRNH(RNH), 
    m_ROSPNH(PNH),
    m_ROSNodeName{ros::this_node::getName().c_str()},
    m_ROSNodePath{ros::package::getPath("forgetful_drones")},

    m_UnitySceneID{flightlib::UnityScene::SPACESHIP_INTERIOR},
    m_UnityReady{false},
    m_UnityRender_ON{false},
    //m_ROSSub_StateEstimate{m_ROSRNH.subscribe("ground_truth/odometry", 1, &ForgetfulSimulator::ROSCB_StateEstimate, this)}, // "updating unity"
    m_ROSSub_SimulatorStart{m_ROSRNH.subscribe("simulator/start", 1, &ForgetfulSimulator::ROSCB_SimulatorStart, this)},
    m_ROSSub_SimulatorStop{m_ROSRNH.subscribe("simulator/stop", 1, &ForgetfulSimulator::ROSCB_SimulatorStop, this)},

    
    m_CamFrameCount{0},
    m_DroneSpawned{false},
    m_DroneModelName{"hummingbird"},
    m_DroneModelDescription(),
    m_ROSSrvCl_GazeboSpawnURDFModel(m_ROSRNH.serviceClient<gazebo_msgs::SpawnModel>("/gazebo/spawn_urdf_model")),
    m_DroneCamFOV(),
    m_DroneCamWidth(),
    m_DroneCamHeight(),
    m_UnityBridgePtr{nullptr},
    m_FM_PointCloudFileName{m_ROSNodePath + "/tmp/point_cloud.ply"},
    m_SimulationReady(false),
    m_GateSTLFilePath{m_ROSNodePath + "/gazebo/rpg_gate/model.stl"},
    m_GateSDFFilePath{m_ROSNodePath + "/gazebo/rpg_gate/model.sdf"},
    m_GroundPlaneID(),
    m_RVizDroneFrameID("hummingbird/base_link"),
    
    m_ROSSrvSv_BuildDroneRacingSimulation(m_ROSRNH.advertiseService("simulator/build_drone_racing_simulation", &ForgetfulSimulator::ROSCB_buildDroneRacingSimulation, this)),
    m_ROSPub_RVizGates(m_ROSRNH.advertise<visualization_msgs::MarkerArray>("rviz/gates", 1, true)),
    m_ROSPub_RVizDrone(m_ROSRNH.advertise<visualization_msgs::MarkerArray>("rviz/drone", 1, true)),
    m_ROSSrvCl_GazeboDeleteModel(m_ROSRNH.serviceClient<gazebo_msgs::DeleteModel>("/gazebo/delete_model")),
    m_ROSSrvCl_GazeboSpawnSDFModel(m_ROSRNH.serviceClient<gazebo_msgs::SpawnModel>("/gazebo/spawn_sdf_model")),

    m_RandFig8_MaxAxShift(),
    m_RandFig8_MaxYawTwist(),
    m_RandFig8_MinScale(),
    m_RandFig8_MaxScale(),
    m_RVizDroneRotorN(),
    m_RVizDroneArmLength(),
    m_RVizDroneBodyWidth(),
    m_RVizDroneBodyHeight(),
    m_RVizDroneScale(),
    m_Debug_ON(false),
    m_MainLoopFreq(),
    m_UnitySceneIdx(),
    m_UnitySiteIdx(),
    m_RacetrackTypeIdx(),
    m_RaceGateTypeIdx(),
    m_RacetrackDataGeneration_ON()

    //m_SimulatorLoopTime()
    //m_RandSprint_GateN(),
    //m_RandSprint_GateMinN(),
    //m_RandSprint_GateMaxN(),    
    //m_RandSprint_SectionGateMinN(),
    //m_RandSprint_SectionGateMaxN(),
    //m_RandSprint_MinSpacing(),
    //m_RandSprint_UnidirStdDevSpacing(),
    //m_RandSprint_LRMeanYaw(),
    //m_RandSprint_LRStdDevYaw(),
    //m_RandSprint_MeanPitch(),
    //m_RandSprint_StdDevPitch(),
    //m_DynGates_MaxAxAmp(),
    //m_DynGates_MaxAxFreq(),
    //m_ROSSub_DynamicGatesSwitch( m_ROSRNH.subscribe("simulator/dynamic_gates_switch", 1, &ForgetfulSimulator::ROSCB_DynamicGatesSwitch, this) ), // DYNGATES
    //m_ROSTimer_SimulatorLoop( m_ROSRNH.createTimer(ros::Duration(m_SimulatorLoopTime), &ForgetfulSimulator::ROSTimerFunc_SimulatorLoop, this, false, false) ), // DYNGATES
    //m_ROSSub_GazeboModelStates( m_ROSRNH.subscribe("/gazebo/model_states", 1, &ForgetfulSimulator::ROS_CB_GazeboModelStates, this ) ),  //DYNGATES
    //m_ROSPub_GazeboSetModelState( m_ROSRNH.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state", 0) ), //DYNGATES
    //m_ROSSrvCl_GazeboResetSimulation( m_ROSRNH.serviceClient<std_srvs::Empty>("/gazebo/reset_simulation") ),
{
    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) 
        ros::console::notifyLoggerLevelsChanged();

    bool InitSuccessful{true};

    m_RandEngine.seed(ros::WallTime::now().toNSec());

    if (!fetchROSParameters(m_ROSPNH, m_KeysBoolPtrs, m_KeysIntPtrs, m_KeysDoublePtrs, m_KeysStringPtrs))
        InitSuccessful = false;

    // Initialize publishers for images from Unity
    image_transport::ImageTransport IT(m_ROSPNH);
    m_ITPub_RGBImg = IT.advertise("/flightmare/rgb", 1);
    //m_ITPub_DepthImg = IT.advertise("/flightmare/depth", 1);
    //m_ITPub_SegmentationImg = IT.advertise("/flightmare/segmentation", 1);
    //m_ITPub_OpticalFlowImg = IT.advertise("/flightmare/opticalflow", 1);

    // Initialize main loop
    m_ROSTimer_MainLoop
        = m_ROSRNH.createTimer(ros::Rate(m_MainLoopFreq), &ForgetfulSimulator::ROSCB_MainLoop, this, false, false); // no oneshot, no autostart
    
    if (InitSuccessful)
    {
        MY_ROS_INFO("[%s]    Node initialized.", m_ROSNodeName);
    }
    else
    {
        MY_ROS_FATAL("[%s]    Node initialization failed -> Shut down the node.", m_ROSNodeName);
        ros::shutdown();
    }

    // Wait for gazebo and unity to load
    ros::Duration(5.0).sleep();

            
    
    if (m_Debug_ON) debugNode();


    if (m_RacetrackDataGeneration_ON)
    {
        std::cout<<std::endl;
        MY_ROS_INFO("[%s]\n  >> GENERATE RACETRACK DATA: Waypoints", m_ROSNodeName);

        int UNITY_SCENES_N = 5;
        int UNITY_SITES_N = 3;
        int GATE_TYPES_N = 2;
        int REPEATED_CONDITIONS_N = 3;

        for (m_UnitySceneIdx = 0; m_UnitySceneIdx < UNITY_SCENES_N; m_UnitySceneIdx++)
        {
            std::cout << "\n\tScene: " << m_UnitySceneIdx <<"\n";
            for (m_UnitySiteIdx = 0; m_UnitySiteIdx < UNITY_SITES_N; m_UnitySiteIdx++)
            {
                std::cout << "\t\tSite: " << m_UnitySiteIdx <<"\n";
                for (int GateTypeIdx = 0; GateTypeIdx < GATE_TYPES_N; GateTypeIdx++)
                {
                    std::cout << "\t\t\tGate Type: " << GateTypeIdx <<"\n";
                    for (int RepCondIdx = 0; RepCondIdx < REPEATED_CONDITIONS_N; RepCondIdx++)
                    {
                        std::cout << "\t\t\t\tRepeated Conditions: " << RepCondIdx <<"\n";
                        setRacetrackPose();
                        computeRaceTrack_Figure8(forgetful_drones_msgs::BuildDroneRacingSimulationRequest::FIGURE8_RANDOMIZED);


                        

                        std::string OutDirPath 
                            = m_ROSNodePath + "/racetracks/" 
                            + std::to_string(m_UnitySceneIdx) + "/"
                            + std::to_string(m_UnitySiteIdx) + "/"
                            + std::to_string(GateTypeIdx) + "/"
                            + std::to_string(RepCondIdx);
                        
                        std::string cmd = "mkdir -p " + OutDirPath;
                        system(cmd.c_str());

                        std::string OutFilePath = OutDirPath +"/waypoints.txt";

                        std::ofstream OutFile(OutFilePath);
                        for (const Pose& GatePose : m_GatesInitPose_Flightmare)
                        {
                            OutFile << GatePose.position.x() << " " << GatePose.position.y() << " " << GatePose.position.z() << "\n";
                        }
                    }
                }
            }
        }

        ros::shutdown();
    }

    // Start main loop for publishing images to ROS topics
    //m_ROSTimer_MainLoop.start();
}


ForgetfulSimulator::~ForgetfulSimulator() {}





void ForgetfulSimulator::debugNode()
{
    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) 
        ros::console::notifyLoggerLevelsChanged();

    std::cout << "\n\nfeksjfkrvshndjgvnrgrsgvnvn\n\n";


    MY_ROS_DEBUG("[%s]    Debug node.", m_ROSNodeName);

    // Set request from ROS parameters
    forgetful_drones_msgs::BuildDroneRacingSimulation::Request req;

        switch (m_UnitySceneIdx)
        {
        case 0: 
            req.UnityScene = forgetful_drones_msgs::BuildDroneRacingSimulation::Request::SPACESHIP_INTERIOR;
            MY_ROS_DEBUG("[%s]    Unity scene: [%s]", m_ROSNodeName, "SPACESHIP_INTERIOR");
            break;
        
        case 1: 
            req.UnityScene = forgetful_drones_msgs::BuildDroneRacingSimulation::Request::DESTROYED_CITY;
            MY_ROS_DEBUG("[%s]    Unity scene: [%s]", m_ROSNodeName, "DESTROYED_CITY");
            break;

        case 2: 
            req.UnityScene = forgetful_drones_msgs::BuildDroneRacingSimulation::Request::INDUSTRIAL_PARK;
            MY_ROS_DEBUG("[%s]    Unity scene: [%s]", m_ROSNodeName, "INDUSTRIAL_PARK");
            break;
        
        case 3: 
            req.UnityScene = forgetful_drones_msgs::BuildDroneRacingSimulation::Request::POLYGON_CITY;
            MY_ROS_DEBUG("[%s]    Unity scene: [%s]", m_ROSNodeName, "POLYGON_CITY");
            break;

        case 4: 
            req.UnityScene = forgetful_drones_msgs::BuildDroneRacingSimulation::Request::DESERT_MOUNTAIN;
            MY_ROS_DEBUG("[%s]    Unity scene: [%s]", m_ROSNodeName, "DESERT_MOUNTAIN");
            break;

        default:
            MY_ROS_ERROR("[%s]    Requested unity scene is unknown.", m_ROSNodeName);
            return; break;
        }

        switch (m_UnitySiteIdx)
        {
        case 0: 
            req.RacetrackSite = forgetful_drones_msgs::BuildDroneRacingSimulation::Request::SITE_A;
            MY_ROS_DEBUG("[%s]    Racetrack site: [%s]", m_ROSNodeName, "SITE_A");
            break;
        
        case 1: 
            req.RacetrackSite = forgetful_drones_msgs::BuildDroneRacingSimulation::Request::SITE_B;
            MY_ROS_DEBUG("[%s]    Racetrack site: [%s]", m_ROSNodeName, "SITE_B");
            break;

        case 2: 
            req.RacetrackSite = forgetful_drones_msgs::BuildDroneRacingSimulation::Request::SITE_C;
            MY_ROS_DEBUG("[%s]    Racetrack site: [%s]", m_ROSNodeName, "SITE_C");
            break;
        
        default:
            MY_ROS_ERROR("[%s]    Requested racetrack site is unknown.", m_ROSNodeName);
            return; break;
        }

        switch (m_RacetrackTypeIdx)
        {
        case 0: 
            req.RacetrackType = forgetful_drones_msgs::BuildDroneRacingSimulation::Request::FIGURE8_DETERMINISTIC;
            MY_ROS_DEBUG("[%s]    Racetrack type: [%s]", m_ROSNodeName, "FIGURE8_DETERMINISTIC");
            break;
        
        case 1: 
            req.RacetrackType = forgetful_drones_msgs::BuildDroneRacingSimulation::Request::FIGURE8_RANDOMIZED;
            MY_ROS_DEBUG("[%s]    Racetrack type: [%s]", m_ROSNodeName, "FIGURE8_RANDOMIZED");
            break;

        case 2: 
            req.RacetrackType = forgetful_drones_msgs::BuildDroneRacingSimulation::Request::CIRCUIT_RANDOMIZED;
            MY_ROS_DEBUG("[%s]    Racetrack type: [%s]", m_ROSNodeName, "CIRCUIT_RANDOMIZED");
            break;

        case 3: 
            req.RacetrackType = forgetful_drones_msgs::BuildDroneRacingSimulation::Request::SPRINT_RANDOMIZED;
            MY_ROS_DEBUG("[%s]    Racetrack type: [%s]", m_ROSNodeName, "SPRINT_RANDOMIZED");
            break;
        
        default:
            MY_ROS_ERROR("[%s]    Requested racetrack type is unknown.", m_ROSNodeName);
            return; break;
        }
            
        switch (m_RaceGateTypeIdx)
        {
        case 0: 
            req.RaceGateType = forgetful_drones_msgs::BuildDroneRacingSimulation::Request::RPG_GATE;
            MY_ROS_DEBUG("[%s]    Race gate type: [%s]", m_ROSNodeName, "RPG_GATE");
            break;
        case 1: 
            req.RaceGateType = forgetful_drones_msgs::BuildDroneRacingSimulation::Request::TUB_DAI_GATE;
            MY_ROS_DEBUG("[%s]    Race gate type: [%s]", m_ROSNodeName, "TUB_DAI_GATE");
            break;
        case 2: 
            req.RaceGateType = forgetful_drones_msgs::BuildDroneRacingSimulation::Request::THU_DME_GATE;
            MY_ROS_DEBUG("[%s]    Race gate type: [%s]", m_ROSNodeName, "THU_DME_GATE");
            break;
        
        default:
            MY_ROS_ERROR("[%s]    Requested race gate type is unknown.", m_ROSNodeName);
            return; break;
        }



    buildDroneRacingSimulation(req.UnityScene, req.RacetrackSite, req.RacetrackType, req.RaceGateType);
}




void ForgetfulSimulator::setRacetrackPose()
{
    Pose SPACESHIP_INTERIOR_A{Eigen::Vector3f{  -96.3   ,   3.4     ,   0.0     }, EigenQuaternionF_From_Yaw(100.0, true)};
    Pose SPACESHIP_INTERIOR_B{Eigen::Vector3f{  -220.5  ,   -10.9   ,   0.0     }, EigenQuaternionF_From_Yaw(-9.0, true)};
    Pose SPACESHIP_INTERIOR_C{Eigen::Vector3f{  -158.94 ,   36.83   ,   0.0     }, EigenQuaternionF_From_Yaw(-20.0, true)};
    std::array<Pose, 3> SPACESHIP_INTERIOR{
        SPACESHIP_INTERIOR_A, SPACESHIP_INTERIOR_B, SPACESHIP_INTERIOR_C};

    Pose DESTROYED_CITY_A{Eigen::Vector3f{  811.8   ,   373.1   ,   -5.0    }, EigenQuaternionF_From_Yaw(-27.5, true)};
    Pose DESTROYED_CITY_B{Eigen::Vector3f{  255.9   ,   349.8   ,   -5.0    }, EigenQuaternionF_From_Yaw(-69.0, true)};
    Pose DESTROYED_CITY_C{Eigen::Vector3f{  549.3   ,   314.6   ,   -5.0    }, EigenQuaternionF_From_Yaw(-38.6, true)};
    std::array<Pose, 3> DESTROYED_CITY{
        DESTROYED_CITY_A, DESTROYED_CITY_B, DESTROYED_CITY_C};

    Pose INDUSTRIAL_PARK_A{Eigen::Vector3f{ 103.5   ,   -35.7   ,   -2.27   }, EigenQuaternionF_From_Yaw(-90.0, true)};
    Pose INDUSTRIAL_PARK_B{Eigen::Vector3f{ -10.64  ,   -35.39  ,   -2.92   }, EigenQuaternionF_From_Yaw(-90.0, true)};
    Pose INDUSTRIAL_PARK_C{Eigen::Vector3f{ 36.31   ,   -111.05 ,   -2.92   }, EigenQuaternionF_From_Yaw(-20.7, true)};
    std::array<Pose, 3> INDUSTRIAL_PARK{
        INDUSTRIAL_PARK_A, INDUSTRIAL_PARK_B, INDUSTRIAL_PARK_C};
        
    Pose POLYGON_CITY_A{Eigen::Vector3f{    -702.5  ,   65.5    ,   139.73  }, EigenQuaternionF_From_Yaw(0.0, true)};
    Pose POLYGON_CITY_B{Eigen::Vector3f{    -695.02 ,   -37.39  ,   139.53  }, EigenQuaternionF_From_Yaw(-90.0, true)};
    Pose POLYGON_CITY_C{Eigen::Vector3f{    -455.81 ,   76.03   ,   139.53  }, EigenQuaternionF_From_Yaw(-110.0, true)};
    std::array<Pose, 3> POLYGON_CITY{
        POLYGON_CITY_A,POLYGON_CITY_B, POLYGON_CITY_C};

    Pose DESERT_MOUNTAIN_A{Eigen::Vector3f{ 1581.43 ,   1063.43 ,   42.0    }, EigenQuaternionF_From_Yaw(0.0, true)};
    Pose DESERT_MOUNTAIN_B{Eigen::Vector3f{ 1662.6  ,   1210.30 ,   41.3    }, EigenQuaternionF_From_Yaw(70.0, true)};
    Pose DESERT_MOUNTAIN_C{Eigen::Vector3f{ 1775.5  ,   1412.30 ,   42.35   }, EigenQuaternionF_From_Yaw(-70.0, true)};
    std::array<Pose, 3> DESERT_MOUNTAIN{
        DESERT_MOUNTAIN_A, DESERT_MOUNTAIN_B, DESERT_MOUNTAIN_C};

    

    std::array<std::array<Pose, 3>, 5> RacetrackPose{
        SPACESHIP_INTERIOR,
        DESTROYED_CITY,
        INDUSTRIAL_PARK,
        POLYGON_CITY,
        DESERT_MOUNTAIN
        };

    m_RacetrackPose_Flightmare = (RacetrackPose[m_UnitySceneIdx])[m_UnitySiteIdx];
    m_RacetrackPose_Gazebo = (RacetrackPose[m_UnitySceneIdx])[m_UnitySiteIdx];
    m_RacetrackPose_Gazebo.position = {0.0, 0.0, 0.0};


    float X = m_RacetrackPose_Flightmare.position.x();
    float Y = m_RacetrackPose_Flightmare.position.y();
    float Z = m_RacetrackPose_Flightmare.position.z();
    float YAW = m_RacetrackPose_Flightmare.yaw();
    MY_ROS_DEBUG("[%s]    Racetrack pose, X: %f, Y: %f, Z: %f, YAW: %f", m_ROSNodeName, X, Y, Z, YAW);
}


void ForgetfulSimulator::startSimulation()
{
    MY_ROS_INFO("[%s]    Start simulation", m_ROSNodeName);

    if (m_SimulationReady) return;

    // Set unity bridge
    if (m_UnityRender_ON)
    {
        MY_ROS_INFO("[%s]    Unity rendering enabled.", m_ROSNodeName);
        if (m_UnityBridgePtr==nullptr)
        {
            m_UnityBridgePtr = flightlib::UnityBridge::getInstance();
            MY_ROS_INFO("[%s]    Unity bridge created.", m_ROSNodeName);
        }
    }
    else
    {
        m_UnityBridgePtr = nullptr;
        MY_ROS_INFO("[%s]    Unity rendering disabled.", m_ROSNodeName);
    }

    

    deleteAllGazeboModelsExceptDrone();
    
    if (m_UnityRender_ON) spawnGatesInUnity();
    spawnGatesInGazebo();
    spawnGatesInRViz();

    
    spawnGroundPlaneInGazebo();

    if (!m_DroneSpawned)
    {
        spawnDroneInGazebo();
        if (m_UnityRender_ON) spawnDroneInUnity();
        spawnDroneInRViz();
        m_DroneSpawned = true;

        if (m_UnityRender_ON && m_UnityBridgePtr!=nullptr)
        {
            MY_ROS_DEBUG("[%s]    Connect Unity.", m_ROSNodeName)
            m_UnityReady = m_UnityBridgePtr->connectUnity(m_UnitySceneID);
        }
    }
    else
    {
        if (m_UnityRender_ON && m_UnityBridgePtr!=nullptr)
        {
            MY_ROS_DEBUG("[%s]    Switch Unity scene.", m_ROSNodeName)
            m_UnitySceneID = flightlib::UnityScene::SPACESHIP_INTERIOR;
            m_UnityBridgePtr->resetUnity();
            m_UnityBridgePtr->setScene(m_UnitySceneID);
            m_UnityBridgePtr->sendInitialSettings();
            spawnDroneInUnity();
        }
        spawnDroneInRViz();
    }

    //MY_ROS_DEBUG("[%s]    Run Unity standalone.", m_ROSNodeName)      // 
    //system("rosrun flightrender ForgetfulDrone_Flightmare.x86_64");   // error when reconnecting     

    m_SimulationReady = true;
    MY_ROS_DEBUG("[%s]    Start main loop.", m_ROSNodeName)
    m_ROSTimer_MainLoop.start(); // publishing imgs
    MY_ROS_DEBUG("[%s]    Create state estimate subscriber.", m_ROSNodeName)
    m_ROSSub_StateEstimate = m_ROSRNH.subscribe("ground_truth/odometry", 1, &ForgetfulSimulator::ROSCB_StateEstimate, this);
}

void ForgetfulSimulator::ROSCB_SimulatorStart(const std_msgs::Empty::ConstPtr& msg)
{
    MY_ROS_DEBUG("[%s]    Received message of type [%s] on topic [%s].", m_ROSNodeName, "std_msgs::Empty", "simulator/start");
    startSimulation();
}



void ForgetfulSimulator::stopSimulation()
{
    MY_ROS_INFO("[%s]    Stop simulation", m_ROSNodeName);
    
    if (!m_SimulationReady) return;

    MY_ROS_DEBUG("[%s]    Stop main loop.", m_ROSNodeName)
    m_ROSTimer_MainLoop.stop(); // publishing imgs
    MY_ROS_DEBUG("[%s]    Shut down state estimate subscriber.", m_ROSNodeName)
    m_ROSSub_StateEstimate.shutdown();

    MY_ROS_DEBUG("[%s]    Reset Unity.", m_ROSNodeName)
    //m_UnityBridgePtr->disconnectUnity(); // crashes
    m_UnityBridgePtr->resetUnity();
    m_UnityBridgePtr->sendInitialSettings();
    ros::Duration(2.0).sleep();
    
    //system("pkill ForgetfulDrone_");              //stop

    m_UnityBridgePtr = nullptr;

    m_SimulationReady = false;
}



void ForgetfulSimulator::ROSCB_SimulatorStop(const std_msgs::Empty::ConstPtr& msg)
{
    MY_ROS_DEBUG("[%s]    Received message of type [%s] on topic [%s].", m_ROSNodeName, "std_msgs::Empty", "simulator/start");
    stopSimulation();
}


void ForgetfulSimulator::ROSCB_StateEstimate(const nav_msgs::Odometry::ConstPtr& msg) 
{
    if (!m_SimulationReady) return;

    m_FM_DroneState.x[flightlib::QS::POSX] = (flightlib::Scalar) msg->pose.pose.position.x + m_RacetrackPose_Flightmare.position.x();
    m_FM_DroneState.x[flightlib::QS::POSY] = (flightlib::Scalar) msg->pose.pose.position.y + m_RacetrackPose_Flightmare.position.y();
    m_FM_DroneState.x[flightlib::QS::POSZ] = (flightlib::Scalar) msg->pose.pose.position.z + m_RacetrackPose_Flightmare.position.z();
    m_FM_DroneState.x[flightlib::QS::ATTW] = (flightlib::Scalar) msg->pose.pose.orientation.w;
    m_FM_DroneState.x[flightlib::QS::ATTX] = (flightlib::Scalar) msg->pose.pose.orientation.x;
    m_FM_DroneState.x[flightlib::QS::ATTY] = (flightlib::Scalar) msg->pose.pose.orientation.y;
    m_FM_DroneState.x[flightlib::QS::ATTZ] = (flightlib::Scalar) msg->pose.pose.orientation.z;

    m_FM_DronePtr->setState(m_FM_DroneState);

    if (m_UnityRender_ON && m_UnityReady)
    {
        m_UnityBridgePtr->getRender(0);
        m_UnityBridgePtr->handleOutput();
    }
}



void ForgetfulSimulator::ROSCB_MainLoop(const ros::TimerEvent& TE)
{
    if (TE.profile.last_duration.toSec() > 1/m_MainLoopFreq)
        ROS_WARN(
            "[%s]\n  >> Last loop iteration took %f s exceeding nominal duration of %f s.",
            m_ROSNodeName, 
            TE.profile.last_duration.toSec(),
            1/m_MainLoopFreq
            );

    if (!m_SimulationReady) return;


    cv::Mat Img;

    m_FM_RGBCam->getRGBImage(Img);
    sensor_msgs::ImagePtr rgb_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", Img).toImageMsg();
    rgb_msg->header.stamp.fromNSec(m_CamFrameCount);
    m_ITPub_RGBImg.publish(rgb_msg);

    /*m_FM_RGBCam->getDepthMap(Img);
    sensor_msgs::ImagePtr depth_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", Img).toImageMsg();
    depth_msg->header.stamp.fromNSec(m_CamFrameCount);
    m_ITPub_DepthImg.publish(depth_msg);

    m_FM_RGBCam->getSegmentation(Img);
    sensor_msgs::ImagePtr segmentation_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", Img).toImageMsg();
    segmentation_msg->header.stamp.fromNSec(m_CamFrameCount);
    m_ITPub_SegmentationImg.publish(segmentation_msg);

    m_FM_RGBCam->getOpticalFlow(Img);
    sensor_msgs::ImagePtr opticflow_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", Img).toImageMsg();
    opticflow_msg->header.stamp.fromNSec(m_CamFrameCount);
    m_ITPub_OpticalFlowImg.publish(opticflow_msg); */

    m_CamFrameCount++;
}










bool ForgetfulSimulator::ROSCB_buildDroneRacingSimulation(
    forgetful_drones_msgs::BuildDroneRacingSimulation::Request& req,
    forgetful_drones_msgs::BuildDroneRacingSimulation::Response& res
    )
{
    buildDroneRacingSimulation(
        req.UnityScene,
        req.RacetrackSite,
        req.RacetrackType,
        req.RaceGateType
        );

    // Set service response
    res.DroneInitPose = m_DroneInitPose_Gazebo.as_geometry_msg();
    res.GatesInitPose.clear();
    for (size_t GateIdx = 0; GateIdx < m_GatesInitPose_Gazebo.size(); GateIdx++)
        res.GatesInitPose.push_back(m_GatesInitPose_Gazebo[GateIdx].as_geometry_msg());

    return true;
}





bool ForgetfulSimulator::buildDroneRacingSimulation(
    //const geometry_msgs::Pose& RaceTrackPose_WorldRF,
    //const forgetful_drones_msgs::BuildDroneRacingSimulation::Request::_RaceType_type& RaceTrackType,
    //const double& GateWaypointHeight, //-> GateType
    //const forgetful_drones_msgs::BuildDroneRacingSimulation::Request::_DynGatesActivated_type& GatesDynamic
    const forgetful_drones_msgs::BuildDroneRacingSimulation::Request::_UnityScene_type UnityScene,
    const forgetful_drones_msgs::BuildDroneRacingSimulation::Request::_RacetrackSite_type& RacetrackSite,
    const forgetful_drones_msgs::BuildDroneRacingSimulation::Request::_RacetrackType_type& RacetrackType,
    const forgetful_drones_msgs::BuildDroneRacingSimulation::Request::_RaceGateType_type& RaceGateType
    )
{
    m_SimulationReady = false;
    m_RacetrackPose_Flightmare = Pose();
    m_RacetrackPose_Gazebo = Pose();
    m_GateWaypointHeight = 0.0;
    m_GatesInitPose_Flightmare.clear();
    m_GatesInitPose_Gazebo.clear();
    m_DroneInitPose_Flightmare = Pose();
    m_DroneInitPose_Gazebo = Pose();
    m_GroundPlaneOrigin_Gazebo = {0.0, 0.0, 0.0};
    m_GroundPlaneSize_Gazebo = {0.0, 0.0};
    
    
    


    if (UnityScene == forgetful_drones_msgs::BuildDroneRacingSimulation::Request::RANDOM_CHOICE)
    {
        std::array<uint8_t, 5> UnityScenes{
            forgetful_drones_msgs::BuildDroneRacingSimulation::Request::SPACESHIP_INTERIOR,
            forgetful_drones_msgs::BuildDroneRacingSimulation::Request::DESTROYED_CITY,
            forgetful_drones_msgs::BuildDroneRacingSimulation::Request::INDUSTRIAL_PARK,
            forgetful_drones_msgs::BuildDroneRacingSimulation::Request::POLYGON_CITY,
            forgetful_drones_msgs::BuildDroneRacingSimulation::Request::DESERT_MOUNTAIN
            };
        std::uniform_int_distribution<size_t> UIDistri_0To4{0, 4};
        const_cast<uint8_t&>(UnityScene) = UnityScenes[UIDistri_0To4(m_RandEngine)];
        MY_ROS_DEBUG("[%s]    Unity scene randomly chosen.", m_ROSNodeName);
    }
    switch (UnityScene) // m_UnitySceneID
    {   
    case forgetful_drones_msgs::BuildDroneRacingSimulation::Request::SPACESHIP_INTERIOR:
        m_UnitySceneID = flightlib::UnityScene::SPACESHIP_INTERIOR;
        const_cast<int&>(m_UnitySceneIdx) = 0;
        MY_ROS_DEBUG("[%s]    Unity scene: [%s]", m_ROSNodeName, "SPACESHIP_INTERIOR");
        break;

    case forgetful_drones_msgs::BuildDroneRacingSimulation::Request::DESTROYED_CITY:
        m_UnitySceneID = flightlib::UnityScene::DESTROYED_CITY;
        const_cast<int&>(m_UnitySceneIdx) = 1;
        MY_ROS_DEBUG("[%s]    Unity scene: [%s]", m_ROSNodeName, "DESTROYED_CITY");
        break;

    case forgetful_drones_msgs::BuildDroneRacingSimulation::Request::INDUSTRIAL_PARK:
        m_UnitySceneID = flightlib::UnityScene::INDUSTRIAL_PARK;
        const_cast<int&>(m_UnitySceneIdx) = 2;
        MY_ROS_DEBUG("[%s]    Unity scene: [%s]", m_ROSNodeName, "INDUSTRIAL_PARK");
        break;
    
    case forgetful_drones_msgs::BuildDroneRacingSimulation::Request::POLYGON_CITY:
        m_UnitySceneID = flightlib::UnityScene::POLYGON_CITY;
        const_cast<int&>(m_UnitySceneIdx) = 3;
        MY_ROS_DEBUG("[%s]    Unity scene: [%s]", m_ROSNodeName, "POLYGON_CITY");
        break;
    
    case forgetful_drones_msgs::BuildDroneRacingSimulation::Request::DESERT_MOUNTAIN:
        m_UnitySceneID = flightlib::UnityScene::DESERT_MOUNTAIN;
        const_cast<int&>(m_UnitySceneIdx) = 4;
        MY_ROS_DEBUG("[%s]    Unity scene: [%s]", m_ROSNodeName, "DESERT_MOUNTAIN");
        break;
    
    default:
        MY_ROS_ERROR("[%s]\n  >> Requested Unity scene is unknown.", m_ROSNodeName);
        return false; break;
    }


    if (RacetrackSite == forgetful_drones_msgs::BuildDroneRacingSimulation::Request::RANDOM_CHOICE)
    {
        std::array<uint8_t, 3> RacetrackSites{
            forgetful_drones_msgs::BuildDroneRacingSimulation::Request::SITE_A,
            forgetful_drones_msgs::BuildDroneRacingSimulation::Request::SITE_B,
            forgetful_drones_msgs::BuildDroneRacingSimulation::Request::SITE_C
            };
        std::uniform_int_distribution<size_t> UIDistri_0To2{0, 2};
        const_cast<uint8_t&>(RacetrackSite) = RacetrackSites[UIDistri_0To2(m_RandEngine)];
        MY_ROS_DEBUG("[%s]    Racetrack site randomly chosen.", m_ROSNodeName);
    }
    switch (RacetrackSite) 
    {
    case forgetful_drones_msgs::BuildDroneRacingSimulation::Request::SITE_A:
        const_cast<int&>(m_UnitySiteIdx) = 0;
        MY_ROS_DEBUG("[%s]    Racetrack site: [%s]", m_ROSNodeName, "SITE_A");
        break;

    case forgetful_drones_msgs::BuildDroneRacingSimulation::Request::SITE_B:
        const_cast<int&>(m_UnitySiteIdx) = 1;
        MY_ROS_DEBUG("[%s]    Racetrack site: [%s]", m_ROSNodeName, "SITE_B");
        break;

    case forgetful_drones_msgs::BuildDroneRacingSimulation::Request::SITE_C:
        const_cast<int&>(m_UnitySiteIdx) = 2;
        MY_ROS_DEBUG("[%s]    Racetrack site: [%s]", m_ROSNodeName, "SITE_C");
        break;

    default:
        MY_ROS_ERROR("[%s]\n  >> Requested racetrack site is unknown.", m_ROSNodeName);
        return false; break;
    }


    setRacetrackPose();



    if (RaceGateType == forgetful_drones_msgs::BuildDroneRacingSimulation::Request::RANDOM_CHOICE)
    {
        std::array<uint8_t, 3> RaceGateTypes{
            forgetful_drones_msgs::BuildDroneRacingSimulation::Request::RPG_GATE,
            forgetful_drones_msgs::BuildDroneRacingSimulation::Request::TUB_DAI_GATE,
            forgetful_drones_msgs::BuildDroneRacingSimulation::Request::THU_DME_GATE
            };
        std::uniform_int_distribution<size_t> UIDistri_0To2{0, 0};
        const_cast<uint8_t&>(RaceGateType) = RaceGateTypes[UIDistri_0To2(m_RandEngine)];
        MY_ROS_DEBUG("[%s]    Race gate type randomly chosen.", m_ROSNodeName);
    }
    switch (RaceGateType)
    {
    case forgetful_drones_msgs::BuildDroneRacingSimulation::Request::RPG_GATE:
        m_GateWaypointHeight = 2.00; //2.02
        m_GatePrefabID = "rpg_gate";
        m_GateSTLFilePath = m_ROSNodePath + "/gazebo/rpg_gate/model.stl";
        m_GateSDFFilePath = m_ROSNodePath + "/gazebo/rpg_gate/model.sdf";
        MY_ROS_DEBUG("[%s]    Race gate type: [%s]", m_ROSNodeName, "RPG_GATE");
        break;

    case forgetful_drones_msgs::BuildDroneRacingSimulation::Request::TUB_DAI_GATE:
        m_GateWaypointHeight = 2.00; //2.02
        m_GatePrefabID = "tub_dai_gate";
        m_GateSTLFilePath = m_ROSNodePath + "/gazebo/tub_dai_gate/model.stl";
        m_GateSDFFilePath = m_ROSNodePath + "/gazebo/tub_dai_gate/model.sdf";
        MY_ROS_DEBUG("[%s]    Race gate type: [%s]", m_ROSNodeName, "TUB_DAI_GATE");
        break;
    
    case forgetful_drones_msgs::BuildDroneRacingSimulation::Request::THU_DME_GATE:
        m_GateWaypointHeight = 2.00; //2.02
        m_GatePrefabID = "thu_dme_gate";
        m_GateSTLFilePath = m_ROSNodePath + "/gazebo/thu_dme_gate/model.stl";
        m_GateSDFFilePath = m_ROSNodePath + "/gazebo/thu_dme_gate/model.sdf";
        MY_ROS_DEBUG("[%s]    Race gate type: [%s]", m_ROSNodeName, "THU_DME_GATE");
        break;
    
    default:
        MY_ROS_ERROR("[%s]\n  >> Requested race gate type is unknown.", m_ROSNodeName);
        return false; break;
    }


    


    if (RacetrackType == forgetful_drones_msgs::BuildDroneRacingSimulation::Request::RANDOM_CHOICE)
    {
        std::array<uint8_t, 3> RacetrackTypes{
            forgetful_drones_msgs::BuildDroneRacingSimulation::Request::FIGURE8_RANDOMIZED,
            //forgetful_drones_msgs::BuildDroneRacingSimulation::Request::CIRCUIT_RANDOMIZED,
            //forgetful_drones_msgs::BuildDroneRacingSimulation::Request::SPRINT_RANDOMIZED
            };
        std::uniform_int_distribution<size_t> UIDistri_0To2{0, 2};
        const_cast<uint8_t&>(RacetrackType) = RacetrackTypes[UIDistri_0To2(m_RandEngine)];
        MY_ROS_DEBUG("[%s]    Racetrack type randomly chosen.", m_ROSNodeName);
    }
    switch (RacetrackType)
    {
        case forgetful_drones_msgs::BuildDroneRacingSimulation::Request::FIGURE8_DETERMINISTIC:
            computeRaceTrack_Figure8(RacetrackType);
            MY_ROS_DEBUG("[%s]    Racetrack type: [%s]", m_ROSNodeName, "FIGURE8_DETERMINISTIC");
            break;

        case forgetful_drones_msgs::BuildDroneRacingSimulation::Request::FIGURE8_RANDOMIZED:
            computeRaceTrack_Figure8(RacetrackType);
            MY_ROS_DEBUG("[%s]    Racetrack type: [%s]", m_ROSNodeName, "FIGURE8_RANDOMIZED");
            break;

        case forgetful_drones_msgs::BuildDroneRacingSimulation::Request::CIRCUIT_RANDOMIZED:
            MY_ROS_ERROR("[%s]\n  >> Requested racetrack type not implemented yet.", m_ROSNodeName);
            return false; break;

        case forgetful_drones_msgs::BuildDroneRacingSimulation::Request::SPRINT_RANDOMIZED: 
            //computeGatePoses_SprintRand(); break;
            MY_ROS_ERROR("[%s]\n  >> Requested racetrack type not implemented yet.", m_ROSNodeName);
            return false; break;

        default:
            MY_ROS_ERROR("[%s]\n  >> Requested racetrack type is unknown.", m_ROSNodeName);
            return false; break;
    }

    if (m_Debug_ON)
    {
        std::vector<float> Gates_X, Gates_Y, Gates_Z;
        float Gates_MaxX = - 1e10;
        float Gates_MinX = + 1e10;
        float Gates_MaxY = - 1e10;
        float Gates_MinY = + 1e10;
        for (int Gate_i = 0; Gate_i < m_GatesInitPose_Flightmare.size(); Gate_i++)
        {
            Gates_X.push_back(m_GatesInitPose_Flightmare[Gate_i].position.x());
            Gates_Y.push_back(m_GatesInitPose_Flightmare[Gate_i].position.y());
            Gates_Z.push_back(m_GatesInitPose_Flightmare[Gate_i].position.z());

            Gates_MaxX = std::max(Gates_MaxX, Gates_X[Gate_i]);
            Gates_MinX = std::min(Gates_MinX, Gates_X[Gate_i]);
            Gates_MaxY = std::max(Gates_MaxY, Gates_Y[Gate_i]);
            Gates_MinY = std::min(Gates_MinY, Gates_Y[Gate_i]);
        }

        matplotlibcpp::figure_size(3840, 2160); // Ultra HD
        matplotlibcpp::title("Initial Position of gates and drone in x/y-Plane");

        matplotlibcpp::scatter(Gates_X, Gates_Y, 100);

        float TextShift_X = (Gates_MaxX - Gates_MinX)/100;
        float TextShift_Y = (Gates_MaxY - Gates_MinY)/100;
        for (int Gate_i = 0; Gate_i < m_GatesInitPose_Flightmare.size(); Gate_i++)
        {
            const std::string Text = "Gate " + std::to_string(Gate_i);
            matplotlibcpp::text(Gates_X[Gate_i] + TextShift_X, Gates_Y[Gate_i] + TextShift_Y, Text);
        }

        std::vector<float> Drone_X {m_DroneInitPose_Flightmare.position.x()};
        std::vector<float> Drone_Y {m_DroneInitPose_Flightmare.position.y()};
        matplotlibcpp::scatter(Drone_X, Drone_Y, 100);
        matplotlibcpp::text(Drone_X[0] + TextShift_X, Drone_Y[0] + TextShift_Y, "Drone start position");

        //matplotlibcpp::legend();
        //matplotlibcpp::save("./basic.png");
        matplotlibcpp::show();
    }


    computeGazeboGroundPlane();

    


    /*
    // Spawn environment in Gazebo for collision
    gazebo_msgs::SpawnModel srv;
        srv.request.reference_frame = "world";
        srv.request.robot_namespace = "environment";
    srv.request.model_name = "spaceship_interior_00";
    std::ifstream ModelXML(m_ROSNodePath + "/gazebo/spaceship_interior/00/model.sdf");
    std::stringstream Buffer; Buffer << ModelXML.rdbuf();
    srv.request.model_xml = Buffer.str();
    //std::cout<<Buffer.str()<<std::endl;
    //srv.request.initial_pose.position 
    //    = GeometryMsgsPoint_From_EigenVector3d( m_GatesInitPos_WorldRF[GateIdx] );
    //srv.request.initial_pose.orientation
    //    = tf::createQuaternionMsgFromYaw( m_GatesInitYaw_WorldRF[GateIdx] );
    
    if ( m_ROSSrvCl_GazeboSpawnSDFModel.call( srv ) )
        MY_ROS_INFO("[%s]\n  > Successfully spawned \"%s\" in Gazebo.", 
            m_ROSNodeName, srv.request.model_name.c_str());
    else
        MY_ROS_ERROR("[%s]\n  > Failed to spawn \"%s\" in Gazebo.",
            m_ROSNodeName, srv.request.model_name.c_str());
    */



    


    /*
    // --> get point cloud from unity and spawn collision model in gazebo
    flightlib::PointCloudMessage_t PointCloudMsg;
    PointCloudMsg.path = m_ROSNodePath + "/tmp/";
    PointCloudMsg.file_name = "point_cloud"; // .ply is added automatically
    PointCloudMsg.origin = {
        (float)RaceTrackPose_WorldRF.position.x, 
        (float)RaceTrackPose_WorldRF.position.y,
        (float)RaceTrackPose_WorldRF.position.z
        };
    PointCloudMsg.range = {
        35.0, 
        35.0, 
        (float)(RaceTrackPose_WorldRF.position.z + 3*GateWaypointHeight)
        };
    PointCloudMsg.resolution = 0.10;
    bool FM_PointCloudReady = m_UnityBridgePtr->getPointCloud(PointCloudMsg);

    if (FM_PointCloudReady)
        MY_ROS_INFO("#############################################################################");
    else
        MY_ROS_INFO("+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++");
    */
    
    
    //initRandParamsOfDynGates();
    //req.DynGatesActivated? m_ROSTimer_SimulatorLoop.start() : m_ROSTimer_SimulatorLoop.stop();
    
    
    return true;
}






void ForgetfulSimulator::computeRaceTrack_Figure8(const forgetful_drones_msgs::BuildDroneRacingSimulation::Request::_RacetrackType_type& RacetrackType)
{
    // Data in the race track reference frame
        // Gate (center points) and drone start position:
        constexpr size_t GatesN = 14;
        std::array<Eigen::Vector3d, GatesN +1> GatesAndDroneInitPos_RaceTrackRF = {
            Eigen::Vector3d{ -20.45 ,   -8.65   ,   m_GateWaypointHeight  }, // #01
            Eigen::Vector3d{ -12.55 ,   -11.15  ,   m_GateWaypointHeight  }, // #02
            Eigen::Vector3d{ -4.15  ,   -5.35   ,   m_GateWaypointHeight  }, // #03
            Eigen::Vector3d{ 3.45   ,   4.25    ,   m_GateWaypointHeight  }, // #04
            Eigen::Vector3d{ 11.95  ,   11.15   ,   m_GateWaypointHeight  }, // #05
            Eigen::Vector3d{ 21.85  ,   6.85    ,   m_GateWaypointHeight  }, // #06
            Eigen::Vector3d{ 24.25  ,   -1.75   ,   m_GateWaypointHeight  }, // #07
            Eigen::Vector3d{ 19.25  ,   -9.55   ,   m_GateWaypointHeight  }, // #08
            Eigen::Vector3d{ 10.55  ,   -10.65  ,   m_GateWaypointHeight  }, // #09
            Eigen::Vector3d{ 2.85   ,   -5.95   ,   m_GateWaypointHeight  }, // #10
            Eigen::Vector3d{ -4.95  ,   4.65    ,   m_GateWaypointHeight  }, // #11
            Eigen::Vector3d{ -12.95 ,   9.65    ,   m_GateWaypointHeight  }, // #12
            Eigen::Vector3d{ -21.05 ,   6.65    ,   m_GateWaypointHeight  }, // #13
            Eigen::Vector3d{ -24.25 ,   -2.15   ,   m_GateWaypointHeight  }, // #14
            Eigen::Vector3d{ -24.00 ,   6.65    ,   0.2                     }  // Drone
            };
        // Gate and drone yaws in race track reference frame:
        std::array<double, GatesN +1> GatesAndDroneInitYaw_RaceTrackRF = {
            -2.01e+00   ,   // #01
            -1.57e+00   ,   // #02
            -6.01e-01   ,   // #03
            -3.77e+00   ,   // #04
            1.93e+00    ,   // #05
            9.99e-01    ,   // #06
            -7.96e-04   ,   // #07
            -4.17e+00   ,   // #08
            1.53e+00    ,   // #09
            -2.57e+00   ,   // #10
            -2.47e+00   ,   // #11
            -4.67e+00   ,   // #12
            -7.71e-01   ,   // #13
            -3.07e+00   ,   // #14
            -M_PI/2        // Drone 
            };

    

    std::vector<float> Gates_X_Det, Gates_Y_Det, Gates_Z_Det;
    float Gates_MaxX, Gates_MinX, Gates_MaxY, Gates_MinY;
    std::vector<float> Drone_X_Det {static_cast<float>(GatesAndDroneInitPos_RaceTrackRF.back().x())};
    std::vector<float> Drone_Y_Det {static_cast<float>(GatesAndDroneInitPos_RaceTrackRF.back().y())};
    if (m_Debug_ON)
    {
        Gates_MaxX = - 1e10;
        Gates_MinX = + 1e10;
        Gates_MaxY = - 1e10;
        Gates_MinY = + 1e10;
        for (int Gate_i = 0; Gate_i < GatesN; Gate_i++)
        {
            Gates_X_Det.push_back(GatesAndDroneInitPos_RaceTrackRF[Gate_i].x());
            Gates_Y_Det.push_back(GatesAndDroneInitPos_RaceTrackRF[Gate_i].y());
            Gates_Z_Det.push_back(GatesAndDroneInitPos_RaceTrackRF[Gate_i].z()); 

            Gates_MaxX = std::max(Gates_MaxX, Gates_X_Det[Gate_i]);
            Gates_MinX = std::min(Gates_MinX, Gates_X_Det[Gate_i]);
            Gates_MaxY = std::max(Gates_MaxY, Gates_Y_Det[Gate_i]);
            Gates_MinY = std::min(Gates_MinY, Gates_Y_Det[Gate_i]);
        }
    }


    // Randomize racetrack if desired
    if (RacetrackType == forgetful_drones_msgs::BuildDroneRacingSimulation::Request::FIGURE8_RANDOMIZED)
    {
        std::uniform_real_distribution<double> URDistri_MinusToPlus1(-1.0, 1.0);
        std::uniform_real_distribution<double> URDistri_Scale(m_RandFig8_MinScale, m_RandFig8_MaxScale);
        double Scale = URDistri_Scale(m_RandEngine);

        for (size_t GateIdx = 0; GateIdx < GatesN; GateIdx++)
        {
            Eigen::Vector3d AxialShift = m_RandFig8_MaxAxShift * Eigen::Vector3d{ 
                    URDistri_MinusToPlus1(m_RandEngine), 
                    URDistri_MinusToPlus1(m_RandEngine), 
                    URDistri_MinusToPlus1(m_RandEngine) +1.0
                };
            GatesAndDroneInitPos_RaceTrackRF[GateIdx] += AxialShift;
            GatesAndDroneInitPos_RaceTrackRF[GateIdx] *= Scale;

            double YawTwist = m_RandFig8_MaxYawTwist * URDistri_MinusToPlus1(m_RandEngine);
            GatesAndDroneInitYaw_RaceTrackRF[GateIdx] += YawTwist;

            if (GateIdx == GatesAndDroneInitPos_RaceTrackRF.size() -3) // drone starting position is next to second last gate
            {
                GatesAndDroneInitPos_RaceTrackRF.back() += AxialShift;
                GatesAndDroneInitPos_RaceTrackRF.back() *= Scale;
            }
        }
    }


    
    
    if (m_Debug_ON)
    {
        std::vector<float> Gates_X_Rand, Gates_Y_Rand, Gates_Z_Rand;
        std::vector<float> Drone_X_Rand {static_cast<float>(GatesAndDroneInitPos_RaceTrackRF.back().x())};
        std::vector<float> Drone_Y_Rand {static_cast<float>(GatesAndDroneInitPos_RaceTrackRF.back().y())};
        for (int Gate_i = 0; Gate_i < GatesN; Gate_i++)
        {
            Gates_X_Rand.push_back(GatesAndDroneInitPos_RaceTrackRF[Gate_i].x());
            Gates_Y_Rand.push_back(GatesAndDroneInitPos_RaceTrackRF[Gate_i].y());
            Gates_Z_Rand.push_back(GatesAndDroneInitPos_RaceTrackRF[Gate_i].z());

            Gates_MaxX = std::max(Gates_MaxX, Gates_X_Rand[Gate_i]);
            Gates_MinX = std::min(Gates_MinX, Gates_X_Rand[Gate_i]);
            Gates_MaxY = std::max(Gates_MaxY, Gates_Y_Rand[Gate_i]);
            Gates_MinY = std::min(Gates_MinY, Gates_Y_Rand[Gate_i]);
        }

        // HERE

        matplotlibcpp::figure_size(3840, 2160); // Ultra HD
        matplotlibcpp::title("Deterministic and randomized figure-8 racetrack in x/y-Plane");

        std::map<std::string, std::string> keywords_det;
        keywords_det.insert(std::pair<std::string, std::string>("label", "Deterministic"));
        keywords_det.insert(std::pair<std::string, std::string>("color", "black"));
        matplotlibcpp::scatter(Gates_X_Det, Gates_Y_Det, 100, keywords_det);

        std::map<std::string, std::string> keywords_rand;
        keywords_rand.insert(std::pair<std::string, std::string>("label", "Randomized"));
        keywords_rand.insert(std::pair<std::string, std::string>("color", "red"));
        matplotlibcpp::scatter(Gates_X_Rand, Gates_Y_Rand, 100, keywords_rand);

        float TextShift_X = (Gates_MaxX - Gates_MinX)/100;
        float TextShift_Y = (Gates_MaxY - Gates_MinY)/100;
        for (int Gate_i = 0; Gate_i < GatesN; Gate_i++)
        {
            const std::string Text = "Gate " + std::to_string(Gate_i);
            matplotlibcpp::text(Gates_X_Det[Gate_i] + TextShift_X, Gates_Y_Det[Gate_i] + TextShift_Y, Text);
        }

        std::map<std::string, std::string> keywords_det2;
        keywords_det2.insert(std::pair<std::string, std::string>("color", "black"));
        matplotlibcpp::scatter(Drone_X_Det, Drone_Y_Det, 200, keywords_det2);
        std::map<std::string, std::string> keywords_rand2;
        keywords_rand2.insert(std::pair<std::string, std::string>("color", "red"));
        matplotlibcpp::scatter(Drone_X_Rand, Drone_Y_Rand, 200, keywords_rand2);
        matplotlibcpp::text(Drone_X_Det[0] + TextShift_X, Drone_Y_Det[0] + TextShift_Y, "Drone start");

        matplotlibcpp::legend();
        //matplotlibcpp::save("./basic.png");
        matplotlibcpp::show();
    }

   

    

    // Resize the member containing the gate poses
    m_GatesInitPose_Flightmare.resize(GatesN);
    m_GatesInitPose_Gazebo.resize(GatesN);

    // Create Transformation between reference frames: race track -> world 
    kindr::minimal::QuatTransformation T_WorldRF_RaceTrackRF_Flightmare;
    tf::poseMsgToKindr(m_RacetrackPose_Flightmare.as_geometry_msg(), &T_WorldRF_RaceTrackRF_Flightmare);
    kindr::minimal::QuatTransformation T_WorldRF_RaceTrackRF_Gazebo;
    tf::poseMsgToKindr(m_RacetrackPose_Gazebo.as_geometry_msg(), &T_WorldRF_RaceTrackRF_Gazebo);


    // Apply transformation to compute gate poses in world reference frame
    for (size_t GateIdx = 0; GateIdx < GatesN; GateIdx++)
    {
        geometry_msgs::Pose GateInitPose_RaceTrackRF;
        GateInitPose_RaceTrackRF.position = GeometryMsgsPoint_From_EigenVector3d(GatesAndDroneInitPos_RaceTrackRF[GateIdx]);
        GateInitPose_RaceTrackRF.orientation = tf::createQuaternionMsgFromYaw(GatesAndDroneInitYaw_RaceTrackRF[GateIdx]);
        
        kindr::minimal::QuatTransformation T_RaceTrackRF_GateInitRF;
        tf::poseMsgToKindr(GateInitPose_RaceTrackRF, &T_RaceTrackRF_GateInitRF);
        kindr::minimal::QuatTransformation T_WorldRF_GateInitRF_Flightmare = T_WorldRF_RaceTrackRF_Flightmare * T_RaceTrackRF_GateInitRF;
        kindr::minimal::QuatTransformation T_WorldRF_GateInitRF_Gazebo = T_WorldRF_RaceTrackRF_Gazebo * T_RaceTrackRF_GateInitRF;

        m_GatesInitPose_Flightmare[GateIdx].position = T_WorldRF_GateInitRF_Flightmare.getPosition().cast<float>();
        m_GatesInitPose_Flightmare[GateIdx].orientation = T_WorldRF_GateInitRF_Flightmare.getEigenQuaternion().cast<float>();

        m_GatesInitPose_Gazebo[GateIdx].position = T_WorldRF_GateInitRF_Gazebo.getPosition().cast<float>();
        m_GatesInitPose_Gazebo[GateIdx].orientation = T_WorldRF_GateInitRF_Gazebo.getEigenQuaternion().cast<float>();
    }

    // Apply transformations to compute drone init pose in world reference frame
    geometry_msgs::Pose DroneInitPose_RaceTrackRF;
    DroneInitPose_RaceTrackRF.position = GeometryMsgsPoint_From_EigenVector3d(GatesAndDroneInitPos_RaceTrackRF[GatesN]);
    DroneInitPose_RaceTrackRF.orientation = tf::createQuaternionMsgFromYaw(GatesAndDroneInitYaw_RaceTrackRF[GatesN]);
    
    kindr::minimal::QuatTransformation T_RaceTrackRF_DroneInitRF;
    tf::poseMsgToKindr(DroneInitPose_RaceTrackRF, &T_RaceTrackRF_DroneInitRF);
    kindr::minimal::QuatTransformation T_WorldRF_DroneInitRF_Flightmare = T_WorldRF_RaceTrackRF_Flightmare * T_RaceTrackRF_DroneInitRF;
    kindr::minimal::QuatTransformation T_WorldRF_DroneInitRF_Gazebo = T_WorldRF_RaceTrackRF_Gazebo * T_RaceTrackRF_DroneInitRF;

    m_DroneInitPose_Flightmare.position = T_WorldRF_DroneInitRF_Flightmare.getPosition().cast<float>();
    m_DroneInitPose_Flightmare.orientation = T_WorldRF_DroneInitRF_Flightmare.getEigenQuaternion().cast<float>();
    m_DroneInitPose_Gazebo.position = T_WorldRF_DroneInitRF_Gazebo.getPosition().cast<float>();
    m_DroneInitPose_Gazebo.orientation = T_WorldRF_DroneInitRF_Gazebo.getEigenQuaternion().cast<float>();
}


void ForgetfulSimulator::spawnGatesInUnity()
{
    MY_ROS_DEBUG("[%s]    Spawn Gates in Unity.", m_ROSNodeName)

    const size_t& GatesN = m_GatesInitPose_Flightmare.size();
    int IDWidth = static_cast<int>(log10(GatesN) +1);
    std::string IDPrefix = "unity_gate_";
    
    m_UnityGates.resize(GatesN);
    for (size_t GateIdx = 0; GateIdx < GatesN; GateIdx++)
    {
        // ObjectID
        std::stringstream IDSuffix; //GateID.str("");
        IDSuffix
            << std::setw(IDWidth) 
            << std::setfill('0') 
            << GateIdx;
        std::string ObjectID = IDPrefix + IDSuffix.str();

        // Init gates with ObjectID, PrefabID, position and orientation
        //m_UnityGates[GateIdx] = std::make_shared<flightlib::StaticGate>(ObjectID, m_GatePrefabID);
        m_UnityGates[GateIdx] = std::make_shared<flightlib::StaticObject>(ObjectID, m_GatePrefabID);
        m_UnityGates[GateIdx]->setPosition(m_GatesInitPose_Flightmare[GateIdx].position);
        m_UnityGates[GateIdx]->setQuaternion(m_GatesInitPose_Flightmare[GateIdx].orientation);

        // Add gates
        m_UnityBridgePtr->addStaticObject(m_UnityGates[GateIdx]);
        
        MY_ROS_INFO("[%s]    Gate (object ID: [%s], prefab ID: [%s]) spawned in Unity.", 
                m_ROSNodeName, m_UnityGates[GateIdx]->getID().c_str(), m_UnityGates[GateIdx]->getPrefabID().c_str());
    }
}



void ForgetfulSimulator::spawnGatesInGazebo()
{
    MY_ROS_DEBUG("[%s]    Spawn Gates in Gazebo.", m_ROSNodeName)

    const size_t& GatesN = m_GatesInitPose_Gazebo.size();
    int IDWidth = static_cast<int>(log10(GatesN) +1);
    std::string IDPrefix = "gazebo_gate_";

    gazebo_msgs::SpawnModel srv;
        srv.request.reference_frame = "world";
        srv.request.robot_namespace = "race_track";
    
    m_UnityGates.resize(GatesN);
    for (size_t GateIdx = 0; GateIdx < GatesN; GateIdx++)
    {
        // ObjectID
        std::stringstream IDSuffix; //GateID.str("");
        IDSuffix
            << std::setw(IDWidth) 
            << std::setfill('0') 
            << GateIdx;
        std::string ObjectID = IDPrefix + IDSuffix.str();

        srv.request.model_name = IDPrefix + IDSuffix.str();
        std::ifstream ModelXML(m_GateSDFFilePath);
        std::stringstream Buffer; Buffer << ModelXML.rdbuf();
        srv.request.model_xml = Buffer.str();
        srv.request.initial_pose = m_GatesInitPose_Gazebo[GateIdx].as_geometry_msg();
        
        if (m_ROSSrvCl_GazeboSpawnSDFModel.call(srv))
            MY_ROS_INFO("[%s]    Gazebo model [%s] spawned.", 
                m_ROSNodeName, srv.request.model_name.c_str())
        else
            MY_ROS_ERROR("[%s]    Failed to spawn Gazebo model [%s].",
                m_ROSNodeName, srv.request.model_name.c_str())
    }
}


void ForgetfulSimulator::spawnGatesInRViz()
{
    MY_ROS_DEBUG("[%s]    Spawn Gates in RViz.", m_ROSNodeName)

    const size_t& GatesN = m_GatesInitPose_Gazebo.size();

    m_RVizGates.markers.resize(GatesN);

    m_RVizGate.header.frame_id = "world";
    m_RVizGate.scale.x = m_RVizGate.scale.y = m_RVizGate.scale.z = 1.0;
    m_RVizGate.action = visualization_msgs::Marker::ADD;
    m_RVizGate.type = visualization_msgs::Marker::MESH_RESOURCE;
    m_RVizGate.mesh_use_embedded_materials = true;

    for (size_t GateIdx = 0; GateIdx < GatesN; GateIdx++)
    {   
        m_RVizGate.id = GateIdx;
        m_RVizGate.pose = m_GatesInitPose_Gazebo[GateIdx].as_geometry_msg();
        m_RVizGate.mesh_resource = "file://" + m_GateSTLFilePath;
        
        m_RVizGates.markers[GateIdx] = m_RVizGate;
    }

    m_ROSPub_RVizGates.publish( m_RVizGates );
}











void ForgetfulSimulator::spawnDroneInGazebo()
{
    MY_ROS_DEBUG("[%s]    Spawn drone in Gazebo.", m_ROSNodeName)

    gazebo_msgs::SpawnModel srv;
    srv.request.model_name = m_DroneModelName;
    srv.request.model_xml = m_DroneModelDescription;
    srv.request.reference_frame = "world";
    srv.request.initial_pose = m_DroneInitPose_Gazebo.as_geometry_msg();
    
    m_ROSSrvCl_GazeboSpawnURDFModel.call(srv);
}

void ForgetfulSimulator::spawnDroneInUnity()
{
    MY_ROS_DEBUG("[%s]    Spawn drone in Unity.", m_ROSNodeName)

    // Init drone
    m_FM_DronePtr = std::make_shared<flightlib::Quadrotor>();

    // Init and add mono camera
    m_FM_RGBCam = std::make_shared<flightlib::RGBCamera>();
    flightlib::Vector<3> B_r_BC{0.0, 0.0, 0.3};

    constexpr double FMDroneCam_YawOffset = -1*M_PI/2;
    tf::Quaternion FMDroneCam_QuaternionOffset = tf::createQuaternionFromYaw(FMDroneCam_YawOffset);
    flightlib::Matrix<3, 3> R_BC = flightlib::Quaternion{FMDroneCam_QuaternionOffset.w(), FMDroneCam_QuaternionOffset.x(), FMDroneCam_QuaternionOffset.y(), FMDroneCam_QuaternionOffset.z()}.toRotationMatrix();
    m_FM_RGBCam->setRelPose(B_r_BC, R_BC);
    m_FM_RGBCam->setFOV(m_DroneCamFOV);
    m_FM_RGBCam->setWidth(m_DroneCamWidth);
    m_FM_RGBCam->setHeight(m_DroneCamHeight);
    m_FM_RGBCam->setPostProcesscing(std::vector<bool>{false, false, false});  // depth, segmentation, optical flow
    m_FM_DronePtr->addRGBCamera(m_FM_RGBCam);

    // Init drone state and set to initial pose
    m_FM_DroneState.setZero();
    m_FM_DronePtr->reset(m_FM_DroneState);   
    m_FM_DroneState.x[flightlib::QS::POSX] = m_DroneInitPose_Flightmare.position.x();
    m_FM_DroneState.x[flightlib::QS::POSY] = m_DroneInitPose_Flightmare.position.y();
    m_FM_DroneState.x[flightlib::QS::POSZ] = m_DroneInitPose_Flightmare.position.z();
    m_FM_DroneState.x[flightlib::QS::ATTW] = m_DroneInitPose_Flightmare.orientation.w();
    m_FM_DroneState.x[flightlib::QS::ATTX] = m_DroneInitPose_Flightmare.orientation.x();
    m_FM_DroneState.x[flightlib::QS::ATTY] = m_DroneInitPose_Flightmare.orientation.y();
    m_FM_DroneState.x[flightlib::QS::ATTZ] = m_DroneInitPose_Flightmare.orientation.z();
    m_FM_DronePtr->setState(m_FM_DroneState);

    // Add quadcopter
    m_UnityBridgePtr->addQuadrotor(m_FM_DronePtr);
}



//void ForgetfulSimulator::ROS_CB_GazeboModelStates(const gazebo_msgs::ModelStates::ConstPtr &msg) 
//{ 
//    m_GazeboModelStates_Ptr = msg;
//}


void ForgetfulSimulator::deleteAllGazeboModelsExceptDrone()
{
    MY_ROS_DEBUG("[%s]    Delete all Gazebo models except drone.", m_ROSNodeName)
    
    gazebo_msgs::ModelStates::ConstPtr msg 
        = ros::topic::waitForMessage<gazebo_msgs::ModelStates>("/gazebo/model_states", ros::Duration(2.0));
    
    if (msg==nullptr)
    {
        MY_ROS_ERROR("[%s]    Failed to receive Gazebo model states.", m_ROSNodeName);
        return;
    }
    else
    {
        gazebo_msgs::DeleteModel srv;
        for (const std::string& ModelName : msg->name)
        {
            if (ModelName == m_DroneModelName) continue;

            srv.request.model_name = ModelName;
            if (m_ROSSrvCl_GazeboDeleteModel.call(srv))
                MY_ROS_INFO("[%s]    Gazebo model [%s] deleted.", m_ROSNodeName, ModelName.c_str())
            else
                MY_ROS_ERROR( "[%s]\n  >> Failed to delete Gazebo model [%s].", m_ROSNodeName, ModelName.c_str() )
        }
    }    
}



void ForgetfulSimulator::spawnGroundPlaneInGazebo()
{   
    MY_ROS_DEBUG("[%s]    Spawn ground plane in Gazebo.", m_ROSNodeName)

    std::array<std::string, 5> Scenes{"spaceship_interior_", "destroyed_city_", "industrial_park_", "polygon_city_", "desert_mountain"};
    std::array<std::string, 3> Sites{"A", "B", "C"};
    const_cast<std::string&>(m_GroundPlaneID) 
        = std::string("ground_plane_") + Scenes[m_UnitySceneIdx] + Sites[m_UnitySiteIdx];

    gazebo_msgs::SpawnModel srv;
        srv.request.reference_frame = "world";
        srv.request.robot_namespace = "Environment";
        srv.request.model_name = m_GroundPlaneID;
        srv.request.model_xml = getGroundPlaneSDF(m_GroundPlaneSize_Gazebo);
        srv.request.initial_pose.position = GeometryMsgsPoint_From_EigenVector3d(m_GroundPlaneOrigin_Gazebo.cast<double>());

        
    if (m_ROSSrvCl_GazeboSpawnSDFModel.call(srv))
        MY_ROS_INFO("[%s]    Gazebo model [%s] spawned.", m_ROSNodeName, m_GroundPlaneID.c_str())
    else
        MY_ROS_ERROR("[%s]    Failed to spawn Gazebo model [%s].", m_ROSNodeName, m_GroundPlaneID.c_str())
}



std::string ForgetfulSimulator::getGroundPlaneSDF(const Eigen::Vector2f& GroundPlaneSize)
{
    return 
    R"(<?xml version="1.0"?>
    <sdf version="1.6">
        <model name='Ground_Plane_)" + m_GroundPlaneID + R"('>
            <pose frame='world'>0 0 0 0 0 0</pose>
            <static>1</static>
            <link name='link'>
                <collision name='collision'>
                    <geometry>
                        <plane>
                            <normal>0 0 1</normal>
                            <size>)" 
                                + std::to_string(GroundPlaneSize.x()) 
                                + " " 
                                + std::to_string(GroundPlaneSize.y()) 
                                + R"(</size>
                        </plane>
                    </geometry>
                    <surface>
                        <friction>
                            <ode>
                                <mu>100</mu>
                                <mu2>50</mu2>
                            </ode>
                            <torsional>
                                <ode/>
                            </torsional>
                        </friction>
                        <contact>
                            <ode/>
                        </contact>
                        <bounce/>
                    </surface>
                    <max_contacts>10</max_contacts>
                </collision>
                <visual name='visual'>
                    <cast_shadows>false</cast_shadows>
                    <geometry>
                        <plane>
                            <normal>0 0 1</normal>
                            <size>)" 
                                + std::to_string(GroundPlaneSize.x()) 
                                + " " 
                                + std::to_string(GroundPlaneSize.y())
                                + R"(</size>
                        </plane>
                    </geometry>
                </visual>
                <self_collide>0</self_collide>
                <enable_wind>0</enable_wind>
                <kinematic>0</kinematic>
            </link>
        </model>
    </sdf>)";
}




void ForgetfulSimulator::spawnDroneInRViz()
{
    MY_ROS_DEBUG("[%s]    Spawn drone in RViz.", m_ROSNodeName)

    visualization_msgs::MarkerArray RVizDrone;
    RVizDrone.markers.reserve(2 * m_RVizDroneRotorN + 1);

    visualization_msgs::Marker Body;
        Body.header.stamp = ros::Time();
        Body.header.frame_id = m_RVizDroneFrameID;
        Body.ns = "vehicle_body";
        Body.action = visualization_msgs::Marker::ADD;
        Body.type = visualization_msgs::Marker::CUBE;
        Body.scale.x = m_RVizDroneBodyWidth * m_RVizDroneScale;
        Body.scale.y = m_RVizDroneBodyWidth * m_RVizDroneScale;
        Body.scale.z = m_RVizDroneBodyHeight * m_RVizDroneScale;
        setRGBOfVisMarker(VisualizationColors::BLACK, Body);
        Body.color.a = 1.0;
        Body.frame_locked = true;
    RVizDrone.markers.push_back(Body);

    visualization_msgs::Marker Rotor;
        Rotor.header.stamp = ros::Time();
        Rotor.header.frame_id = m_RVizDroneFrameID;
        Rotor.ns = "vehicle_rotor";
        Rotor.action = visualization_msgs::Marker::ADD;
        Rotor.type = visualization_msgs::Marker::CYLINDER;
        Rotor.scale.x = 0.2 * m_RVizDroneScale;
        Rotor.scale.y = 0.2 * m_RVizDroneScale;
        Rotor.scale.z = 0.01 * m_RVizDroneScale;
        setRGBOfVisMarker(VisualizationColors::BLUE, Rotor);
        Rotor.color.a = 0.7;
        Rotor.pose.position.z = 0;
        Rotor.frame_locked = true;
    
    visualization_msgs::Marker Arm;
        Arm.header.stamp = ros::Time();
        Arm.header.frame_id = m_RVizDroneFrameID;
        Arm.ns = "vehicle_arm";
        Arm.action = visualization_msgs::Marker::ADD;
        Arm.type = visualization_msgs::Marker::CUBE;
        Arm.scale.x = m_RVizDroneArmLength * m_RVizDroneScale;
        Arm.scale.y = 0.02 * m_RVizDroneScale;
        Arm.scale.z = 0.01 * m_RVizDroneScale;
        setRGBOfVisMarker(VisualizationColors::BLACK, Arm);
        Arm.color.a = 1.0;
        Arm.pose.position.z = -0.015 * m_RVizDroneScale;
        Arm.frame_locked = true;

        const double RotorAngleIncrement = 2*M_PI / m_RVizDroneRotorN;
        for (double Angle = RotorAngleIncrement /2; Angle <= 2*M_PI; Angle += RotorAngleIncrement) 
        {
            Rotor.pose.position.x = m_RVizDroneArmLength * cos(Angle) * m_RVizDroneScale;
            Rotor.pose.position.y = m_RVizDroneArmLength * sin(Angle) * m_RVizDroneScale;
            Rotor.id++;

            Arm.pose.position.x = Rotor.pose.position.x /2;
            Arm.pose.position.y = Rotor.pose.position.y /2;
            Arm.pose.orientation = tf::createQuaternionMsgFromYaw(Angle);
            Arm.id++;

            RVizDrone.markers.push_back(Rotor);
            RVizDrone.markers.push_back(Arm);
        }

    m_ROSPub_RVizDrone.publish(RVizDrone);
}






void ForgetfulSimulator::computeGazeboGroundPlane()
{
    constexpr float BufferDistance = 10.0;

    float MinX = m_DroneInitPose_Gazebo.position.x(), MaxX = MinX;
    float MinY = m_DroneInitPose_Gazebo.position.y(), MaxY = MinY;

    for (size_t GateIdx = 0; GateIdx < m_GatesInitPose_Gazebo.size(); GateIdx++)
    {
        MinX = std::min(MinX, m_GatesInitPose_Gazebo[ GateIdx ].position.x());
        MaxX = std::max(MaxX, m_GatesInitPose_Gazebo[ GateIdx ].position.x());
        MinY = std::min(MinY, m_GatesInitPose_Gazebo[ GateIdx ].position.y());
        MaxY = std::max(MaxY, m_GatesInitPose_Gazebo[ GateIdx ].position.y());
    }

    MinX -= BufferDistance; MaxX += BufferDistance;
    MinY -= BufferDistance; MaxY += BufferDistance;

    const float OriginX = (MinX + MaxX) /2;
    const float OriginY = (MinY + MaxY) /2;
    const float OriginZ = m_RacetrackPose_Flightmare.position.z();
    const float SizeX = MaxX - MinX;
    const float SizeY = MaxY - MinY;

    m_GroundPlaneOrigin_Gazebo = {OriginX, OriginY, OriginZ};
    m_GroundPlaneSize_Gazebo = {SizeX, SizeY};

    MY_ROS_DEBUG("[%s]    Ground plane, origin: [%f, %f, %f], size: [%f, %f]", 
        m_ROSNodeName, OriginX, OriginY, OriginZ, SizeX, SizeY);
}





//void ForgetfulSimulator::computeRaceTrack_Fig8Rand()
//{
//    computeRaceTrack_Figure8();
//
//    std::uniform_real_distribution<float> URDistri_MinusToPlus1(-1.0, 1.0);
//    std::uniform_real_distribution<float> URDistri_Scale(m_RandFig8_MinScale, m_RandFig8_MaxScale);
//    float Scale = URDistri_Scale(m_RandEngine);
//
//    for (size_t GateIdx = 0; GateIdx < m_GatesInitPose_Flightmare.size(); GateIdx++)
//    {
//        Eigen::Vector3f AxialShift 
//            = m_RandFig8_MaxAxShift * Eigen::Vector3f{ 
//            URDistri_MinusToPlus1(m_RandEngine), 
//            URDistri_MinusToPlus1(m_RandEngine), 
//            URDistri_MinusToPlus1(m_RandEngine) +1.0
//            };
//
//        m_GatesInitPose_Flightmare[GateIdx].position += AxialShift;
//        m_GatesInitPose_Flightmare[GateIdx].position *= Scale;
//
//        if (GateIdx == m_GatesInitPose_Flightmare.size() -2) // drone starting position is next to second last gate
//        {
//            m_DroneInitPose_Flightmare.position += AxialShift;
//            m_DroneInitPose_Flightmare.position *= Scale;
//        }
//    }
//}























































































/*










/----------- Dynamic Gates



void ForgetfulSimulator::ROSCB_DynamicGatesSwitch( const std_msgs::Bool::ConstPtr& msg )
{
    MY_ROS_INFO( 
        "[%s]\n  >> Dynamic gates %s.", 
        m_ROSNodeName,
        msg->data? "enabled" : "disabled"
        );

    msg->data? m_ROSTimer_SimulatorLoop.start() : m_ROSTimer_SimulatorLoop.stop();
}


void ForgetfulSimulator::ROSTimerFunc_SimulatorLoop( const ros::TimerEvent& TimerEvent )
{   
    if ( TimerEvent.profile.last_duration.toSec() > m_SimulatorLoopTime )
        ROS_WARN(
            "[%s]\n  >> Last loop iteration took %f s exceeding nominal duration of %f s.",
            m_ROSNodeName, 
            TimerEvent.profile.last_duration.toSec(),
            m_SimulatorLoopTime
            );


    
    if ( ! m_SimulationReady ) return;


    double SimTime = ros::Time::now().toSec();

    gazebo_msgs::ModelState ModelState;
    ModelState.reference_frame = "world";
    Eigen::Vector3d AxialPhases;

    for ( size_t GateIdx = 0; GateIdx < m_GatesInitPos_WorldRF.size(); GateIdx++ )
    {
        ModelState.model_name = m_GatesID[ GateIdx ];

        AxialPhases
            = m_DynGates_InitAxPhases[ GateIdx ]
            + 2*M_PI*m_DynGates_AxFreqs[ GateIdx ] * SimTime;

        ModelState.pose.position = GeometryMsgsPoint_From_EigenVector3d(
            m_GatesInitPos_WorldRF[ GateIdx ] + m_DynGates_AxAmps[ GateIdx ].cwiseProduct(
                Eigen::Vector3d{ 
                    std::sin(AxialPhases.x()), 
                    std::sin(AxialPhases.y()), 
                    std::sin(AxialPhases.z()) 
                    }
                )
            );
        
        ModelState.pose.orientation 
            = tf::createQuaternionMsgFromYaw( m_GatesInitYaw_WorldRF[ GateIdx ] );

        
        //m_RVizGate.id = GateIdx;
        //m_RVizGate.pose.position = ModelState.pose.position;
        //m_RVizGate.pose.orientation = ModelState.pose.orientation;
        //m_RVizGates.markers.push_back( m_RVizGate );

        m_RVizGates.markers[ GateIdx ].pose = ModelState.pose;


        m_ROSPub_GazeboSetModelState.publish( ModelState );
    }

    m_ROSPub_RVizGates.publish( m_RVizGates );
    
}


void ForgetfulSimulator::initRandParamsOfDynGates()
{
    std::uniform_real_distribution<double> m_UniRealDistri_0To1{0.0, 1.0};

    m_DynGates_AxAmps.resize( m_GatesInitPos_WorldRF.size() );
    m_DynGates_AxFreqs.resize( m_GatesInitPos_WorldRF.size() );
    m_DynGates_InitAxPhases.resize( m_GatesInitPos_WorldRF.size() );

    for ( size_t GateIdx = 0; GateIdx < m_GatesInitPos_WorldRF.size(); GateIdx++ )
    {
        m_DynGates_AxAmps[ GateIdx ] 
            = m_DynGates_MaxAxAmp
            * Eigen::Vector3d{
                m_UniRealDistri_0To1( m_RandEngine ),
                m_UniRealDistri_0To1( m_RandEngine ),
                m_UniRealDistri_0To1( m_RandEngine )
                };
        
        
        m_DynGates_AxFreqs[ GateIdx ] 
            = m_DynGates_MaxAxFreq
            * Eigen::Vector3d{ 
                m_UniRealDistri_0To1( m_RandEngine ),
                m_UniRealDistri_0To1( m_RandEngine ),
                m_UniRealDistri_0To1( m_RandEngine )
                };

        m_DynGates_InitAxPhases[ GateIdx ] 
            = 2*M_PI 
            * Eigen::Vector3d{
                m_UniRealDistri_0To1( m_RandEngine ),
                m_UniRealDistri_0To1( m_RandEngine ),
                m_UniRealDistri_0To1( m_RandEngine )
                };
    }
}


































//------------------ SPRINT RACE 



///////////////////////////////////////////////////////////////////////////////////////////////////////
/// \brief [ Set member variables: ´m_GatesInitPos_WorldRF´ and ´m_GatesInitYaw_WorldRF´ ]
/// Computes the position and yaw of gates of a randomized sprint race.
/// \param m_RandSprint_GateN Number of gates if not == 0.
/// \param m_RandSprint_GateMinN If ´m_RandSprint_GateN´ == 0, lower limit for random number of gates.
/// \param m_RandSprint_GateMaxN If ´m_RandSprint_GateN´ == 0, upper limit for random number of gates.
/// \param m_RandSprint_SectionGateMinN Lower limit for random number of gates in race section.
/// \param m_RandSprint_SectionGateMaxN Upper limit for random number of gates in race section.
/// \param m_GateBaseMinAltitude Lower limit for altitude of gates.
/// \param m_RandSprint_MinSpacing Lower limit for spacing between gates.
/// \param m_RandSprint_UnidirStdDevSpacing Standard deviation of values 
/// whose absolute value is added to ´m_RandSprint_MinSpacing´ to get random spacing between gates.
/// \param m_RandSprint_LRMeanYaw +/- mean of yaw between gates in race section where +/- is randomly chosen.
/// \param m_RandSprint_LRStdDevYaw Standard deviation of yaw between gates in race section.
/// \param m_RandSprint_MeanPitch Mean of pitch between gates in race section.
/// \param m_RandSprint_StdDevPitch Standard deviation of pitch between gates in race section.
/// \return True if position and yaw of gates successfully computed, false otherwise.
///////////////////////////////////////////////////////////////////////////////////////////////////////
void ForgetfulSimulator::computeGatePoses_SprintRand()
{
    MY_ROS_INFO( "[%s] Computing position and yaw of gates for randomized sprint race...", 
        m_ROSNodeName );

    // If number of gates was not specified, i.e., `m_RandSprint_GateN` = 0, //
    // set to random value in [ `m_RandSprint_GateMinN`, m_RandSprint_GateMaxN ]    //
    /////////////////////////////////////////////////////////////////
        unsigned int Gates_N;
        if ( m_RandSprint_GateN == 0)
        {
            // Lambda function to get random number of gates for sprint race.
            std::uniform_int_distribution<unsigned int> UnifIntDistri_Gates_N( m_RandSprint_GateMinN, m_RandSprint_GateMaxN );
            Gates_N = UnifIntDistri_Gates_N( m_RandEngine );
        }
        else Gates_N = m_RandSprint_GateN;
    

    // Lambda functions to get random values for the design of the race track //
    ////////////////////////////////////////////////////////////////////////////
    std::uniform_int_distribution< unsigned int > UnifIntDistri_01{ 0, 1 };
    std::uniform_int_distribution< unsigned int > UnifIntDistri_Section_N{ 
        static_cast<unsigned int>( m_RandSprint_SectionGateMinN ), 
        static_cast<unsigned int>( m_RandSprint_SectionGateMaxN ) 
        };
    std::normal_distribution< double > NormalDistri_Spacing{ 0, m_RandSprint_UnidirStdDevSpacing };
    std::normal_distribution< double > NormalDistri_LeftYaw{ m_RandSprint_LRMeanYaw, m_RandSprint_LRStdDevYaw };
    std::normal_distribution< double > NormalDistri_RightYaw{ -m_RandSprint_LRMeanYaw, m_RandSprint_LRStdDevYaw };
    std::normal_distribution< double > NormalDistri_Pitch{ m_RandSprint_MeanPitch, m_RandSprint_StdDevPitch };
    
    auto getRand_Bool = [ & ]() { 
        return static_cast<bool>( UnifIntDistri_01( m_RandEngine ) ); };
    auto getRand_Section_N = [ & ]() { 
        return UnifIntDistri_Section_N( m_RandEngine ); };
    auto getRand_Spacing = [ & ]() { 
        return m_RandSprint_MinSpacing + abs( NormalDistri_Spacing( m_RandEngine ) ); };
    auto getRand_Yaw = [ & ]() { 
        return getRand_Bool()? NormalDistri_LeftYaw( m_RandEngine ) : NormalDistri_RightYaw( m_RandEngine ); };
    auto getRand_Pitch = [ & ]() { 
        return NormalDistri_Pitch( m_RandEngine ); };

    
    // Compute random gate positions and yaws //
    ///////////////////////////////////////////
    m_GatesInitPos_WorldRF.resize( Gates_N );
    m_GatesInitYaw_WorldRF.resize( Gates_N );
    
    // Position
    unsigned int Section_RemainedGates_n = 0;
    unsigned int Section_N;
    double SectionYaw;
    double SectionPitch;
    double Yaw = m_Drone_InitYaw;
    double Pitch;
    double StaticYaw;
    Eigen::Vector3d PrevGate_Pos = m_Drone_InitPosition + Eigen::Vector3d{ 0.0, 0.0, m_GateBaseMinAltitude };


    for ( size_t Gate_i = 0; Gate_i < Gates_N; Gate_i++ )
    {
        if ( Section_RemainedGates_n == 0 )
        {
            Section_N = getRand_Section_N();
            Section_RemainedGates_n = Section_N;
            SectionYaw = getRand_Yaw();
            SectionPitch = getRand_Pitch();
            StaticYaw = Yaw;
        }
        else Section_RemainedGates_n--;

        Yaw = StaticYaw + SectionYaw * ( Section_N - Section_RemainedGates_n );
        Pitch = SectionPitch * ( Section_N - Section_RemainedGates_n );

        m_GatesInitPos_WorldRF[ Gate_i ]
            = PrevGate_Pos 
            + Eigen::Vector3d{
                getRand_Spacing() * cos( Yaw ) * cos( Pitch ),
                getRand_Spacing() * sin( Yaw ) * cos( Pitch ),
                getRand_Spacing()              * sin( Pitch )
                };

        m_GatesInitPos_WorldRF[ Gate_i ].z() = std::max( m_GateBaseMinAltitude, m_GatesInitPos_WorldRF[ Gate_i ].z() );

        PrevGate_Pos = m_GatesInitPos_WorldRF[ Gate_i ];
    }
    // Yaw
    PrevGate_Pos = m_Drone_InitPosition + Eigen::Vector3d{ 0.0, 0.0, m_GateBaseMinAltitude };
    for ( size_t Gates_i = 0; Gates_i < Gates_N - 1; Gates_i++ )
    {
        Eigen::Vector3d Prev2Next = m_GatesInitPos_WorldRF[ Gates_i + 1 ] - PrevGate_Pos;
        
        m_GatesInitYaw_WorldRF[ Gates_i ] = atan2( Prev2Next.y(), Prev2Next.x() );

        PrevGate_Pos = m_GatesInitPos_WorldRF[ Gates_i ];
    }
    Eigen::Vector3d SecondLast2Last = m_GatesInitPos_WorldRF[ Gates_N - 1 ] - m_GatesInitPos_WorldRF[ Gates_N - 2 ];
    m_GatesInitYaw_WorldRF[ Gates_N - 1 ] = atan2( SecondLast2Last.y(), SecondLast2Last.x() );


    MY_ROS_INFO( "[%s] Position and yaw of gates for randomized sprint race successfully computed:\n"
        "\tNumber of Gates:           <%d>\n"
        "\tPosition of first Gate:    <%f, %f, %f>\n"
        "\tPosition of last Gate:     <%f, %f, %f>\n",
        m_ROSNodeName, Gates_N, 
        m_GatesInitPos_WorldRF[ 0 ].x(), m_GatesInitPos_WorldRF[ 0 ].y(), m_GatesInitPos_WorldRF[ 0 ].z(),
        m_GatesInitPos_WorldRF[ m_GatesInitPos_WorldRF.size() - 1 ].x(), m_GatesInitPos_WorldRF[ m_GatesInitPos_WorldRF.size() - 1 ].y(), m_GatesInitPos_WorldRF[ m_GatesInitPos_WorldRF.size() - 1 ].z()
        );
}

*/










}



