#include "forgetful_drones/forgetful_simulator.hpp"







namespace forgetful_drone{


ForgetfulSimulator::ForgetfulSimulator (const ros::NodeHandle& RNH, const ros::NodeHandle& PNH)
    :
    m_ROSRNH(RNH), 
    m_ROSPNH(PNH),
    m_ROSNodeName{ros::this_node::getName().c_str()},
    m_FM_SceneID{flightlib::UnityScene::WAREHOUSE},
    m_FM_UnityReady{false},
    m_FM_UnityRenderOn{false},
    m_FM_receive_id_{0},
    m_MainLoopFreq{50.0},
    m_ROSSub_StateEstimate{m_ROSRNH.subscribe("ground_truth/odometry", 1, &ForgetfulSimulator::ROSCB_StateEstimate, this)}, // "flight_pilot/state_estimate"
    m_CamFrameCount{0},
    m_DroneSpawned{false},
    m_DroneModelName{"hummingbird"},
    m_DroneModelDescription(), // parameter
    m_ROSSrvCl_GazeboSpawnURDFModel(m_ROSRNH.serviceClient<gazebo_msgs::SpawnModel>("/gazebo/spawn_urdf_model")),
    m_DroneCamFOV(),
    m_DroneCamWidth(),
    m_DroneCamHeight(),
    m_FM_UnityBridgePtr{nullptr}
    //m_UniRealDistri_0To1{0.0, 1.0},
    //m_SimulationReady(false),
    //m_Models_DirPath( ros::package::getPath("forgetful_drones") + "/gazebo/models" ),
    //m_Gate_NamePrefix( "Gate#" ),
    //m_Gate_TexDirPath ( m_Models_DirPath + "/gate/materials/textures/resources" ),
    //m_Gate_TexFileNames( getFileNamesInDir(m_Gate_TexDirPath) ),
    //m_Gate_DAEDirPath ( m_Models_DirPath + "/gate/meshes/resources/dae" ),
    //m_Gate_DAEFilePath( "file://" + m_Models_DirPath + "/gate/meshes/gate.dae" ),
    //m_Gate_DAEFileNames( getFileNamesInDir(m_Gate_DAEDirPath) ),
    //m_Gate_STLDirPath( m_Models_DirPath + "/gate/meshes/resources/stl" ),
    //m_Gate_STLFileNames( getFileNamesInDir(m_Gate_STLDirPath) ),
    //m_Gate_RandIllumScriptFilePath( ros::package::getPath("forgetful_drones") + "/src/adjust_ambient_and_emission_tag_of_XML_file.py" ),
    //m_Gate_TmpDirPath( m_Models_DirPath + "/gate/meshes/.tmp" ),
    //m_Ground_Name( "GroundPlane" ),
    //m_Ground_TexDirPath( m_Models_DirPath + "/ground_plane/materials/textures/resources" ),
    //m_Ground_TexFileNames( getFileNamesInDir(m_Ground_TexDirPath) ),
    //m_Ground_TmpDirPath( m_Models_DirPath + "/ground_plane/materials/scripts/.tmp" ),
    //m_Wall_NamePrefix( "Wall#" ),
    //m_Wall_TexDirPath( m_Models_DirPath + "/wall/materials/textures/resources" ),
    //m_Wall_TexFileNames( getFileNamesInDir( m_Wall_TexDirPath ) ),
    //m_Wall_TmpDirPath( m_Models_DirPath + "/wall/materials/scripts/.tmp" ),
    //m_Drone_FrameID( "hummingbird/base_link" ),
    //m_ROSSub_DynamicGatesSwitch( m_ROSRNH.subscribe("simulator/dynamic_gates_switch", 1, &ForgetfulSimulator::ROSCB_DynamicGatesSwitch, this) ),
    //m_ROSSrvSv_BuildDroneRacingSimulation( m_ROSRNH.advertiseService("simulator/build_drone_racing_simulation", &ForgetfulSimulator::ROSSrvFunc_buildDroneRacingSimulation, this) ),
    //m_ROSTimer_SimulatorLoop( m_ROSRNH.createTimer(ros::Duration(m_SimulatorLoopTime), &ForgetfulSimulator::ROSTimerFunc_SimulatorLoop, this, false, false) ),
    //m_ROSPub_RvizGates( m_ROSRNH.advertise<visualization_msgs::MarkerArray>("rviz/gates", 1, true) ),
    //m_ROSPub_RvizDrone( m_ROSRNH.advertise<visualization_msgs::MarkerArray>("rviz/drone", 1, true) ),
    //m_ROSSub_GazeboModelStates( m_ROSRNH.subscribe("/gazebo/model_states", 1, &ForgetfulSimulator::ROS_CB_GazeboModelStates, this ) ),
    //m_ROSPub_GazeboSetModelState( m_ROSRNH.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state", 0) ),
    //m_ROSSrvCl_GazeboDeleteModel( m_ROSRNH.serviceClient<gazebo_msgs::DeleteModel>("/gazebo/delete_model") ),
    //m_ROSSrvCl_GazeboResetSimulation( m_ROSRNH.serviceClient<std_srvs::Empty>("/gazebo/reset_simulation") ),
    //m_ROSSrvCl_GazeboSpawnSDFModel( m_ROSRNH.serviceClient<gazebo_msgs::SpawnModel>("/gazebo/spawn_sdf_model") ),
    
    //m_GateWaypointHeight(),
    //m_GateBaseMinAltitude(),
    //m_RandFig8_MaxAxShift(),
    //m_RandFig8_MinScale(),
    //m_RandFig8_MaxScale(),
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
    //m_WallBufferDistance(),
    //m_WallHeight(),
    //m_WallThickness(),
    //m_Drone_InitX(),
    //m_Drone_InitY(),
    //m_Drone_InitZ(),
    //m_Drone_InitPosition(),
    //m_Drone_InitYaw(),
    //m_Drone_RotorN(),
    //m_Drone_ArmLength(),
    //m_Drone_BodyWidth(),
    //m_Drone_BodyHeight(),
    //m_Drone_Scale(),
    //m_SimulatorLoopTime()
{
    //m_RandEngine.seed( ros::WallTime::now().toNSec() );
    //const char* m_ROSNodeName = ros::this_node::getName().c_str();

    bool InitializationSuccessful{true};

    if (!fetchROSParameters(m_ROSPNH, m_KeysBoolPtrs, m_KeysIntPtrs, m_KeysDoublePtrs, m_KeysStringPtrs))
        InitializationSuccessful = false;


    
    if (InitializationSuccessful) 
        ROS_INFO("[%s]\n  >> Initialization successful.", m_ROSNodeName);
    else
    {
        ROS_FATAL("[%s]\n  >> Initialization failed -> Shut down ROS node.", m_ROSNodeName);
        ros::shutdown();
    }

    // Initialize publishers for images
    image_transport::ImageTransport IT(m_ROSPNH);
    m_ITPub_RGBImg = IT.advertise("/rgb", 1);
    m_ITPub_DepthImg = IT.advertise("/depth", 1);
    m_ITPub_SegmentationImg = IT.advertise("/segmentation", 1);
    m_ITPub_OpticalFlowImg = IT.advertise("/opticalflow", 1);

    // Initialize main loop
    m_ROSTimer_MainLoop
        = m_ROSRNH.createTimer(ros::Rate(m_MainLoopFreq), &ForgetfulSimulator::ROSCB_MainLoop, this, false, false); // no oneshot, no autostart


    // wait for gazebo and unity to load
    ros::Duration(5.0).sleep();

    // Set unity bridge
    if (m_FM_UnityRenderOn && m_FM_UnityBridgePtr==nullptr) 
    {
        m_FM_UnityBridgePtr = flightlib::UnityBridge::getInstance();
        ROS_INFO("[%s] Unity Bridge created.", m_ROSNodeName);
    }



    // TESTING & DEBUGGING
    //-------------------------

    // -> req
    geometry_msgs::Pose RaceTrackPose_WorldRF; // for a spot in FM INDUSTRIAL SCENE: ->parameter
    RaceTrackPose_WorldRF.position.x = 67.0;
    RaceTrackPose_WorldRF.position.y = -17.0;
    RaceTrackPose_WorldRF.position.z = -17.0;
    RaceTrackPose_WorldRF.orientation = tf::createQuaternionMsgFromYaw( 0.0 );
    constexpr double GateWaypointHeight = 2.0; // for FM RPG GATE: ->parameter

    
    

    buildDroneRacingSimulation(
        RaceTrackPose_WorldRF,
        forgetful_drones_msgs::BuildDroneRacingSimulation::Request::FIG8_DET, 
        GateWaypointHeight,
        (uint8_t)false
        );

    // Start main loop for publishing images to ROS topics
    m_ROSTimer_MainLoop.start();
    //m_ROSTimer_MainLoop.stop();
}


ForgetfulSimulator::~ForgetfulSimulator() {}




void ForgetfulSimulator::ROSCB_StateEstimate(const nav_msgs::Odometry::ConstPtr& msg) 
{
    m_FM_DroneState.x[flightlib::QS::POSX] = (flightlib::Scalar) msg->pose.pose.position.x;
    m_FM_DroneState.x[flightlib::QS::POSY] = (flightlib::Scalar) msg->pose.pose.position.y;
    m_FM_DroneState.x[flightlib::QS::POSZ] = (flightlib::Scalar) msg->pose.pose.position.z;
    m_FM_DroneState.x[flightlib::QS::ATTW] = (flightlib::Scalar) msg->pose.pose.orientation.w;
    m_FM_DroneState.x[flightlib::QS::ATTX] = (flightlib::Scalar) msg->pose.pose.orientation.x;
    m_FM_DroneState.x[flightlib::QS::ATTY] = (flightlib::Scalar) msg->pose.pose.orientation.y;
    m_FM_DroneState.x[flightlib::QS::ATTZ] = (flightlib::Scalar) msg->pose.pose.orientation.z;

    m_FM_DronePtr->setState(m_FM_DroneState);

    if (m_FM_UnityRenderOn && m_FM_UnityReady)
    {
        m_FM_UnityBridgePtr->getRender(0);
        m_FM_UnityBridgePtr->handleOutput();
    }
}

void ForgetfulSimulator::ROSCB_MainLoop(const ros::TimerEvent& TE)
{
    if ( TE.profile.last_duration.toSec() > 1/m_MainLoopFreq )
        ROS_WARN(
            "[%s]\n  >> Last loop iteration took %f s exceeding nominal duration of %f s.",
            ros::this_node::getName().c_str(), 
            TE.profile.last_duration.toSec(),
            1/m_MainLoopFreq
            );


    cv::Mat Img;
    //sensor_msgs::ImagePtr Msg; Try to use only one instance for better preformance.

    m_FM_RGBCam->getRGBImage(Img);
    sensor_msgs::ImagePtr rgb_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", Img).toImageMsg();
    rgb_msg->header.stamp.fromNSec(m_CamFrameCount);
    m_ITPub_RGBImg.publish(rgb_msg);

    m_FM_RGBCam->getDepthMap(Img);
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
    m_ITPub_OpticalFlowImg.publish(opticflow_msg);

    m_CamFrameCount++;
}









bool ForgetfulSimulator::ROSCB_buildDroneRacingSimulation(
    forgetful_drones_msgs::BuildDroneRacingSimulation::Request& req,
    forgetful_drones_msgs::BuildDroneRacingSimulation::Response& res
    )
{
    // -> req
    geometry_msgs::Pose RaceTrackPose_WorldRF; // for a spot in FM INDUSTRIAL SCENE: ->parameter
    RaceTrackPose_WorldRF.position.x = 67.0;
    RaceTrackPose_WorldRF.position.y = -17.0;
    RaceTrackPose_WorldRF.position.z = -17.0;
    RaceTrackPose_WorldRF.orientation = tf::createQuaternionMsgFromYaw( 0.0 );
    constexpr double GateWaypointHeight = 2.0; // for FM RPG GATE: ->parameter


    buildDroneRacingSimulation(
        RaceTrackPose_WorldRF,
        req.RaceType, 
        GateWaypointHeight,
        req.DynGatesActivated
        );

    // Set service response
    //res.Drone_InitPose = m_DroneInitPose_WorldRF.as_geometry_msg();
    //res.Gates_WaypointPose.clear();
    //for (size_t GateIdx = 0; GateIdx < m_GatesInitPose_WorldRF.size(); GateIdx++)
    //    res.Gates_WaypointPose.push_back(m_GatesInitPose_WorldRF[GateIdx].as_geometry_msg());
}





bool ForgetfulSimulator::buildDroneRacingSimulation(
    const geometry_msgs::Pose& RaceTrackPose_WorldRF,
    const forgetful_drones_msgs::BuildDroneRacingSimulation::Request::_RaceType_type& RaceTrackType,
    const double& GateWaypointHeight, //-> GateType
    const forgetful_drones_msgs::BuildDroneRacingSimulation::Request::_DynGatesActivated_type& GatesDynamic
    )
{


    //////////////////////////////
    //////////////////////////////

    //m_SimulationReady = false;
    //req.DynGatesActivated? m_ROSTimer_SimulatorLoop.start() : m_ROSTimer_SimulatorLoop.stop();


    //deleteAllGazeboModelsExceptDrone();

    std::vector<forgetful_drone::Pose> GatesInitPose_WorldRF;
    forgetful_drone::Pose DroneInitPose_WorldRF;
    switch ( RaceTrackType )
    {
        case forgetful_drones_msgs::BuildDroneRacingSimulation::Request::FIG8_DET: 
            compGatesAndDroneInitPose_Fig8Det(
                RaceTrackPose_WorldRF,
                GateWaypointHeight,
                GatesInitPose_WorldRF,
                DroneInitPose_WorldRF
                ); break;
        //case forgetful_drones_msgs::BuildDroneRacingSimulation::Request::FIG8_RAND: 
        //    computeGatePoses_Fig8Rand(); break;
        //case forgetful_drones_msgs::BuildDroneRacingSimulation::Request::CIRCUIT_RAND:
        //    ROS_ERROR("[%s]\n  >> Not yet implemented specified race type.", ros::this_node::getName().c_str());
        //    return false; break;
        //case forgetful_drones_msgs::BuildDroneRacingSimulation::Request::SPRINT_RAND: 
        //    computeGatePoses_SprintRand(); break;
        default:
            ROS_ERROR("[%s]\n  >> Unknown specified race type.", ros::this_node::getName().c_str());
            return false; break;
    }


    //initRandParamsOfDynGates();
    
    spawnGatesInUnity(GatesInitPose_WorldRF);
    //computePositionAndSizeOfGroundAndWalls();
    //spawnRandGatesInGazeboAndRVIZ();
    //spawnRandGroundInGazebo();
    //spawnRandWallsInGazebo();
    // --> get point cloud from unity and spawn collision model in gazebo


    if (!m_DroneSpawned)
    {
        spawnDroneInGazebo(DroneInitPose_WorldRF);
        spawnDroneInUnity(DroneInitPose_WorldRF);
        m_DroneSpawned = true;
    }

    

    //rvizDrone();






        
    // Connect Unity
    if (m_FM_UnityRenderOn && m_FM_UnityBridgePtr!=nullptr) 
        m_FM_UnityReady = m_FM_UnityBridgePtr->connectUnity(m_FM_SceneID);
    //m_SimulationReady = true;
    
    return true;
}






void ForgetfulSimulator::compGatesAndDroneInitPose_Fig8Det(
    const geometry_msgs::Pose& in_RaceTrackPose_WorldRF, 
    const double& in_GatesWaypointHeight,
    std::vector<forgetful_drone::Pose>& out_GatesInitPose_WorldRF,
    forgetful_drone::Pose& out_DroneInitPose_WorldRF
    )
{
    // Data
    constexpr size_t GatesN = 14;
    constexpr double PosOffsetX = 22.7;
    constexpr double PosOffsetY = 14.5;
    std::array<Eigen::Vector3d, GatesN> GatesInitPos_RaceTrackRF = {
        Eigen::Vector3d{ 2.7  - PosOffsetX ,     6.7  - PosOffsetY ,   in_GatesWaypointHeight  }, // #01
        Eigen::Vector3d{ 10.6 - PosOffsetX ,     4.2  - PosOffsetY ,   in_GatesWaypointHeight  }, // #02
        Eigen::Vector3d{ 19.0 - PosOffsetX ,     10.0 - PosOffsetY ,   in_GatesWaypointHeight  }, // #03
        Eigen::Vector3d{ 26.6 - PosOffsetX ,     19.6 - PosOffsetY ,   in_GatesWaypointHeight  }, // #04
        Eigen::Vector3d{ 35.1 - PosOffsetX ,     26.5 - PosOffsetY ,   in_GatesWaypointHeight  }, // #05
        Eigen::Vector3d{ 45.0 - PosOffsetX ,     22.2 - PosOffsetY ,   in_GatesWaypointHeight  }, // #06
        Eigen::Vector3d{ 47.4 - PosOffsetX ,     13.6 - PosOffsetY ,   in_GatesWaypointHeight  }, // #07
        Eigen::Vector3d{ 42.4 - PosOffsetX ,     5.8  - PosOffsetY ,   in_GatesWaypointHeight  }, // #08
        Eigen::Vector3d{ 33.7 - PosOffsetX ,     4.7  - PosOffsetY ,   in_GatesWaypointHeight  }, // #09
        Eigen::Vector3d{ 26.0 - PosOffsetX ,     9.4  - PosOffsetY ,   in_GatesWaypointHeight  }, // #10
        Eigen::Vector3d{ 18.2 - PosOffsetX ,     20.0 - PosOffsetY ,   in_GatesWaypointHeight  }, // #11
        Eigen::Vector3d{ 10.2 - PosOffsetX ,     25.0 - PosOffsetY ,   in_GatesWaypointHeight  }, // #12
        Eigen::Vector3d{ 2.1  - PosOffsetX ,     22.0 - PosOffsetY ,   in_GatesWaypointHeight  }, // #13
        Eigen::Vector3d{ -1.1 - PosOffsetX ,     13.2 - PosOffsetY ,   in_GatesWaypointHeight  }  // #14
        };
    std::array<double, GatesN> GatesInitYaw_RaceTrackRF = {
        -0.44,  // #01
        0.0  ,  // #02
        0.97 ,  // #03
        -2.2 ,  // #04
        3.5  ,  // #05
        2.57 ,  // #06
        1.57 ,  // #07
        -2.6 ,  // #08
        3.1  ,  // #09
        -1.0 ,  // #10
        -0.9 ,  // #11
        -3.1 ,  // #12
        0.8  ,  // #13
        -1.5 ,  // #14
        };
    const geometry_msgs::Pose DroneInitPose_RaceTrackRF;
        const_cast<geometry_msgs::Pose&>(DroneInitPose_RaceTrackRF).position.x = -0.5;
        const_cast<geometry_msgs::Pose&>(DroneInitPose_RaceTrackRF).position.y = 22.0;
        const_cast<geometry_msgs::Pose&>(DroneInitPose_RaceTrackRF).position.z = 0.0;
        const_cast<geometry_msgs::Pose&>(DroneInitPose_RaceTrackRF).orientation
            = tf::createQuaternionMsgFromYaw(-M_PI / 2.0);

    // Create transformation between reference frames
        // race track -> world
    kindr::minimal::QuatTransformation T_WorldRF_RaceTrackRF;
    tf::poseMsgToKindr(in_RaceTrackPose_WorldRF, &T_WorldRF_RaceTrackRF);
        // drone init -> world
    kindr::minimal::QuatTransformation T_RaceTrackRF_DroneInitRF;
    tf::poseMsgToKindr(DroneInitPose_RaceTrackRF, &T_RaceTrackRF_DroneInitRF);
    kindr::minimal::QuatTransformation T_WorldRF_DroneInitRF = T_WorldRF_RaceTrackRF * T_RaceTrackRF_DroneInitRF;


    // Resize the vector containing gates
    out_GatesInitPose_WorldRF.resize(GatesN);


    // Apply transformations...
        // ...to the gate data
    for (size_t GateIdx = 0; GateIdx < GatesN; GateIdx++)
    {
        
        
        
        
        geometry_msgs::Pose GateInitPose_RaceTrackRF;
        GateInitPose_RaceTrackRF.position = GeometryMsgsPoint_From_EigenVector3d(GatesInitPos_RaceTrackRF[GateIdx]);
        GateInitPose_RaceTrackRF.orientation = tf::createQuaternionMsgFromYaw(GatesInitYaw_RaceTrackRF[GateIdx]);
        
        kindr::minimal::QuatTransformation T_RaceTrackRF_GateInitRF;
        tf::poseMsgToKindr(GateInitPose_RaceTrackRF, &T_RaceTrackRF_GateInitRF);
        kindr::minimal::QuatTransformation T_WorldRF_GateInitRF = T_WorldRF_RaceTrackRF * T_RaceTrackRF_GateInitRF;

        out_GatesInitPose_WorldRF[GateIdx].position = T_WorldRF_GateInitRF.getPosition().cast<float>();
        out_GatesInitPose_WorldRF[GateIdx].orientation = T_WorldRF_GateInitRF.getEigenQuaternion().cast<float>();
    }
        // ...to the drone data
    out_DroneInitPose_WorldRF.position = T_WorldRF_DroneInitRF.getPosition().cast<float>();
    out_DroneInitPose_WorldRF.orientation = T_WorldRF_DroneInitRF.getEigenQuaternion().cast<float>();
}


void ForgetfulSimulator::spawnGatesInUnity(const std::vector<forgetful_drone::Pose> GatesInitPose_WorldRF)
{
    const size_t& GatesN = GatesInitPose_WorldRF.size();
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

        // PrefabID
        std::string PrefabID = "rpg_gate";

        // Init gates with ObjectID, PrefabID, position and orientation
        m_UnityGates[GateIdx] = std::make_shared<flightlib::StaticGate>(ObjectID, PrefabID);
        m_UnityGates[GateIdx]->setPosition(GatesInitPose_WorldRF[GateIdx].position);
        m_UnityGates[GateIdx]->setQuaternion(GatesInitPose_WorldRF[GateIdx].orientation);

        // Add gates
        m_FM_UnityBridgePtr->addStaticObject(m_UnityGates[GateIdx]);
    }
}


void ForgetfulSimulator::spawnDroneInGazebo(const forgetful_drone::Pose& DroneInitPose_WorldRF)
{
    gazebo_msgs::SpawnModel srv;
        srv.request.model_name = m_DroneModelName;
        srv.request.model_xml = m_DroneModelDescription;
        srv.request.reference_frame = "world";
        srv.request.initial_pose = DroneInitPose_WorldRF.as_geometry_msg();
    m_ROSSrvCl_GazeboSpawnURDFModel.call(srv);
}

void ForgetfulSimulator::spawnDroneInUnity(const forgetful_drone::Pose& DroneInitPose_WorldRF)
{
    // Init drone
    m_FM_DronePtr = std::make_shared<flightlib::Quadrotor>();

    // Init and add mono camera
    m_FM_RGBCam = std::make_shared<flightlib::RGBCamera>();
    flightlib::Vector<3> B_r_BC{0.0, 0.0, 0.3};
    flightlib::Matrix<3, 3> R_BC = flightlib::Quaternion{1.0, 0.0, 0.0, 0.0}.toRotationMatrix();
    m_FM_RGBCam->setRelPose(B_r_BC, R_BC);
    m_FM_RGBCam->setFOV(m_DroneCamFOV);
    m_FM_RGBCam->setWidth(m_DroneCamWidth);
    m_FM_RGBCam->setHeight(m_DroneCamHeight);
    m_FM_RGBCam->setPostProcesscing(std::vector<bool>{false, false, false});  // depth, segmentation, optical flow
    m_FM_DronePtr->addRGBCamera(m_FM_RGBCam);

    // Init drone state and set to initial pose
    m_FM_DroneState.setZero();
    m_FM_DronePtr->reset(m_FM_DroneState);   
    m_FM_DroneState.x[flightlib::QS::POSX] = DroneInitPose_WorldRF.position.x();
    m_FM_DroneState.x[flightlib::QS::POSY] = DroneInitPose_WorldRF.position.y();
    m_FM_DroneState.x[flightlib::QS::POSZ] = DroneInitPose_WorldRF.position.z();
    m_FM_DroneState.x[flightlib::QS::ATTW] = DroneInitPose_WorldRF.orientation.w();
    m_FM_DroneState.x[flightlib::QS::ATTX] = DroneInitPose_WorldRF.orientation.x();
    m_FM_DroneState.x[flightlib::QS::ATTY] = DroneInitPose_WorldRF.orientation.y();
    m_FM_DroneState.x[flightlib::QS::ATTZ] = DroneInitPose_WorldRF.orientation.z();
    m_FM_DronePtr->setState(m_FM_DroneState);

    // Add quadcopter
    m_FM_UnityBridgePtr->addQuadrotor(m_FM_DronePtr);
}






/*
/// \brief !
bool ForgetfulSimulator::ROSSrvFunc_buildDroneRacingSimulation(
    forgetful_drones_msgs::BuildDroneRacingSimulation::Request& req,
    forgetful_drones_msgs::BuildDroneRacingSimulation::Response& res
    )
{
    m_SimulationReady = false;
    req.DynGatesActivated? m_ROSTimer_SimulatorLoop.start() : m_ROSTimer_SimulatorLoop.stop();


    deleteAllGazeboModelsExceptDrone();

    
        


    switch ( req.RaceType )
    {
        case req.FIG8_DET: compGatesAndDroneInitPose_Fig8Det(); break;
        case req.FIG8_RAND: computeGatePoses_Fig8Rand(); break;
        case req.CIRCUIT_RAND:
            ROS_ERROR( "[%s]\n  >> Specified race type not implemented yet.", ros::this_node::getName().c_str() );
            return false;
            break;
        case req.SPRINT_RAND: computeGatePoses_SprintRand(); break;
        default:
            ROS_ERROR( "[%s]\n  >> Specified race type unknown.", ros::this_node::getName().c_str() );
            return false;
            break;
    }
    
    setGatesID();

    initRandParamsOfDynGates();
    
    computePositionAndSizeOfGroundAndWalls();
        
    spawnRandGatesInGazeboAndRVIZ();
    
    spawnRandGroundInGazebo();

    spawnRandWallsInGazebo();


    if ( ! m_DroneSpawned )
    {
        spawnDroneInGazebo();
        m_DroneSpawned = true;
    }
    //else
    //    resetDroneModelState();

    

    rvizDrone();





    // --- Set service response ---
        res.Drone_InitPose.position = GeometryMsgsPoint_From_EigenVector3d( m_Drone_InitPosition );
        res.Drone_InitPose.orientation = tf::createQuaternionMsgFromYaw( m_Drone_InitYaw );

        res.Gates_WaypointPose.clear();
        geometry_msgs::Pose WaypointPose;
        for ( size_t GateIdx = 0; GateIdx < m_GatesInitPos_WorldRF.size(); GateIdx++ )
        {
            WaypointPose.position 
                = GeometryMsgsPoint_From_EigenVector3d( m_GatesInitPos_WorldRF[GateIdx] );
            WaypointPose.position.z += m_GateWaypointHeight;
            WaypointPose.orientation = tf::createQuaternionMsgFromYaw( m_GatesInitYaw_WorldRF[GateIdx] );

            res.Gates_WaypointPose.push_back( WaypointPose );
        }
        


    m_SimulationReady = true;
    return true;
}
/*


void ForgetfulSimulator::compGatesAndDroneInitPose_Fig8Det()
{
    size_t GatesN = 14;
    m_GatesInitPos_WorldRF.resize( GatesN );
    m_GatesInitYaw_WorldRF.resize( GatesN );

    m_GatesInitPos_WorldRF = {
        { 2.7   ,     6.7   ,    2.0    },
        { 10.6  ,     4.2   ,    2.0    },
        { 19.0  ,     10.0  ,   1.9     },
        { 26.6  ,     19.6  ,   2.0     },
        { 35.1  ,     26.5  ,   2.0     },
        { 45.0  ,     22.2  ,   1.9     },
        { 47.4  ,     13.6  ,   2.0     },
        { 42.4  ,     5.8   ,    2.0    },
        { 33.7  ,     4.7   ,    1.9    },
        { 26.0  ,     9.4   ,    2.0    },
        { 18.2  ,     20.0  ,   2.0     },
        { 10.2  ,     25.0  ,   2.0     },
        { 2.1   ,     22.0  ,   1.9     },
        { -1.1  ,     13.2  ,   2.0     }
        };

    m_GatesInitYaw_WorldRF = {
        -0.44,
        0.0  , 
        0.97 ,
        -2.2 ,
        3.5  , 
        2.57 ,
        1.57 ,
        -2.6 ,
        3.1  , 
        -1.0 ,
        -0.9 ,
        -3.1 ,
        0.8  , 
        -1.5 ,
        };


        geometry_msgs::Pose DroneInitRF_WRF;
        DroneInitRF_WRF.position = GeometryMsgsPoint_From_EigenVector3d( m_Drone_InitPosition );
        DroneInitRF_WRF.position.z = 0.0;
        DroneInitRF_WRF.orientation = tf::createQuaternionMsgFromYaw( m_Drone_InitYaw );
    kindr::minimal::QuatTransformation T_WRF_DroneInitRF;
        tf::poseMsgToKindr( DroneInitRF_WRF, &T_WRF_DroneInitRF );

        geometry_msgs::Pose RaceTrackRF_DroneInitRF;
        RaceTrackRF_DroneInitRF.position.x = -0.5;
        RaceTrackRF_DroneInitRF.position.y = 22.0;
        RaceTrackRF_DroneInitRF.position.z = 0.0;
        RaceTrackRF_DroneInitRF.orientation = tf::createQuaternionMsgFromYaw( - M_PI / 2.0 );
    kindr::minimal::QuatTransformation T_RaceTrackRF_WRF;
        tf::poseMsgToKindr( RaceTrackRF_DroneInitRF, &T_RaceTrackRF_WRF );

    kindr::minimal::QuatTransformation T_WRF_RaceTrackRF = T_WRF_DroneInitRF * T_RaceTrackRF_WRF.inverse();


        geometry_msgs::Pose GateInitPose_RaceTrackRF;
        kindr::minimal::QuatTransformation T_RaceTrackRF_GateInitRF;
        kindr::minimal::QuatTransformation T_WorldRF_GateInitRF;    
    for ( size_t GateIdx = 0; GateIdx < GatesN; GateIdx++ )
    {
        GateInitPose_RaceTrackRF.position 
            = GeometryMsgsPoint_From_EigenVector3d( m_GatesInitPos_WorldRF[ GateIdx ] );
        GateInitPose_RaceTrackRF.orientation 
            = tf::createQuaternionMsgFromYaw( m_GatesInitYaw_WorldRF[ GateIdx ] );
        tf::poseMsgToKindr (
            GateInitPose_RaceTrackRF, 
            &T_RaceTrackRF_GateInitRF
            );
        T_WorldRF_GateInitRF = T_WRF_RaceTrackRF * T_RaceTrackRF_GateInitRF;

        m_GatesInitPos_WorldRF[ GateIdx ] = T_WorldRF_GateInitRF.getPosition();
        m_GatesInitYaw_WorldRF[ GateIdx ] 
            = T_WorldRF_GateInitRF.getEigenQuaternion().toRotationMatrix().eulerAngles(0, 1, 2).z();
    }
}


void ForgetfulSimulator::computeGatePoses_Fig8Rand()
{
    compGatesAndDroneInitPose_Fig8Det();


    std::uniform_real_distribution<double> URDistri_M1ToP1( -1.0, 1.0 );
    std::uniform_real_distribution<double> URDistri_Scale( m_RandFig8_MinScale, m_RandFig8_MaxScale );
    double Scale = URDistri_Scale( m_RandEngine );

    for ( size_t GateIdx = 0; GateIdx < m_GatesInitPos_WorldRF.size(); GateIdx++ )
    {
        m_GatesInitPos_WorldRF[ GateIdx ].x() 
            += m_RandFig8_MaxAxShift * URDistri_M1ToP1( m_RandEngine );
        
        m_GatesInitPos_WorldRF[ GateIdx ].y() 
            += m_RandFig8_MaxAxShift * URDistri_M1ToP1( m_RandEngine );

        m_GatesInitPos_WorldRF[ GateIdx ].z() 
            += m_RandFig8_MaxAxShift * URDistri_M1ToP1( m_RandEngine );

        m_GatesInitPos_WorldRF[ GateIdx ].z() = std::max( m_GateBaseMinAltitude, m_GatesInitPos_WorldRF[ GateIdx ].z() );

        m_GatesInitPos_WorldRF[ GateIdx ].x() *= Scale;
        m_GatesInitPos_WorldRF[ GateIdx ].y() *= Scale;
    }

    const_cast< Eigen::Vector3d& >( m_Drone_InitPosition ) *= Scale;
}







void ForgetfulSimulator::setGatesID()
{
    m_GatesID.resize( m_GatesInitPos_WorldRF.size() );
    std::stringstream GateID;
    int IDWidth = static_cast<int>( log10(m_GatesInitPos_WorldRF.size()) +1 );

    for ( size_t GateIdx = 0; GateIdx < m_GatesInitPos_WorldRF.size(); GateIdx++ )
    {
        GateID.str( "" );
        GateID 
            << std::setw( IDWidth ) 
            << std::setfill('0') 
            << GateIdx;

        m_GatesID[ GateIdx ] = m_Gate_NamePrefix + GateID.str();
    }
}










void ForgetfulSimulator::computePositionAndSizeOfGroundAndWalls()
{

    double MinX = m_Drone_InitPosition.x(), MaxX = MinX;
    double MinY = m_Drone_InitPosition.y(), MaxY = MinY;

    for ( size_t GateIdx = 0; GateIdx < m_GatesInitPos_WorldRF.size(); GateIdx++ )
    {
        MinX = std::min( MinX, m_GatesInitPos_WorldRF[ GateIdx ].x() );
        MaxX = std::max( MaxX, m_GatesInitPos_WorldRF[ GateIdx ].x() );
        MinY = std::min( MinY, m_GatesInitPos_WorldRF[ GateIdx ].y() );
        MaxY = std::max( MaxY, m_GatesInitPos_WorldRF[ GateIdx ].y() );
    }

    MinX -= m_WallBufferDistance; MaxX += m_WallBufferDistance;
    MinY -= m_WallBufferDistance; MaxY += m_WallBufferDistance;

    const double Ground_BaseX = (MinX + MaxX) /2;
    const double Ground_BaseY = (MinY + MaxY) /2;
    const double Ground_SizeX = MaxX - MinX;
    const double Ground_SizeY = MaxY - MinY;

    m_Ground_BasePoint = {Ground_BaseX, Ground_BaseY};
    m_Ground_Size = {Ground_SizeX, Ground_SizeY};

    m_Walls_BasePoint[ 0 ] = {MinX, Ground_BaseY, m_WallHeight/2};
    m_Walls_BasePoint[ 1 ] = {MaxX, Ground_BaseY, m_WallHeight/2};
    m_Walls_BasePoint[ 2 ] = {Ground_BaseX, MinY, m_WallHeight/2};
    m_Walls_BasePoint[ 3 ] = {Ground_BaseX, MaxY, m_WallHeight/2};

    m_Walls_Size[ 0 ] = {m_WallThickness, Ground_SizeY, m_WallHeight};
    m_Walls_Size[ 1 ] = {m_WallThickness, Ground_SizeY, m_WallHeight};
    m_Walls_Size[ 2 ] = {Ground_SizeX, m_WallThickness, m_WallHeight};
    m_Walls_Size[ 3 ] = {Ground_SizeX, m_WallThickness, m_WallHeight};
}










void ForgetfulSimulator::spawnDroneInGazebo()
{
    gazebo_msgs::SpawnModel srv;
        srv.request.model_name = m_DroneModelName;
        srv.request.reference_frame = "world";
        srv.request.initial_pose.position 
            = GeometryMsgsPoint_From_EigenVector3d( m_Drone_InitPosition );
        srv.request.initial_pose.orientation 
            = tf::createQuaternionMsgFromYaw( m_Drone_InitYaw );
        srv.request.model_xml = m_DroneModelDescription;

    m_ROSSrvCl_GazeboSpawnURDFModel.call( srv );
}



void ForgetfulSimulator::resetDroneModelState()
{
    gazebo_msgs::ModelState ModelState;
    ModelState.reference_frame = "world";
    ModelState.model_name = m_DroneModelName;
    ModelState.pose.position = GeometryMsgsPoint_From_EigenVector3d (m_Drone_InitPosition);
    ModelState.pose.orientation = tf::createQuaternionMsgFromYaw( m_Drone_InitYaw );
    m_ROSPub_GazeboSetModelState.publish( ModelState );
}
*/
































































/*

/----------- RVIZ

/// in Comstructor body
// --- Marker for gates in RVIZ ---
m_GateMarker.header.frame_id = "world";
m_GateMarker.scale.x = m_GateMarker.scale.y = m_GateMarker.scale.z = 1.0;
m_GateMarker.action = visualization_msgs::Marker::ADD;
m_GateMarker.type = visualization_msgs::Marker::MESH_RESOURCE;
m_GateMarker.mesh_use_embedded_materials = true;


void ForgetfulSimulator::rvizDrone()
{
    // --- Marker array for drone in RVIZ ---
    visualization_msgs::MarkerArray m_DroneMarkerArray;
    m_DroneMarkerArray.markers.reserve( 2 * m_Drone_RotorN + 1 );

        visualization_msgs::Marker DroneMarker_Body;
            DroneMarker_Body.header.stamp = ros::Time();
            DroneMarker_Body.header.frame_id = m_Drone_FrameID;
            DroneMarker_Body.ns = "vehicle_body";
            DroneMarker_Body.action = visualization_msgs::Marker::ADD;
            DroneMarker_Body.type = visualization_msgs::Marker::CUBE;
            DroneMarker_Body.scale.x = m_Drone_BodyWidth * m_Drone_Scale;
            DroneMarker_Body.scale.y = m_Drone_BodyWidth * m_Drone_Scale;
            DroneMarker_Body.scale.z = m_Drone_BodyHeight * m_Drone_Scale;
            setRGBOfVisMarker( VisualizationColors::BLACK, DroneMarker_Body);
            DroneMarker_Body.color.a = 1.0;
            DroneMarker_Body.frame_locked = true;
        m_DroneMarkerArray.markers.push_back( DroneMarker_Body );

        visualization_msgs::Marker DroneMarker_Rotor;
            DroneMarker_Rotor.header.stamp = ros::Time();
            DroneMarker_Rotor.header.frame_id = m_Drone_FrameID;
            DroneMarker_Rotor.ns = "vehicle_rotor";
            DroneMarker_Rotor.action = visualization_msgs::Marker::ADD;
            DroneMarker_Rotor.type = visualization_msgs::Marker::CYLINDER;
            DroneMarker_Rotor.scale.x = 0.2 * m_Drone_Scale;
            DroneMarker_Rotor.scale.y = 0.2 * m_Drone_Scale;
            DroneMarker_Rotor.scale.z = 0.01 * m_Drone_Scale;
            setRGBOfVisMarker( VisualizationColors::BLUE, DroneMarker_Rotor);
            DroneMarker_Rotor.color.a = 0.7;
            DroneMarker_Rotor.pose.position.z = 0;
            DroneMarker_Rotor.frame_locked = true;
        
        visualization_msgs::Marker DroneMarker_Arm;
            DroneMarker_Arm.header.stamp = ros::Time();
            DroneMarker_Arm.header.frame_id = m_Drone_FrameID;
            DroneMarker_Arm.ns = "vehicle_arm";
            DroneMarker_Arm.action = visualization_msgs::Marker::ADD;
            DroneMarker_Arm.type = visualization_msgs::Marker::CUBE;
            DroneMarker_Arm.scale.x = m_Drone_ArmLength * m_Drone_Scale;
            DroneMarker_Arm.scale.y = 0.02 * m_Drone_Scale;
            DroneMarker_Arm.scale.z = 0.01 * m_Drone_Scale;
            setRGBOfVisMarker( VisualizationColors::BLACK, DroneMarker_Arm);
            DroneMarker_Arm.color.a = 1.0;
            DroneMarker_Arm.pose.position.z = -0.015 * m_Drone_Scale;
            DroneMarker_Arm.frame_locked = true;

            const float Drone_RotorAngleIncrement = 2 * M_PI / m_Drone_RotorN;
            for 
            (
                float Angle = Drone_RotorAngleIncrement / 2; 
                Angle <= 2 * M_PI; 
                Angle += Drone_RotorAngleIncrement
            ) 
            {
                DroneMarker_Rotor.pose.position.x 
                    = m_Drone_ArmLength * cos( Angle ) * m_Drone_Scale;
                DroneMarker_Rotor.pose.position.y 
                    = m_Drone_ArmLength * sin( Angle ) * m_Drone_Scale;
                DroneMarker_Rotor.id++;

                DroneMarker_Arm.pose.position.x = DroneMarker_Rotor.pose.position.x / 2;
                DroneMarker_Arm.pose.position.y = DroneMarker_Rotor.pose.position.y / 2;
                DroneMarker_Arm.pose.orientation = tf::createQuaternionMsgFromYaw( Angle );
                DroneMarker_Arm.id++;

                m_DroneMarkerArray.markers.push_back( DroneMarker_Rotor );
                m_DroneMarkerArray.markers.push_back( DroneMarker_Arm );
            }

    m_ROSPub_RvizDrone.publish( m_DroneMarkerArray );
}


/// \brief Spawning gates (with pose from ´m_GatesInitPos_WorldRF´ and ´m_GatesInitYaw_WorldRF´) in Gazebo.
/// Each gates is randomized, i.e., the texture and mesh is randomly chosen from resources
/// and the illumination is randomly set.
/// \return True if all gates successfully randomized and spawned, false otherwise.
void ForgetfulSimulator::spawnRandGatesInGazeboAndRVIZ()
{
    gazebo_msgs::SpawnModel srv;
        srv.request.reference_frame = "world";
        srv.request.robot_namespace = "RaceTrack";
    
    m_GateMarkerArray.markers.clear();
    m_GateMarkerArray.markers.reserve( m_GatesInitPos_WorldRF.size() );
    size_t TmpGateIdx;
    for ( size_t GateIdx = 0; GateIdx < m_GatesInitPos_WorldRF.size(); GateIdx++ )
    {   
        // --- Gazebo ---
        srv.request.model_name = m_GatesID[ GateIdx ];
        srv.request.model_xml = getRandGateSDF( TmpGateIdx );
        srv.request.initial_pose.position 
            = GeometryMsgsPoint_From_EigenVector3d( m_GatesInitPos_WorldRF[GateIdx] );
        srv.request.initial_pose.orientation
            = tf::createQuaternionMsgFromYaw( m_GatesInitYaw_WorldRF[GateIdx] );
        
        if ( m_ROSSrvCl_GazeboSpawnSDFModel.call( srv ) )
            ROS_INFO( "[%s]\n  > Model \"%s\" successfully spawned in Gazebo.", 
                ros::this_node::getName().c_str(), srv.request.model_name.c_str() );
        else
            ROS_ERROR( "[%s]\n  > Failed to spawn model \"%s\" in Gazebo.",
                ros::this_node::getName().c_str(), srv.request.model_name.c_str() );


        // --- RVIZ ---
        m_GateMarker.id = GateIdx;
        m_GateMarker.pose = srv.request.initial_pose;
        m_GateMarker.mesh_resource = "file://" + m_Models_DirPath + "/gate/meshes/.tmp/" + std::to_string( TmpGateIdx ) + "/gate.dae";
        
        m_GateMarkerArray.markers.push_back( m_GateMarker );
    }


    m_ROSPub_RvizGates.publish( m_GateMarkerArray );
}





/----------- Delete all Gazebo Models

void ForgetfulSimulator::ROS_CB_GazeboModelStates( const gazebo_msgs::ModelStates::ConstPtr &msg ) 
{ 
    m_GazeboModelStates = *msg;
}

//////////////////////////////////////////////////////////////////////////////
/// \brief Deletes all current models in Gazebo.                           //
/// \return True if all models got successfully deleted, False otherwise. //
///////////////////////////////////////////////////////////////////////////
void ForgetfulSimulator::deleteAllGazeboModelsExceptDrone()
{   
    // Copy required here otherwise not all models get deleted.
    std::vector<std::string> ModelNames = m_GazeboModelStates.name;
    
    gazebo_msgs::DeleteModel srv;
    for ( const std::string& ModelName : ModelNames )
    {
        if ( ModelName == m_DroneModelName ) continue;

        srv.request.model_name = ModelName;

        if ( m_ROSSrvCl_GazeboDeleteModel.call( srv ) )
            ROS_INFO( "[%s]\n  >> Model \"%s\" successfully deleted from Gazebo", 
                ros::this_node::getName().c_str(), ModelName.c_str() );
        else
            ROS_ERROR( "[%s]\n  >> Failed to delete model \"%s\" from Gazebo.",
                ros::this_node::getName().c_str(), ModelName.c_str() );
    }
}





/----------- Dynamic Gates



void ForgetfulSimulator::ROSCB_DynamicGatesSwitch( const std_msgs::Bool::ConstPtr& msg )
{
    ROS_INFO( 
        "[%s]\n  >> Dynamic gates %s.", 
        ros::this_node::getName().c_str(),
        msg->data? "enabled" : "disabled"
        );

    msg->data? m_ROSTimer_SimulatorLoop.start() : m_ROSTimer_SimulatorLoop.stop();
}


void ForgetfulSimulator::ROSTimerFunc_SimulatorLoop( const ros::TimerEvent& TimerEvent )
{   
    if ( TimerEvent.profile.last_duration.toSec() > m_SimulatorLoopTime )
        ROS_WARN(
            "[%s]\n  >> Last loop iteration took %f s exceeding nominal duration of %f s.",
            ros::this_node::getName().c_str(), 
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

        
        //m_GateMarker.id = GateIdx;
        //m_GateMarker.pose.position = ModelState.pose.position;
        //m_GateMarker.pose.orientation = ModelState.pose.orientation;
        //m_GateMarkerArray.markers.push_back( m_GateMarker );

        m_GateMarkerArray.markers[ GateIdx ].pose = ModelState.pose;


        m_ROSPub_GazeboSetModelState.publish( ModelState );
    }

    m_ROSPub_RvizGates.publish( m_GateMarkerArray );
    
}


void ForgetfulSimulator::initRandParamsOfDynGates()
{
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
    ROS_INFO( "[%s] Computing position and yaw of gates for randomized sprint race...", 
        ros::this_node::getName().c_str() );

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


    ROS_INFO( "[%s] Position and yaw of gates for randomized sprint race successfully computed:\n"
        "\tNumber of Gates:           <%d>\n"
        "\tPosition of first Gate:    <%f, %f, %f>\n"
        "\tPosition of last Gate:     <%f, %f, %f>\n",
        ros::this_node::getName().c_str(), Gates_N, 
        m_GatesInitPos_WorldRF[ 0 ].x(), m_GatesInitPos_WorldRF[ 0 ].y(), m_GatesInitPos_WorldRF[ 0 ].z(),
        m_GatesInitPos_WorldRF[ m_GatesInitPos_WorldRF.size() - 1 ].x(), m_GatesInitPos_WorldRF[ m_GatesInitPos_WorldRF.size() - 1 ].y(), m_GatesInitPos_WorldRF[ m_GatesInitPos_WorldRF.size() - 1 ].z()
        );
}

*/










}



