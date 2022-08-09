#include "forgetful_drones/forgetful_simulator.hpp"
#include "forgetful_drones/forgetful_helpers.hpp"








namespace forgetful_drone {


ForgetfulSimulator::ForgetfulSimulator (const ros::NodeHandle& rnh, const ros::NodeHandle& pnh, const bool& as_ros_node)
    :
    m_runAsROSNode {as_ros_node},

    // ROS node handles
        m_rosRNH {rnh}, 
        m_rosPNH {pnh},
    
    // ROS publishers
        m_rosPUB_RVIZ_GATES {m_rosRNH.advertise<visualization_msgs::MarkerArray>("rviz/gates", 1, true)},
        m_rosPUB_RVIZ_DRONE {m_rosRNH.advertise<visualization_msgs::MarkerArray>("rviz/drone", 1, true)},
        m_ROSPub_GazeboSetModelState {m_rosRNH.advertise <gazebo_msgs::ModelState> ("/gazebo/set_model_state", 0)},
        m_rosPUB_FLIGHTMARE_RGB {},
        //m_rosPUB_FLIGHTMARE_DEPTH {},
        //m_rosPUB_FLIGHTMARE_SEGMENTATION {},
        //m_rosPUB_FLIGHTMARE_OPTICALFLOW {},

    
    // ROS subscribers
        m_ROS_SUB_GROUND_TRUTH_ODOMETRY {m_rosRNH.subscribe("ground_truth/odometry", 1, &ForgetfulSimulator::ROSCB_GROUND_TRUTH_ODOMETRY, this)},
        //m_ROSSub_DynamicGatesSwitch( m_rosRNH.subscribe("simulator/dynamic_gates_switch", 1, &ForgetfulSimulator::ROSCB_DynamicGatesSwitch, this) ), // DYNGATES
        //m_ROSSub_GazeboModelStates( m_rosRNH.subscribe("/gazebo/model_states", 1, &ForgetfulSimulator::ROS_CB_GazeboModelStates, this ) ),  //DYNGATES

    // ROS service clients
        m_rosSVC_GAZEBO_SPAWN_URDF_MODEL {m_rosRNH.serviceClient<gazebo_msgs::SpawnModel>("/gazebo/spawn_urdf_model")},
        m_rosSVC_GAZEBO_DELETE_MODEL {m_rosRNH.serviceClient<gazebo_msgs::DeleteModel>("/gazebo/delete_model")},
        m_rosSVC_GAZEBO_SPAWN_SDF_MODEL {m_rosRNH.serviceClient<gazebo_msgs::SpawnModel>("/gazebo/spawn_sdf_model")},
        m_rosSVC_GAZEBO_GET_MODEL_STATE {m_rosRNH.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state")},
        m_rosSVC_RVIZ_LOAD_CONFIG {m_rosRNH.serviceClient<rviz::SendFilePath>("rviz/load_config")},
        //m_ROSSrvCl_GazeboResetSimulation( m_rosRNH.serviceClient<std_srvs::Empty>("/gazebo/reset_simulation") ),

    // ROS service servers
        m_rosSVS_SIMULATOR_BUILD {m_rosRNH.advertiseService("simulator/build", &ForgetfulSimulator::ROSCB_SIMULATOR_BUILD, this)},
        m_rosSVS_SIMULATOR_START {m_rosRNH.advertiseService("simulator/start", &ForgetfulSimulator::ROSCB_SIMULATOR_START, this)},
        m_rosSVS_SIMULATOR_STOP {m_rosRNH.advertiseService("simulator/stop", &ForgetfulSimulator::ROSCB_SIMULATOR_STOP, this)},
        m_rosSVS_SIMULATOR_TELEPORT {m_rosRNH.advertiseService("simulator/teleport_drone", &ForgetfulSimulator::ROSCB_SIMULATOR_TELEPORT, this)},

    // ROS timers
        //m_ROSTimer_SimulatorLoop( m_rosRNH.createTimer(ros::Duration(m_SimulatorLoopTime), &ForgetfulSimulator::ROSTimerFunc_SimulatorLoop, this, false, false) ), // DYNGATES



    // Dynamic Members
    m_UnitySceneIdx {static_cast<uint8_t>(fdBSReq::RANDOM_CHOICE)},
    m_SceneSiteIdx {static_cast<uint8_t>(fdBSReq::RANDOM_CHOICE)},
    m_TrackTypeIdx {static_cast<uint8_t>(fdBSReq::RANDOM_CHOICE)},
    m_TrackGenerationIdx {static_cast<uint8_t>(fdBSReq::RANDOM_CHOICE)},
    m_TrackDirectionIdx {static_cast<uint8_t>(fdBSReq::RANDOM_CHOICE)},
    m_GateTypeIdx {static_cast<uint8_t>(fdBSReq::RANDOM_CHOICE)},

    m_CamFrameCnt {0},
    m_CamFrameFailsCnt {0},
    m_GazeboDroneSpawned {false},
    m_RVizDroneSpawned {false},
    m_UnityLaunched {false},
    m_UnityConnected {false},
    m_SimulationRunning {false},
    m_UnityBridgePtr {nullptr}
{
    // Seed random engine
    m_RandEngine.seed (ros::WallTime::now().toNSec());


    
    // Init flightmare image publishers
    image_transport::ImageTransport IT (m_rosPNH);
    m_rosPUB_FLIGHTMARE_RGB = IT.advertise ("/flightmare/rgb", 1);
    //m_rosPUB_FLIGHTMARE_DEPTH = IT.advertise("/flightmare/depth", 1);
    //m_rosPUB_FLIGHTMARE_SEGMENTATION = IT.advertise("/flightmare/segmentation", 1);
    //m_rosPUB_FLIGHTMARE_OPTICALFLOW = IT.advertise("/flightmare/opticalflow", 1);
    

    if (initROSParameters ()) {
        ROSINFO("Init succeeded");
    } else {
        ROSERROR("Init failed, terminate ROS");
        ros::shutdown();
    }

    // Init main loop timer
    if (m_runAsROSNode) {
        double rate = p_MAIN_LOOP_FREQ;
        m_rosTMR_MAIN_LOOP = m_rosRNH.createTimer (
            /*rate*/ ros::Rate(rate),//1 / p_MAIN_LOOP_FREQ,
            /*callback*/ &ForgetfulSimulator::ROSCB_MAIN_LOOP, 
            /*obj*/ this, /*oneshot*/ false, /*autostart*/ false
        );
    }

    
    // Test simulator if specified
    if (p_TEST_ENABLED) test ();
}



ForgetfulSimulator::~ForgetfulSimulator () {}






bool ForgetfulSimulator::initROSParameters () {

    // ROS param keys and destinations of type bool
    std::vector <std::pair<const char*, const bool*>> kd_bool {
        {"SIM_UNITY_ENABLED", &p_UNITY_ENABLED},
        {"SIM_RVIZ_ENABLED", &p_RVIZ_ENABLED},
        {"SIM_TEST_ENABLED", &p_TEST_ENABLED},
    };

    // ROS param keys and destinations of type int
    std::vector <std::pair<const char*, const int*>> kd_int {
        {"SIM_UNITY_DRONE_CAMERA_WIDTH" , &p_DRONE_CAM_WIDTH},
        {"SIM_UNITY_DRONE_CAMERA_HEIGHT" , &p_DRONE_CAM_HEIGHT},
        {"SIM_RANDSPRINT_NUM_GATES" , &p_RAND_SPRINT_GATE_N},
        {"SIM_RANDSPRINT_NUM_GATES_MIN", &p_RAND_SPRINT_GATE_MIN_N},
        {"SIM_RANDSPRINT_NUM_GATES_MAX", &p_RAND_SPRINT_GATE_MAX_N},
        {"SIM_RANDSPRINT_SECTION_NUM_GATES_MIN", &p_RAND_SPRINT_SECTION_GATE_MIN_N},
        {"SIM_RANDSPRINT_SECTION_NUM_GATES_MAX", &p_RAND_SPRINT_SECTION_GATE_MAX_N},
        {"SIM_RVIZ_DRONE_NUM_ROTORS", &p_RVIZ_DRONE_ROTOR_N},
    };

    // ROS param keys and destinations of type double
    std::vector <std::pair<const char*, const double*>> kd_double
    {
        {"SIM_MAINLOOP_FREQ", &p_MAIN_LOOP_FREQ},
        {"SIM_UNITY_DRONE_CAMERA_FOV", &p_DRONE_CAM_FOV},
        {"SIM_TRACKRAND_AXSHIFT_MAX", &p_RAND_FIG8_AXIAL_SHIFT_MAX},
        {"SIM_TRACKRAND_YAWTWIST_MAX", &p_RAND_FIG8_YAW_TWIST_MAX},
        {"SIM_TRACKRAND_SCALE_MIN", &p_RAND_FIG8_SCALE_MIN},
        {"SIM_TRACKRAND_SCALE_MAX", &p_RAND_FIG8_SCALE_MAX},
        {"SIM_RANDSPRINT_SPACING_MIN", &p_RAND_SPRINT_SPACING_MIN},
        {"SIM_RANDSPRINT_SPACING_UNIDIRECTIONAL_STD_DEV", &p_RAND_SPRINT_SPACING_UNI_DIRECT_STD_DEV},
        {"SIM_RANDSPRINT_YAW_LR_MEAN", &p_RAND_SPRINT_LR_YAW_MEAN},
        {"SIM_RANDSPRINT_YAW_STD_DEV", &p_RAND_SPRINT_LR_YAW_STD_DEV},
        {"SIM_RANDSPRINT_PITCH_MEAN", &p_RAND_SPRINT_PITCH_MEAN},
        {"SIM_RANDSPRINT_PITCH_STD_DEV", &p_RAND_SPRINT_PITCH_STD_DEV},
        {"SIM_DYNGATES_AXAMP_MAX", &p_DYN_GATES_AXIAL_AMP_MAX},
        {"SIM_DYNGATES_AXFREQ_MAX", &p_DYN_GATES_AXIAL_FREQ_MAX},
        {"SIM_RVIZ_DRONE_ARM_LENGTH", &p_RVIZ_DRONE_ARM_LENGTH},
        {"SIM_RVIZ_DRONE_BODY_WIDTH", &p_RVIZ_DRONE_BODY_WIDTH},
        {"SIM_RVIZ_DRONE_BODY_HEIGHT", &p_RVIZ_DRONE_BODY_HEIGHT},
        {"SIM_RVIZ_DRONE_SCALE", &p_RVIZ_DRONE_SCALE} 
    };

    // ROS param keys and destinations of type string
    std::vector <std::pair<const char*, const std::string*>> kd_string
    {
        {"robot_description", &p_DRONE_MODEL_DESCRIPTION}
    };


    
    
    // Fetch ROS parameters, return true if successful
    return fetchROSParameters(
        m_rosRNH, kd_bool, kd_int, kd_double, kd_string, /*log_enabled*/ false);
}






void ForgetfulSimulator::test () {
    ros::Duration(5.0).sleep();
    ROSINFO(">>> TEST FORGETFUL SIMULATOR");



    // Fetch ROS Parameters for testing
    auto fRP = [this] (const std::string& key, int& dest) {
        std::string prefix = "SIM_TEST_";
        return fetchROSParameter<int>(m_rosRNH, (prefix + key).c_str(), dest);
    };
    bool all_found {true};
    int unity_scene; if (!fRP("UNITYSCENE", unity_scene)) all_found = false;
    int scene_site; if (!fRP("SCENESITE", scene_site)) all_found = false;
    int track_type; if (!fRP("TRACKTYPE", track_type)) all_found = false;
    int track_generation; if (!fRP("TRACKGENERATION", track_generation)) all_found = false;
    int track_direction; if (!fRP("TRACKDIRECTION", track_direction)) all_found = false;
    int gate_type; if (!fRP("GATETYPE", gate_type)) all_found = false;
    if (!all_found) return;



    // Set BuildSimulation request from fetched parameters
    fdBSReq req;
    req.unity_scene = static_cast<fdBSReq::_unity_scene_type>(unity_scene);
    req.scene_site = static_cast<fdBSReq::_scene_site_type>(scene_site);
    req.track_type = static_cast<fdBSReq::_track_type_type>(track_type);
    req.track_generation = static_cast<fdBSReq::_track_generation_type>(track_generation);
    req.track_direction = static_cast<fdBSReq::_track_direction_type>(track_direction);
    req.gate_type = static_cast<fdBSReq::_gate_type_type>(gate_type);


    req.unity_scene = 0;
    req.scene_site = 0;
    req.track_type = 0;
    req.track_generation = 0;
    req.track_direction = 0;
    req.gate_type = 1;


    // Call function to build simulation
    std::cout << std::endl << std::endl << std::endl;
    buildSimulation (req);
    startSimulation ();
        std::thread t1 (runMultiThreadedSpinner);
        ROSINFO("Run 10 s");
        ros::Duration (10.0).sleep();
        t1.detach();
        t1.~thread();
    stopSimulation ();
    ROSINFO("Got " << m_CamFrameCnt << "rgb frames and " << m_CamFrameFailsCnt << "fails");
    m_CamFrameCnt = 0; m_CamFrameFailsCnt = 0;
    

    // Call function to build simulation
    std::cout << std::endl << std::endl << std::endl;
    buildSimulation (req);
    startSimulation ();
        std::thread t2 (runMultiThreadedSpinner);
        ROSINFO("Run 10 s");
        ros::Duration (10.0).sleep();
        t2.detach();
        t2.~thread();
    stopSimulation ();
    ROSINFO("Got " << m_CamFrameCnt << "rgb frames and " << m_CamFrameFailsCnt << "fails");
    m_CamFrameCnt = 0; m_CamFrameFailsCnt = 0;

    
    // Call function to build simulation
    std::cout << std::endl << std::endl << std::endl;
    req.unity_scene = 1;
    buildSimulation (req);
    startSimulation ();
        std::thread t3 (runMultiThreadedSpinner);
        ROSINFO("Run 10 s");
        ros::Duration (10.0).sleep();
        t3.detach();
        t3.~thread();
    stopSimulation ();
    ROSINFO("Got " << m_CamFrameCnt << "rgb frames and " << m_CamFrameFailsCnt << "fails");
    m_CamFrameCnt = 0; m_CamFrameFailsCnt = 0;

    ros::shutdown();
}




void ForgetfulSimulator::buildSimulation_setTrackPoses () {
    const float& x = h_UNITY_TRACK_XYZYAW_WRF[m_UnitySceneIdx][m_SceneSiteIdx][0];
    const float& y = h_UNITY_TRACK_XYZYAW_WRF[m_UnitySceneIdx][m_SceneSiteIdx][1];
    const float& z = h_UNITY_TRACK_XYZYAW_WRF[m_UnitySceneIdx][m_SceneSiteIdx][2];
    const float& yaw = h_UNITY_TRACK_XYZYAW_WRF[m_UnitySceneIdx][m_SceneSiteIdx][3];

    m_UnityTrackPose = {{x, y, z}, EQf_from_Yaw(yaw, true)}; //ROSDEBUG(GET_VAR_REP(m_UnityTrackPose));
    m_GazeboTrackPose.position = {0.0, 0.0, 0.0};
    m_GazeboTrackPose.orientation = m_UnityTrackPose.orientation; //ROSDEBUG(GET_VAR_REP(m_GazeboTrackPose));
}


bool ForgetfulSimulator::startSimulation () {
    if (m_SimulationRunning) {ROSWARN("Tried to start simulation which is already running"); return true;}

    std::cout << std::endl;
    ROSINFO("Start simulation");
    std::cout << std::endl;

    
    // Gazebo
    startSimulation_deleteGazeboModelsExceptDrone();
    startSimulation_spawnGazeboGates();
    startSimulation_spawnGazeboGroundPlane();
    if (!m_GazeboDroneSpawned) {
        startSimulation_spawnGazeboDrone(); 
        m_GazeboDroneSpawned = true;
        ros::Duration(1.0).sleep();
    }


    // RViz
    if (p_RVIZ_ENABLED) {
        //resetRVIZ();
        startSimulation_spawnRVizGates();
        if (!m_RVizDroneSpawned) {
            startSimulation_spawnRVizDrone(); 
            m_RVizDroneSpawned = true; 
        }
    }

    
    // Unity
    if (p_UNITY_ENABLED) {
        if (!m_UnityLaunched) {
            startSimulation_launchUnity();
            startSimulation_addUnityGates();
            startSimulation_addUnityDrone();
            m_UnityLaunched = true;
        }

        ROSINFO("Unity: connect");
        m_UnityConnected = m_UnityBridgePtr->connectUnity(std::get<1>(h_UNITY_SCENES[m_UnitySceneIdx]));
        if (!startSimulation_configUnityGates()) return false;;
    }


    
    if (m_runAsROSNode) m_rosTMR_MAIN_LOOP.start();
    m_SimulationRunning = true;
    return true;
}





bool ForgetfulSimulator::stopSimulation() {
    if (!m_SimulationRunning) {ROSWARN("Tried to stop simulation which is not running."); return true;}

    ROSINFO("Stop simulation");

    // Unity
    if (p_UNITY_ENABLED) {
        if (m_UnityConnected) {
            ROSINFO("Unity: disconnect");
            if (m_UnityBridgePtr->disconnectUnity()) {
                m_UnityConnected = false;
            } else {
                ROSERROR("Unity: Failed to disconnect");
                return false;
            }

        }
    } 

    if (m_runAsROSNode) m_rosTMR_MAIN_LOOP.stop();
    m_SimulationRunning = false;
    return true;
}




void ForgetfulSimulator::ROSCB_GROUND_TRUTH_ODOMETRY (const nav_msgs::Odometry::ConstPtr& msg) {
    if ((!m_SimulationRunning) || (!m_UnityConnected)) return;

    m_UnityDroneState.x[flightlib::QS::POSX] = (flightlib::Scalar) msg->pose.pose.position.x + m_UnityTrackPose.position.x();
    m_UnityDroneState.x[flightlib::QS::POSY] = (flightlib::Scalar) msg->pose.pose.position.y + m_UnityTrackPose.position.y();
    m_UnityDroneState.x[flightlib::QS::POSZ] = (flightlib::Scalar) msg->pose.pose.position.z + m_UnityTrackPose.position.z();
    m_UnityDroneState.x[flightlib::QS::ATTW] = (flightlib::Scalar) msg->pose.pose.orientation.w;
    m_UnityDroneState.x[flightlib::QS::ATTX] = (flightlib::Scalar) msg->pose.pose.orientation.x;
    m_UnityDroneState.x[flightlib::QS::ATTY] = (flightlib::Scalar) msg->pose.pose.orientation.y;
    m_UnityDroneState.x[flightlib::QS::ATTZ] = (flightlib::Scalar) msg->pose.pose.orientation.z;
    
    m_UnityDronePtr->setState(m_UnityDroneState);
    m_UnityBridgePtr->getRender(0);
    m_UnityBridgePtr->handleOutput();
}


void ForgetfulSimulator::spinOnce (const ros::TimerEvent& te) {
    ROSCB_MAIN_LOOP (te);
}

void ForgetfulSimulator::ROSCB_MAIN_LOOP (const ros::TimerEvent& te) {
    checkTimerPeriod(ROS_LOG_PREFIX, te, 1 / p_MAIN_LOOP_FREQ);
    
    if ((!m_SimulationRunning) || (!m_UnityConnected)) return;

    cv::Mat img;
    if (m_UnityRGBCamPtr->getRGBImage(img)) {
        sensor_msgs::ImagePtr rgb_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
        rgb_msg->header.stamp.fromSec(ros::Time::now().toSec()); //rgb_msg->header.stamp.fromNSec(m_CamFrameCnt);
        m_rosPUB_FLIGHTMARE_RGB.publish(rgb_msg);
        m_CamFrameCnt++;
    } else {
        m_CamFrameFailsCnt++;
        //ROSWARN("Unity: failed to get RGB image");
    }
    

    /*m_UnityRGBCamPtr->getDepthMap(img);
    sensor_msgs::ImagePtr depth_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
    depth_msg->header.stamp.fromNSec(m_CamFrameCnt);
    m_ITPub_DepthImg.publish(depth_msg);

    m_UnityRGBCamPtr->getSegmentation(img);
    sensor_msgs::ImagePtr segmentation_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
    segmentation_msg->header.stamp.fromNSec(m_CamFrameCnt);
    m_ITPub_SegmentationImg.publish(segmentation_msg);

    m_UnityRGBCamPtr->getOpticalFlow(img);
    sensor_msgs::ImagePtr opticflow_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
    opticflow_msg->header.stamp.fromNSec(m_CamFrameCnt);
    m_ITPub_OpticalFlowImg.publish(opticflow_msg); */

    
}



bool ForgetfulSimulator::init (fdBS& srv) {
    return ROSCB_SIMULATOR_BUILD (srv.request, srv.response);
}
bool ForgetfulSimulator::start (fdStartS& srv) {
    return ROSCB_SIMULATOR_START (srv.request, srv.response);
}
bool ForgetfulSimulator::stop (fdStopS& srv) {
    return ROSCB_SIMULATOR_STOP (srv.request, srv.response);
}


bool ForgetfulSimulator::ROSCB_SIMULATOR_START (fdStartSReq& req, fdStartSRes& res) {
    return startSimulation();
}
bool ForgetfulSimulator::ROSCB_SIMULATOR_STOP (fdStopSReq& req, fdStopSRes& res) {
    return stopSimulation();
}


bool ForgetfulSimulator::ROSCB_SIMULATOR_TELEPORT (std_srvs::Empty::Request& req, std_srvs::Empty::Response& res) {
    gazebo_msgs::GetModelState srv;
    srv.request.model_name = h_DRONE_MODEL_NAME;

    if (!callRosSrv (ROS_LOG_PREFIX, m_rosSVC_GAZEBO_GET_MODEL_STATE, srv)) return false;
    forgetful_drone::Pose p0 (srv.response.pose);
    const forgetful_drone::Pose& p1 = m_GazeboDroneInitPose;
    
    gazebo_msgs::ModelState msg;
    msg.model_name = h_DRONE_MODEL_NAME;
    
    ros::Rate r (10.0);
    for (float t = 0; t <= 1; t += 1.0 / 100 ) {
        ROSINFO (t);
        
        forgetful_drone::Pose pt {
            p0.position * (1 - t) + p1.position * (t),
            p0.orientation.slerp (t, p1.orientation)
        };
        msg.pose = pt.as_geometry_msg ();
        m_ROSPub_GazeboSetModelState.publish (msg);
        r.sleep ();
    }
    return true;




    //ROSINFO("Gazebo: delete drone");
//
    //gazebo_msgs::DeleteModel srv;
    //srv.request.model_name = h_DRONE_MODEL_NAME;
    //if (!m_rosSVC_GAZEBO_DELETE_MODEL.call (srv)) return false;
//
    //m_GazeboDroneSpawned = false;    
    //return true;
}




bool ForgetfulSimulator::ROSCB_SIMULATOR_BUILD (fdBSReq& req, fdBSRes& res) {
    // Process request
    if (!buildSimulation (req)) return false;

    // Process response
    res.drone_init_pose = m_GazeboDroneInitPose.as_geometry_msg();
    res.gate_init_poses.clear();
    for (const Pose& p : m_GazeboGateInitPoses) {
        res.gate_init_poses.push_back(p.as_geometry_msg());
    }

    return true;
}


void ForgetfulSimulator::buildSimulation_resetMembers () {
    m_SimulationRunning = false;
    m_UnityTrackPose = Pose();
    m_GazeboTrackPose = Pose();
    m_UnityGateInitPoses.clear();
    m_GazeboGateInitPoses.clear();
    m_UnityDroneInitPose = Pose();
    m_GazeboDroneInitPose = Pose();
    m_GazeboGroundPlaneOrigin = {0.0, 0.0, 0.0};
    m_GazeboGroundPlaneSize = {0.0, 0.0};
}


void ForgetfulSimulator::buildSimulation_handleRandRequest (const fdBSReq& req) {
    std::array<const uint8_t*, 6> req_parsed = parseRequest (req);

    m_ReqLogRand = {"", "", "", "", "", ""};
    
    //bool any_random {false};
    //std::stringstream info_ss;
    for (size_t i = 0; i < req_parsed.size(); i++) {
        const uint8_t& val = *req_parsed[i];
        const uint8_t& max_val = std::get<1>(h_REQUEST[i]);
        //const char* name = std::get<0>(h_REQUEST[i]);

        if (val == fdBSReq::RANDOM_CHOICE) {
            //info_ss << name << ", ";
            std::uniform_int_distribution<uint8_t> uid {0, max_val};
            const_cast<uint8_t&>(val) = uid (m_RandEngine);
            //any_random = true;
            m_ReqLogRand[i] = " (randomly chosen)";
        }
    }

    //if (any_random) {
    //    std::string info_s = info_ss.str();
    //    info_s.pop_back(); info_s.pop_back();
    //    ROSINFO("{" << info_s << "} randomly chosen.");
    //}
}


bool ForgetfulSimulator::buildSimulation_RequestValid (const fdBSReq& req) {
    std::array<const uint8_t*, 6> req_parsed = parseRequest (req);
    
    bool all_valid {true};
    for (size_t i = 0; i < req_parsed.size(); i++) {
        const uint8_t& val = *req_parsed[i];
        const uint8_t& max_val = std::get<1>(h_REQUEST[i]);
        const char* name = std::get<0>(h_REQUEST[i]);

        if (val > max_val) {
            ROSERROR(name << ": " << val << " not in {0, ..., " << max_val << "}");
            all_valid = false;
        }
    }
    return all_valid;
}

void ForgetfulSimulator::buildSimulation_logRequest () {
    std::cout << std::endl;
    ROSINFO("Build simulation:" << std::endl
        << "\t\t- " << std::get<0>(h_REQUEST[0]) << ": " << std::get<0>(h_UNITY_SCENES[m_UnitySceneIdx])           << m_ReqLogRand[0] << std::endl
        << "\t\t- " << std::get<0>(h_REQUEST[1]) << ": " << std::get<0>(h_SCENE_SITES[m_SceneSiteIdx])             << m_ReqLogRand[1] << std::endl
        << "\t\t- " << std::get<0>(h_REQUEST[2]) << ": " << std::get<0>(h_TRACK_TYPES[m_TrackTypeIdx])             << m_ReqLogRand[2] << std::endl
        << "\t\t- " << std::get<0>(h_REQUEST[3]) << ": " << std::get<0>(h_TRACK_GENERATIONS[m_TrackGenerationIdx]) << m_ReqLogRand[3] << std::endl
        << "\t\t- " << std::get<0>(h_REQUEST[4]) << ": " << std::get<0>(h_TRACK_DIRECTIONS[m_TrackDirectionIdx])   << m_ReqLogRand[4] << std::endl
        << "\t\t- " << std::get<0>(h_REQUEST[5]) << ": " << std::get<0>(h_GATE_TYPES[m_GateTypeIdx])               << m_ReqLogRand[5]);
    std::cout << std::endl;
}



bool ForgetfulSimulator::buildSimulation (const fdBSReq& req) {
    buildSimulation_resetMembers ();
    buildSimulation_handleRandRequest (req);
    if (!buildSimulation_RequestValid (req)) return false;

    // Set members from request
    m_UnitySceneIdx = req.unity_scene;
    m_SceneSiteIdx = req.scene_site;
    m_TrackTypeIdx = req.track_type;
    m_TrackGenerationIdx = req.track_generation;
    m_TrackDirectionIdx = req.track_direction;
    m_GateTypeIdx = req.gate_type;

    buildSimulation_logRequest();
    buildSimulation_setTrackPoses();

    if (!buildSimulation_generateTrack()) return false;
    buildSimulation_generateGazeboGroundPlane();
    return true;
}


bool ForgetfulSimulator::buildSimulation_generateTrack () {
    switch (m_TrackTypeIdx) {
        case fdBSReq::track_type_FIGURE8:
            buildSimulation_generateTrack_FIGURE8(); return true;
        case fdBSReq::track_type_GAP:
            buildSimulation_generateTrack_GAP(); return true;
        
        case fdBSReq::track_type_RANDOM_CIRCUIT:
        case fdBSReq::track_type_RANDOM_SPRINT:
        default:
            ROSERROR("No implementation for " << std::get<0>(h_TRACK_TYPES[m_TrackTypeIdx])); return false;
    }
}



std::tuple<std::vector<Pose>, std::vector<Pose>> 
ForgetfulSimulator::buildSimulation_generateTrack_GAP_getInitGatePosesTRF () {
    
    // gate init poses (in track RF)
    std::vector<Pose> /*narrow*/gip_nr, /*wide*/gip_wd;
    for (const std::array<std::array<float, 3>, 2> xyyaw: h_GAP_GATE_INIT_XYYAW_TRF) {
        const float& x_nr = xyyaw[0][0], y_nr = xyyaw[0][1], yaw_nr = xyyaw[0][2];
        const float& x_wd = xyyaw[1][0], y_wd = xyyaw[1][1], yaw_wd = xyyaw[1][2];
        const float& z = std::get<1>(h_GATE_TYPES[m_GateTypeIdx]);
        
        gip_nr.push_back ({{x_nr, y_nr, z}, EQf_from_Yaw (yaw_nr, false)});
        gip_wd.push_back ({{x_wd, y_wd, z}, EQf_from_Yaw (yaw_wd, false)});
    }

    //// drone init pose (in track RF)
    //const std::array<float, 4>&  xyzyaw = h_GAP_DRONE_INIT_XYZYAW_TRF;
    //const float& x = xyzyaw[0], y = xyzyaw[1], z = xyzyaw[2], yaw = xyzyaw[3];
    //Pose dip {{x, y, z}, EQf_from_Yaw (yaw, false)};

    // return tuple
    return std::make_tuple (gip_nr, gip_wd); //, dip);
}




std::vector<Pose>
ForgetfulSimulator::buildSimulation_generateTrack_FIGURE8_getInitGatePosesTRF () {

    // gate init poses (in track RF)
    std::vector<Pose> gip;
    for (const std::array<float, 3> xyyaw: h_FIGURE8_GATE_INIT_XYYAW_TRF) {
        const float& x = xyyaw[0], y = xyyaw[1], yaw = xyyaw[2];
        const float& z = std::get<1>(h_GATE_TYPES[m_GateTypeIdx]);
        gip.push_back ({{x, y, z}, EQf_from_Yaw (yaw, false)});
    }

    //// drone init pose (in track RF)
    //const std::array<float, 4>&  xyzyaw = h_FIGURE8_DRONE_INIT_XYZYAW_TRF;
    //const float& x = xyzyaw[0], y = xyzyaw[1], z = xyzyaw[2], yaw = xyzyaw[3];
    //Pose dip {{x, y, z}, EQf_from_Yaw (yaw, false)};

    // return tuple
    return gip;//std::make_tuple (gip, dip);
}



void ForgetfulSimulator::buildSimulation_generateTrack_GAP () {
    // gate init poses and drone init pose (in track reference frame)
    std::vector<Pose> 
        /*determinstic narrow*/gip_det_nr, /*determinstic wide*/gip_det_wd,
        /*determinstic*/gip_det,
        /*averaged randomized*/gip_avg_rand, /*randomized*/gip_rand,
        /*redirected*/gip_red;
    Pose 
        /*determinstic*/dip_det,
        /*randomized*/dip_rand,
        /*redirected*/dip_red;

    // Pointers to poses in use
    std::vector<Pose>* gip_ptr; Pose* dip_ptr;

    // function to average between narrow and wide
    auto average_gip_nr_wd = [] (
        const std::vector<Pose>& gp0,
        const std::vector<Pose>& gp1,
        const float& slider_0t1
    ) {
        std::vector<Pose> gpc;
        for (size_t i = 0; i < gp0.size(); i++) {
            const Eigen::Vector3f& p0 = gp0[i].position;
            const Eigen::Vector3f& p1 = gp1[i].position;
            const Eigen::Quaternionf& q0 = gp0[i].orientation;
            const Eigen::Quaternionf& q1 = gp1[i].orientation; q0.slerp(slider_0t1, q1);

            Eigen::Vector3f pc {p0 + (p1 - p0) * slider_0t1}; 
            ROSDEBUG(GET_VAR_REP(p0.transpose()) << " | " << GET_VAR_REP(p1.transpose()) << " | " << GET_VAR_REP(pc.transpose()));
            
            Eigen::Quaternionf qc {q0.slerp(slider_0t1, q1)};
            double yaw0 = Yaw_From_EQf(q0);
            double yaw1 = Yaw_From_EQf(q0);
            double yawc = Yaw_From_EQf(qc);
            ROSDEBUG(GET_VAR_REP(yaw0) << " | " << GET_VAR_REP(yaw1) << " | " << GET_VAR_REP(yawc));

            ////yawc = yaw0 + (yaw1 - yaw0) * slider_0t1;
            //double yaw0 = Yaw_From_EQf(q0), 
            //    yaw1 = Yaw_From_EQf(q0), 
            //    yawc = yaw0 + (yaw1 - yaw0) * slider_0t1;
            ////const Eigen::Quaternionf qc {slerp(q0, q1, slider_0t1)};
            //const Eigen::Quaternionf qc {EQf_from_Yaw(p0 + (p1 - p0) * slider_0t1, false)};
            
            
            
            gpc.push_back ({/*weighted averaged position*/ pc, /*... quaternion*/ qc});
        }
        return gpc;
    };
    // determinstic poses
    std::tie (gip_det_nr, gip_det_wd) 
        = buildSimulation_generateTrack_GAP_getInitGatePosesTRF ();
    if (m_TrackGenerationIdx != fdBSReq::track_generation_RANDOMIZED) {
        gip_det = average_gip_nr_wd (gip_det_nr, gip_det_wd, 0.5);
        dip_det = buildSimulation_generateTrack_getInitDronePoseTRF (gip_det);
        gip_ptr = &gip_det; dip_ptr = &dip_det;
    } 
    
    // randomized poses
    else {
        std::uniform_real_distribution<float> urd {0.0, 1.0};
        gip_avg_rand = average_gip_nr_wd (gip_det_nr, gip_det_wd, urd(m_RandEngine));
        gip_rand = buildSimulation_generateTrack_randomizeInitGatePosesTRF (gip_avg_rand);
        dip_rand = buildSimulation_generateTrack_getInitDronePoseTRF(gip_rand);
        gip_ptr = &gip_rand; dip_ptr = &dip_rand;
    }

    // redirected poses
    if (m_TrackDirectionIdx == fdBSReq::track_direction_CLOCKWISE) {
        gip_red = buildSimulation_generateTrack_redirectInitGatePosesTRF (*gip_ptr);
        dip_red = buildSimulation_generateTrack_getInitDronePoseTRF(gip_red);
        gip_ptr = &gip_red; dip_ptr = &dip_red;
    }
    if (p_TEST_ENABLED) {
        Pose dip_det_nr = buildSimulation_generateTrack_getInitDronePoseTRF (gip_det_nr);
        Pose dip_det_wd = buildSimulation_generateTrack_getInitDronePoseTRF (gip_det_wd);
        
        gip_det = average_gip_nr_wd (gip_det_nr, gip_det_wd, 0.5);
        dip_det = buildSimulation_generateTrack_getInitDronePoseTRF (gip_det);
        
        std::uniform_real_distribution<float> urd {0.0, 1.0};
        gip_avg_rand = average_gip_nr_wd (gip_det_nr, gip_det_wd, urd(m_RandEngine));
        gip_rand = buildSimulation_generateTrack_randomizeInitGatePosesTRF (gip_avg_rand);
        dip_rand = buildSimulation_generateTrack_getInitDronePoseTRF(gip_rand);
        
        gip_red = buildSimulation_generateTrack_redirectInitGatePosesTRF (gip_rand);
        dip_red = buildSimulation_generateTrack_getInitDronePoseTRF(gip_red);

        plotTracks (
            "",
            {gip_det_nr, gip_det_wd, gip_det},
            {dip_det_nr, dip_det_wd, dip_det},
            {"narrow", "wide", "deterministic"},
            {"green", "blue", "black"}
        );

        plotTracks (
            "",
            {gip_det, gip_rand},
            {dip_det, dip_rand},
            {"determinstic", "randomized"},
            {"green", "blue"}
        );

        plotTracks (
            "",
            {gip_rand, gip_red},
            {dip_rand, dip_red},
            {"counterclockwise", "clockwise"},
            {"green", "blue"}
        );
    }

    // Transform poses from track to world reference frame and set members
    buildSimulation_generateTrack_transformInitPosesTRFtoWRF (*gip_ptr, *dip_ptr);
}


std::vector<Pose>
ForgetfulSimulator::buildSimulation_generateTrack_randomizeInitGatePosesTRF (
    const std::vector<Pose>& gps
) {
    const size_t& num_gates = gps.size();

    std::uniform_real_distribution<float> urd_mp1 {-1.0, 1.0};
    std::uniform_real_distribution<float> urd_scale {
        static_cast<float>(p_RAND_FIG8_SCALE_MIN), 
        static_cast<float>(p_RAND_FIG8_SCALE_MAX)
    };
    float scale = urd_scale(m_RandEngine);

    std::vector<Pose> gp_rand = gps;
    //Pose dp_rand = drone_pose;

    for (size_t i = 0; i < num_gates; i++) {
        Eigen::Vector3f axial_shift 
            = Eigen::Vector3f {
                urd_mp1 (m_RandEngine), 
                urd_mp1 (m_RandEngine), 
                urd_mp1 (m_RandEngine) + 1.0F
            } * static_cast<float>(p_RAND_FIG8_AXIAL_SHIFT_MAX);
        gp_rand[i].position += axial_shift;
        gp_rand[i].position *= scale;

        float yaw_twist = p_RAND_FIG8_YAW_TWIST_MAX * urd_mp1 (m_RandEngine);
            //double yaw_before_randomization = gp_rand[i].yaw();
        gp_rand[i].orientation *= EQf_from_Yaw(yaw_twist, false);
            //double yaw_after_randomization = gp_rand[i].yaw();
            //ROSDEBUG(
            //    GET_VAR_REP(yaw_before_randomization) << " | " 
            //    << GET_VAR_REP(yaw_after_randomization) << " | " 
            //    << GET_VAR_REP(p_RAND_FIG8_YAW_TWIST_MAX));

        //// drone starting position is next to second last gate
        //if (i == num_gates - 2) {
        //    dp_rand.position += axial_shift;
        //    dp_rand.position *= scale;
        //}
    }

    return gp_rand; //std::make_tuple (gp_rand, dp_rand);
}

std::vector<Pose> 
ForgetfulSimulator::buildSimulation_generateTrack_redirectInitGatePosesTRF (
    const std::vector<Pose>& gps
) {
    const size_t& num_gates = gps.size();
    const size_t start_gate_idx = num_gates - 2;

    


    //for (size_t i = 0; i < num_gates; i++) {
    //    gp_red[i].position.y() *= -1;
    //
    //    if (i != num_gates - 1) {
    //        gp_red[i] = gps[num_gates - 2 - i];
    //    }
    //}

    std::vector<forgetful_drone::Pose> gp_red = gps;
    //Pose dp_red = drone_pose;
    
    for (size_t i = 0; i < num_gates; i++) {
        gp_red[i] = gps[(start_gate_idx - i + num_gates) % num_gates];
        gp_red[i].orientation *= EQf_from_Yaw (180, true);
    }

    //dp_red.position.y() *= -1;
    //dp_red.orientation *= EQf_from_Yaw (180.0, true);

    return gp_red; // std::make_tuple (gp_red, dp_red);
}



Pose ForgetfulSimulator::buildSimulation_generateTrack_getInitDronePoseTRF (
    std::vector<forgetful_drone::Pose> gps
) {
    const size_t& num_gates = gps.size();
    const size_t idx_front = num_gates - 1 - h_DRONE_INIT_POSE_NUM_GATES_SETBACK;
    const size_t idx_back = idx_front - 1;
    const Pose& pose_f = gps[idx_front], pose_b = gps[idx_back];
    const Eigen::Vector3f& pos_f = pose_f.position, pos_b = pose_b.position;
    const Eigen::Quaternionf& ori_f = pose_f.orientation/*, ori_b = pose_b.orientation*/;

    // Set drone orientation equal to orientation of front gate
    Eigen::Quaternionf d_ori = ori_f * EQf_from_Yaw (-h_GATE_GAZEBO_MODEL_YAW_TWIST, false);
    //Eigen::Quaternionf d_ori = slerp(ori_b, ori_f, h_DRONE_INIT_POSE_BACK2FRONT_SLIDER);

    // Set drone position between back and front gate
    Eigen::Vector3f d_pos = pos_b + (pos_f - pos_b) * h_DRONE_INIT_POSE_BACK2FRONT_SLIDER;
    // x/y-project drone position onto line traverssing front gate
    Eigen::Vector2f ldv { // line direction vector
        std::cos(pose_f.yaw() - h_GATE_GAZEBO_MODEL_YAW_TWIST), 
        std::sin(pose_f.yaw() - h_GATE_GAZEBO_MODEL_YAW_TWIST)
    };
    Eigen::Matrix2f opm // orthogonal projection matrix
        = ldv * ldv.transpose() / (ldv.transpose() * ldv);
    Eigen::Vector2f p0 { // arbitrary point on line
        pos_f.x(), 
        pos_f.y()
    };
    Eigen::Vector2f p { // point to project on line
        d_pos.x(), 
        d_pos.y()
    }; 
    Eigen::Vector2f d_pos_xy = opm * (p - p0) + p0;
    d_pos.x() = d_pos_xy.x();
    d_pos.y() = d_pos_xy.y();


    //ROSDEBUG(std::endl
    //    << GET_VAR_REP(p) << std::endl
    //    << GET_VAR_REP(p0) << std::endl
    //    << GET_VAR_REP(ldv) << std::endl
    //    << GET_VAR_REP(opm) << std::endl
    //    << GET_VAR_REP(d_pos_xy) << std::endl
    //);


    
    return {d_pos, d_ori};
}


void ForgetfulSimulator::buildSimulation_generateTrack_transformInitPosesTRFtoWRF (
    const std::vector<Pose>& gate_init_poses_TRF,
    const Pose& drone_init_pose_TRF
) {
    using kmQT = kindr::minimal::QuatTransformation;
    // u: unity, g: gazebo

    // Transformations: track -> world reference frame
    kmQT uWRF_TRF; tf::poseMsgToKindr(m_UnityTrackPose.as_geometry_msg(), &uWRF_TRF);
    kmQT gWRF_TRF; tf::poseMsgToKindr(m_GazeboTrackPose.as_geometry_msg(), &gWRF_TRF);

    // Apply transformations to gate poses
    m_UnityGateInitPoses.clear(); 
    m_GazeboGateInitPoses.clear();
    for (const Pose& gip_TRF: gate_init_poses_TRF) {
        kmQT TRF_GRF; tf::poseMsgToKindr(gip_TRF.as_geometry_msg(), &TRF_GRF);
        kmQT uWRF_GRF = uWRF_TRF * TRF_GRF;
        kmQT gWRF_GRF = gWRF_TRF * TRF_GRF;

        m_UnityGateInitPoses.push_back({
            uWRF_GRF.getPosition().cast<float>(),
            uWRF_GRF.getEigenQuaternion().cast<float>()
        });
        m_GazeboGateInitPoses.push_back({
                gWRF_GRF.getPosition().cast<float>(),
                gWRF_GRF.getEigenQuaternion().cast<float>()
        });
    }
    // Apply transformations to drone pose
    {
        kmQT TRF_DRF; tf::poseMsgToKindr(drone_init_pose_TRF.as_geometry_msg(), &TRF_DRF);
        kmQT uWRF_DRF = uWRF_TRF * TRF_DRF;
        kmQT gWRF_DRF = gWRF_TRF * TRF_DRF;

        m_UnityDroneInitPose = {
            uWRF_DRF.getPosition().cast<float>(),
            uWRF_DRF.getEigenQuaternion().cast<float>()
        };
        m_GazeboDroneInitPose = {
            gWRF_DRF.getPosition().cast<float>(),
            gWRF_DRF.getEigenQuaternion().cast<float>()
        };
    }
}



void ForgetfulSimulator::buildSimulation_generateTrack_FIGURE8 () {
    // gate init poses and drone init pose (in track reference frame)
    std::vector<Pose> 
        /*determinstic*/gip_det, 
        /*randomized*/gip_rand, 
        /*redirected*/gip_red;
    Pose 
        /*determinstic*/dip_det, 
        /*randomized*/dip_rand, 
        /*redirected*/dip_red;

    // Pointers to poses in use
    std::vector<Pose>* gip_ptr; Pose* dip_ptr;
    
    // deterministic poses
    gip_det = buildSimulation_generateTrack_FIGURE8_getInitGatePosesTRF();
    dip_det = buildSimulation_generateTrack_getInitDronePoseTRF(gip_det);
    gip_ptr = &gip_det; dip_ptr = &dip_det;

    // randomized poses
    if (m_TrackGenerationIdx == fdBSReq::track_generation_RANDOMIZED) {
        gip_rand = buildSimulation_generateTrack_randomizeInitGatePosesTRF (*gip_ptr);
        dip_rand = buildSimulation_generateTrack_getInitDronePoseTRF(gip_rand);
        gip_ptr = &gip_rand; dip_ptr = &dip_rand;
    }

    // redirected poses
    if (m_TrackDirectionIdx == fdBSReq::track_direction_CLOCKWISE) {
        gip_red = buildSimulation_generateTrack_redirectInitGatePosesTRF (*gip_ptr);
        dip_red = buildSimulation_generateTrack_getInitDronePoseTRF(gip_red);
        gip_ptr = &gip_red; dip_ptr = &dip_red;
    }

    if (p_TEST_ENABLED) {
        std::vector<Pose>* gip_ccw_ptr; Pose* dip_ccw_ptr;
        if (m_TrackGenerationIdx == fdBSReq::track_generation_RANDOMIZED) {
            gip_ccw_ptr = &gip_rand; dip_ccw_ptr = &dip_rand;
        } else {
            gip_rand = buildSimulation_generateTrack_randomizeInitGatePosesTRF (gip_det);
            dip_rand = buildSimulation_generateTrack_getInitDronePoseTRF(gip_rand);
            gip_ccw_ptr = &gip_det; dip_ccw_ptr = &dip_det;
        }

        plotTracks (
            "",
            {gip_det, gip_rand},
            {dip_det, dip_rand},
            {"deterministic", "randomized"},
            {"black", "red"}
        );

        plotTracks (
            "",
            {*gip_ccw_ptr, gip_red},
            {*dip_ccw_ptr, dip_red},
            {"counterclockwise", "clockwise"},
            {"green", "blue"}
        );
    }

    // Transform poses from track to world reference frame and set members
    buildSimulation_generateTrack_transformInitPosesTRFtoWRF (*gip_ptr, *dip_ptr);
}

/*
bool ForgetfulSimulator::buildSimulation_OLD (
    const forgetful_drones::BuildDroneRacingSimulation::Request::_UnityScene_type unity_scene,
    const forgetful_drones::BuildDroneRacingSimulation::Request::_RacetrackSite_type& scene_site,
    const forgetful_drones::BuildDroneRacingSimulation::Request::_RacetrackType_type& racetrack_type,
    const forgetful_drones::BuildDroneRacingSimulation::Request::_RaceGateType_type& gate_type,

    const forgetful_drones::BuildDroneRacingSimulation::Request::_RacetrackClockwise_type& RacetrackClockwise,
    const forgetful_drones::BuildDroneRacingSimulation::Request::_RacetrackMode_type& RacetrackMode  
) {
    ROSINFO("Build drone racing simulation.");

    m_SimulationRunning = false;
    m_UnityTrackPose = Pose();
    m_GazeboTrackPose = Pose();
    m_UnityGateInitPoses.clear();
    m_GazeboGateInitPoses.clear();
    m_UnityDroneInitPose = Pose();
    m_GazeboDroneInitPose = Pose();
    m_GazeboGroundPlaneOrigin = {0.0, 0.0, 0.0};
    m_GazeboGroundPlaneSize = {0.0, 0.0};
    
    
    

    std::string rc_tag = "";
    if (unity_scene == forgetful_drones::BuildDroneRacingSimulation::Request::RANDOM_CHOICE) {
        std::array<uint8_t, 5> UnityScenes{
            forgetful_drones::BuildDroneRacingSimulation::Request::SPACESHIP_INTERIOR,
            forgetful_drones::BuildDroneRacingSimulation::Request::DESTROYED_CITY,
            forgetful_drones::BuildDroneRacingSimulation::Request::INDUSTRIAL_PARK,
            forgetful_drones::BuildDroneRacingSimulation::Request::POLYGON_CITY,
            forgetful_drones::BuildDroneRacingSimulation::Request::DESERT_MOUNTAIN
            };
        std::uniform_int_distribution<size_t> UIDistri_0To4{0, 4};
        const_cast<uint8_t&>(unity_scene) = UnityScenes[UIDistri_0To4(m_RandEngine)];
        rc_tag = " (randomly chosen)";
    }
    switch (unity_scene) {   
        case forgetful_drones::BuildDroneRacingSimulation::Request::SPACESHIP_INTERIOR:
            ROSDEBUG("Unity scene: " << "SPACESHIP_INTERIOR" << rc_tag);
            break;
        case forgetful_drones::BuildDroneRacingSimulation::Request::DESTROYED_CITY:
            ROSDEBUG("Unity scene: " << "DESTROYED_CITY" << rc_tag);
            break;
        case forgetful_drones::BuildDroneRacingSimulation::Request::INDUSTRIAL_PARK:
            ROSDEBUG("Unity scene: " << "INDUSTRIAL_PARK" << rc_tag);
            break;
        case forgetful_drones::BuildDroneRacingSimulation::Request::POLYGON_CITY:
            ROSDEBUG("Unity scene: " << "POLYGON_CITY" << rc_tag);
            break;
        case forgetful_drones::BuildDroneRacingSimulation::Request::DESERT_MOUNTAIN:
            ROSDEBUG("Unity scene: " << "DESERT_MOUNTAIN" << rc_tag);
            break;
        default:
            ROSERROR("No implementation for " << GET_VAR_REP(unity_scene));
            return false; break;
    }
    m_UnitySceneIdx = unity_scene;

    rc_tag = "";
    if (scene_site == forgetful_drones::BuildDroneRacingSimulation::Request::RANDOM_CHOICE) {
        std::array<uint8_t, 3> RacetrackSites{
            forgetful_drones::BuildDroneRacingSimulation::Request::SITE_A,
            forgetful_drones::BuildDroneRacingSimulation::Request::SITE_B,
            forgetful_drones::BuildDroneRacingSimulation::Request::SITE_C
            };
        std::uniform_int_distribution<size_t> UIDistri_0To2{0, 2};
        const_cast<uint8_t&>(scene_site) = RacetrackSites[UIDistri_0To2(m_RandEngine)];
        rc_tag = " (randomly chosen)";
    }
    switch (scene_site) {
        case forgetful_drones::BuildDroneRacingSimulation::Request::SITE_A:
            ROSDEBUG("Scene site: " << "A" << rc_tag);
            break;
        case forgetful_drones::BuildDroneRacingSimulation::Request::SITE_B:
            ROSDEBUG("Scene site: " << "B" << rc_tag);
            break;
        case forgetful_drones::BuildDroneRacingSimulation::Request::SITE_C:
            ROSDEBUG("Scene site: " << "C" << rc_tag);
            break;
        default:
            ROSERROR("No implementation for " << GET_VAR_REP(scene_site));
            return false; break;
    }
    m_SceneSiteIdx = scene_site;

    buildSimulation_setTrackPoses();


    rc_tag = "";
    if (gate_type == forgetful_drones::BuildDroneRacingSimulation::Request::RANDOM_CHOICE) {
        std::array<uint8_t, 3> RaceGateTypes{
            forgetful_drones::BuildDroneRacingSimulation::Request::RPG_GATE,
            forgetful_drones::BuildDroneRacingSimulation::Request::TUB_DAI_GATE,
            forgetful_drones::BuildDroneRacingSimulation::Request::THU_DME_GATE
            };
        std::uniform_int_distribution<size_t> UIDistri_0To2{0, 0};
        const_cast<uint8_t&>(gate_type) = RaceGateTypes[UIDistri_0To2(m_RandEngine)];
        rc_tag = " (randomly chosen)";
    }
    switch (gate_type) {
        case forgetful_drones::BuildDroneRacingSimulation::Request::RPG_GATE:
            m_GateMinZ = 2.00;
            m_GatePrefabID = "rpg_gate";
            m_GateSTLFpath = ROS_PACKAGE_PATH + "/gazebo/rpg_gate/model.stl";
            m_GateSDFFpath = ROS_PACKAGE_PATH + "/gazebo/rpg_gate/model.sdf";
            ROSDEBUG("Gate type: " << "RPG_GATE" << rc_tag);
            break;
        case forgetful_drones::BuildDroneRacingSimulation::Request::TUB_DAI_GATE:
            m_GateMinZ = 2.00;
            m_GatePrefabID = "tub_dai_gate";
            m_GateSTLFpath = ROS_PACKAGE_PATH + "/gazebo/tub_dai_gate/model.stl";
            m_GateSDFFpath = ROS_PACKAGE_PATH + "/gazebo/tub_dai_gate/model.sdf";
            ROSDEBUG("Gate type: " << "TUB_DAI_GATE" << rc_tag);
            break;
        case forgetful_drones::BuildDroneRacingSimulation::Request::THU_DME_GATE:
            m_GateMinZ = 2.00;
            m_GatePrefabID = "thu_dme_gate";
            m_GateSTLFpath = ROS_PACKAGE_PATH + "/gazebo/thu_dme_gate/model.stl";
            m_GateSDFFpath = ROS_PACKAGE_PATH + "/gazebo/thu_dme_gate/model.sdf";
            ROSDEBUG("Gate type: " << "THU_DME_GATE" << rc_tag);
            break;
        default:
            ROSERROR("No implementation for " << GET_VAR_REP(gate_type));
            return false; break;
    }


    

    rc_tag = "";
    if (racetrack_type == forgetful_drones::BuildDroneRacingSimulation::Request::RANDOM_CHOICE) {
        std::array<uint8_t, 3> RacetrackTypes{
            forgetful_drones::BuildDroneRacingSimulation::Request::FIGURE8_RANDOMIZED,
            //forgetful_drones::BuildDroneRacingSimulation::Request::CIRCUIT_RANDOMIZED,
            //forgetful_drones::BuildDroneRacingSimulation::Request::SPRINT_RANDOMIZED
            };
        std::uniform_int_distribution<size_t> UIDistri_0To2{0, 2};
        const_cast<uint8_t&>(racetrack_type) = RacetrackTypes[UIDistri_0To2(m_RandEngine)];
        rc_tag = " (randomly chosen)";
    }
    switch (racetrack_type) {
        case forgetful_drones::BuildDroneRacingSimulation::Request::FIGURE8_DETERMINISTIC:
            computeRaceTrack_Figure8(racetrack_type);
            ROSDEBUG("Racetrack type: " << "FIGURE8_DETERMINISTIC" << rc_tag);
            break;
        case forgetful_drones::BuildDroneRacingSimulation::Request::FIGURE8_RANDOMIZED:
            computeRaceTrack_Figure8(racetrack_type);
            ROSDEBUG("Racetrack type: " << "FIGURE8_RANDOMIZED" << rc_tag);
            break;
        case forgetful_drones::BuildDroneRacingSimulation::Request::INTERMEDIATE_TARGET_LOSS:
            computeRaceTrack_IntermediateTargetLoss(RacetrackClockwise, RacetrackMode);
            ROSDEBUG("Racetrack type: " << "INTERMEDIATE_TARGET_LOSS" << rc_tag 
                << ", clockwise: " << RacetrackClockwise << ", mode: " << RacetrackMode);
            break;
        case forgetful_drones::BuildDroneRacingSimulation::Request::CIRCUIT_RANDOMIZED:
        case forgetful_drones::BuildDroneRacingSimulation::Request::SPRINT_RANDOMIZED: 
            //computeGatePoses_SprintRand(); break;
        default:
            ROSERROR("No implementation for " << GET_VAR_REP(racetrack_type));
            return false; break;
    }


    if (p_TEST_ENABLED) { // Plot gate positions
        std::vector<float> Gates_X, Gates_Y, Gates_Z;
        float Gates_MaxX = - 1e10;
        float Gates_MinX = + 1e10;
        float Gates_MaxY = - 1e10;
        float Gates_MinY = + 1e10;
        for (size_t Gate_i = 0; Gate_i < m_UnityGateInitPoses.size(); Gate_i++)
        {
            Gates_X.push_back(m_UnityGateInitPoses[Gate_i].position.x());
            Gates_Y.push_back(m_UnityGateInitPoses[Gate_i].position.y());
            Gates_Z.push_back(m_UnityGateInitPoses[Gate_i].position.z());

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
        for (size_t Gate_i = 0; Gate_i < m_UnityGateInitPoses.size(); Gate_i++)
        {
            const std::string Text = "Gate " + std::to_string(Gate_i);
            matplotlibcpp::text(Gates_X[Gate_i] + TextShift_X, Gates_Y[Gate_i] + TextShift_Y, Text);
        }

        std::vector<float> Drone_X {m_UnityDroneInitPose.position.x()};
        std::vector<float> Drone_Y {m_UnityDroneInitPose.position.y()};
        matplotlibcpp::scatter(Drone_X, Drone_Y, 100);
        matplotlibcpp::text(Drone_X[0] + TextShift_X, Drone_Y[0] + TextShift_Y, "Drone start position");

        //matplotlibcpp::legend();
        //matplotlibcpp::save("./basic.png");
        matplotlibcpp::show();
    }


    buildSimulation_generateGazeboGroundPlane();

    


    
    //// Spawn environment in Gazebo for collision
    //gazebo_msgs::SpawnModel srv;
    //    srv.request.reference_frame = "world";
    //    srv.request.robot_namespace = "environment";
    //srv.request.model_name = "spaceship_interior_00";
    //std::ifstream ModelXML(ROS_PACKAGE_PATH + "/gazebo/spaceship_interior/00/model.sdf");
    //std::stringstream Buffer; Buffer << ModelXML.rdbuf();
    //srv.request.model_xml = Buffer.str();
    ////std::cout<<Buffer.str()<<std::endl;
    ////srv.request.initial_pose.position 
    ////    = GMP_From_EV3d( m_GatesInitPos_WorldRF[GateIdx] );
    ////srv.request.initial_pose.orientation
    ////    = tf::createQuaternionMsgFromYaw( m_GatesInitYaw_WorldRF[GateIdx] );
    //
    //if ( m_rosSVC_GAZEBO_SPAWN_SDF_MODEL.call( srv ) )
    //    MY_ROS_INFO("[%s]\n  > Successfully spawned \"%s\" in Gazebo.", 
    //        m_ROS_NODE_NAME, srv.request.model_name.c_str());
    //else
    //    MY_ROS_ERROR("[%s]\n  > Failed to spawn \"%s\" in Gazebo.",
    //        m_ROS_NODE_NAME, srv.request.model_name.c_str());
    

    
    //// --> get point cloud from unity and spawn collision model in gazebo
    //flightlib::PointCloudMessage_t PointCloudMsg;
    //PointCloudMsg.path = ROS_PACKAGE_PATH + "/tmp/";
    //PointCloudMsg.file_name = "point_cloud"; // .ply is added automatically
    //PointCloudMsg.origin = {
    //    (float)RaceTrackPose_WorldRF.position.x, 
    //    (float)RaceTrackPose_WorldRF.position.y,
    //    (float)RaceTrackPose_WorldRF.position.z
    //    };
    //PointCloudMsg.range = {
    //    35.0, 
    //    35.0, 
    //    (float)(RaceTrackPose_WorldRF.position.z + 3*GateWaypointHeight)
    //    };
    //PointCloudMsg.resolution = 0.10;
    //bool FM_PointCloudReady = m_UnityBridgePtr->getPointCloud(PointCloudMsg);
    //
    //if (FM_PointCloudReady)
    //    MY_ROS_INFO("#############################################################################");
    //else
    //    MY_ROS_INFO("+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++");
    
    
    
    //initRandParamsOfDynGates();
    //req.DynGatesActivated? m_ROSTimer_SimulatorLoop.start() : m_ROSTimer_SimulatorLoop.stop();
    
    
    return true;
}
*/

/*
void ForgetfulSimulator::computeRaceTrack_IntermediateTargetLoss(const bool& IsClockwise, const int& Mode)
{
    constexpr int GatesN = 7;
    std::array<Eigen::Vector3d, GatesN +1> GatesAndDroneInitPos_RaceTrackRF;
    std::array<double, GatesN +1> GatesAndDroneInitYaw_RaceTrackRF;

    switch (Mode) // WIDE OR NARROW GAP
    {
    case 0: // sharp 
         GatesAndDroneInitPos_RaceTrackRF = {
            Eigen::Vector3d{ -20.45 +10,   -8.65 * 0.8  ,   m_GateMinZ  }, // #01
            Eigen::Vector3d{ -12.55 +10 ,   -11.15 * 0.6 ,   m_GateMinZ  }, // #02
            //Eigen::Vector3d{ -4.15  ,   -5.35   ,   m_GateMinZ  }, // #03
            
            Eigen::Vector3d{ 2.85 +10  ,   -5.95 * 0.6  ,   m_GateMinZ  }, // #10

            Eigen::Vector3d{ 3.45 +10   ,   4.25 * 0.6   ,   m_GateMinZ  }, // #04
            //Eigen::Vector3d{ 11.95  ,   11.15   ,   m_GateMinZ  }, // #05
            //Eigen::Vector3d{ 21.85  ,   6.85    ,   m_GateMinZ  }, // #06
            //Eigen::Vector3d{ 24.25  ,   -1.75   ,   m_GateMinZ  }, // #07
            //Eigen::Vector3d{ 19.25  ,   -9.55   ,   m_GateMinZ  }, // #08
            //Eigen::Vector3d{ 10.55  ,   -10.65  ,   m_GateMinZ  }, // #09
            
            //Eigen::Vector3d{ -4.95  ,   4.65    ,   m_GateMinZ  }, // #11
            Eigen::Vector3d{ -12.95 +10,   9.65 * 0.6   ,   m_GateMinZ  }, // #12
            Eigen::Vector3d{ -21.05 +10,   6.65 * 0.8   ,   m_GateMinZ  }, // #13
            Eigen::Vector3d{ -24.25 +10 ,   -2.15   ,   m_GateMinZ  }, // #14
            Eigen::Vector3d{ -24.00 +10 ,   6.65    ,   0.2                     }  // Drone
            };
        // Gate and drone yaws in race track reference frame:
        GatesAndDroneInitYaw_RaceTrackRF = {
            -2.01e+00   ,   // #01s
            -1.57e+00   ,   // #02
            //-6.01e-01   ,   // #03
            
            (-1.57e+00) * 0.9,//-2.57e+00   ,   // #10

            (-1.57e+00) * 1.1, //-3.77e+00   ,   // #04
            //1.93e+00    ,   // #05
            //9.99e-01    ,   // #06
            //-7.96e-04   ,   // #07
            //-4.17e+00   ,   // #08
            //1.53e+00    ,   // #09
            
            //-2.47e+00   ,   // #11
            -4.67e+00   ,   // #12
            -7.71e-01   ,   // #13
            -3.07e+00   ,   // #14
            -M_PI/2        // Drone 
            };
        break;
    case 1: //wide
        GatesAndDroneInitPos_RaceTrackRF = {
            Eigen::Vector3d{ -20.45 +10,   -8.65 * 0.8  ,   m_GateMinZ  }, // #01
            Eigen::Vector3d{ -12.55 +10 ,   -11.15 * 0.7 ,   m_GateMinZ  }, // #02
            //Eigen::Vector3d{ -4.15  ,   -5.35   ,   m_GateMinZ  }, // #03
            //Eigen::Vector3d{ 3.45 +10   ,   4.25 * 1.2   ,   m_GateMinZ  }, // #04

            Eigen::Vector3d{ 10.55  ,   -10.65 *0.9 ,   m_GateMinZ  }, // #09
            
            Eigen::Vector3d{ 11.95  ,   11.15  *0.9 ,   m_GateMinZ  }, // #05

            //Eigen::Vector3d{ 21.85  ,   6.85    ,   m_GateMinZ  }, // #06
            //Eigen::Vector3d{ 24.25  ,   -1.75   ,   m_GateMinZ  }, // #07
            //Eigen::Vector3d{ 19.25  ,   -9.55   ,   m_GateMinZ  }, // #08
            
            //Eigen::Vector3d{ 2.85     ,   -5.95,   m_GateMinZ  }, // #10
            //Eigen::Vector3d{ -4.95  ,   4.65    ,   m_GateMinZ  }, // #11
            Eigen::Vector3d{ -12.95 +10,   9.65 * 0.7   ,   m_GateMinZ  }, // #12
            Eigen::Vector3d{ -21.05 +10,   6.65 * 0.8   ,   m_GateMinZ  }, // #13
            Eigen::Vector3d{ -24.25 +10 ,   -2.15   ,   m_GateMinZ  }, // #14
            Eigen::Vector3d{ -24.00 +10 ,   6.65    ,   0.2                     }  // Drone
            };
        // Gate and drone yaws in race track reference frame:
        GatesAndDroneInitYaw_RaceTrackRF = {
            -2.01e+00   ,   // #01s
            -1.57e+00   ,   // #02
            //-6.01e-01   ,   // #03
            //-3.77e+00   ,   // #04
            
            -4.17e+00 , //1.53e+00    ,   // #09
            
            9.99e-01, //1.93e+00    ,   // #05
            //9.99e-01    ,   // #06
            //-7.96e-04   ,   // #07
            //-4.17e+00   ,   // #08
            
            //-2.57e+00   ,   // #10
            //-2.47e+00   ,   // #11
            -4.67e+00   ,   // #12
            -7.71e-01   ,   // #13
            -3.07e+00   ,   // #14
            -M_PI/2        // Drone 
            };
    default:
        break;
    }


    if (IsClockwise)
    {
        //std::array<Eigen::Vector3d, GatesN> GatesInitPos;
        //std::array<double, GatesN> GatesInitYaw;
        //for (size_t Gate_i = 0; Gate_i < GatesN; Gate_i++)
        //{
        //    GatesInitPos[Gate_i] = GatesAndDroneInitPos_RaceTrackRF[Gate_i];
        //    GatesInitYaw[Gate_i] = GatesAndDroneInitYaw_RaceTrackRF[Gate_i];
        //}
        //std::reverse(GatesInitPos.begin(), GatesInitPos.end());
        //std::reverse(GatesInitYaw.begin(), GatesInitYaw.end());
        //for (size_t Gate_i = 0; Gate_i < GatesN; Gate_i++)
        //{
        //    GatesAndDroneInitPos_RaceTrackRF[Gate_i] = GatesInitPos[Gate_i];
        //    GatesAndDroneInitYaw_RaceTrackRF[Gate_i] = GatesInitYaw[Gate_i];
        //}            
        //GatesAndDroneInitPos_RaceTrackRF[3].y() *= -1;
        //GatesAndDroneInitYaw_RaceTrackRF[3] = M_PI/2;

        for (size_t Gate_i = 0; Gate_i < GatesN; Gate_i++)
        {
            GatesAndDroneInitPos_RaceTrackRF[Gate_i].y() *= -1;            
        }
        GatesAndDroneInitPos_RaceTrackRF.back().y() *= -1;
        GatesAndDroneInitYaw_RaceTrackRF.back() = M_PI/2;   

        std::array<double, 8UL> _ = GatesAndDroneInitYaw_RaceTrackRF;
        for (size_t Gate_i = 0; Gate_i < GatesN - 1; Gate_i++)
        {
            GatesAndDroneInitYaw_RaceTrackRF[Gate_i] = _[GatesN - 2 - Gate_i];
        }

    }

    

    std::vector<float> Gates_X_Det, Gates_Y_Det, Gates_Z_Det;
    float Gates_MaxX, Gates_MinX, Gates_MaxY, Gates_MinY;
    std::vector<float> Drone_X_Det {static_cast<float>(GatesAndDroneInitPos_RaceTrackRF.back().x())};
    std::vector<float> Drone_Y_Det {static_cast<float>(GatesAndDroneInitPos_RaceTrackRF.back().y())};
    if (p_TEST_ENABLED)
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
    //if (RacetrackType == forgetful_drones::BuildDroneRacingSimulation::Request::FIGURE8_RANDOMIZED)
    //{
        std::uniform_real_distribution<double> URDistri_MinusToPlus1(-1.0, 1.0);
        std::uniform_real_distribution<double> URDistri_Scale(p_RAND_FIG8_SCALE_MIN, p_RAND_FIG8_SCALE_MAX);
        double Scale = URDistri_Scale(m_RandEngine);

        for (size_t GateIdx = 0; GateIdx < GatesN; GateIdx++)
        {
            Eigen::Vector3d AxialShift = p_RAND_FIG8_AXIAL_SHIFT_MAX * Eigen::Vector3d{ 
                    URDistri_MinusToPlus1(m_RandEngine), 
                    URDistri_MinusToPlus1(m_RandEngine), 
                    URDistri_MinusToPlus1(m_RandEngine) +1.0
                };
            GatesAndDroneInitPos_RaceTrackRF[GateIdx] += AxialShift;
            GatesAndDroneInitPos_RaceTrackRF[GateIdx] *= Scale;

            double YawTwist = p_RAND_FIG8_YAW_TWIST_MAX * URDistri_MinusToPlus1(m_RandEngine);
            GatesAndDroneInitYaw_RaceTrackRF[GateIdx] += YawTwist;

            if (GateIdx == GatesAndDroneInitPos_RaceTrackRF.size() -3) // drone starting position is next to second last gate
            {
                GatesAndDroneInitPos_RaceTrackRF.back() += AxialShift;
                GatesAndDroneInitPos_RaceTrackRF.back() *= Scale;
            }
        }
    //}


    
    
    if (p_TEST_ENABLED)
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
        //keywords_det.insert(std::make_pair("label", "Deterministic"));
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
    m_UnityGateInitPoses.resize(GatesN);
    m_GazeboGateInitPoses.resize(GatesN);

    // Create Transformation between reference frames: race track -> world 
    kindr::minimal::QuatTransformation T_WorldRF_RaceTrackRF_Flightmare;
    tf::poseMsgToKindr(m_UnityTrackPose.as_geometry_msg(), &T_WorldRF_RaceTrackRF_Flightmare);
    kindr::minimal::QuatTransformation T_WorldRF_RaceTrackRF_Gazebo;
    tf::poseMsgToKindr(m_GazeboTrackPose.as_geometry_msg(), &T_WorldRF_RaceTrackRF_Gazebo);


    // Apply transformation to compute gate poses in world reference frame
    for (size_t GateIdx = 0; GateIdx < GatesN; GateIdx++)
    {
        geometry_msgs::Pose GateInitPose_RaceTrackRF;
        GateInitPose_RaceTrackRF.position = GMPoint__from__EV3d(GatesAndDroneInitPos_RaceTrackRF[GateIdx]);
        GateInitPose_RaceTrackRF.orientation = tf::createQuaternionMsgFromYaw(GatesAndDroneInitYaw_RaceTrackRF[GateIdx]);
        
        kindr::minimal::QuatTransformation T_RaceTrackRF_GateInitRF;
        tf::poseMsgToKindr(GateInitPose_RaceTrackRF, &T_RaceTrackRF_GateInitRF);
        kindr::minimal::QuatTransformation T_WorldRF_GateInitRF_Flightmare = T_WorldRF_RaceTrackRF_Flightmare * T_RaceTrackRF_GateInitRF;
        kindr::minimal::QuatTransformation T_WorldRF_GateInitRF_Gazebo = T_WorldRF_RaceTrackRF_Gazebo * T_RaceTrackRF_GateInitRF;

        m_UnityGateInitPoses[GateIdx].position = T_WorldRF_GateInitRF_Flightmare.getPosition().cast<float>();
        m_UnityGateInitPoses[GateIdx].orientation = T_WorldRF_GateInitRF_Flightmare.getEigenQuaternion().cast<float>();

        m_GazeboGateInitPoses[GateIdx].position = T_WorldRF_GateInitRF_Gazebo.getPosition().cast<float>();
        m_GazeboGateInitPoses[GateIdx].orientation = T_WorldRF_GateInitRF_Gazebo.getEigenQuaternion().cast<float>();
    }

    // Apply transformations to compute drone init pose in world reference frame
    geometry_msgs::Pose DroneInitPose_RaceTrackRF;
    DroneInitPose_RaceTrackRF.position = GMPoint__from__EV3d(GatesAndDroneInitPos_RaceTrackRF[GatesN]);
    DroneInitPose_RaceTrackRF.orientation = tf::createQuaternionMsgFromYaw(GatesAndDroneInitYaw_RaceTrackRF[GatesN]);
    
    kindr::minimal::QuatTransformation T_RaceTrackRF_DroneInitRF;
    tf::poseMsgToKindr(DroneInitPose_RaceTrackRF, &T_RaceTrackRF_DroneInitRF);
    kindr::minimal::QuatTransformation T_WorldRF_DroneInitRF_Flightmare = T_WorldRF_RaceTrackRF_Flightmare * T_RaceTrackRF_DroneInitRF;
    kindr::minimal::QuatTransformation T_WorldRF_DroneInitRF_Gazebo = T_WorldRF_RaceTrackRF_Gazebo * T_RaceTrackRF_DroneInitRF;

    m_UnityDroneInitPose.position = T_WorldRF_DroneInitRF_Flightmare.getPosition().cast<float>();
    m_UnityDroneInitPose.orientation = T_WorldRF_DroneInitRF_Flightmare.getEigenQuaternion().cast<float>();
    m_GazeboDroneInitPose.position = T_WorldRF_DroneInitRF_Gazebo.getPosition().cast<float>();
    m_GazeboDroneInitPose.orientation = T_WorldRF_DroneInitRF_Gazebo.getEigenQuaternion().cast<float>();
}
*/





void ForgetfulSimulator::plotTracks (
    const std::string& title,
    const std::vector<std::vector<Pose>>& vec_gp, 
    const std::vector<Pose>& vec_dp,
    const std::vector<std::string>& vec_label,
    const std::vector<std::string>& vec_color
) {
    // Parameters
    std::string dtxt = "Drone, start";
    auto gtxt = [] (const std::string& lbl) {return std::string("Gate, ") + lbl;};
    size_t fig_size_x = 1920, fig_size_y = 1080;
    double /*gsize = 100, */dsize = 200;
    float txtshft_norm = 50;
    const std::vector<std::string> vec_marker {"1", "2", "3", "4"};
    const float gate_width = 4.0;
    const float arrow_head_l {0.25F}, arrow_head_w {0.4F};

    
    matplotlibcpp::figure_size (fig_size_x, fig_size_y);
    //matplotlibcpp::title("Initial Position of gates and drone in x/y-Plane");
    if (title == "") const_cast<std::string&>(title) = "Racetracks in x/y-Plane";
    matplotlibcpp::title(title);

    std::string fname = "tracks_xy_plane";
    for (size_t i = 0; i < vec_gp.size(); i++) {

        const std::vector<Pose>& gp {vec_gp[i]};
        const Pose& dp {vec_dp[i]};
        const std::string& label {vec_label[i]};
        const std::string& color {vec_color[i]};
        //const std::string& marker {vec_marker[i]};

        // Prepare plot 
        std::vector<float> gx, gy, gz;
        float gx_min {1e10}, gx_max {-1e10}, gy_min {1e10}, gy_max {-1e10};
        for (size_t j = 0; j < gp.size(); j++) {
            gx.push_back (gp[j].position.x());
            gy.push_back (gp[j].position.y());
            gz.push_back (gp[j].position.z());

            gx_min = std::min(gx_min, gx[j]); gx_max = std::max(gx_max, gx[j]);
            gy_min = std::min(gy_min, gy[j]); gy_max = std::max(gy_max, gy[j]);
        }
        float txtshft_x = std::pow(-1, i) * (gx_max - gx_min) / txtshft_norm;
        float txtshft_y = - std::pow(-1, i) * (gy_max - gy_min) / txtshft_norm;
        std::vector<float> dx {dp.position.x()};
        std::vector<float> dy {dp.position.y()};


        // Scatter gates
        //matplotlibcpp::scatter(
        //    gx, gy, gsize, 
        //    {
        //        {"label", gtxt(label)},
        //        {"color", color},
        //        {"marker", marker},
        //    }
        //);

        for (size_t i = 0; i < gp.size(); i++) {
            const Pose& pose = gp[i];
            const float& xc = pose.position.x();
            const float& yc = pose.position.y();
            const float yaw = static_cast<float>(pose.yaw());

            const float x0 = xc + gate_width/2 * std::cos(yaw);
            const float x1 = xc - gate_width/2 * std::cos(yaw);
            const float y0 = yc + gate_width/2 * std::sin(yaw);
            const float y1 = yc - gate_width/2 * std::sin(yaw);
            
            std::map<std::string, std::string> kw;
            if (i == 0) 
                kw = {{"label", gtxt(label)}, {"color", color}};
            else 
                kw = {{"color", color}};
            
            matplotlibcpp::plot(
                std::vector<double>{x0, x1}, 
                std::vector<double>{y0, y1},
                kw
            );
        }
        

        
        
        // Mark gates
        for (size_t j = 0; j < gp.size(); j++) {
            matplotlibcpp::text(gx[j] + txtshft_x, gy[j] + txtshft_y, std::to_string(j));
        }

        // arrow-plot drone heading in grey
        Eigen::Vector2f dheading {
            std::cos(dp.yaw()),
            std::sin(dp.yaw())
        };
        matplotlibcpp::arrow(
            dp.position.x(),
            dp.position.y(),
            dheading.x(),
            dheading.y(),
            color, color, arrow_head_l, arrow_head_w
        );

        // Scatter drone
        matplotlibcpp::scatter(
            dx, dy, dsize,
            {
                //{"label", label},
                {"color", color}
            }
        );

        

        

        //// Mark drone
        //matplotlibcpp::text(dx[0] + txtshft_x, dy[0] + txtshft_y, dtxt);

        // extend filename
        fname += "_" + label;
        
        
        /*{
            const size_t& num_gates = gp.size();
            const size_t idx_front = num_gates - 1 - h_DRONE_INIT_POSE_NUM_GATES_SETBACK;
            const size_t idx_back = idx_front - 1;
            const Pose& pose_f = gp[idx_front], pose_b = gp[idx_back];
            const Eigen::Vector3f& pos_f = pose_f.position, pos_b = pose_b.position;
            const Eigen::Quaternionf& ori_f = pose_f.orientation, ori_b = pose_b.orientation;

            // Set drone orientation equal to orientation of front gate
            Eigen::Quaternionf d_ori = ori_f; 
            //Eigen::Quaternionf d_ori = slerp(ori_b, ori_f, h_DRONE_INIT_POSE_BACK2FRONT_SLIDER);

            // Set drone position between back and front gate
            Eigen::Vector3f d_pos = pos_b + (pos_f - pos_b) * h_DRONE_INIT_POSE_BACK2FRONT_SLIDER;
            // x/y-project drone position onto line traverssing front gate
            Eigen::Vector2f ldv { // line direction vector
                std::cos(pose_f.yaw() + M_PI/2), 
                std::sin(pose_f.yaw() + M_PI/2)
            };
            Eigen::Matrix2f opm // orthogonal projection matrix
                = ldv * ldv.transpose() / (ldv.transpose() * ldv);
            Eigen::Vector2f p0 { // arbitrary point on line
                pos_f.x(), 
                pos_f.y()
            };
            Eigen::Vector2f p { // point to project on line
                d_pos.x(), 
                d_pos.y()
            }; 
            Eigen::Vector2f d_pos_xy = opm * (p - p0) + p0;
            d_pos.x() = d_pos_xy.x();
            d_pos.y() = d_pos_xy.y();

            std::vector<double> line_x {p0.x(), p0.x() + 10 * ldv.x()};
            std::vector<double> line_y {p0.y(), p0.y() + 10 * ldv.y()};
            
            matplotlibcpp::plot(line_x, line_y, "y--");
            matplotlibcpp::scatter(
                std::vector<float> {d_pos.x()}, std::vector<float> {d_pos.y()}, dsize,
                {
                    //std::make_pair("label", label),
                    {"color", "yellow"}
                }
            );
        }*/
        
    }

    matplotlibcpp::scatter(
        std::vector<float> {}, std::vector<float> {}, dsize,
        {
            {"label", dtxt},
            {"color", "grey"}
        }
    );

    matplotlibcpp::axis("scaled");
    matplotlibcpp::legend();
    matplotlibcpp::save(ROS_PACKAGE_PATH + "/tmp/" + fname + ".pdf");
    matplotlibcpp::show (true);
};





/*
void ForgetfulSimulator::computeRaceTrack_Figure8(const forgetful_drones::BuildDroneRacingSimulation::Request::_RacetrackType_type& RacetrackType)
{
    // Data in the race track reference frame
        // Gate (center points) and drone start position:
        constexpr int GatesN = 14;
        std::array<Eigen::Vector3d, GatesN +1> GatesAndDroneInitPos_RaceTrackRF = {
            Eigen::Vector3d{ -20.45 ,   -8.65   ,   m_GateMinZ  }, // #01
            Eigen::Vector3d{ -12.55 ,   -11.15  ,   m_GateMinZ  }, // #02
            Eigen::Vector3d{ -4.15  ,   -5.35   ,   m_GateMinZ  }, // #03
            Eigen::Vector3d{ 3.45   ,   4.25    ,   m_GateMinZ  }, // #04
            Eigen::Vector3d{ 11.95  ,   11.15   ,   m_GateMinZ  }, // #05
            Eigen::Vector3d{ 21.85  ,   6.85    ,   m_GateMinZ  }, // #06
            Eigen::Vector3d{ 24.25  ,   -1.75   ,   m_GateMinZ  }, // #07
            Eigen::Vector3d{ 19.25  ,   -9.55   ,   m_GateMinZ  }, // #08
            Eigen::Vector3d{ 10.55  ,   -10.65  ,   m_GateMinZ  }, // #09
            Eigen::Vector3d{ 2.85   ,   -5.95   ,   m_GateMinZ  }, // #10
            Eigen::Vector3d{ -4.95  ,   4.65    ,   m_GateMinZ  }, // #11
            Eigen::Vector3d{ -12.95 ,   9.65    ,   m_GateMinZ  }, // #12
            Eigen::Vector3d{ -21.05 ,   6.65    ,   m_GateMinZ  }, // #13
            Eigen::Vector3d{ -24.25 ,   -2.15   ,   m_GateMinZ  }, // #14
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
    if (p_TEST_ENABLED)
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
    if (RacetrackType == forgetful_drones::BuildDroneRacingSimulation::Request::FIGURE8_RANDOMIZED)
    {
        std::uniform_real_distribution<double> URDistri_MinusToPlus1(-1.0, 1.0);
        std::uniform_real_distribution<double> URDistri_Scale(p_RAND_FIG8_SCALE_MIN, p_RAND_FIG8_SCALE_MAX);
        double Scale = URDistri_Scale(m_RandEngine);

        for (size_t GateIdx = 0; GateIdx < GatesN; GateIdx++)
        {
            Eigen::Vector3d AxialShift = p_RAND_FIG8_AXIAL_SHIFT_MAX * Eigen::Vector3d{ 
                    URDistri_MinusToPlus1(m_RandEngine), 
                    URDistri_MinusToPlus1(m_RandEngine), 
                    URDistri_MinusToPlus1(m_RandEngine) +1.0
                };
            GatesAndDroneInitPos_RaceTrackRF[GateIdx] += AxialShift;
            GatesAndDroneInitPos_RaceTrackRF[GateIdx] *= Scale;

            double YawTwist = p_RAND_FIG8_YAW_TWIST_MAX * URDistri_MinusToPlus1(m_RandEngine);
            GatesAndDroneInitYaw_RaceTrackRF[GateIdx] += YawTwist;

            if (GateIdx == GatesAndDroneInitPos_RaceTrackRF.size() -3) // drone starting position is next to second last gate
            {
                GatesAndDroneInitPos_RaceTrackRF.back() += AxialShift;
                GatesAndDroneInitPos_RaceTrackRF.back() *= Scale;
            }
        }
    }


    
    
    if (p_TEST_ENABLED)
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
    m_UnityGateInitPoses.resize(GatesN);
    m_GazeboGateInitPoses.resize(GatesN);

    // Create Transformation between reference frames: race track -> world 
    kindr::minimal::QuatTransformation T_WorldRF_RaceTrackRF_Flightmare;
    tf::poseMsgToKindr(m_UnityTrackPose.as_geometry_msg(), &T_WorldRF_RaceTrackRF_Flightmare);
    kindr::minimal::QuatTransformation T_WorldRF_RaceTrackRF_Gazebo;
    tf::poseMsgToKindr(m_GazeboTrackPose.as_geometry_msg(), &T_WorldRF_RaceTrackRF_Gazebo);


    // Apply transformation to compute gate poses in world reference frame
    for (size_t GateIdx = 0; GateIdx < GatesN; GateIdx++)
    {
        geometry_msgs::Pose GateInitPose_RaceTrackRF;
        GateInitPose_RaceTrackRF.position = GMPoint__from__EV3d(GatesAndDroneInitPos_RaceTrackRF[GateIdx]);
        GateInitPose_RaceTrackRF.orientation = tf::createQuaternionMsgFromYaw(GatesAndDroneInitYaw_RaceTrackRF[GateIdx]);
        
        kindr::minimal::QuatTransformation T_RaceTrackRF_GateInitRF;
        tf::poseMsgToKindr(GateInitPose_RaceTrackRF, &T_RaceTrackRF_GateInitRF);
        kindr::minimal::QuatTransformation T_WorldRF_GateInitRF_Flightmare = T_WorldRF_RaceTrackRF_Flightmare * T_RaceTrackRF_GateInitRF;
        kindr::minimal::QuatTransformation T_WorldRF_GateInitRF_Gazebo = T_WorldRF_RaceTrackRF_Gazebo * T_RaceTrackRF_GateInitRF;

        m_UnityGateInitPoses[GateIdx].position = T_WorldRF_GateInitRF_Flightmare.getPosition().cast<float>();
        m_UnityGateInitPoses[GateIdx].orientation = T_WorldRF_GateInitRF_Flightmare.getEigenQuaternion().cast<float>();

        m_GazeboGateInitPoses[GateIdx].position = T_WorldRF_GateInitRF_Gazebo.getPosition().cast<float>();
        m_GazeboGateInitPoses[GateIdx].orientation = T_WorldRF_GateInitRF_Gazebo.getEigenQuaternion().cast<float>();
    }

    // Apply transformations to compute drone init pose in world reference frame
    geometry_msgs::Pose DroneInitPose_RaceTrackRF;
    DroneInitPose_RaceTrackRF.position = GMPoint__from__EV3d(GatesAndDroneInitPos_RaceTrackRF[GatesN]);
    DroneInitPose_RaceTrackRF.orientation = tf::createQuaternionMsgFromYaw(GatesAndDroneInitYaw_RaceTrackRF[GatesN]);
    
    kindr::minimal::QuatTransformation T_RaceTrackRF_DroneInitRF;
    tf::poseMsgToKindr(DroneInitPose_RaceTrackRF, &T_RaceTrackRF_DroneInitRF);
    kindr::minimal::QuatTransformation T_WorldRF_DroneInitRF_Flightmare = T_WorldRF_RaceTrackRF_Flightmare * T_RaceTrackRF_DroneInitRF;
    kindr::minimal::QuatTransformation T_WorldRF_DroneInitRF_Gazebo = T_WorldRF_RaceTrackRF_Gazebo * T_RaceTrackRF_DroneInitRF;

    m_UnityDroneInitPose.position = T_WorldRF_DroneInitRF_Flightmare.getPosition().cast<float>();
    m_UnityDroneInitPose.orientation = T_WorldRF_DroneInitRF_Flightmare.getEigenQuaternion().cast<float>();
    m_GazeboDroneInitPose.position = T_WorldRF_DroneInitRF_Gazebo.getPosition().cast<float>();
    m_GazeboDroneInitPose.orientation = T_WorldRF_DroneInitRF_Gazebo.getEigenQuaternion().cast<float>();
}
*/

void ForgetfulSimulator::startSimulation_launchUnity () {
    //system("rosrun flightrender ForgetfulDrone_Flightmare.x86_64 &");
    //ros::Duration(3.0).sleep();
    m_UnityBridgePtr = flightlib::UnityBridge::getInstance();
    //ros::Duration(1.0).sleep();
}


void ForgetfulSimulator::startSimulation_addUnityGates() {
    ROSINFO("Unity: add gates");

    const size_t& num_gates = m_UnityGateInitPoses.size();
    int id_width = static_cast<int>(log10(num_gates) +1);
    std::string id_prefix = "unity_gate_";
    
    const std::array<std::string, 2> prefab_ids = {"tub_dai_gate", "thu_dme_gate"};
    const size_t& num_prefabs = prefab_ids.size();
    
    m_UnityGates.resize(num_gates * prefab_ids.size());

    for (size_t prefab_i = 0; prefab_i < num_prefabs; prefab_i++) {
        const std::string& prefab_id = prefab_ids[prefab_i];
        
        for (size_t gate_i = 0; gate_i < num_gates; gate_i++) {
            size_t unity_gate_i = prefab_i * num_gates + gate_i;
            
            // object_id
            std::stringstream id_suffix;
            id_suffix << std::setw(id_width) << std::setfill('0') << unity_gate_i;
            std::string object_id = id_prefix + id_suffix.str();

            // Init gates with object_id, prefab_id, position and orientation
            m_UnityGates[unity_gate_i] = std::make_shared<flightlib::StaticObject>(object_id, prefab_id);
            m_UnityGates[unity_gate_i]->setPosition(m_UnityGateInitPoses[gate_i].position);
            m_UnityGates[unity_gate_i]->setQuaternion(m_UnityGateInitPoses[gate_i].orientation);

            // Add gates
            m_UnityBridgePtr->addStaticObject(m_UnityGates[unity_gate_i]);
            
            //ROSINFO("Unity: Added ["
            //    << m_UnityGates[unity_gate_i]->getID() << ", "
            //    << m_UnityGates[unity_gate_i]->getPrefabID() << "]");
        }
    
    }
    
    
    
}


bool ForgetfulSimulator::startSimulation_configUnityGates()
{
    ROSINFO("Unity: configure gates");

    const size_t& gate_n = m_UnityGateInitPoses.size();
    int object_id_width = static_cast<int>(log10(gate_n) +1);
    std::string object_id_prefix = "unity_gate_";
    
    const std::array<std::string, 2> prefab_ids = {"tub_dai_gate", "thu_dme_gate"};
    

    std::array<Eigen::Vector3f, 2> prefab_offsets {
        Eigen::Vector3f {0.0, 0.0, -1000.0}, 
        Eigen::Vector3f {0.0, 0.0, -1000.0}
    };
    
    const std::string& gate_prefab_id = std::get<2>(h_GATE_TYPES[m_GateTypeIdx]);
    if (gate_prefab_id == "tub_dai_gate") {
        prefab_offsets[0] = Eigen::Vector3f {0.0, 0.0, 0.0};
    } else if (gate_prefab_id == "thu_dme_gate") {
        prefab_offsets[1] = Eigen::Vector3f{0.0, 0.0, 0.0};
    } else {
        ROSERROR ("No implementation for " << GET_VAR_REP (gate_prefab_id));
        return false;
    }
    

    for (size_t prefab_i = 0; prefab_i < prefab_ids.size(); prefab_i++) {
        const std::string& prefab_id = prefab_ids[prefab_i];
        
        for (size_t gate_i = 0; gate_i < gate_n; gate_i++)
        {
            size_t unity_gate_i = prefab_i * gate_n + gate_i;
            // object_id
            std::stringstream ss; ss << std::setw(object_id_width) << std::setfill('0') << unity_gate_i;
            std::string object_id = object_id_prefix + ss.str();

            // Init gates with object_id, prefab_id, position and orientation
            m_UnityGates[unity_gate_i] = std::make_shared<flightlib::StaticObject>(object_id, prefab_id);
            m_UnityGates[unity_gate_i]->setPosition(
                m_UnityGateInitPoses[gate_i].position + prefab_offsets[prefab_i]);
            m_UnityGates[unity_gate_i]->setQuaternion(
                m_UnityGateInitPoses[gate_i].orientation);

            // Add gates
            m_UnityBridgePtr->configureStaticObject(unity_gate_i, m_UnityGates[unity_gate_i]);
            
            //ROSDEBUG("Unity: Configured ["
            //    << m_UnityGates[unity_gate_i]->getID() << ", "
            //    << m_UnityGates[unity_gate_i]->getPrefabID() << "]");
        }
    }

    return true;
}





void ForgetfulSimulator::startSimulation_spawnGazeboGates() {
    ROSINFO("Gazebo: spawn gates");

    const size_t& num_gates = m_GazeboGateInitPoses.size();
    int id_width = static_cast<int>(log10(num_gates) +1);
    std::string id_prefix = "gazebo_gate_";

    gazebo_msgs::SpawnModel srv;
        srv.request.reference_frame = "world";
        srv.request.robot_namespace = "race_track";
    
    for (size_t gate_i = 0; gate_i < num_gates; gate_i++) {
        // model_name
        std::stringstream id_suffix; id_suffix
            << std::setw(id_width) 
            << std::setfill('0') 
            << gate_i;
        srv.request.model_name = id_prefix + id_suffix.str();
        // model_xml
        std::ifstream ifs((ROS_PACKAGE_PATH + std::get<3>(h_GATE_TYPES[m_GateTypeIdx])) + "/model.sdf");
        std::stringstream ss; ss << ifs.rdbuf();
        srv.request.model_xml = ss.str();
        // initial_pose
        srv.request.initial_pose = m_GazeboGateInitPoses[gate_i].as_geometry_msg();
        // Call service
        spawnGazeboModel(srv, "sdf");
    }
}


void ForgetfulSimulator::startSimulation_spawnRVizGates()
{
    ROSINFO("RViz: spawn gates");
    
    visualization_msgs::MarkerArray msg;

    visualization_msgs::Marker marker;
    marker.header.frame_id = "world";
    marker.scale.x = marker.scale.y = marker.scale.z = 1.0;
    marker.action = visualization_msgs::Marker::ADD;
    marker.type = visualization_msgs::Marker::MESH_RESOURCE;
    marker.mesh_use_embedded_materials = true;

    for (size_t Gate_i = 0; Gate_i < m_GazeboGateInitPoses.size(); Gate_i++)
    {   
        marker.id = Gate_i;
        marker.pose = m_GazeboGateInitPoses[Gate_i].as_geometry_msg();
        marker.mesh_resource = std::string("file://") + (ROS_PACKAGE_PATH + std::get<3>(h_GATE_TYPES[m_GateTypeIdx])) + "/model.stl";
        msg.markers.push_back(marker);
    }

    m_rosPUB_RVIZ_GATES.publish(msg);
}











void ForgetfulSimulator::startSimulation_spawnGazeboDrone () {
    ROSINFO("Gazebo: spawn drone");

    gazebo_msgs::SpawnModel srv;
    srv.request.model_name = h_DRONE_MODEL_NAME;
    srv.request.model_xml = p_DRONE_MODEL_DESCRIPTION;
    srv.request.reference_frame = "world";
    srv.request.initial_pose = m_GazeboDroneInitPose.as_geometry_msg();
    
    spawnGazeboModel(srv, "urdf");
}

void ForgetfulSimulator::startSimulation_addUnityDrone() {
    ROSINFO("Unity: add drone with RGB camera");

    //ROSDEBUG(GET_VAR_REP(sizeof(flightlib::Quadrotor)));
    // Init drone
    //ROSDEBUG("Init Unity drone.");
    m_UnityDronePtr = std::make_shared<flightlib::Quadrotor>();
    
    //ROSDEBUG(GET_VAR_REP(sizeof(flightlib::RGBCamera)));
    // Init and add mono camera
    //ROSDEBUG("Init Unity RGB camera.");
    m_UnityRGBCamPtr = std::make_shared<flightlib::RGBCamera>();
    flightlib::Vector<3> B_r_BC{0.0, 0.0, 0.3};
    constexpr double FMDroneCam_YawOffset = -1*M_PI/2;
    tf::Quaternion FMDroneCam_QuaternionOffset = tf::createQuaternionFromYaw(FMDroneCam_YawOffset);

    flightlib::Matrix<3, 3> R_BC = flightlib::Quaternion{
        static_cast<float>(FMDroneCam_QuaternionOffset.w()), 
        static_cast<float>(FMDroneCam_QuaternionOffset.x()), 
        static_cast<float>(FMDroneCam_QuaternionOffset.y()), 
        static_cast<float>(FMDroneCam_QuaternionOffset.z())}.toRotationMatrix();
    m_UnityRGBCamPtr->setRelPose(B_r_BC, R_BC);
    m_UnityRGBCamPtr->setFOV(p_DRONE_CAM_FOV);
    m_UnityRGBCamPtr->setWidth(p_DRONE_CAM_WIDTH);
    m_UnityRGBCamPtr->setHeight(p_DRONE_CAM_HEIGHT);
    m_UnityRGBCamPtr->setPostProcesscing(std::vector<bool>{false, false, false});  // depth, segmentation, optical flow
    //ROSDEBUG("Add Unity RGB camera.");
    m_UnityDronePtr->addRGBCamera(m_UnityRGBCamPtr);

    // Init drone state and set to initial pose
    m_UnityDroneState.setZero();
    m_UnityDronePtr->reset(m_UnityDroneState);   
    m_UnityDroneState.x[flightlib::QS::POSX] = m_UnityDroneInitPose.position.x();
    m_UnityDroneState.x[flightlib::QS::POSY] = m_UnityDroneInitPose.position.y();
    m_UnityDroneState.x[flightlib::QS::POSZ] = m_UnityDroneInitPose.position.z();
    m_UnityDroneState.x[flightlib::QS::ATTW] = m_UnityDroneInitPose.orientation.w();
    m_UnityDroneState.x[flightlib::QS::ATTX] = m_UnityDroneInitPose.orientation.x();
    m_UnityDroneState.x[flightlib::QS::ATTY] = m_UnityDroneInitPose.orientation.y();
    m_UnityDroneState.x[flightlib::QS::ATTZ] = m_UnityDroneInitPose.orientation.z();
    m_UnityDronePtr->setState(m_UnityDroneState);

    // Add quadcopter
    //ROSDEBUG("Add Unity drone.");
    m_UnityBridgePtr->addQuadrotor(m_UnityDronePtr);
}



//void ForgetfulSimulator::ROS_CB_GazeboModelStates(const gazebo_msgs::ModelStates::ConstPtr &msg) 
//{ 
//    m_GazeboModelStates_Ptr = msg;
//}


void ForgetfulSimulator::startSimulation_deleteGazeboModelsExceptDrone () {
    ROSINFO("Gazebo: delete all models except drone");

    const double d = 2.0;
    
    gazebo_msgs::ModelStates::ConstPtr msg 
        = ros::topic::waitForMessage<gazebo_msgs::ModelStates>(
            "/gazebo/model_states", ros::Duration(d));

    if (msg == nullptr) {
        ROSERROR("Gazebo: failed to receive model states within " << d << " s");
        return;
    } else {
        gazebo_msgs::DeleteModel srv;
        for (const std::string& model_name : msg->name) {
            if (model_name == h_DRONE_MODEL_NAME) continue;
            
            srv.request.model_name = model_name;
            if (m_rosSVC_GAZEBO_DELETE_MODEL.call(srv)) {
                //ROSINFO("Gazebo: deleted \"" << model_name << "\"");
            } else {
                ROSERROR("Gazebo: failed to delete \"" << model_name << "\"");
            }
        }
    }
}


void ForgetfulSimulator::_spawnGazeboModel (
    gazebo_msgs::SpawnModel& srv,
    ros::ServiceClient& srv_cl
) {
    if (srv_cl.call(srv)) {
            //ROSINFO("Gazebo: spawned \"" << srv.request.model_name << "\"");
    } else {
        ROSERROR("Gazebo: failed to spawn \"" << srv.request.model_name << "\"");
    }
}


void ForgetfulSimulator::spawnGazeboModel (
    gazebo_msgs::SpawnModel& srv,
    const std::string& model_type
) {
    if (model_type == "sdf") {
        _spawnGazeboModel(srv, m_rosSVC_GAZEBO_SPAWN_SDF_MODEL);
    } else if (model_type == "urdf") {
        _spawnGazeboModel(srv, m_rosSVC_GAZEBO_SPAWN_URDF_MODEL);
    } else {
        ROSERROR(GET_VAR_REP(model_type) << " must be \"sdf\" or \"urdf\".");
    }
}


void ForgetfulSimulator::startSimulation_spawnGazeboGroundPlane () { 
    ROSINFO("Gazebo: spawn ground plane");

    const std::string ground_plane_id {"ground_plane"};

    gazebo_msgs::SpawnModel srv;
    srv.request.reference_frame = "world";
    srv.request.robot_namespace = "environment";
    srv.request.model_name = ground_plane_id;
    srv.request.model_xml = getGroundPlaneSDF(ground_plane_id, m_GazeboGroundPlaneSize);
    srv.request.initial_pose.position = GMPoint__from__EV3d(m_GazeboGroundPlaneOrigin.cast<double>());

    spawnGazeboModel(srv, "sdf");
}



std::string ForgetfulSimulator::getGroundPlaneSDF (
    const std::string& id,
    const Eigen::Vector2f& size
) {
    return 
    R"(<?xml version="1.0"?>
    <sdf version="1.6">
        <model name='Ground_Plane_)" + id + R"('>
            <pose frame='world'>0 0 0 0 0 0</pose>
            <static>1</static>
            <link name='link'>
                <collision name='collision'>
                    <geometry>
                        <plane>
                            <normal>0 0 1</normal>
                            <size>)" 
                                + std::to_string(size.x()) 
                                + " " 
                                + std::to_string(size.y()) 
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
                                + std::to_string(size.x()) 
                                + " " 
                                + std::to_string(size.y())
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




void ForgetfulSimulator::startSimulation_spawnRVizDrone()
{
    ROSDEBUG_FUNCTION_ENTERED();

    visualization_msgs::MarkerArray RVizDrone;
    RVizDrone.markers.reserve(2 * p_RVIZ_DRONE_ROTOR_N + 1);

    visualization_msgs::Marker Body;
        Body.header.stamp = ros::Time();
        Body.header.frame_id = h_DRONE_FRAME_ID;
        Body.ns = "vehicle_body";
        Body.action = visualization_msgs::Marker::ADD;
        Body.type = visualization_msgs::Marker::CUBE;
        Body.scale.x = p_RVIZ_DRONE_BODY_WIDTH * p_RVIZ_DRONE_SCALE;
        Body.scale.y = p_RVIZ_DRONE_BODY_WIDTH * p_RVIZ_DRONE_SCALE;
        Body.scale.z = p_RVIZ_DRONE_BODY_HEIGHT * p_RVIZ_DRONE_SCALE;
        setRGBOfVisMarker(VisualizationColors::BLACK, Body);
        Body.color.a = 1.0;
        Body.frame_locked = true;
    RVizDrone.markers.push_back(Body);

    visualization_msgs::Marker Rotor;
        Rotor.header.stamp = ros::Time();
        Rotor.header.frame_id = h_DRONE_FRAME_ID;
        Rotor.ns = "vehicle_rotor";
        Rotor.action = visualization_msgs::Marker::ADD;
        Rotor.type = visualization_msgs::Marker::CYLINDER;
        Rotor.scale.x = 0.2 * p_RVIZ_DRONE_SCALE;
        Rotor.scale.y = 0.2 * p_RVIZ_DRONE_SCALE;
        Rotor.scale.z = 0.01 * p_RVIZ_DRONE_SCALE;
        setRGBOfVisMarker(VisualizationColors::BLUE, Rotor);
        Rotor.color.a = 0.7;
        Rotor.pose.position.z = 0;
        Rotor.frame_locked = true;
    
    visualization_msgs::Marker Arm;
        Arm.header.stamp = ros::Time();
        Arm.header.frame_id = h_DRONE_FRAME_ID;
        Arm.ns = "vehicle_arm";
        Arm.action = visualization_msgs::Marker::ADD;
        Arm.type = visualization_msgs::Marker::CUBE;
        Arm.scale.x = p_RVIZ_DRONE_ARM_LENGTH * p_RVIZ_DRONE_SCALE;
        Arm.scale.y = 0.02 * p_RVIZ_DRONE_SCALE;
        Arm.scale.z = 0.01 * p_RVIZ_DRONE_SCALE;
        setRGBOfVisMarker(VisualizationColors::BLACK, Arm);
        Arm.color.a = 1.0;
        Arm.pose.position.z = -0.015 * p_RVIZ_DRONE_SCALE;
        Arm.frame_locked = true;

        const double RotorAngleIncrement = 2*M_PI / p_RVIZ_DRONE_ROTOR_N;
        for (double Angle = RotorAngleIncrement /2; Angle <= 2*M_PI; Angle += RotorAngleIncrement) 
        {
            Rotor.pose.position.x = p_RVIZ_DRONE_ARM_LENGTH * cos(Angle) * p_RVIZ_DRONE_SCALE;
            Rotor.pose.position.y = p_RVIZ_DRONE_ARM_LENGTH * sin(Angle) * p_RVIZ_DRONE_SCALE;
            Rotor.id++;

            Arm.pose.position.x = Rotor.pose.position.x /2;
            Arm.pose.position.y = Rotor.pose.position.y /2;
            Arm.pose.orientation = tf::createQuaternionMsgFromYaw(Angle);
            Arm.id++;

            RVizDrone.markers.push_back(Rotor);
            RVizDrone.markers.push_back(Arm);
        }

    m_rosPUB_RVIZ_DRONE.publish(RVizDrone);
}






void ForgetfulSimulator::buildSimulation_generateGazeboGroundPlane () {
    const float& buffer = h_GAZEBO_GROUND_PLANE_BUFFER;

    float x_min = m_GazeboDroneInitPose.position.x(), x_max = x_min;
    float y_min = m_GazeboDroneInitPose.position.y(), y_max = y_min;

    for (size_t i = 0; i < m_GazeboGateInitPoses.size(); i++) {
        x_min = std::min(x_min, m_GazeboGateInitPoses[i].position.x());
        x_max = std::max(x_max, m_GazeboGateInitPoses[i].position.x());
        y_min = std::min(y_min, m_GazeboGateInitPoses[i].position.y());
        y_max = std::max(y_max, m_GazeboGateInitPoses[i].position.y());
    }

    x_min -= buffer; x_max += buffer;
    y_min -= buffer; y_max += buffer;

    const float origin_x = (x_min + x_max) / 2;
    const float origin_y = (y_min + y_max) / 2;
    const float origin_z = m_GazeboTrackPose.position.z();
    const float size_x = x_max - x_min;
    const float size_y = y_max - y_min;

    m_GazeboGroundPlaneOrigin = {origin_x, origin_y, origin_z};
    m_GazeboGroundPlaneSize = {size_x, size_y};

    //ROSDEBUG("Gazebo ground plane, origin: [" << std::setprecision(3)
    //    << origin_x << ", " << origin_y << ", " << origin_z << "], size: ["
    //    << size_x << ", " << size_y << "]");
}







void ForgetfulSimulator::resetRVIZ () {
    auto logError = [] () {ROSERROR("Failed to reset RViz.");};

    std::string RVIZ_CONFIG_FILE;
    if (fetchROSParameter<std::string>(m_rosPNH, "SIM_RVIZ_CONFIG_FPATH", RVIZ_CONFIG_FILE)) {
        rviz::SendFilePath srv;
        srv.request.path.data = ROS_PACKAGE_PATH + RVIZ_CONFIG_FILE;

        if (!m_rosSVC_RVIZ_LOAD_CONFIG.call(srv))
            logError();
    }
    else
        logError();
}



//void ForgetfulSimulator::computeRaceTrack_Fig8Rand()
//{
//    computeRaceTrack_Figure8();
//
//    std::uniform_real_distribution<float> URDistri_MinusToPlus1(-1.0, 1.0);
//    std::uniform_real_distribution<float> URDistri_Scale(p_RAND_FIG8_SCALE_MIN, p_RAND_FIG8_SCALE_MAX);
//    float Scale = URDistri_Scale(m_RandEngine);
//
//    for (size_t GateIdx = 0; GateIdx < m_UnityGateInitPoses.size(); GateIdx++)
//    {
//        Eigen::Vector3f AxialShift 
//            = p_RAND_FIG8_AXIAL_SHIFT_MAX * Eigen::Vector3f{ 
//            URDistri_MinusToPlus1(m_RandEngine), 
//            URDistri_MinusToPlus1(m_RandEngine), 
//            URDistri_MinusToPlus1(m_RandEngine) +1.0
//            };
//
//        m_UnityGateInitPoses[GateIdx].position += AxialShift;
//        m_UnityGateInitPoses[GateIdx].position *= Scale;
//
//        if (GateIdx == m_UnityGateInitPoses.size() -2) // drone starting position is next to second last gate
//        {
//            m_UnityDroneInitPose.position += AxialShift;
//            m_UnityDroneInitPose.position *= Scale;
//        }
//    }
//}























































































/*










/----------- Dynamic Gates



void ForgetfulSimulator::ROSCB_DynamicGatesSwitch( const std_msgs::Bool::ConstPtr& msg )
{
    MY_ROS_INFO( 
        "[%s]\n  >> Dynamic gates %s.", 
        m_ROS_NODE_NAME,
        msg->data? "enabled" : "disabled"
        );

    msg->data? m_ROSTimer_SimulatorLoop.start() : m_ROSTimer_SimulatorLoop.stop();
}


void ForgetfulSimulator::ROSTimerFunc_SimulatorLoop( const ros::TimerEvent& TimerEvent )
{   
    if ( TimerEvent.profile.last_duration.toSec() > m_SimulatorLoopTime )
        ROS_WARN(
            "[%s]\n  >> Last loop iteration took %f s exceeding nominal duration of %f s.",
            m_ROS_NODE_NAME, 
            TimerEvent.profile.last_duration.toSec(),
            m_SimulatorLoopTime
            );


    
    if ( ! m_SimulationRunning ) return;


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

        ModelState.pose.position = GMP_From_EV3d(
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

    m_rosPUB_RVIZ_GATES.publish( m_RVizGates );
    
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
            = p_DYN_GATES_AXIAL_AMP_MAX
            * Eigen::Vector3d{
                m_UniRealDistri_0To1( m_RandEngine ),
                m_UniRealDistri_0To1( m_RandEngine ),
                m_UniRealDistri_0To1( m_RandEngine )
                };
        
        
        m_DynGates_AxFreqs[ GateIdx ] 
            = p_DYN_GATES_AXIAL_FREQ_MAX
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
/// \param p_RAND_SPRINT_GATE_N Number of gates if not == 0.
/// \param p_RAND_SPRINT_GATE_MIN_N If ´p_RAND_SPRINT_GATE_N´ == 0, lower limit for random number of gates.
/// \param p_RAND_SPRINT_GATE_MAX_N If ´p_RAND_SPRINT_GATE_N´ == 0, upper limit for random number of gates.
/// \param p_RAND_SPRINT_SECTION_GATE_MIN_N Lower limit for random number of gates in race section.
/// \param p_RAND_SPRINT_SECTION_GATE_MAX_N Upper limit for random number of gates in race section.
/// \param m_GateBaseMinAltitude Lower limit for altitude of gates.
/// \param p_RAND_SPRINT_SPACING_MIN Lower limit for spacing between gates.
/// \param p_RAND_SPRINT_SPACING_UNI_DIRECT_STD_DEV Standard deviation of values 
/// whose absolute value is added to ´p_RAND_SPRINT_SPACING_MIN´ to get random spacing between gates.
/// \param p_RAND_SPRINT_LR_YAW_MEAN +/- mean of yaw between gates in race section where +/- is randomly chosen.
/// \param p_RAND_SPRINT_LR_YAW_STD_DEV Standard deviation of yaw between gates in race section.
/// \param p_RAND_SPRINT_PITCH_MEAN Mean of pitch between gates in race section.
/// \param p_RAND_SPRINT_PITCH_STD_DEV Standard deviation of pitch between gates in race section.
/// \return True if position and yaw of gates successfully computed, false otherwise.
///////////////////////////////////////////////////////////////////////////////////////////////////////
void ForgetfulSimulator::computeGatePoses_SprintRand()
{
    MY_ROS_INFO( "[%s] Computing position and yaw of gates for randomized sprint race...", 
        m_ROS_NODE_NAME );

    // If number of gates was not specified, i.e., `p_RAND_SPRINT_GATE_N` = 0, //
    // set to random value in [ `p_RAND_SPRINT_GATE_MIN_N`, p_RAND_SPRINT_GATE_MAX_N ]    //
    /////////////////////////////////////////////////////////////////
        unsigned int Gates_N;
        if ( p_RAND_SPRINT_GATE_N == 0)
        {
            // Lambda function to get random number of gates for sprint race.
            std::uniform_int_distribution<unsigned int> UnifIntDistri_Gates_N( p_RAND_SPRINT_GATE_MIN_N, p_RAND_SPRINT_GATE_MAX_N );
            Gates_N = UnifIntDistri_Gates_N( m_RandEngine );
        }
        else Gates_N = p_RAND_SPRINT_GATE_N;
    

    // Lambda functions to get random values for the design of the race track //
    ////////////////////////////////////////////////////////////////////////////
    std::uniform_int_distribution< unsigned int > UnifIntDistri_01{ 0, 1 };
    std::uniform_int_distribution< unsigned int > UnifIntDistri_Section_N{ 
        static_cast<unsigned int>( p_RAND_SPRINT_SECTION_GATE_MIN_N ), 
        static_cast<unsigned int>( p_RAND_SPRINT_SECTION_GATE_MAX_N ) 
        };
    std::normal_distribution< double > NormalDistri_Spacing{ 0, p_RAND_SPRINT_SPACING_UNI_DIRECT_STD_DEV };
    std::normal_distribution< double > NormalDistri_LeftYaw{ p_RAND_SPRINT_LR_YAW_MEAN, p_RAND_SPRINT_LR_YAW_STD_DEV };
    std::normal_distribution< double > NormalDistri_RightYaw{ -p_RAND_SPRINT_LR_YAW_MEAN, p_RAND_SPRINT_LR_YAW_STD_DEV };
    std::normal_distribution< double > NormalDistri_Pitch{ p_RAND_SPRINT_PITCH_MEAN, p_RAND_SPRINT_PITCH_STD_DEV };
    
    auto getRand_Bool = [ & ]() { 
        return static_cast<bool>( UnifIntDistri_01( m_RandEngine ) ); };
    auto getRand_Section_N = [ & ]() { 
        return UnifIntDistri_Section_N( m_RandEngine ); };
    auto getRand_Spacing = [ & ]() { 
        return p_RAND_SPRINT_SPACING_MIN + abs( NormalDistri_Spacing( m_RandEngine ) ); };
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
        m_ROS_NODE_NAME, Gates_N, 
        m_GatesInitPos_WorldRF[ 0 ].x(), m_GatesInitPos_WorldRF[ 0 ].y(), m_GatesInitPos_WorldRF[ 0 ].z(),
        m_GatesInitPos_WorldRF[ m_GatesInitPos_WorldRF.size() - 1 ].x(), m_GatesInitPos_WorldRF[ m_GatesInitPos_WorldRF.size() - 1 ].y(), m_GatesInitPos_WorldRF[ m_GatesInitPos_WorldRF.size() - 1 ].z()
        );
}

*/










}



