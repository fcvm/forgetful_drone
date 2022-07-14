#include "forgetful_drones/forgetful_drone.hpp"
#include "forgetful_drones/forgetful_helpers.hpp"
#include "forgetful_drones/forgetful_global_trajectory.hpp"
#include "forgetful_drones/forgetful_simulator.hpp"








namespace forgetful_drone {

ForgetfulDrone::ForgetfulDrone(const ros::NodeHandle& rnh, const ros::NodeHandle& pnh)
    :
    m_rosRNH {rnh},
    m_rosPNH {pnh},
    
    m_rosSUB_GROUND_TRUTH_ODOMETRY {m_rosRNH.subscribe("ground_truth/odometry", 1, &ForgetfulDrone::ROSCB_GROUND_TRUTH_ODOMETRY, this)},
    m_rosSUB_GROUND_TRUTH_IMU {m_rosRNH.subscribe("ground_truth/imu", 1, &ForgetfulDrone::ROSCB_GROUND_TRUTH_IMU, this)},
    m_rosSUB_AUTOPILOT_FEEDBACK {m_rosRNH.subscribe("autopilot/feedback", 1, &ForgetfulDrone::ROSCB_AUTOPILOT_FEEDBACK, this)},
    m_rosSUB_CONTROL_COMMAND {m_rosRNH.subscribe("control_command", 1, &ForgetfulDrone::ROSCB_CONTROL_COMMAND, this)},
    
    m_rosSUB_FLIGHTMARE_RGB {m_rosRNH.subscribe("/flightmare/rgb", 1, &ForgetfulDrone::ROSCB_FLIGHTMARE_RGB, this)},
    
    m_rosPUB_RVIZ_NAVIGATION_POINTS {m_rosRNH.advertise<visualization_msgs::Marker>("rviz/navigation_points", 0)},
    m_rosPUB_RVIZ_RGB_LABELED {m_rosRNH.advertise<sensor_msgs::Image>("rviz/rgb/labeled", 1)},
    m_rosPUB_RVIZ_GLOBAL_TRAJECTORY {m_rosRNH.advertise<visualization_msgs::Marker>("rviz/global_trajectory", 1)},
    //m_ROSPUB_SIMULATOR_DYNAMIC_GATES_SWITCH {m_rosRNH.advertise<std_msgs::Bool>("simulator/dynamic_gates_switch", 1))} 
    m_rosPUB_AUTOPILOT_OFF {m_rosRNH.advertise<std_msgs::Empty>("autopilot/off", 1)},
    m_rosPUB_AUTOPILOT_START {m_rosRNH.advertise<std_msgs::Empty>("autopilot/start", 1)},
    m_rosPUB_AUTOPILOT_LAND {m_rosRNH.advertise<std_msgs::Empty>("autopilot/land", 1) },
    m_rosPUB_BRIDGE_ARM {m_rosRNH.advertise<std_msgs::Bool>("bridge/arm", 1)},
    //m_ROSPUB_NAVIGATOR_STATE_SWITCH {m_rosRNH.advertise<std_msgs::Bool>("navigator/state_switch", 1)},
    m_rosPUB_AUTOPILOT_REFERENCE_STATE {m_rosRNH.advertise<quadrotor_msgs::TrajectoryPoint>("autopilot/reference_state", 1)},
    m_rosPUB_RVIZ_LOCAL_TRAJECTORY {m_rosRNH.advertise<visualization_msgs::Marker>("rviz/local_trajectory", 1)},
    m_rosPUB_AUTOPILOT_POSE_COMMAND {m_rosRNH.advertise<geometry_msgs::PoseStamped>("autopilot/pose_command", 1)},


    m_rosSVC_BRAIN_TRAIN_ANN {m_rosRNH.serviceClient<std_srvs::Empty/*forgetful_drones::TrainBrain*/>("brain/train_ann")},
    m_rosSVC_SIMULATOR_BUILD {m_rosRNH.serviceClient<fdBS>("simulator/build")},
    m_rosSVC_SIMULATOR_START {m_rosRNH.serviceClient<fdStartS>("simulator/start", 1)},
    m_rosSVC_SIMULATOR_STOP {m_rosRNH.serviceClient<fdStopS>("simulator/stop", 1)},
    //m_rosSVC_RVIZ_LOAD_CONFIG {m_rosRNH.serviceClient<rviz::SendFilePath>("rviz/load_config")},



    m_AutopilotState {quadrotor_msgs::AutopilotFeedback::OFF},
    m_ExpertOutput {0.0, 0.0, 0.0},
    m_BrainOutput {0.0, 0.0, 0.0},
    m_NavigatorInput {0.0, 0.0, 0.0},
    m_GloTraj {},
    m_TrackWaypoints {},
    m_GloTrajExpertStateIdx {0},
    m_Expert_MaxSpeed {std::numeric_limits<double>::min()},
    m_Expert_MinSpeed {std::numeric_limits<double>::max()},
    m_T_WRF_DRF {},
    m_T_ARF_DRF {},
    m_RefStatePos_WRF {},
    m_CurrWaypointIdx {0},
    m_LastWaypointIdx {0},
    m_Dist2CurrWaypoint {0.0},
    m_Dist2LastWaypoint {0.0},
    

    m_NavigatorENABLED {false},
    m_DataSavingENABLED {false},
    m_LocTrajSubseqInfeasibleCnt {0},
    m_LocTrajStartTime {},
    m_LocTraj {Vec3(), Vec3(), Vec3(), Vec3()},
    m_MainLoopIterCnt {0},
    m_LocTrajFeasibleCnt {0},
    m_TotalRunCnt {0},
    m_RGBCnt {0},
    m_RunLapCnt {-1}
{
    bool init_successful {true};
        if (!initROSParameters()) init_successful = false;
        if (!initExperimentID()) init_successful = false;
        if (!initTrafoArf2Wrf()) init_successful = false;
        if (!initBrain()) init_successful = false;
    if (!init_successful) {
        ROSERROR("Initialization failed. Shut ROS down.");
        ros::shutdown();
    }
    
    constexpr double wt {3.0};
    ROSINFO("Initialized node");
    ros::Duration(wt).sleep();

    if (p_TEST_ENABLED) test();
    else {
        switch (p_FLIGHTMISSION) {
            case 0: performFlightMission_GlobalTrajectoryTracking(); break;
            case 1: performFlightMission_NavigationByExpert(); break;
            case 2: performFlightMission_NavigationByBrain(); break;
            case 3: performFlightMission_TrainingDataGeneration(); break;
            case 4: performFlightMission_DAGGER(); break;
            default: 
                ROSERROR("No implementation for " << GET_VAR_REP(p_FLIGHTMISSION)); 
                ros::shutdown(); break;
        }
    }
}



ForgetfulDrone::~ForgetfulDrone () {}





std::string ForgetfulDrone::ExperimentDpath () {
    return ROS_PACKAGE_PATH + "/" + std::string(h_EXPERIMENTS_DNAME) + "/" + p_EXPERIMENT_ID;
}
std::string ForgetfulDrone::ConfigDpath () {
    return ExperimentDpath() + "/" + std::string(h_CONFIG_DNAME);
}
std::string ForgetfulDrone::OutputDpath () {
    return ExperimentDpath() + "/" + std::string(h_OUTPUT_DNAME);
}
std::string ForgetfulDrone::DataDpath () {
    return ExperimentDpath() + "/" + std::string(h_DATA_DNAME);
}
std::string ForgetfulDrone::RawDpath () {
    return DataDpath() + "/" + std::string(h_RAW_DNAME);
}

std::string ForgetfulDrone::ConfigFpath () {
    return OutputDpath() + "/" + std::string(h_CONFIG_FNAME);
}
std::string ForgetfulDrone::ScriptModuleFpath () {
    return OutputDpath() + "/" + std::string(h_SCRIPTMODULE_FNAME);
}
std::string ForgetfulDrone::ParametersFpath (const bool& src_not_dst) {
    if (src_not_dst) return ROS_PACKAGE_PATH + "/" + std::string(h_PARAMETERS_DNAME) + "/" + std::string(h_PARAMETERS_FNAME);
    else return ConfigDpath() + "/" + std::string(h_PARAMETERS_FNAME);
}



std::string ForgetfulDrone::RunID () {
    std::ostringstream oss_tr; oss_tr << std::setw(4) << std::setfill('0') << m_TotalRunCnt;
    std::ostringstream oss_dr; oss_dr << std::setw(3) << std::setfill('0') << m_DaggerRepCnt;
    std::ostringstream oss_ms; oss_ms << std::setfill('0') << std::setw(5) << std::fixed << std::setprecision(2) << m_LocTrajMaxSpeed;
    std::ostringstream oss_dm; oss_dm << std::setfill('0') << std::setw(5) << std::fixed << std::setprecision(2)  << m_DaggerMargin;

    std::ostringstream oss; oss
        << oss_tr.str() << "___"
        << std::to_string(m_UnitySceneIdx) << '_'
        << std::to_string(m_SceneSiteIdx) << '_'
        << std::to_string(m_TrackTypeIdx) << '_'
        << std::to_string(m_TrackGenerationIdx) << '_'
        << std::to_string(m_TrackDirectionIdx) << '_'
        << std::to_string(m_GateTypeIdx) << '_'
        << oss_ms.str() << '_'
        << oss_dm.str() << "___"
        << oss_dr.str();
    
    return oss.str();
}
std::string ForgetfulDrone::RunDPath () {
    return RawDpath() + '/' + RunID();
}


bool ForgetfulDrone::initBrain () {
    const char* torch_interface_name = std::get<0>(h_BRAIN_TORCHINTERFACES[p_TORCHINTERFACE_IDX]);
    ROSINFO("Interface torch with: " << torch_interface_name);
    
    if (p_TORCHINTERFACE_IDX == 0) return initBrain_Python();
    else if (p_TORCHINTERFACE_IDX == 1) return initBrain_Cpp();
    else return false;
}


bool ForgetfulDrone::initBrain_Python () {
    m_rosSUB_BRAIN_OUTPUT = m_rosRNH.subscribe("brain/output", 1, &ForgetfulDrone::ROSCB_BRAIN_OUTPUT, this);

    m_rosPUB_BRAIN_ENABLEINFERENCE = m_rosRNH.advertise<std_msgs::Bool>("brain/enable_inference", 1);
    m_rosPUB_BRAIN_TRIGGERINFERENCE = m_rosRNH.advertise<std_msgs::Empty>("brain/trigger_inference", 1);
    m_rosPUB_BRAIN_LOADCHECKPOINT = m_rosRNH.advertise<std_msgs::String>("brain/load_checkpoint", 1);

    return true;
}


bool ForgetfulDrone::initROSParameters () { 
    ROSINFO("Fetch ROS parameters");

    // ROS param keys and destinations of type bool
    std::vector <std::pair<const char*, const bool*>> kd_bool {
        //{"DYNAMIC_GATES_ON" , &m_DYNAMIC_GATES_ON},
        {"RVIZ_LOCTRAJ_ELAPSEDDISPLAYED", &p_RVIZ_LOCAL_TRAJECTORY_DISPLAY_ELAPSED_ENABLED},
        {"DRONE_TEST_ENABLED", &p_TEST_ENABLED},
        {"RVIZ_LABELEDRGB_SAVED", &p_RVIZ_LABELEDRGB_SAVED},
        {"RVIZ_LABELEDRGB_ENABLED", &p_RVIZ_LABELEDRGB_ENABLED},
    };

    // ROS param keys and destinations of type int
    std::vector <std::pair<const char*, const int*>> kd_int {
        {"CONFIG_NUMRUNS" , &p_CONFIG_NUMRUNS},
        {"GLOTRAJ_POLYNOMIALORDER" , &p_GLOBAL_TRAJECTORY_POLYNOMIAL_ORDER},
        {"GLOTRAJ_CONTINUITYORDER", &p_GLOBAL_TRAJECTORY_CONTINUITY_ORDER},
        {"NAV_REPLANNING_MAINLOOPITERSCNT" , &p_NAV_REPLANNING_MAINLOOPITERSCNT},
        {"NAV_LOCTRAJ_FEASIBILITY_MAXSUCCESSIVEFAILSCNT", &p_NAV_LOCTRAJ_FEASIBILITY_MAXSUCCESSIVEFAILSCNT},
        {"RUN_NUMLAPS", &p_RUN_NUMLAPS},
        {"WAYPARRIV_MAXDURATION", &p_WPARR_MAXDUR},
        {"DRONE_FLIGHTMISSION", &p_FLIGHTMISSION},
        {"DRONE_BRAIN_TORCHINTERFACE", &p_TORCHINTERFACE_IDX},
        {"DRONE_BRAIN_TORCHDEVICE", &p_BRAIN_TORCHDEVICE},
        
        {"DRONE_DAGGER_FIRSTRUN_NUMLAPS", &p_DAGGER_FIRSTRUN_NUMLAPS},
        {"DRONE_FLIGHTMISSION_DAGGER_NUM_EPOCHS", &p_DAGGER_NUM_EPOCHS},
    };

    // ROS param keys and destinations of type double
    std::vector <std::pair<const char*, const double*>> kd_dbl {
        {"DRONE_CAMERA_HALF_YAW_AOV", &p_DRONE_CAMERA_HALF_YAW_AOV},
        {"DRONE_CAMERA_HALF_PITCH_AOV", &p_DRONE_CAMERA_HALF_PITCH_AOV},
        {"GLOTRAJ_MAXTHRUST", &p_GLOBAL_TRAJECTORY_MAX_THRUST},
        {"GLOTRAJ_MAXROLLPITCHRATE", &p_GLOBAL_TRAJECTORY_MAX_ROLL_PITCH_RATE},
        {"GLOTRAJ_MAXSPEED", &p_GLOBAL_TRAJECTORY_MAX_SPEED}, 
        {"GLOTRAJ_NONDIMTEMPORALRANGE", &p_GLOBAL_TRAJECTORY_NON_DIMENSIONAL_TEMPORAL_RANGE},
        {"GLOTRAJ_NONDIMSPATIALRANGE", &p_GLOBAL_TRAJECTORY_NON_DIMENSIONAL_SPATIAL_RANGE},
        {"WAYPARRIV_THRESHOLDDIST", &p_WAYPOINT_ARRIVAL_THRESHOLD_DISTANCE},
        {"EXPERT_MINWAYPHORIZON", &p_EXPERT_MIN_HORIZON},
        {"EXPERT_SPEEDHORIZON", &p_EXPERT_SPEED_HORIZON},
        {"NAV_REFSTATE_MAXDIVERGENCE", &p_NAV_REFSTATE_MAXDIVERGENCE},
        {"EXPERT_PROJECTIONMAXDIVERGENCE", &p_EXPERT_PROJECTION_MAX_DIVERGENCE_FROM_GLOBAL_TRAJECTORY},
        {"DRONE_MAINLOOP_FREQ", &p_MAIN_LOOP_FREQUENCY},
        {"NAV_LOCTRAJ_MINSPEED", &p_LOCAL_TRAJECTORY_MIN_SPEED},
        {"NAV_LOCTRAJ_MAXSPEEDINCREMENT", &p_LOCAL_TRAJECTORY_MAX_SPEED_INCREMENT},
        {"NAV_LOCTRAJ_DURATION", &p_LOCAL_TRAJECTORY_DURATION},
        {"NAV_LOCTRAJ_MINDIST", &p_LOCAL_TRAJECTORY_MIN_DISTANCE},
        {"NAV_LOCTRAJ_MAXDIST", &p_LOCAL_TRAJECTORY_MAX_DISTANCE},
        {"NAV_LOCTRAJ_FEASIBILITY_POSITION_MAXALTITUDE", &p_LOCAL_TRAJECTORY_MAX_ALTITUDE},
        {"NAV_LOCTRAJ_FEASIBILITY_POSITION_MINALTITUDE", &p_LOCAL_TRAJECTORY_MIN_ALTITUDE},
        {"NAV_LOCTRAJ_FEASIBILITY_INPUT_MINTHRUST", &p_LOCAL_TRAJECTORY_MIN_THRUST},
        {"NAV_LOCTRAJ_FEASIBILITY_INPUT_MAXTHRUST", &p_LOCAL_TRAJECTORY_MAX_THRUST},
        {"NAV_LOCTRAJ_FEASIBILITY_INPUT_MAXBODYRATES", &p_LOCAL_TRAJECTORY_MAX_BODY_RATES},
        {"NAV_LOCTRAJ_FEASIBILITY_INPUT_MINSAMPLINGTIME", &p_LOCAL_TRAJECTORY_INPUT_FEASIBILITY_CHECK_MIN_SAMPLING_TIME},
        {"NAV_INPUTPERTURBATION_ELEMENTWISEAMP", &p_NAV_INPUTPERTURBATION_ELEMENTWISEAMP},
        {"RVIZ_LOCTRAJ_SAMPLINGDURATION", &p_RVIZ_LOCAL_TRAJECTORY_DURATION},
        {"RVIZ_LOCTRAJ_SAMPLINGFREQUENCY", &p_RVIZ_LOCAL_TRAJECTORY_SAMPLING_FREQUENCY},
        {"DRONE_FLIGHTMISSION_DAGGER_EXPERT_INTERVENTION_SHARE_THRESHOLD", &p_DAGGER_EXPERT_INTERVENTION_SHARE_THRESHOLD},
    };

    // ROS param keys and destinations of type string
    std::vector <std::pair<const char*, const std::string*>> kd_str
    {        
        {"DRONE_EXPERIMENT_ID", &p_EXPERIMENT_ID},
    };
    
    
    return true
    && fetchROSArrayParameter("DRONE_FLIGHTMISSION_UNITYSCENES",      p_FLIGHTMISSION_UNITYSCENES,      m_rosRNH, false)
    && fetchROSArrayParameter("DRONE_FLIGHTMISSION_SCENESITES",       p_FLIGHTMISSION_SCENESITES,       m_rosRNH, false)
    && fetchROSArrayParameter("DRONE_FLIGHTMISSION_TRACKTYPES",       p_FLIGHTMISSION_TRACKTYPES,       m_rosRNH, false)
    && fetchROSArrayParameter("DRONE_FLIGHTMISSION_TRACKGENERATIONS", p_FLIGHTMISSION_TRACKGENERATIONS, m_rosRNH, false)
    && fetchROSArrayParameter("DRONE_FLIGHTMISSION_TRACKDIRECTIONS",  p_FLIGHTMISSION_TRACKDIRECTIONS,  m_rosRNH, false)
    && fetchROSArrayParameter("DRONE_FLIGHTMISSION_GATETYPES",        p_FLIGHTMISSION_GATETYPES,        m_rosRNH, false)
    && fetchROSArrayParameter("DRONE_AUTOPILOT_REFFRAME_WRF",         p_AUTOPILOT_REFFRAME_WRF,         m_rosRNH, false)
    && fetchROSArrayParameter("NAV_LOCTRAJ_MAXSPEEDS",                p_LOCTRAJ_MAXSPEEDS,              m_rosRNH, false)
    && fetchROSArrayParameter("DRONE_DAGGER_MARGINS",                 p_DAGGERMARGINS,                  m_rosRNH, false)
    && fetchROSParameters(m_rosRNH, kd_bool, kd_int, kd_dbl, kd_str, /*log_enabled*/ false);
}



bool ForgetfulDrone::initBrain_Cpp () {
    // params from json file
    try {
        std::ifstream ifs (ConfigFpath());
        nlohmann::json jf = nlohmann::json::parse(ifs);
        
        nlohmann::json val = jf["ann"]["gru"]["hidden_size"];
        if (val.is_null()) const_cast<int&>(p_ANN_GRU_HIDDENSIZE) = 1;
        else const_cast<int&>(p_ANN_GRU_HIDDENSIZE) = val;

        val = jf["ann"]["gru"]["num_layers"];
        if (val.is_null()) const_cast<int&>(p_ANN_GRU_NUMLAYERS) = 1;
        else const_cast<int&>(p_ANN_GRU_NUMLAYERS) = val;

        const_cast<int&>(p_DATA_PROCESSED_RGB_HEIGHT) = jf["data"]["processed"]["rgb"]["height"];
        const_cast<int&>(p_DATA_PROCESSED_RGB_WIDTH) = jf["data"]["processed"]["rgb"]["width"];
    }
    catch(const std::exception& e) {
        ROSERROR(e.what());
        return false;
    }

    // deserialize the ScriptModule
    try {
        m_TorchDevice = std::get<1>(h_BRAIN_TORCHDEVICES[p_BRAIN_TORCHDEVICE]);
        ROSINFO("Use torch device: " << std::get<0>(h_BRAIN_TORCHDEVICES[p_BRAIN_TORCHDEVICE]));

        m_TorchTensorOptions = torch::TensorOptions()
            .dtype(torch::kFloat32)
            .layout(torch::kStrided)
            //.device(torch::Device(m_TorchDevice, 0))
            .requires_grad(false);

        m_TorchScriptModule = torch::jit::load (ScriptModuleFpath());
        m_TorchScriptModule.to(torch::Device(m_TorchDevice, 0));

        m_ANNGRUHiddenState = std::vector<float>(
            p_ANN_GRU_NUMLAYERS * h_BATCHSIZE * p_ANN_GRU_HIDDENSIZE, 0.0);
    }
    catch (const c10::Error& e) {
        ROSERROR("Error loading model: " << e.what());
        return false;
    }
    catch(const std::exception& e) {
        ROSERROR(e.what());
        return false;
    }

    return true;
}



bool ForgetfulDrone::checkPaths () {
    bool successful {true};

    auto is_dir = [&successful] (const std::string& path) {
        if (!isDirectory(path)) {
            ROSERROR("Experiment ID was specified but \"" << path << "\" is not a directory");
            successful = false;
        }
    };

    auto is_file = [&successful] (const std::string& path) {
        if (!isFile(path)) {
            ROSERROR("Experiment ID was specified but \"" << path << "\" is not a file");
            successful = false;
        }
    };

    is_dir(ExperimentDpath());
    is_dir(DataDpath());
    is_dir(RawDpath());
    is_dir(ConfigDpath());
    is_dir(OutputDpath());

    is_file(ConfigFpath());
    is_file(ScriptModuleFpath());

    return successful;
}


bool ForgetfulDrone::initExperimentID () {
    try {
        if (p_EXPERIMENT_ID == "") {
            const_cast<bool&>(p_EXPERIMENT_NEW) = true;
            const_cast<std::string&>(p_EXPERIMENT_ID) = getUTCDateTimeString();
            ROSINFO("Start new experiment: " << p_EXPERIMENT_ID);

            initExperimentDirectory();
        }
        else {
            ROSINFO("Continue experiment: " << p_EXPERIMENT_ID);
            const_cast<bool&>(p_EXPERIMENT_NEW) = false;

            if (!checkPaths()) return false;
        }
        m_rosRNH.setParam("EXPERIMENT_DPATH", ExperimentDpath());
    } 
    catch (const std::exception& e) {
        ROSERROR(e.what());
        return false;
    }
    
    return true;
}

bool ForgetfulDrone::initTrafoArf2Wrf () {
    // transformation from autopilot to world reference frame
    try {
        geometry_msgs::Pose p;
        p.position.x = p_AUTOPILOT_REFFRAME_WRF[0];
        p.position.y = p_AUTOPILOT_REFFRAME_WRF[1];
        p.position.z = p_AUTOPILOT_REFFRAME_WRF[2];
        p.orientation.w = p_AUTOPILOT_REFFRAME_WRF[3];
        p.orientation.x = p_AUTOPILOT_REFFRAME_WRF[4];
        p.orientation.y = p_AUTOPILOT_REFFRAME_WRF[5];
        p.orientation.z = p_AUTOPILOT_REFFRAME_WRF[6];
        tf::poseMsgToKindr(p, &const_cast<kindr::minimal::QuatTransformation&>(p_T_WRF_ARF));
    } 
    catch(const std::exception& e) {
        ROSERROR(e.what());
        return false;
    }

    return true;
}







void ForgetfulDrone::test () {
    // set logger level to Debug
    namespace rc = ros::console;
    if (rc::set_logger_level(ROSCONSOLE_DEFAULT_NAME, rc::levels::Debug)) rc::notifyLoggerLevelsChanged();

    ROSDEBUG("--- Testing Network in C++ ---");


}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////// ROS CALLBACKS //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void ForgetfulDrone::ROSCB_CONTROL_COMMAND (const quadrotor_msgs::ControlCommand::ConstPtr& msg) { 
    m_CtrlCmdMtx.lock(); 
    m_CtrlCmdPtr = msg;
    m_CtrlCmdMtx.unlock();
}

void ForgetfulDrone::ROSCB_AUTOPILOT_FEEDBACK (const quadrotor_msgs::AutopilotFeedback::ConstPtr& msg) {
    m_AutopilotState = msg->autopilot_state;
}

void ForgetfulDrone::ROSCB_GROUND_TRUTH_ODOMETRY (const nav_msgs::OdometryConstPtr& msg) {
    tf::poseMsgToKindr(GMPose_From_NMO(*msg), &m_T_ARF_DRF);
    m_T_WRF_DRF = p_T_WRF_ARF * m_T_ARF_DRF;
    m_ActualNormSpeed = EV3d_From_GMV3(msg->twist.twist.linear).norm() / m_LocTrajMaxSpeed;
}

void ForgetfulDrone::ROSCB_GROUND_TRUTH_IMU (const sensor_msgs::ImuConstPtr& msg) {
    m_IMUMtx.lock();
    m_IMUPtr = msg;
    m_IMUMtx.unlock();
}

void ForgetfulDrone::ROSCB_BRAIN_OUTPUT (const geometry_msgs::PointConstPtr& msg) {
    m_BrainOutput = {msg->x, msg->y, msg->z};
}

void ForgetfulDrone::ROSCB_FLIGHTMARE_RGB (const sensor_msgs::ImageConstPtr& msg) {
    //std::cout << *msg <<std::endl;
    m_RGBMtx.lock();
    m_RGBPtr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
    m_RGBMtx.unlock();
}



void ForgetfulDrone::rvizLabeledCamFrame() {
    if (isEmpty(ROS_LOG_PREFIX, m_RGBData)) return;

    
    const int& xPx_N = m_RGBData.cols;
    const int& yPx_N = m_RGBData.rows;
    int xPx, yPx;

    // Add vertical grid lines
    constexpr double angle_inc = 30.0 * M_PI/180.0;
    for (double yaw = 0.0; yaw <= p_DRONE_CAMERA_HALF_YAW_AOV; yaw += angle_inc) {
        xPx = static_cast<int>((yaw/p_DRONE_CAMERA_HALF_YAW_AOV + 1.0) * xPx_N/2.0);
        cv::line(m_RGBData, cv::Point(xPx, 0), cv::Point(xPx, yPx_N), cv::Scalar(100, 100, 100, 0));

        xPx = static_cast<int>((-yaw/p_DRONE_CAMERA_HALF_YAW_AOV + 1.0) * xPx_N/2.0);
        cv::line(m_RGBData, cv::Point(xPx, 0), cv::Point(xPx, yPx_N), cv::Scalar(100, 100, 100, 0));
    }
    // Add horizontal grid lines
    for (double pitch = 0; pitch <= p_DRONE_CAMERA_HALF_PITCH_AOV; pitch += angle_inc) {
        yPx = static_cast<int>((pitch/p_DRONE_CAMERA_HALF_PITCH_AOV + 1.0) * yPx_N/2.0);
        cv::line(m_RGBData, cv::Point(0, yPx), cv::Point(xPx_N, yPx), cv::Scalar(100, 100, 100, 0));

        yPx = static_cast<int>((-pitch/p_DRONE_CAMERA_HALF_PITCH_AOV + 1.0) * yPx_N/2.0);
        cv::line(m_RGBData, cv::Point(0, yPx), cv::Point(xPx_N, yPx), cv::Scalar(100, 100, 100, 0));
    }

    // Add expert output
    constexpr double alpha = 0.8;
    xPx = static_cast<int>(xPx_N * (1.0 + m_ExpertOutput.x()) / 2.0);
    yPx = static_cast<int>(yPx_N * (1.0 - m_ExpertOutput.y()) / 2.0);
    cv::circle(m_RGBData, cv::Point(xPx, yPx), 0, cv::Scalar(255, 0, 0), 12, 8, 0);
    cv::Mat roi = m_RGBData(cv::Rect(xPx_N - 200, yPx_N - 50, 190, 40));
    cv::Mat color = cv::Mat(roi.size(), CV_8UC3, cv::Scalar(125, 125, 125));
    cv::addWeighted(color, alpha, roi, 1.0 - alpha, 0.0, roi); // grey box
    cv::putText(m_RGBData, 
        ("Expert " + std::to_string(m_ExpertOutput.z()).substr(0, 5)).c_str(), 
        cv::Point2f(xPx_N - 190, yPx_N - 23), 
        cv::FONT_ITALIC, 0.7, cv::Scalar(255, 0, 0, 255));

    // Add ANN output
    xPx = static_cast<int>(xPx_N * (1.0 + m_BrainOutput.x()) / 2.0);
    yPx = static_cast<int>(yPx_N * (1.0 - m_BrainOutput.y()) / 2.0);
    cv::circle(m_RGBData, cv::Point(xPx, yPx), 0, cv::Scalar(0, 255, 0), 12, 8, 0);
    roi = m_RGBData(cv::Rect(10, yPx_N - 50, 190, 40));
    color = cv::Mat(roi.size(), CV_8UC3, cv::Scalar(125, 125, 125));
    cv::addWeighted(color, alpha, roi, 1.0 - alpha, 0.0, roi);
    cv::putText(m_RGBData, 
        ("Network " + std::to_string(m_BrainOutput.z()).substr(0, 5)).c_str(), 
        cv::Point2f(20, yPx_N - 23), 
        cv::FONT_ITALIC, 0.7, cv::Scalar(0, 255, 0, 255));

    // --- Add actual normalized speed
    roi = m_RGBData(cv::Rect(static_cast<int>(xPx_N/2-95), yPx_N - 50, 190, 40));
    color = cv::Mat(roi.size(), CV_8UC3, cv::Scalar(125, 125, 125));
    cv::addWeighted(color, alpha, roi, 1.0 - alpha, 0.0, roi);
    cv::putText(m_RGBData, 
        ("Estimate " + std::to_string(m_ActualNormSpeed).substr(0, 5)).c_str(), 
        cv::Point2f(static_cast<int>(xPx_N/2-85), yPx_N - 23), 
        cv::FONT_ITALIC, 0.7, cv::Scalar(0, 0, 255, 255));

    // --- Add brain decision / expert intervention bar
    int total_cnt = m_ExpertInterventionsCnt + m_BrainDecisionsCnt;
    if (total_cnt != 0) {
        double exp_share = (double)m_ExpertInterventionsCnt / (double)total_cnt;
        double brain_share = 1.0 - exp_share;
        int total_width = xPx_N - 20;
            // - brain
            int brain_width = (int)(total_width * brain_share);
            roi = m_RGBData(cv::Rect(10, 10, brain_width, 40));
            color = cv::Mat(roi.size(), CV_8UC3, cv::Scalar(0, 255, 0));
            cv::addWeighted(color, alpha, roi, 1.0 - alpha, 0.0, roi);
            // - expert
            int exp_width = total_width - brain_width;
            roi = m_RGBData(cv::Rect(10 + brain_width + 1, 10, exp_width, 40));
            color = cv::Mat(roi.size(), CV_8UC3, cv::Scalar(255, 0, 0));
            cv::addWeighted(color, alpha, roi, 1.0 - alpha, 0.0, roi);

            cv::putText(m_RGBData, 
                (std::to_string(100 * brain_share).substr(0, 5) + " %").c_str(), 
                cv::Point2f(20, 50 - 13), 
                cv::FONT_ITALIC, 0.7, cv::Scalar(125, 125, 125, 255));
                
            cv::putText(m_RGBData, 
                (std::to_string(100 * exp_share).substr(0, 5) + " %").c_str(), 
                cv::Point2f(xPx_N - 130, 50 - 13), 
                cv::FONT_ITALIC, 0.7, cv::Scalar(125, 125, 125, 255));
    }
    

    

    // rviz labeled camera frame
    cv_bridge::CvImage msg;
    msg.header.stamp = ros::Time::now();
    msg.encoding = sensor_msgs::image_encodings::BGR8;
    msg.image = m_RGBData;
    m_rosPUB_RVIZ_RGB_LABELED.publish(msg.toImageMsg());

    // save labeled camera frame
    if (p_RVIZ_LABELEDRGB_SAVED)
    {
        ROSWARN("Implementation of saving labeled camera frames not tested.");

        std::ostringstream oss; oss << std::setw(5) << std::setfill('0') << m_RGBCnt;
        std::string img_path = m_RunConfigDpath + "/labeled_images/camera_frame_" + oss.str() + ".jpg";
        
        saveCVMat (ROS_LOG_PREFIX, img_path, m_RGBData);
        
        std::ofstream ofs;
        std::string stamps_path = m_RunConfigDpath + "/labeled_images/stamps.txt";
        ofs.open(stamps_path, std::ios_base::app);
        ofs << ros::Time::now().toSec() << std::endl;
        ofs.close();
    }
}



//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////// FLIGHT MISSIONS //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


void ForgetfulDrone::waitForAutopilotState (
    const uint8_t& ap_state,
    const bool& exit_state
) const {
    //ROSDEBUG_FUNCTION_ENTERED();

    std::string aps_string;
    switch (ap_state)
    {
    case quadrotor_msgs::AutopilotFeedback::BREAKING: aps_string = "BREAKING"; break;
    case quadrotor_msgs::AutopilotFeedback::HOVER: aps_string = "HOVER"; break;
    case quadrotor_msgs::AutopilotFeedback::OFF: aps_string = "OFF"; break;
    default:
        aps_string = "?"; break;
    }
    
    
    ros::Rate Rate(100.0);
    if (!exit_state) {
        //ROSDEBUG("Wait for autopilot state " << aps_string);
        while (m_AutopilotState != ap_state) {
            ros::spinOnce(); 
            Rate.sleep(); 
        }
    }
    else {
        //ROSDEBUG("Wait for any autopilot state but " << aps_string);
        while (m_AutopilotState == ap_state) {
            ros::spinOnce(); 
            Rate.sleep(); 
        }
    }
}

void ForgetfulDrone::launchDroneOffGround () {
    ROSINFO("Launch drone off ground");

    //ROSDEBUG("Switch autopilot off.");
    m_rosPUB_AUTOPILOT_OFF.publish(std_msgs::Empty());

    //ROSDEBUG("Arm bridge.");
    std_msgs::Bool msg; msg.data = true; m_rosPUB_BRIDGE_ARM.publish(msg);

    //ROSDEBUG("Start autopilot.");
    m_rosPUB_AUTOPILOT_START.publish(std_msgs::Empty());

    waitForAutopilotState(quadrotor_msgs::AutopilotFeedback::BREAKING);
    waitForAutopilotState(quadrotor_msgs::AutopilotFeedback::HOVER);
    //ROSDEBUG("Drone launched off ground.");
    ros::Duration(1.0).sleep();
}

void ForgetfulDrone::landDroneOnGround () {
    ROSINFO("Land drone on ground");

    //ROSDEBUG("Land autopilot.");
    m_rosPUB_AUTOPILOT_LAND.publish(std_msgs::Empty());
    
    waitForAutopilotState(quadrotor_msgs::AutopilotFeedback::OFF);

    //ROSDEBUG("Disarm bridge.");
    std_msgs::Bool msg; msg.data = false; m_rosPUB_BRIDGE_ARM.publish(msg);

    //ROSDEBUG("Drone landed on ground.");
    ros::Duration(1.0).sleep();
}


void ForgetfulDrone::flyDroneToPose (const geometry_msgs::Pose& target_pose) const {
    ROSINFO(" - Fly drone to, position: "
        << EV3d_From_GMP(target_pose.position).transpose()
        << ", yaw: " << Yaw_From_EQd(EQd_From_GMQ(target_pose.orientation)));
    
    geometry_msgs::PoseStamped msg;
    msg.pose = target_pose;
    m_rosPUB_AUTOPILOT_POSE_COMMAND.publish(msg);

    waitForAutopilotState(quadrotor_msgs::AutopilotFeedback::HOVER);
    waitForAutopilotState(quadrotor_msgs::AutopilotFeedback::HOVER, true);
    waitForAutopilotState(quadrotor_msgs::AutopilotFeedback::HOVER);
    
    ROSINFO(" - Drone arrived at, position: " 
        << EV3d_From_GMP(target_pose.position).transpose()
        << ", yaw: " << Yaw_From_EQd(EQd_From_GMQ(target_pose.orientation)));
    ros::Duration(1.0).sleep();
}


void ForgetfulDrone::flyDroneAboveTrack () {
    ROSINFO ("Fly drone above init pose");

    // fly between the two closest gates
    Eigen::Vector3d curr_pos = m_T_ARF_DRF.getPosition();
    double dmin = 1e10;
    size_t imin {0};
    for (size_t i = 0; i < m_TrackWaypoints.size(); i++) {
        double d = (curr_pos - m_TrackWaypoints[i]).norm();
        if (d < dmin) {
            imin = i;
            dmin = d;
        }
    }
    dmin = 1e10;
    size_t jmin {1};
    for (size_t j = 0; j < m_TrackWaypoints.size(); j++) {
        if (j == imin) continue;
        double d = (curr_pos - m_TrackWaypoints[j]).norm();
        if (d < dmin) {
            jmin = j;
            dmin = d;
        }
    }
    //Eigen::Vector3d target_p = (m_TrackWaypoints.back() + m_TrackWaypoints[0]) / 2;
    Eigen::Vector3d target_p = (m_TrackWaypoints[imin] + m_TrackWaypoints[jmin]) / 2;
    Eigen::Quaterniond target_q = m_T_ARF_DRF.getEigenQuaternion();
    flyDroneToPose(GMPose_from_EV3d_EQd(target_p, target_q));

    // fly upwards above track
    target_p += Eigen::Vector3d{0, 0, m_DroneInitPose.position.z + 6.0};
    flyDroneToPose(GMPose_from_EV3d_EQd(target_p, target_q));

    
}




void ForgetfulDrone::flyDroneToInitPose () {
    ROSINFO ("Fly drone to init pose");

    // fly above drone start position
    geometry_msgs::Pose pose = m_DroneInitPose;
    pose.position.z += 6.0;
    flyDroneToPose(pose);

    
    flyDroneToPose(m_DroneInitPose);
}








void ForgetfulDrone::initMainLoopTimer (void (forgetful_drone::ForgetfulDrone::*callback)(const ros::TimerEvent&)) {
    m_rosTMR_MAINLOOP = m_rosRNH.createTimer(
        /*rate*/ ros::Rate(p_MAIN_LOOP_FREQUENCY), 
        /*callback*/ callback, /*obj*/ this, 
        /*oneshot*/ false, /*autostart*/ false
    );
};




void ForgetfulDrone::performFlightMission_NavigationByExpert () {
    ROSDEBUG_FUNCTION_ENTERED();

    m_TotalRunCnt = 0;
    m_SimulatorPtr = nullptr;
    logFlightMissionInfo();
    initMainLoopTimer(&ForgetfulDrone::ROSCB_NAVIGATION_BY_EXPERT);

    m_RunNumLaps = p_RUN_NUMLAPS;

    m_TotalRunCnt = 0;
    do {
        reset4NewRun();
        logRunInfo(
            m_TotalRunCnt, p_CONFIG_NUMRUNS,
            p_UNITY_SCENE, p_SCENE_SITE,
            p_RACETRACK_TYPE, p_GATE_TYPE,
            p_INTERMEDIATE_TARGET_LOSS_GAP_TYPE,
            p_INTERMEDIATE_TARGET_LOSS_DIRECTION
        );
        do {
            buildSimulation(
                p_RACETRACK_TYPE,
                p_UNITY_SCENE,
                p_SCENE_SITE,
                p_GATE_TYPE,
                p_INTERMEDIATE_TARGET_LOSS_DIRECTION,
                p_INTERMEDIATE_TARGET_LOSS_GAP_TYPE
            );
            ros::Duration(1.0).sleep();
        } while (!computeGlobalTrajectory());
        
        startSimulation();
        rvizGloTraj();
            if (m_TotalRunCnt == 0) launchDroneOffGround();
            else flyDroneToInitPose();
            runNavigation ();
        stopSimulation();
        flyDroneAboveTrack();
        m_TotalRunCnt++;
    } while (m_TotalRunCnt < p_CONFIG_NUMRUNS);
    
    landDroneOnGround();
    logFlightMissionResults();
}







void ForgetfulDrone::performFlightMission_NavigationByBrain () {
    logFlightMissionInfo();
    initMainLoopTimer(std::get<1>(h_BRAIN_TORCHINTERFACES[p_TORCHINTERFACE_IDX]));

    m_TotalRunCnt = 0;
    m_SimulatorPtr = nullptr;
    int TotalRunSuccessfulCnt = 0;
    bool drone_launched {false};

    for (const int& unity_scene : p_FLIGHTMISSION_UNITYSCENES)
    for (const int& scene_site : p_FLIGHTMISSION_SCENESITES)
    for (const int& track_type : p_FLIGHTMISSION_TRACKTYPES)
    for (const int& track_generation : p_FLIGHTMISSION_TRACKGENERATIONS)
    for (const int& track_direction : p_FLIGHTMISSION_TRACKDIRECTIONS)
    for (const int& gate_type : p_FLIGHTMISSION_GATETYPES) 
    for (const double& loctraj_maxspeed: p_LOCTRAJ_MAXSPEEDS) {
        

        m_UnitySceneIdx = static_cast<uint8_t>(unity_scene);
        m_SceneSiteIdx = static_cast<uint8_t>(scene_site);
        m_TrackTypeIdx = static_cast<uint8_t>(track_type);
        m_TrackGenerationIdx = static_cast<uint8_t>(track_generation);
        m_TrackDirectionIdx = static_cast<uint8_t>(track_direction);
        m_GateTypeIdx = static_cast<uint8_t>(gate_type);
        m_LocTrajMaxSpeed = loctraj_maxspeed;
        m_rosRNH.setParam("brain_MAX_SPEED", m_LocTrajMaxSpeed);

        int config_run_cnt {0};
        int config_run_successful_cnt {0};
        do {
            if (config_run_cnt == 0) {
                std::string spacing = "  |  ";
                std::cout << std::endl;
                ROSINFO(std::endl
                    << "Switch to run configuration:"
                    << "  unity scene: " << std::get<0>(ForgetfulSimulator::getUnityScenes()->at(m_UnitySceneIdx)) << spacing
                    << "  scene site: " << std::get<0>(ForgetfulSimulator::getSceneSites()->at(m_SceneSiteIdx)) << spacing
                    << "  track type: " << std::get<0>(ForgetfulSimulator::getTrackTypes()->at(m_TrackTypeIdx)) << std::endl
                    << "  track generation: " << std::get<0>(ForgetfulSimulator::getTrackGenerations()->at(m_TrackGenerationIdx)) << spacing
                    << "  track direction: " << std::get<0>(ForgetfulSimulator::getTrackDirections()->at(m_TrackDirectionIdx)) << spacing
                    << "  gate type: " << std::get<0>(ForgetfulSimulator::getGateTypes()->at(m_GateTypeIdx)) << std::endl
                    << "  max speed: " << m_LocTrajMaxSpeed << " m/s" << spacing
                );
            }

            ROSINFO(std::endl << "Start run #" << m_TotalRunCnt << " (repetition " << config_run_cnt + 1 << "/" << p_CONFIG_NUMRUNS << ")");

                        
            reset4NewRun();
            buildSimulation();
            setTrackWaypoints ();
            initWaypointIndices();
            startSimulation();

            if (!drone_launched) {launchDroneOffGround (); drone_launched = true;}
            flyDroneToInitPose();


            if (!p_EXPERIMENT_NEW) {
                if (p_TORCHINTERFACE_IDX == 0) {
                    std_msgs::String lc_msg; lc_msg.data = p_EXPERIMENT_ID;
                    m_rosPUB_BRAIN_LOADCHECKPOINT.publish(lc_msg);
                    ros::Duration(1.0).sleep();

                    std_msgs::Bool ei_msg; ei_msg.data = true;
                    m_rosPUB_BRAIN_ENABLEINFERENCE.publish(ei_msg);
                    ros::Duration(1.0).sleep();
                }
                if (p_TORCHINTERFACE_IDX == 1) {
                    m_TorchScriptModule = torch::jit::load (ScriptModuleFpath());
                    m_TorchScriptModule.to(torch::Device(m_TorchDevice, 0));

                    m_ANNGRUHiddenState = std::vector<float>(
                        p_ANN_GRU_NUMLAYERS * h_BATCHSIZE * p_ANN_GRU_HIDDENSIZE, 0.0);
                }
            }
            

            bool track_completed = runNavigation();
            if (track_completed) {
                TotalRunSuccessfulCnt++;
                config_run_successful_cnt++;
            }
            stopSimulation();
            flyDroneAboveTrack();

            m_TotalRunCnt++;
        } while (config_run_cnt++ < p_CONFIG_NUMRUNS);
        ROSINFO("Completed run configuration: " << config_run_successful_cnt << "/" << config_run_cnt << "runs were successful.");
    }
    
    landDroneOnGround(); drone_launched = false;
    ROSINFO("Completed flight mission: " << h_FLIGHTMISSIONS[p_FLIGHTMISSION] << ": " << TotalRunSuccessfulCnt << "/" << m_TotalRunCnt << "runs were successful.");
}


void ForgetfulDrone::initExperimentDirectory () {
    createDirectory (ROS_LOG_PREFIX, ExperimentDpath());
    createDirectory (ROS_LOG_PREFIX, RawDpath());
    createDirectory (ROS_LOG_PREFIX, ConfigDpath());

    copyFile(ROS_LOG_PREFIX, ParametersFpath(true), ParametersFpath(false));
}







void ForgetfulDrone::initRunDirectory () {
    m_RunID = RunID();
    m_rosRNH.setParam("LATEST_RUN_ID", m_RunID);

    m_RunDpath = RunDPath();
    m_RunRGBDpath = m_RunDpath + '/' + h_RUN_IMAGES_DNAME;
    m_RunDataFpath = m_RunDpath + '/' + h_DATA_FNAME;


    createDirectory (ROS_LOG_PREFIX, m_RunDpath);
    createDirectory (ROS_LOG_PREFIX, m_RunRGBDpath);

    std::ofstream ofs;
    const char delimiter = ',';
    ofs.open(m_RunDataFpath, std::ios_base::app); ofs
        << "expert_intervened" << delimiter
        << "rgb_dt" << delimiter
        << "imu_dt" << delimiter
        << "imu_linacc_x" << delimiter
        << "imu_linacc_y" << delimiter
        << "imu_linacc_z" << delimiter
        << "imu_angvel_x" << delimiter
        << "imu_angvel_y" << delimiter
        << "imu_angvel_z" << delimiter
        << "max_speed" << delimiter
        << "exp_waypoint_x" << delimiter
        << "exp_waypoint_y" << delimiter
        << "exp_normspeed" << delimiter
        << "ctrlcmd_bodyrates_x" << delimiter
        << "ctrlcmd_bodyrates_y" << delimiter
        << "ctrlcmd_bodyrates_z" << delimiter
        << "ctrlcmd_angacc_x" << delimiter
        << "ctrlcmd_angacc_y" << delimiter
        << "ctrlcmd_angacc_z" << delimiter
        << "ctrlcmd_collthrust" << std::endl;
    ofs.close(); ofs.clear();
}

void ForgetfulDrone::createRunConfigDirectory () {
    
    m_RunConfigDpath = RawDpath() + "/"
        + std::string(h_RUN_DNAME_PREFIX) + "_"
        + std::to_string(m_UnitySceneIdx) + "_"
        + std::to_string(m_SceneSiteIdx) + "_"
        + std::to_string(m_TrackTypeIdx) + "_"
        + std::to_string(m_TrackGenerationIdx) + "_"
        + std::to_string(m_TrackDirectionIdx) + "_"
        + std::to_string(m_GateTypeIdx);


    
    std::array<std::string, 1> dpaths {
        m_RunConfigDpath, 
        //m_RunConfigDpath + "/" + h_RUN_IMAGES_DNAME,
        //m_RunConfigDpath + "/" + h_RUN_LABELEDIMAGES_DNAME,
    };
    for (const std::string& dir_path : dpaths) {
        createDirectory (ROS_LOG_PREFIX, dir_path);
    }
}

void ForgetfulDrone::initRunDaggerDirectory () {
    
    std::ostringstream oss; oss << std::setw(3) << std::setfill('0') << m_DaggerRepCnt;


    
    m_RunDpath
        = m_RunConfigDpath + "/"
        + h_RUN_DAGGER_DNAME_PREFIX + "_" 
        + std::to_string(m_DaggerMargin) + "_" + oss.str();
    m_RunRGBDpath
        = m_RunDpath + "/" + h_RUN_IMAGES_DNAME;
    m_RunDaggerLabeledFramesDpath
        = m_RunDpath + "/" + h_RUN_LABELEDIMAGES_DNAME;

    std::array<std::string, 3> dpaths {
        m_RunDpath, 
        m_RunRGBDpath,
        m_RunDaggerLabeledFramesDpath,
    };
    for (const std::string& dir_path : dpaths) {
        createDirectory (ROS_LOG_PREFIX, dir_path);
    }


    std::ofstream ofs;
    const char delimiter = ',';
    ofs.open(m_RunDataFpath, std::ios_base::app); ofs
        << "expert_intervened" << delimiter
        << "rgb_dt" << delimiter
        << "imu_dt" << delimiter
        << "imu_linacc_x" << delimiter
        << "imu_linacc_y" << delimiter
        << "imu_linacc_z" << delimiter
        << "imu_angvel_x" << delimiter
        << "imu_angvel_y" << delimiter
        << "imu_angvel_z" << delimiter
        << "exp_waypoint_x" << delimiter
        << "exp_waypoint_y" << delimiter
        << "exp_normspeed" << delimiter
        << "ctrlcmd_bodyrates_x" << delimiter
        << "ctrlcmd_bodyrates_y" << delimiter
        << "ctrlcmd_bodyrates_z" << delimiter
        << "ctrlcmd_angacc_x" << delimiter
        << "ctrlcmd_angacc_y" << delimiter
        << "ctrlcmd_angacc_z" << delimiter
        << "ctrlcmd_collthrust" << std::endl;
    ofs.close(); ofs.clear();
}

void ForgetfulDrone::createRunDirectory (
    const int& scene_idx,
    const int& site_idx,
    const int& gate_idx,
    const int& rep_idx,
    const int& dir_idx,
    const int& gap_idx
) {
    ROSDEBUG_FUNCTION_ENTERED();

    if (m_STR2VAL_MAP.at(p_RACETRACK_TYPE) != BDRSR::INTERMEDIATE_TARGET_LOSS) {
        m_RunConfigDpath = RawDpath() + "/run_"
        + std::to_string(scene_idx) + "_" + std::to_string(site_idx) + "_" 
        + std::to_string(gate_idx) + "_" + std::to_string(rep_idx) + "_X_X";
    } 
    else {
        m_RunConfigDpath = RawDpath() + "/run_" 
        + std::to_string(scene_idx) + "_" + std::to_string(site_idx) + "_" 
        + std::to_string(gate_idx) + "_X_" + std::to_string(dir_idx)
        + "_" + std::to_string(gap_idx);
    }
    
    std::array<std::string, 3> dir_paths {
        m_RunConfigDpath, 
        m_RunConfigDpath + "/images", 
        m_RunConfigDpath + "/labeled_images"
    };
    for (const std::string& dir_path : dir_paths) {
        createDirectory(ROS_LOG_PREFIX, dir_path);
    }    
}


void ForgetfulDrone::logRunInfo (const bool& num_runs_known) {

    std::stringstream ss;
    if (num_runs_known) {
        const size_t num_run_configurations 
            = p_FLIGHTMISSION_UNITYSCENES.size()
            * p_FLIGHTMISSION_SCENESITES.size()
            * p_FLIGHTMISSION_TRACKTYPES.size()
            * p_FLIGHTMISSION_TRACKGENERATIONS.size()
            * p_FLIGHTMISSION_TRACKDIRECTIONS.size()
            * p_FLIGHTMISSION_GATETYPES.size();
        ss << "Run " << m_TotalRunCnt + 1 << "/" << num_run_configurations << ": ";
    } else {
        ss << "Run " << m_TotalRunCnt + 1 << "/?: ";
    }
    
    ROSINFO(ss.str() << ", configuration: "
        << std::get<0>(ForgetfulSimulator::getUnityScenes()->at(m_UnitySceneIdx)) << " - "
        << std::get<0>(ForgetfulSimulator::getSceneSites()->at(m_SceneSiteIdx)) << " - "
        << std::get<0>(ForgetfulSimulator::getTrackTypes()->at(m_TrackTypeIdx)) << " - "
        << std::get<0>(ForgetfulSimulator::getTrackGenerations()->at(m_TrackGenerationIdx)) << " - "
        << std::get<0>(ForgetfulSimulator::getTrackDirections()->at(m_TrackDirectionIdx)) << " - "
        << std::get<0>(ForgetfulSimulator::getGateTypes()->at(m_GateTypeIdx))
    );
};

void ForgetfulDrone::logRunInfo (
    const int& run_cnt,
    const int& run_n,
    const std::string& unity_scene,
    const std::string& scene_site,
    const std::string& racetrack_type,
    const std::string& gate_type,
    const std::string& gap_type,
    const std::string& direction,
    const int& rep_i,
    const int& rep_n
) const {
    ROSDEBUG_FUNCTION_ENTERED();

    std::stringstream ss; ss 
        << "Run " << run_cnt + 1 << "/" << run_n << ": "
        << unity_scene << " - " 
        << scene_site << " - " 
        << racetrack_type << " - " 
        << gate_type;
    if (rep_i != -1 && rep_n != -1) {
        ss  << " - repetition " << rep_i + 1 << "/" << rep_n;
    }
    if (m_STR2VAL_MAP.at(racetrack_type) == BDRSR::INTERMEDIATE_TARGET_LOSS) {
        ss  << " - "
            << gap_type << " - "
            << direction;
    }

    ROSINFO(ss.str());
}

void ForgetfulDrone::logFlightMissionInfo () const {
    ROSINFO("Start flight mission: " << h_FLIGHTMISSIONS[p_FLIGHTMISSION]);
}

void ForgetfulDrone::logFlightMissionResults () const {
    ROSINFO("Finished flight mission: " << h_FLIGHTMISSIONS[p_FLIGHTMISSION]);
}


void ForgetfulDrone::reset4NewRun () {
    m_ExpertInterventionsCnt = 0;
    m_BrainDecisionsCnt = 0;
    m_RunLapCnt = -1;
    m_MainLoopIterCnt = 0;
    m_LocTrajSubseqInfeasibleCnt = 0;
    m_LocTrajFeasibleCnt = 0;
    m_LocTraj = {Vec3(), Vec3(), Vec3(), Vec3()};
    m_LocTrajStartTime = {};
    m_RGBCnt = 0;
    m_DataSavingENABLED = false;
}

void ForgetfulDrone::buildSimulation (
    const std::string& racetrack_type,
    const std::string& unity_scene,
    const std::string& scene_site,
    const std::string& gate_type,
    const std::string& itl_direction,
    const std::string& itl_gap_type
) {
    ROSDEBUG_FUNCTION_ENTERED();
    
    BDRS srv;
    srv.request.RacetrackType = m_STR2VAL_MAP.at(racetrack_type);
    srv.request.UnityScene = m_STR2VAL_MAP.at(unity_scene);
    srv.request.RacetrackSite = m_STR2VAL_MAP.at(scene_site);
    srv.request.RaceGateType = m_STR2VAL_MAP.at(gate_type);
    srv.request.RacetrackClockwise = m_STR2VAL_MAP.at(itl_direction);
    srv.request.RacetrackMode = m_STR2VAL_MAP.at(itl_gap_type);

    bool SimReady {false}; 
    do {
        if (callROSService<BDRS>(ROS_LOG_PREFIX, m_rosSVC_SIMULATOR_BUILD, srv)) {
            m_GateInitPoses = srv.response.GatesInitPose;
            m_DroneInitPose = srv.response.DroneInitPose;

            if (m_STR2VAL_MAP.at(p_RACETRACK_TYPE) == BDRSR::INTERMEDIATE_TARGET_LOSS)
                insertITLWaypoint();

            SimReady = true;
        }
        /*wait before next call try*/ ros::Duration(1.0).sleep();
    } while (!SimReady);

    // --- Set Waypoint Idx for deterministic 8 RaceTrack 
    m_CurrWaypointIdx = m_GateInitPoses.size() - 1;
    m_LastWaypointIdx = (m_CurrWaypointIdx + m_GateInitPoses.size() - 1) % m_GateInitPoses.size();

    // Set waypoints of global trajectory from gate poses
    m_TrackWaypoints.clear();
    m_TrackWaypoints.reserve(m_GateInitPoses.size());
    for (const geometry_msgs::Pose& pose : m_GateInitPoses)
        m_TrackWaypoints.push_back(EV3d_From_GMP(pose.position));

    // Set max horizon of expert to max straight distance between waypoints of global trajetory
    m_Expert_MaxHorizon = 0.0;
    size_t wp_idx_last = m_TrackWaypoints.size() - 1;
    for (size_t wp_idx_curr = 0; wp_idx_curr < m_TrackWaypoints.size(); wp_idx_curr++) {
        double wp_dist = (m_TrackWaypoints[wp_idx_curr] - m_TrackWaypoints[wp_idx_last]).norm();
        m_Expert_MaxHorizon = std::max(m_Expert_MaxHorizon, wp_dist);
        wp_idx_last = wp_idx_curr;
    }
}


void ForgetfulDrone::buildSimulation (forgetful_drone::ForgetfulSimulator& fs) {
    fdBS srv;   srv.request.unity_scene = m_UnitySceneIdx;
                srv.request.scene_site = m_SceneSiteIdx;
                srv.request.track_type = m_TrackTypeIdx;
                srv.request.track_generation = m_TrackGenerationIdx;
                srv.request.track_direction = m_TrackDirectionIdx;
                srv.request.gate_type = m_GateTypeIdx;
    while (!fs.init(srv)) {
        ros::Duration(1.0).sleep();
    }
    m_GateInitPoses = srv.response.gate_init_poses;
    m_DroneInitPose = srv.response.drone_init_pose;
}

void ForgetfulDrone::buildSimulation () {
    fdBS srv;   srv.request.unity_scene = m_UnitySceneIdx;
                srv.request.scene_site = m_SceneSiteIdx;
                srv.request.track_type = m_TrackTypeIdx;
                srv.request.track_generation = m_TrackGenerationIdx;
                srv.request.track_direction = m_TrackDirectionIdx;
                srv.request.gate_type = m_GateTypeIdx;


    if (m_SimulatorPtr) {
        while (!m_SimulatorPtr->init(srv)) {
            ros::Duration(1.0).sleep();
        }
    } else {
        m_rosSVC_SIMULATOR_BUILD = m_rosRNH.serviceClient<fdBS>("simulator/build");
        while (!callROSService<fdBS>(ROS_LOG_PREFIX, m_rosSVC_SIMULATOR_BUILD, srv)) {
            ros::Duration(1.0).sleep();
        }
    }

    m_GateInitPoses = srv.response.gate_init_poses;
    m_DroneInitPose = srv.response.drone_init_pose;
}

void ForgetfulDrone::setTrackWaypoints () {

    // Set waypoints of global trajectory from gate poses
    m_TrackWaypoints.clear(); 
    m_TrackWaypoints.reserve(m_GateInitPoses.size());
    for (const geometry_msgs::Pose& pose : m_GateInitPoses) {
        m_TrackWaypoints.push_back(EV3d_From_GMP(pose.position));
    }

    if (m_TrackTypeIdx == 1) // Gap
        insertGapWaypoint();

    //for (size_t i = 0; i < m_TrackWaypoints.size(); i++) {
    //    ROSWARN("Waypoint " << i + 1 << ": " << m_TrackWaypoints[i].transpose());
    //}
}

void ForgetfulDrone::insertGapWaypoint () {
    int i0 = static_cast<int>(m_TrackWaypoints.size() / 2) - 1;
    int i1 = i0 + 1;

    const Eigen::Vector3d& wp_0 = m_TrackWaypoints.back();
    const Eigen::Vector3d& wp_gap_0 = m_TrackWaypoints[i0];
    const Eigen::Vector3d& wp_gap_1 = m_TrackWaypoints[i1];

    Eigen::Vector3d wp_gap_c = (wp_gap_0 + wp_gap_1) / 2;
    

    Eigen::Vector3d direction = wp_gap_c - wp_0; 
    direction.normalize();
    double distance = (wp_gap_0 -wp_gap_1).norm();

    Eigen::Vector3d wp_gap = wp_gap_c + direction * distance / 2;

    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>::iterator it = m_TrackWaypoints.begin();
    m_TrackWaypoints.insert(it + i1, wp_gap);
}


void ForgetfulDrone::initWaypointIndices () {

    m_CurrWaypointIdx = m_GateInitPoses.size() - 1 - *ForgetfulSimulator::getDroneInitPoseNumGatesSetback();
    if (m_TrackTypeIdx == 1) {
        m_CurrWaypointIdx++; // easy fix, no time to code better
    }
    m_LastWaypointIdx = (m_CurrWaypointIdx + m_GateInitPoses.size() - 1) % m_GateInitPoses.size();
}

void ForgetfulDrone::setExpertMaxHorizon () {

    m_Expert_MaxHorizon = 0.0;
    size_t i0 = m_TrackWaypoints.size() - 1;
    
    for (size_t i1 = 0; i1 < m_TrackWaypoints.size(); i1++) {
        double distance = (m_TrackWaypoints[i1] - m_TrackWaypoints[i0]).norm();
        m_Expert_MaxHorizon = std::max(m_Expert_MaxHorizon, distance);
        i0 = i1;
    }
}



void ForgetfulDrone::insertITLWaypoint () {
    // Necessary since otherwise the trajectory through the gap building gates
    // is to sharp. Better would be to implement global traj generation
    // with constraints on angle at gate waypoints

    geometry_msgs::Pose _1 = m_GateInitPoses[2];
    geometry_msgs::Pose _2 = m_GateInitPoses[3];

    geometry_msgs::Pose _;
    _.position.x = (_1.position.x + _2.position.x) / 2;
    _.position.y = (_1.position.y + _2.position.y) / 2;
    _.position.z = (_1.position.z + _2.position.z) / 2;

    _1 = m_GateInitPoses.back();
    Eigen::Vector3d __ = {
        _.position.x - _1.position.x,
        _.position.y - _1.position.y,
        _.position.z - _1.position.z,
        };
    __.normalize();


    double ___ = (m_STR2VAL_MAP.at(p_INTERMEDIATE_TARGET_LOSS_GAP_TYPE) == BDRSR::NARROW_GAP)? 4:6;
    _.position.x += __.x() * ___;
    _.position.y += __.y() * ___;
    _.position.z += __.z() * ___;
    
    std::vector<geometry_msgs::Pose, std::allocator<geometry_msgs::Pose>>::iterator it = m_GateInitPoses.begin();

    m_GateInitPoses.insert(it + 3, _);
}

void ForgetfulDrone::startSimulation (ForgetfulSimulator& fs) {
    fdStartS srv;
    //while (!callROSService<fdStartS>(ROS_LOG_PREFIX, m_rosSVC_SIMULATOR_START, srv)) {
    //    ros::Duration(1.0).sleep();
    //}
    while (!fs.start(srv)) {
        ros::Duration(1.0).sleep();
    }
}

void ForgetfulDrone::startSimulation () {
    fdStartS srv;

    if (m_SimulatorPtr) {
        while (!m_SimulatorPtr->start(srv)) {
            ros::Duration(1.0).sleep();
        }
    } else {
        m_rosSVC_SIMULATOR_START = m_rosRNH.serviceClient<fdStartS>("simulator/start", 1);
        while (!callROSService<fdStartS>(ROS_LOG_PREFIX, m_rosSVC_SIMULATOR_START, srv)) {
            ros::Duration(1.0).sleep();
        }
    }
}


void ForgetfulDrone::stopSimulation (ForgetfulSimulator& fs) {
    fdStopS srv;
    //while (!callROSService<fdStopS>(ROS_LOG_PREFIX, m_rosSVC_SIMULATOR_STOP, srv)) {
    //    ros::Duration(1.0).sleep();
    //}
    while (!fs.stop(srv)) {
        ros::Duration(1.0).sleep();
    }
}

void ForgetfulDrone::stopSimulation () {
    fdStopS srv;
    
    if (m_SimulatorPtr) {
        while (!m_SimulatorPtr->stop(srv)) {
            ros::Duration(1.0).sleep();
        }
    } else {
        m_rosSVC_SIMULATOR_STOP = m_rosRNH.serviceClient<fdStopS>("simulator/stop", 1);
        while (!callROSService<fdStopS>(ROS_LOG_PREFIX, m_rosSVC_SIMULATOR_STOP, srv)) {
            ros::Duration(1.0).sleep();
        }
    }
}


bool ForgetfulDrone::runNavigation () {
    ROSINFO("Start navigation");
    playAudioFromText("Start - Navigation.");
    

    switchNavigator(true);
    m_rosTMR_MAINLOOP.start();
    std::thread t1(runMultiThreadedSpinner);
    m_WpArrLastTime = ros::Time::now();  
    ros::Rate rate(1.0);
    
    do {
        rate.sleep();
        ros::Duration time_passed = ros::Time::now() - m_WpArrLastTime;
        int time_left = p_WPARR_MAXDUR - static_cast<int>(time_passed.toSec());
        ROSINFO(" - Time left to pass waypoint: " << time_left << "/" << p_WPARR_MAXDUR << " s");
        
        if (time_left < 0) {
            ROSERROR("No waypoint passed in " << p_WPARR_MAXDUR << " s -> abort run");
            switchNavigator (false);
            break;
        }
    } while (m_RunLapCnt < m_RunNumLaps && m_NavigatorENABLED);

    m_DataSavingENABLED = false;
    
    m_rosTMR_MAINLOOP.stop();
    t1.detach();
    t1.~thread();

    if (m_NavigatorENABLED) {
        ROSINFO("Run " << m_TotalRunCnt + 1 << " succeeded");
        switchNavigator (false);
        return true;
    } else {
        ROSWARN ("Run " << m_TotalRunCnt + 1 << " aborted ahead completion");
        return false;
    }
}



void ForgetfulDrone::performFlightMission_DAGGER () {
    logFlightMissionInfo();
    m_TotalRunCnt = 0;
    m_SimulatorPtr = nullptr;
    initMainLoopTimer(&ForgetfulDrone::ROSCB_DAGGER_PYTHON);


    bool drone_launched {false};

    for (const int& unity_scene : p_FLIGHTMISSION_UNITYSCENES)
    for (const int& scene_site : p_FLIGHTMISSION_SCENESITES)
    for (const int& track_type : p_FLIGHTMISSION_TRACKTYPES)
    for (const int& track_generation : p_FLIGHTMISSION_TRACKGENERATIONS)
    for (const int& track_direction : p_FLIGHTMISSION_TRACKDIRECTIONS)
    for (const int& gate_type : p_FLIGHTMISSION_GATETYPES) 
    for (const double& loctraj_maxspeed: p_LOCTRAJ_MAXSPEEDS)
    for (const double& dagger_margin: p_DAGGERMARGINS) {
        

        m_UnitySceneIdx = static_cast<uint8_t>(unity_scene);
        m_SceneSiteIdx = static_cast<uint8_t>(scene_site);
        m_TrackTypeIdx = static_cast<uint8_t>(track_type);
        m_TrackGenerationIdx = static_cast<uint8_t>(track_generation);
        m_TrackDirectionIdx = static_cast<uint8_t>(track_direction);
        m_GateTypeIdx = static_cast<uint8_t>(gate_type);
        m_LocTrajMaxSpeed = loctraj_maxspeed;
        m_rosRNH.setParam("brain_MAX_SPEED", m_LocTrajMaxSpeed);


        m_DaggerRepCnt = 0;
        bool repeat_run_config {true};
        while (repeat_run_config) {
            playAudioFromText("Start - Run - " + std::to_string(m_TotalRunCnt));

            if (m_TotalRunCnt == 0) {
                m_DaggerMargin = 0;
                initMainLoopTimer(&ForgetfulDrone::ROSCB_DAGGER_FIRST_RUN);
                m_RunNumLaps = p_DAGGER_FIRSTRUN_NUMLAPS;
            }
            else {
                m_DaggerMargin = dagger_margin;
                initMainLoopTimer(&ForgetfulDrone::ROSCB_DAGGER_PYTHON);
                m_RunNumLaps = p_RUN_NUMLAPS;
            }

            std::string spacing = "    ";
            ROSINFO(std::endl
                << "Run #" << m_TotalRunCnt << ":\n"
                << " - Unity scene: " << std::get<0>(ForgetfulSimulator::getUnityScenes()->at(m_UnitySceneIdx)) << spacing
                << " - Scene site: " << std::get<0>(ForgetfulSimulator::getSceneSites()->at(m_SceneSiteIdx)) << spacing
                << " - Track type: " << std::get<0>(ForgetfulSimulator::getTrackTypes()->at(m_TrackTypeIdx)) << std::endl
                << " - Track generation: " << std::get<0>(ForgetfulSimulator::getTrackGenerations()->at(m_TrackGenerationIdx)) << spacing
                << " - Track direction: " << std::get<0>(ForgetfulSimulator::getTrackDirections()->at(m_TrackDirectionIdx)) << spacing
                << " - Gate types: " << std::get<0>(ForgetfulSimulator::getGateTypes()->at(m_GateTypeIdx)) << std::endl
                << " - Max speed: " << m_LocTrajMaxSpeed << " m/s" << spacing
                << " - Dagger margin: " << m_DaggerMargin << " m" << spacing
                << " - Repetition: #" << m_DaggerRepCnt
            );
            reset4NewRun();

            if (isDirectory(RunDPath())) {
                std::string fpath = OutputDpath() + "/" + h_RECORDINGS_FNAME;
                std::ifstream ifs (fpath);
                nlohmann::json jf = nlohmann::json::parse(ifs);
                double exp_iv_share = jf["expert_intervention_share"][m_TotalRunCnt];
                if (exp_iv_share > p_DAGGER_EXPERT_INTERVENTION_SHARE_THRESHOLD) {
                    ROSINFO(
                        "This run configuration was performed with " 
                        << 100*exp_iv_share <<" % (> " << 100*p_DAGGER_EXPERT_INTERVENTION_SHARE_THRESHOLD << " %) expert interventions -> Repetition required");
                    m_DaggerRepCnt ++;
                } else {
                    ROSINFO(
                        "This run configuration was performed with " 
                        << 100*exp_iv_share <<" % (<= " << 100*p_DAGGER_EXPERT_INTERVENTION_SHARE_THRESHOLD << " %) expert interventions -> No repetition required");
                    repeat_run_config = false;
                }

                m_TotalRunCnt ++;
                continue;
            }

            initRunDirectory ();
            
            do {
                buildSimulation ();
                setTrackWaypoints ();
                ros::Duration(1.0).sleep ();
            } while (!computeGlobalTrajectory ());

            initWaypointIndices ();
            setExpertMaxHorizon ();
            startSimulation ();
            rvizGloTraj ();

            if (!drone_launched) {
                launchDroneOffGround ();
                drone_launched = true;
            }
            flyDroneToInitPose ();

            if (m_TotalRunCnt > 0) {
                if (p_TORCHINTERFACE_IDX == 0) {
                    
                    std_msgs::String lc_msg; lc_msg.data = p_EXPERIMENT_ID;
                    m_rosPUB_BRAIN_LOADCHECKPOINT.publish(lc_msg);
                    ros::Duration(1.0).sleep();
                    

                    std_msgs::Bool ei_msg; ei_msg.data = true;
                    m_rosPUB_BRAIN_ENABLEINFERENCE.publish(ei_msg);
                    ros::Duration(1.0).sleep();
                }
                if (p_TORCHINTERFACE_IDX == 1) {
                    m_TorchScriptModule = torch::jit::load (ScriptModuleFpath());
                    m_TorchScriptModule.to(torch::Device(m_TorchDevice, 0));

                    m_ANNGRUHiddenState = std::vector<float>(
                        p_ANN_GRU_NUMLAYERS * h_BATCHSIZE * p_ANN_GRU_HIDDENSIZE, 0.0);
                }
            }



            bool track_completed = runNavigation();
            stopSimulation ();
            flyDroneAboveTrack ();

            if (!track_completed) {
                playAudioFromText("Run failed. Delete data and repeat run.");
                ROSINFO("Expert failed to complete the track -> delete data & repeat run.");
                std::experimental::filesystem::remove_all (m_RunDpath);
            } else {
                playAudioFromText("Train - A.N.N.");
                std_srvs::Empty srv;
                callROSService(ROS_LOG_PREFIX, m_rosSVC_BRAIN_TRAIN_ANN, srv);

                std::string fpath = OutputDpath() + "/" + h_RECORDINGS_FNAME;
                std::ifstream ifs (fpath);
                nlohmann::json jf = nlohmann::json::parse(ifs);
                double exp_iv_share = jf["expert_intervention_share"][m_TotalRunCnt];

                auto logExpInterv = [this, exp_iv_share] (const std::string& comp_op) {
                    std::stringstream ss; ss << std::setprecision(5)
                        << "Expert interventions: " << 100*exp_iv_share << " % (" << comp_op << " "
                        << 100 * p_DAGGER_EXPERT_INTERVENTION_SHARE_THRESHOLD << " %)";
                    return ss.str();
                };
                
                if (exp_iv_share > p_DAGGER_EXPERT_INTERVENTION_SHARE_THRESHOLD) {
                    ROSINFO(logExpInterv(">") << " -> aggregate more training data for this configuration.");
                } else {
                    ROSINFO(logExpInterv("<=") << " -> completed data aggregation for this configuration.");
                    repeat_run_config = false;
                }

                m_DaggerRepCnt ++;
                m_TotalRunCnt ++;
            }
        }
    }

    landDroneOnGround();
    drone_launched = false;
    ROSINFO("Completed flight mission: " << h_FLIGHTMISSIONS[p_FLIGHTMISSION]);
}


void ForgetfulDrone::performFlightMission_TrainingDataGeneration () {
    logFlightMissionInfo();
    m_TotalRunCnt = 0;
    m_SimulatorPtr = nullptr;
    m_RunNumLaps = p_RUN_NUMLAPS;
    initMainLoopTimer(&ForgetfulDrone::ROSCB_TRAINING_DATA_GENERATION);


    
    

    

    //const size_t& scene_n = h_UNITY_SCENES.size();
    const size_t& site_n = h_SCENE_SITES.size();
    const size_t& gate_n = h_GATE_TYPES.size();
    size_t rep_n = p_REPEATED_SETUP_RUN_N;
    size_t dir_n = m_ITL_DIRECTION_STRS.size();
    size_t gap_n = m_ITL_GAP_TYPE_STRS.size();

    switch (m_STR2VAL_MAP.at(p_RACETRACK_TYPE)) {
        case BDRSR::FIGURE8_DETERMINISTIC:
        case BDRSR::FIGURE8_RANDOMIZED:
            dir_n = 1;
            gap_n = 1;
            break;
        case BDRSR::INTERMEDIATE_TARGET_LOSS:
            rep_n = 1;
            break;
        default:
            break;
    }

    const size_t run_n = site_n * gate_n * rep_n * dir_n * gap_n;
    m_TotalRunCnt = 0;





    for (size_t site_i = 0; site_i < site_n; site_i++)
    for (size_t gate_i = 0; gate_i < gate_n; gate_i++)
    for (size_t rep_i = 0; rep_i < rep_n; rep_i++)
    for (size_t dir_i = 0; dir_i < dir_n; dir_i++)
    for (size_t gap_i = 0; gap_i < gap_n; gap_i++) {

        const std::string& site_str = h_SCENE_SITES[site_i];
        const std::string& gate_str = h_GATE_TYPES[gate_i];
        const std::string& dir_str = m_ITL_DIRECTION_STRS[dir_i];
        const std::string& gap_str = m_ITL_GAP_TYPE_STRS[gap_i];

        reset4NewRun();
        logRunInfo(
            m_TotalRunCnt, run_n,
            p_UNITY_SCENE, site_str,
            p_RACETRACK_TYPE, gate_str,
            gap_str,
            dir_str
        );

    

        createRunDirectory(
            m_STR2VAL_MAP.at(p_UNITY_SCENE),
            site_i,
            gate_i,
            rep_i,
            dir_i,
            gap_i
        );                
                

        
        do {
            buildSimulation(
                p_RACETRACK_TYPE,
                p_UNITY_SCENE,
                site_str,
                gate_str,
                dir_str,
                gap_str
            );
            ros::Duration(1.0).sleep();
        } while (!computeGlobalTrajectory());

        startSimulation();
        rvizGloTraj();
                

                if (m_TotalRunCnt == 0) launchDroneOffGround();
                else flyDroneToInitPose();


                runNavigation();

                stopSimulation();
                flyDroneAboveTrack();
                m_TotalRunCnt++;
    }
    
    landDroneOnGround();
    logFlightMissionResults();
}




//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////// ROS TIMER FUNCTIONS //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



void ForgetfulDrone::ROSCB_NAVIGATION_BY_EXPERT (const ros::TimerEvent& te) {
    checkROSTimerPeriodTime(ROS_LOG_PREFIX, te, 1 / p_MAIN_LOOP_FREQUENCY);
    
    runWaypointUpdater();
    runExpert();
    m_NavigatorInput = m_ExpertOutput;
    bool _;
    runNavigator(_, &ForgetfulDrone::doNothing);
}

void ForgetfulDrone::ROSCB_NAVIGATIONBYANN_CPP (const ros::TimerEvent& te) {
    checkROSTimerPeriodTime(ROS_LOG_PREFIX, te, 1 / p_MAIN_LOOP_FREQUENCY);
    
    runWaypointUpdater();

    setRGBData();
    setIMUData();
    
    runBrain();
    m_NavigatorInput = m_BrainOutput;

    bool _;
    runNavigator(_, &ForgetfulDrone::doNothing);

    if (p_RVIZ_LABELEDRGB_ENABLED) rvizLabeledCamFrame();
}

void ForgetfulDrone::ROSCB_NAVIGATION_BY_ANN_PYTHON (const ros::TimerEvent& te) {
    checkROSTimerPeriodTime(ROS_LOG_PREFIX, te, 1 / p_MAIN_LOOP_FREQUENCY);
    runWaypointUpdater();
    
    setRGBData();
    setIMUData();
    
    triggerBrain();
    m_NavigatorInput = m_BrainOutput;

    bool _;
    runNavigator(_, &ForgetfulDrone::doNothing);
    
    if (p_RVIZ_LABELEDRGB_ENABLED) rvizLabeledCamFrame();
}

void ForgetfulDrone::ROSCB_TRAINING_DATA_GENERATION (const ros::TimerEvent& te) {
    checkROSTimerPeriodTime(ROS_LOG_PREFIX, te, 1 / p_MAIN_LOOP_FREQUENCY);
    
    runWaypointUpdater();
    runExpert();
    m_NavigatorInput = m_ExpertOutput;
    runDataSaver(true);
    bool _;
    runNavigator(_, &ForgetfulDrone::doNothing);
}

void ForgetfulDrone::setRGBData () {
    m_RGBMtx.lock ();
        m_RGBData = m_RGBPtr->image.clone();
        m_RGBTimeIncrement = m_RGBPtr->header.stamp.toSec() - m_RGBLastStampTime;
        m_RGBLastStampTime = m_RGBPtr->header.stamp.toSec();
    m_RGBMtx.unlock ();
}

void ForgetfulDrone::setIMUData () {
    m_IMUMtx.lock();
        m_IMUData = {
            m_IMUPtr->linear_acceleration.x,
            m_IMUPtr->linear_acceleration.y,
            m_IMUPtr->linear_acceleration.z,
            m_IMUPtr->angular_velocity.x,
            m_IMUPtr->angular_velocity.y,
            m_IMUPtr->angular_velocity.z,
        };
        m_IMUTimeIncrement = m_IMUPtr->header.stamp.toSec() - m_IMULastStampTime;
        m_IMULastStampTime = m_IMUPtr->header.stamp.toSec();
    m_IMUMtx.unlock();
}

void ForgetfulDrone::setCtrlCmdData () {
    m_CtrlCmdMtx.lock();
    m_CtrlCmdData = {
        m_CtrlCmdPtr->bodyrates.x,
        m_CtrlCmdPtr->bodyrates.y,
        m_CtrlCmdPtr->bodyrates.z,
        m_CtrlCmdPtr->angular_accelerations.x,
        m_CtrlCmdPtr->angular_accelerations.y,
        m_CtrlCmdPtr->angular_accelerations.z,
        m_CtrlCmdPtr->collective_thrust
    };
    m_CtrlCmdMtx.unlock();
}



void ForgetfulDrone::ROSCB_DAGGER_PYTHON (const ros::TimerEvent& te) {
    checkROSTimerPeriodTime(ROS_LOG_PREFIX, te, 1 / p_MAIN_LOOP_FREQUENCY); // or m_SimulatorPtr->spinOnce (te);

    runWaypointUpdater();

    setRGBData();
    setIMUData();
    setCtrlCmdData();

    triggerBrain();
    m_NavigatorInput = m_BrainOutput;
    
    runExpert();
    bool exp_intervened {false};
    runNavigator(exp_intervened, &ForgetfulDrone::interveneBrainDecisionWithExpert);

    runDataSaver(exp_intervened);
    
    if (p_RVIZ_LABELEDRGB_ENABLED) rvizLabeledCamFrame();
}


void ForgetfulDrone::ROSCB_DAGGER_FIRST_RUN (const ros::TimerEvent& te) {
    checkROSTimerPeriodTime(ROS_LOG_PREFIX, te, 1 / p_MAIN_LOOP_FREQUENCY); // or m_SimulatorPtr->spinOnce (te);

    runWaypointUpdater();

    setRGBData();
    setIMUData();
    setCtrlCmdData();

    m_NavigatorInput = {0.0, 0.0, 0.0};
    
    runExpert();
    bool exp_intervened {false};
    runNavigator(exp_intervened, &ForgetfulDrone::interveneBrainDecisionWithExpert_firstRun);

    runDataSaver(exp_intervened);
    
    if (p_RVIZ_LABELEDRGB_ENABLED) rvizLabeledCamFrame();
}






//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////// SIMULATOR //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



/*
void ForgetfulDrone::switchDynamicGates( const bool& Enabled )
{
        std_msgs::Bool msg; 
        msg.data = Enabled; 
    m_ROSPUB_SIMULATOR_DYNAMIC_GATES_SWITCH.publish( msg );
}
*/



//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////// GLOBAL TRAJECTORY //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////





bool ForgetfulDrone::computeGlobalTrajectory () {
    // Compute global trajectory
    ROSINFO("Compute Global Trajectory:\n"
        << " - #Waypoints: " << static_cast<int>(m_TrackWaypoints.size()) << "\n"
        << " - Poly.Order: " << p_GLOBAL_TRAJECTORY_POLYNOMIAL_ORDER << "\n"
        << " - Cont.Order: " << p_GLOBAL_TRAJECTORY_CONTINUITY_ORDER << "\n"
        << " - Max.Speed: " << p_GLOBAL_TRAJECTORY_MAX_SPEED << " m/s\n"
        << " - Max.Thrust: " << p_GLOBAL_TRAJECTORY_MAX_THRUST << " m/s^2\n"
        << " - Max.RP-Rate: " << p_GLOBAL_TRAJECTORY_MAX_ROLL_PITCH_RATE << " 1/s\n"
        << " - Sampl.Time: " << 1 / p_MAIN_LOOP_FREQUENCY << " s"
    );
    std::shared_ptr<ForgetfulGlobalTrajectory<long double>> gt_ptr
        = std::make_shared<ForgetfulGlobalTrajectory<long double>>(
            m_TrackWaypoints, 
            p_GLOBAL_TRAJECTORY_POLYNOMIAL_ORDER, 
            p_GLOBAL_TRAJECTORY_CONTINUITY_ORDER, 
            p_GLOBAL_TRAJECTORY_MAX_SPEED, 
            p_GLOBAL_TRAJECTORY_MAX_THRUST, 
            p_GLOBAL_TRAJECTORY_MAX_ROLL_PITCH_RATE, 
            1 / p_MAIN_LOOP_FREQUENCY,
            3, 
            p_GLOBAL_TRAJECTORY_NON_DIMENSIONAL_TEMPORAL_RANGE, 
            p_GLOBAL_TRAJECTORY_NON_DIMENSIONAL_SPATIAL_RANGE, 
            false
        );
    if (!gt_ptr->m_Successful) return false;
    
    // Set global trajectory points
    m_GloTraj.clear(); 
    m_GloTraj.reserve(gt_ptr->m_Traj_T_Dim.size());
    for (size_t i = 0; i < static_cast<size_t>(gt_ptr->m_Traj_T_Dim.size()); i++) {
        quadrotor_common::TrajectoryPoint tp;
        tp.position.x() = static_cast<double>(gt_ptr->m_Traj_PosX_Dim(i));
        tp.position.y() = static_cast<double>(gt_ptr->m_Traj_PosY_Dim(i));
        tp.position.z() = static_cast<double>(gt_ptr->m_Traj_PosZ_Dim(i));
        tp.velocity.x() = static_cast<double>(gt_ptr->m_Traj_VelX_Dim(i));
        tp.velocity.y() = static_cast<double>(gt_ptr->m_Traj_VelY_Dim(i));
        tp.velocity.z() = static_cast<double>(gt_ptr->m_Traj_VelZ_Dim(i));
        m_GloTraj.push_back(tp);
    }

    // Set max and min speed along global trajectory.
    m_Expert_MaxSpeed = 0.0;
    m_Expert_MinSpeed = std::numeric_limits<double>::max();
    for (size_t i = 0; i < m_GloTraj.size(); i++) {
        double speed = m_GloTraj[i].velocity.norm();
        m_Expert_MaxSpeed = std::max(m_Expert_MaxSpeed, speed);
        m_Expert_MinSpeed = std::min(m_Expert_MinSpeed, speed);
    }


    ROSINFO("Global Trajectory:"
        << " - #States " << static_cast<int>(m_GloTraj.size()) << "\n"
        << " - Duration " << static_cast<int>(m_GloTraj.size()) / p_MAIN_LOOP_FREQUENCY << " s\n"
        << " - Speed Range [" << static_cast<double>(gt_ptr->m_Traj_MinSpeed) << ", " << static_cast<double>(gt_ptr->m_Traj_MaxSpeed) << "] m/s\n"
        << " - Thrust Range [" << static_cast<double>(gt_ptr->m_Traj_MinThrust) << ", " << static_cast<double>(gt_ptr->m_Traj_MaxThrust) << "] m/s^2\n"
        << " - RP-Rate Range [" << static_cast<double>(gt_ptr->m_Traj_MinRollPitchRate) << ", " << static_cast<double>(gt_ptr->m_Traj_MaxRollPitchRate) << "] 1/s"
    );

    // return whether computation was successful
    return true;
}






void ForgetfulDrone::rvizGloTraj()
{
    visualization_msgs::Marker marker = getTrajMarker();
    setRGBOfVisMarker(VisualizationColors::PURPLE, marker);
    marker.id = 0;

    for (const quadrotor_common::TrajectoryPoint& tp : m_GloTraj) {
        marker.points.push_back(GMPoint__from__EV3d(tp.position));
        marker.points.push_back(GMPoint__from__EV3d(tp.position + tp.velocity/20.0));  
    }

    m_rosPUB_RVIZ_GLOBAL_TRAJECTORY.publish(marker);
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////// WAYPOINT UPDATER //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


void ForgetfulDrone::runWaypointUpdater () {
    // Distance from reference state to current and last gate waypoint.
    m_Dist2CurrWaypoint = (m_RefStatePos_WRF - m_TrackWaypoints[m_CurrWaypointIdx]).norm();
    m_Dist2LastWaypoint = (m_RefStatePos_WRF - m_TrackWaypoints[m_LastWaypointIdx]).norm();
    
    if (m_Dist2CurrWaypoint < p_WAYPOINT_ARRIVAL_THRESHOLD_DISTANCE) {
        runWaypointUpdater_updateWaypointIndices();
    }

    runWaypointUpdater_rvizCurrWaypoint();
}


void ForgetfulDrone::runWaypointUpdater_updateWaypointIndices()
{
    ++m_CurrWaypointIdx %= m_TrackWaypoints.size();
    m_LastWaypointIdx = (m_CurrWaypointIdx + m_TrackWaypoints.size() - 1) % m_TrackWaypoints.size();
    m_WpArrLastTime = ros::Time::now();

    ROSINFO("Pass gate " << m_LastWaypointIdx + 1 << "/" << m_TrackWaypoints.size());

    if (m_CurrWaypointIdx == 0) {
        m_RunLapCnt++;
        ROSINFO("Start lap " << m_RunLapCnt + 1 << "/" << m_RunNumLaps);
    }
}

void ForgetfulDrone::runWaypointUpdater_rvizCurrWaypoint () {
    rvizPosition(
        m_TrackWaypoints[m_CurrWaypointIdx], 
        VisPosTypes::CURRGATE, 
        m_rosPUB_RVIZ_NAVIGATION_POINTS
    );
}





//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////// NAVIGATOR //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


void ForgetfulDrone::doNothing (Eigen::Vector3d& _0, double& _1, bool& _2) {
    return;
}

void ForgetfulDrone::interveneBrainDecisionWithExpert (Eigen::Vector3d& target_pos_ARF, double& speed_to_target, bool& exp_intervened) {

    // Set position of reference state in world RF
    geometry_msgs::Pose target_pose_ARF;
        target_pose_ARF.position = GMPoint__from__EV3d (target_pos_ARF);
        target_pose_ARF.orientation.w = 1.0;
        target_pose_ARF.orientation.x = 0.0;
        target_pose_ARF.orientation.y = 0.0;
        target_pose_ARF.orientation.z = 0.0;
    kindr::minimal::QuatTransformation ARF_TRF; 
    tf::poseMsgToKindr (target_pose_ARF, &ARF_TRF);
    Eigen::Vector3d target_pos_WRF = (p_T_WRF_ARF * ARF_TRF).getPosition();
    
    double dist_2_glotraj; size_t _;
    findDistance2GlobalTrajectory(target_pos_WRF, dist_2_glotraj, _);
    
    if (dist_2_glotraj > m_DaggerMargin * (m_LocTrajMaxSpeed / 5.0 + 1.0/5.0)) {
        m_NavigatorInput = m_ExpertOutput;
        processNavigatorInput(target_pos_ARF, speed_to_target);
        if (m_RunLapCnt >= 0) {
            m_ExpertInterventionsCnt++;
            exp_intervened = true;
        }
    } else {
        if (m_RunLapCnt >= 0) {
            m_BrainDecisionsCnt++;
        }
    }
}


void ForgetfulDrone::interveneBrainDecisionWithExpert_firstRun (Eigen::Vector3d& target_pos_ARF, double& speed_to_target, bool& exp_intervened) {

    // Set position of reference state in world RF
    geometry_msgs::Pose target_pose_ARF;
        target_pose_ARF.position = GMPoint__from__EV3d (target_pos_ARF);
        target_pose_ARF.orientation.w = 1.0;
        target_pose_ARF.orientation.x = 0.0;
        target_pose_ARF.orientation.y = 0.0;
        target_pose_ARF.orientation.z = 0.0;
    kindr::minimal::QuatTransformation ARF_TRF; 
    tf::poseMsgToKindr (target_pose_ARF, &ARF_TRF);
    Eigen::Vector3d target_pos_WRF = (p_T_WRF_ARF * ARF_TRF).getPosition();
    
    double dist_2_glotraj; size_t _;
    findDistance2GlobalTrajectory(target_pos_WRF, dist_2_glotraj, _);
    
    if (dist_2_glotraj > m_DaggerMargin * (m_LocTrajMaxSpeed / 5.0 + 1.0/5.0)) {
        m_NavigatorInput = m_ExpertOutput + Eigen::Vector3d::Ones() * p_NAV_INPUTPERTURBATION_ELEMENTWISEAMP * std::sin(0.5*ros::WallTime::now().toSec());
        processNavigatorInput(target_pos_ARF, speed_to_target);
        m_ExpertInterventionsCnt++;
        exp_intervened = true;
    } else {
        m_BrainDecisionsCnt++;
    }
}

void ForgetfulDrone::runNavigator (bool& intervened, void (forgetful_drone::ForgetfulDrone::*intervention_fct)(Eigen::Vector3d& _0, double& _1, bool& _2)) {
    if (!m_NavigatorENABLED) {
        ROSINFO("Wait for enabled Navigator."); ros::Duration(1.0).sleep();
        return;
    }

    double div_from_refstate = (m_RefState_ARF.position - m_T_ARF_DRF.getPosition()).norm();
    if (p_NAV_REFSTATE_MAXDIVERGENCE < div_from_refstate) {
        ROSERROR("Drone position estimate diverged " << div_from_refstate << ">" << p_NAV_REFSTATE_MAXDIVERGENCE << "(max) m from autopilot reference position.");
        switchNavigator(false);
        return;
    }

    if (m_MainLoopIterCnt % p_NAV_REPLANNING_MAINLOOPITERSCNT == 0) {

        // Compute goal position and speed to reach goal position from navigator input
        Eigen::Vector3d target_pos_ARF; double speed_to_target;
        processNavigatorInput(target_pos_ARF, speed_to_target);


        (this->*intervention_fct)(target_pos_ARF, speed_to_target, intervened);


        // Compute local trajectory
        if (computeLocalTrajectory (target_pos_ARF, speed_to_target)) {
            m_LocTrajSubseqInfeasibleCnt = 0;
            m_LocTrajStartTime = ros::Time::now();
            rvizLocTraj();
        } 
        else {
            if (m_LocTrajSubseqInfeasibleCnt < p_NAV_LOCTRAJ_FEASIBILITY_MAXSUCCESSIVEFAILSCNT) {
                m_LocTrajSubseqInfeasibleCnt ++;
            } 
            else {
                ROSERROR(p_NAV_LOCTRAJ_FEASIBILITY_MAXSUCCESSIVEFAILSCNT
                    << "(max) subsequent local trajectories been infeasible.");
                switchNavigator(false);
                return;
            }
        }
    }
    

    publishReferenceState2Autopilot();
    rvizReferenceState();
    m_MainLoopIterCnt++;
}












void ForgetfulDrone::rvizReferenceState () {
    rvizState(
        m_RefStatePos_WRF, m_RefState_ARF.acceleration, 
        VisPosTypes::REFERENCE, m_rosPUB_RVIZ_NAVIGATION_POINTS);
}

void ForgetfulDrone::switchNavigator (const bool& enabled) {

    std::string last_state = m_NavigatorENABLED? "ENABLED" : "DISABLED";

    auto pubCurrPoseAsAutopilotRefState = [this] () {
        // To use autopilot in REFERENCE_CONTROL state,
        // the first published reference state must be close to current pose.
        m_RefState_ARF = quadrotor_common::TrajectoryPoint();
        m_RefState_ARF.position = m_T_ARF_DRF.getPosition();
        m_RefState_ARF.heading = Yaw_From_EQd(m_T_ARF_DRF.getEigenQuaternion().normalized());
        m_rosPUB_AUTOPILOT_REFERENCE_STATE.publish(m_RefState_ARF.toRosMessage());
        ros::Duration(1.0).sleep();
    };

    auto switchENABLED = [last_state, pubCurrPoseAsAutopilotRefState, this] () {
        pubCurrPoseAsAutopilotRefState();
        m_NavigatorENABLED = true;
        ROSINFO("Switch navigator from " << last_state << " to " << "ENABLED");
        ros::Duration(1.0).sleep();
    };

    auto switchDISABLED = [last_state, pubCurrPoseAsAutopilotRefState, this] () {
        m_NavigatorENABLED = false;
        pubCurrPoseAsAutopilotRefState();
        ROSINFO("Switch navigator from " << last_state << " to " << "DISABLED");
        ros::Duration(1.0).sleep();
    };

    enabled? switchENABLED() : switchDISABLED();
}


void ForgetfulDrone::publishReferenceState2Autopilot()
{
    // Get time of current state of current local trajectory
    const double t = (ros::Time::now() - m_LocTrajStartTime).toSec() + 1 / p_MAIN_LOOP_FREQUENCY;
    
    // Set reference state from local trajectory
    m_RefState_ARF = quadrotor_common::TrajectoryPoint();
    m_RefState_ARF.position        = EigenVector3d_From_Vec3(m_LocTraj.GetPosition(t));
    m_RefState_ARF.velocity        = EigenVector3d_From_Vec3(m_LocTraj.GetVelocity(t));
    m_RefState_ARF.acceleration    = EigenVector3d_From_Vec3(m_LocTraj.GetAcceleration(t));
    m_RefState_ARF.jerk            = EigenVector3d_From_Vec3(m_LocTraj.GetJerk(t));
    m_RefState_ARF.snap            = {};
    m_RefState_ARF.heading = std::atan2(m_RefState_ARF.velocity.y(), m_RefState_ARF.velocity.x());

    // Set position of reference state in world RF
    geometry_msgs::Pose ref_pose_ARF;
        ref_pose_ARF.position = GMPoint__from__EV3d (m_RefState_ARF.position);
        ref_pose_ARF.orientation.w = 1.0;
        ref_pose_ARF.orientation.x = 0.0;
        ref_pose_ARF.orientation.y = 0.0;
        ref_pose_ARF.orientation.z = 0.0;
    kindr::minimal::QuatTransformation ARF_RRF; 
    tf::poseMsgToKindr (ref_pose_ARF, &ARF_RRF);
    m_RefStatePos_WRF = (p_T_WRF_ARF * ARF_RRF).getPosition();
    
    //ROSDEBUG("Reference state (ARF),\n\tposition: [" 
    //    << m_RefState_ARF.position.x() << ", "
    //    << m_RefState_ARF.position.y() << ", "
    //    << m_RefState_ARF.position.z() << "]\n\theading:"
    //    << m_RefState_ARF.heading << "\n\tvelocity: [" 
    //    << m_RefState_ARF.velocity.x() << ", "
    //    << m_RefState_ARF.velocity.y() << ", "
    //    << m_RefState_ARF.velocity.z() << "]\n\tacceleration: [" 
    //    << m_RefState_ARF.acceleration.x() << ", "
    //    << m_RefState_ARF.acceleration.y() << ", "
    //    << m_RefState_ARF.acceleration.z() << "]");


    // Publish reference state to autopilot
    m_rosPUB_AUTOPILOT_REFERENCE_STATE.publish (m_RefState_ARF.toRosMessage());
}


void ForgetfulDrone::rvizLocTraj()
{
    visualization_msgs::Marker msg = getTrajMarker();

    setRGBOfVisMarker(VisualizationColors::GREEN, msg);

    msg.id = p_RVIZ_LOCAL_TRAJECTORY_DISPLAY_ELAPSED_ENABLED? 
        ++m_LocTrajFeasibleCnt : 0;

    double VisDuration 
        = (p_RVIZ_LOCAL_TRAJECTORY_DURATION == 0.0)? 
            m_LocTraj.GetEndTime() 
            : std::min(m_LocTraj.GetEndTime(), p_RVIZ_LOCAL_TRAJECTORY_DURATION);

    
    const Eigen::Quaterniond Q = p_T_WRF_ARF.getEigenQuaternion();
    const Eigen::Vector3d T = p_T_WRF_ARF.getPosition();

    for (double t = 0.0; t <= VisDuration; t += 1/p_RVIZ_LOCAL_TRAJECTORY_SAMPLING_FREQUENCY)
        {
            Eigen::Vector3d Pos = Q * (EigenVector3d_From_Vec3(m_LocTraj.GetPosition(t))) + T;
            Eigen::Vector3d Vel = Q * EigenVector3d_From_Vec3(m_LocTraj.GetVelocity(t));

            msg.points.push_back(GMPoint__from__EV3d(Pos));
            msg.points.push_back(GMPoint__from__EV3d(Pos + Vel/20.0));  
        }

    m_rosPUB_RVIZ_LOCAL_TRAJECTORY.publish(msg);
}


bool ForgetfulDrone::computeLocalTrajectory (
    /*Target position in reference frame of autopilot*/ const Eigen::Vector3d& target_pos_ARF, 
    /*Desired speed on way to target position*/ const double& speed_to_target
) {
    // In ARF, set start position, velocity, acceleration; end position; gravity.
    const Vec3 pos_0_ARF = Vec3_from_EV3d (m_RefState_ARF.position);
    const Vec3 vel_0_ARF = Vec3_from_EV3d (m_RefState_ARF.velocity);
    const Vec3 acc_0_ARF = Vec3_from_EV3d (m_RefState_ARF.acceleration);
    const Vec3 pos_1_ARF = Vec3_from_EV3d(target_pos_ARF);
    const Vec3 Gravity {0.0, 0.0, -9.81};

    // Linear distance of the trajectory
    const double lin_dist = (pos_1_ARF - pos_0_ARF).GetNorm2();

    // Actual speed on way to target position
    const double act_speed_to_target = std::min ({
        speed_to_target,
        vel_0_ARF.GetNorm2() + p_LOCAL_TRAJECTORY_MAX_SPEED_INCREMENT,
        m_LocTrajMaxSpeed
    });

    // Duration of the trajectory
    const double lin_duration = lin_dist / act_speed_to_target;

    // Compute trajectory
    RQTG::RapidTrajectoryGenerator loc_traj {pos_0_ARF, vel_0_ARF, acc_0_ARF, Gravity};
    loc_traj.SetGoalPosition (pos_1_ARF);
    loc_traj.Generate (lin_duration);
    
    // Take/discard computed trajectory if feasible/infeasible
    if (checkFeasibility(loc_traj)) {
        m_LocTraj = loc_traj;
        return true;
    } else {
        ROSWARN("Discard infeasible local trajectory.");
        return false;
    }
}



bool ForgetfulDrone::checkFeasibility (RQTG::RapidTrajectoryGenerator& lt) {
    using RTG = RQTG::RapidTrajectoryGenerator;

    // Input feasibility
    RTG::InputFeasibilityResult ifr = lt.CheckInputFeasibility (
        p_LOCAL_TRAJECTORY_MIN_THRUST, 
        p_LOCAL_TRAJECTORY_MAX_THRUST, 
        p_LOCAL_TRAJECTORY_MAX_BODY_RATES, 
        p_LOCAL_TRAJECTORY_INPUT_FEASIBILITY_CHECK_MIN_SAMPLING_TIME
    );

    // Position feasibility: check if trajectory violates min/max altitude
    Vec3 boundary_pnt {0.0, 0.0, p_LOCAL_TRAJECTORY_MIN_ALTITUDE};  
    Vec3 boundary_vec {0.0, 0.0, 1.0}; 
    RTG::StateFeasibilityResult sfr_floor
        = lt.CheckPositionFeasibility (boundary_pnt, boundary_vec);
    boundary_pnt[2] = p_LOCAL_TRAJECTORY_MAX_ALTITUDE;
    boundary_vec[2] = -1.0;
    RTG::StateFeasibilityResult sfr_ceiling 
        = lt.CheckPositionFeasibility (boundary_pnt, boundary_vec);


    if (ifr != RTG::InputFeasible) {
        ROSWARN("Local trajectory input-infeasible w.r.t. thrust range: [" 
            << p_LOCAL_TRAJECTORY_MIN_THRUST << ", " << p_LOCAL_TRAJECTORY_MAX_THRUST << "] m/s^2, max. body rates: "
            << p_LOCAL_TRAJECTORY_MAX_BODY_RATES << " 1/s. (Checked with min. sampling time: "
            << p_LOCAL_TRAJECTORY_INPUT_FEASIBILITY_CHECK_MIN_SAMPLING_TIME << " s.");
        return false;
    }
    if (sfr_floor != RTG::StateFeasible) {
        ROSWARN("Local trajectory position-infeasible w.r.t. min. altitude: "
            << p_LOCAL_TRAJECTORY_MIN_ALTITUDE << " m.");
        return false;
    }
    if (sfr_ceiling != RTG::StateFeasible) {
        ROSWARN("Local trajectory position-infeasible w.r.t. max. altitude: "
            << p_LOCAL_TRAJECTORY_MAX_ALTITUDE << " m.");
        return false;
    }

    return true;
}



void ForgetfulDrone::processNavigatorInput (
    Eigen::Vector3d& OUT_GoalPos_ARF, 
    double& OUT_Speed2Goal 
) {
    //ROSDEBUG("Navigator Input: [" << m_NavigatorInput.transpose() << "]");
    const double& GoalX_IRF = m_NavigatorInput.x();
    const double& GoalY_IRF = m_NavigatorInput.y();
    const double& NormSpeed2Goal = m_NavigatorInput.z();

    

    // Denormalize speed
    OUT_Speed2Goal = std::max(p_LOCAL_TRAJECTORY_MIN_SPEED, m_LocTrajMaxSpeed * NormSpeed2Goal); 
    //ROSDEBUG("Speed to Goal: " << OUT_Speed2Goal);

    

    // Compute distance from drone to goal position:
    double LT_LinDist = std::max(p_LOCAL_TRAJECTORY_MIN_DISTANCE, std::min(p_LOCAL_TRAJECTORY_MAX_DISTANCE, p_LOCAL_TRAJECTORY_DURATION * OUT_Speed2Goal)); 
    //ROSDEBUG("Linear distance to goal position: " << LT_LinDist);

    // Compute goal position
    OUT_GoalPos_ARF 
        = XYZ_ARF_From_XYZ_DRF( 
            XYZ_DRF_From_XYDist_IRF(GoalX_IRF, GoalY_IRF, LT_LinDist));
    //ROSDEBUG("Goal Position: [" 
    //    << OUT_GoalPos_ARF.x() << ", " 
    //    << OUT_GoalPos_ARF.y() << ", " 
    //    << OUT_GoalPos_ARF.z() << "]");
}



Eigen::Vector3d ForgetfulDrone::XYZ_DRF_From_XYDist_IRF( 
    const double& X_IRF, 
    const double& Y_IRF, 
    const double& Dist_IRF 
){
    // Compute yaw and pitch angle to position in reference frame of drone.
    const double Yaw = -p_DRONE_CAMERA_HALF_YAW_AOV * X_IRF;
    const double Pitch = p_DRONE_CAMERA_HALF_PITCH_AOV * Y_IRF;
    //ROSDEBUG("Yaw: "<< Yaw <<" (DRF), Pitch: " << Pitch << " (DRF)");

    // Compute vector in drone reference frame.
    return {
        Dist_IRF * cos (Pitch) * cos (Yaw),
        Dist_IRF * cos (Pitch) * sin (Yaw),
        Dist_IRF * sin (Pitch)
    };
}

Eigen::Vector3d ForgetfulDrone::XYZ_ARF_From_XYZ_DRF (const Eigen::Vector3d& pos_drf)
{
    geometry_msgs::Pose Pose_DRF;
        Pose_DRF.position = GMPoint__from__EV3d(pos_drf);
        Pose_DRF.orientation.w = 1.0;
        Pose_DRF.orientation.x = 0.0;
        Pose_DRF.orientation.y = 0.0;
        Pose_DRF.orientation.z = 0.0;
    kindr::minimal::QuatTransformation T_DRF_XYZ;
    tf::poseMsgToKindr(Pose_DRF, &T_DRF_XYZ);
    kindr::minimal::QuatTransformation T_ARF_XYZ = m_T_ARF_DRF * T_DRF_XYZ;
    
    return T_ARF_XYZ.getPosition();
}









//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////// EXPERT //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



void ForgetfulDrone::runExpert () {
    // Horizon: Distance 2 the closer one of current/last gate but within specified range
    double horizon = std::min(m_Dist2CurrWaypoint, m_Dist2LastWaypoint);
    horizon = capMinMax<double>(horizon, p_EXPERT_MIN_HORIZON, m_Expert_MaxHorizon);

    findExpertStateWithProjection(); 
    findHorizonState(horizon);
    rvizExpertAndHorizonState();

    Eigen::Vector2d target_pos_IRF = PosIRF_From_PosDRF(PosDRF_From_PosWRF(m_GloTraj[m_GT_HorizonState_i].position));

    findSpeedState();
    double norm_speed_2_target = m_GloTraj[m_GT_SpeedState_i].velocity.norm() / m_Expert_MaxSpeed;


    m_ExpertOutput = {
        target_pos_IRF.x(), target_pos_IRF.y(), norm_speed_2_target
    };
}



void ForgetfulDrone::triggerBrain () {
    m_rosPUB_BRAIN_TRIGGERINFERENCE.publish(std_msgs::Empty());
}






void ForgetfulDrone::runBrain () {
    if (isEmpty(ROS_LOG_PREFIX, m_RGBData)) return;
    
    //ROSDEBUG("Resize camera frame.");
    cv::Mat bgr_resized;
    cv::resize(
        m_RGBData, 
        bgr_resized, 
        cv::Size(p_DATA_PROCESSED_RGB_WIDTH, p_DATA_PROCESSED_RGB_HEIGHT)
    );

    //ROSDEBUG(GET_VAR_REP(bgr_resized.rows));
    //ROSDEBUG(GET_VAR_REP(bgr_resized.cols));
    

    //ROSDEBUG("Convert camera frame from BGR to RGB.");
    cv::Mat rgb_resized;
    cv::cvtColor(
        bgr_resized, 
        rgb_resized, 
        cv::COLOR_BGR2RGB
    );

    cv::Mat rgb_resized_normalized;
    rgb_resized.convertTo(rgb_resized_normalized, CV_32F, 1.0 / 255);


    at::Tensor x_rgb = torch::from_blob(
        rgb_resized_normalized.data,
        {h_SEQUENCE_LENGTH, h_BATCHSIZE, rgb_resized_normalized.rows, rgb_resized_normalized.cols, rgb_resized_normalized.channels()},
        m_TorchTensorOptions
    );
    x_rgb = x_rgb.permute({0, 1, 4, 2, 3});
    x_rgb = x_rgb.to(torch::Device(m_TorchDevice, 0));

    
    
    //ROSDEBUG("Create tensor from IMU data.");
    double cat_data[9] = {
        m_RGBTimeIncrement,
        m_IMUTimeIncrement,
        m_IMUData[0],
        m_IMUData[1],
        m_IMUData[2],
        m_IMUData[3],
        m_IMUData[4],
        m_IMUData[5],
        m_LocTrajMaxSpeed,
    };

    at::Tensor x_cat = torch::from_blob(
        cat_data,
        {h_SEQUENCE_LENGTH, h_BATCHSIZE, 8},
        m_TorchTensorOptions
    );
    x_cat = x_cat.to(torch::Device(m_TorchDevice, 0));

    //ROSDEBUG("Create tensor from hidden state.");
    at::Tensor h = torch::from_blob(
        m_ANNGRUHiddenState.data(), 
        {p_ANN_GRU_NUMLAYERS, h_BATCHSIZE, p_ANN_GRU_HIDDENSIZE},
        m_TorchTensorOptions
    );
    h = h.to(torch::Device(m_TorchDevice, 0));

    //ROSDEBUG("Create input vector from image, imu and hidden state tensor.");
    std::vector<torch::jit::IValue> input {x_rgb, x_cat, h};

    //ROSDEBUG("Compute output of ANN.");
    //at::Tensor Output = m_TorchScriptModule.forward(input).toTensor().cpu();
    auto output = m_TorchScriptModule.forward(input).toTuple();

    //ROSDEBUG("Identitfy prediction and hidden state from output.");
    torch::Tensor output_x = output->elements()[0].toTensor().cpu();
    torch::Tensor output_h = output->elements()[1].toTensor().cpu().view(
        {p_ANN_GRU_NUMLAYERS * h_BATCHSIZE * p_ANN_GRU_HIDDENSIZE}); 

    auto acc_h = output_h.accessor<float, 1>();
    for (int i = 0; i < acc_h.size(0); i++) {
        m_ANNGRUHiddenState[i] = acc_h[i];
    }
    //ROSDEBUG(GET_VAR_REP(m_ANNGRUHiddenState));
        
    
    auto acc_x = output_x.accessor<float, 2>();
    m_BrainOutput = {acc_x[0][0], acc_x[0][1], acc_x[0][2]};
    //ROSDEBUG(GET_VAR_REP(m_BrainOutput.transpose()));
}







void ForgetfulDrone::findExpertStateWithProjection () {
    // Find "expert state" with projection.
    Eigen::Vector3d ExpState_Direction;
    double ExpState_SpeedLvl;
    Eigen::Vector3d Exp2RefState_Direction;
    double ProjLvl; // "Projection level" of Exp2RefState_Direction onto ExpState_Direction



    ExpState_Direction
        = m_GloTraj[ m_GloTrajExpertStateIdx     ].position 
        - m_GloTraj[ m_GloTrajExpertStateIdx - 1 ].position;

    ExpState_SpeedLvl = ExpState_Direction.norm();

    ExpState_Direction /= ExpState_SpeedLvl;

    Exp2RefState_Direction
        = m_RefStatePos_WRF
        - m_GloTraj [ m_GloTrajExpertStateIdx - 1 ].position;

    ProjLvl = Exp2RefState_Direction.dot( ExpState_Direction );

    while ( ProjLvl > ExpState_SpeedLvl )
    {
        m_GloTrajExpertStateIdx++;
        m_GloTrajExpertStateIdx %= m_GloTraj.size();


        ExpState_Direction
            = m_GloTraj[ m_GloTrajExpertStateIdx     ].position 
            - m_GloTraj[ m_GloTrajExpertStateIdx - 1 ].position;

        ExpState_SpeedLvl = ExpState_Direction.norm();

        ExpState_Direction /= ExpState_SpeedLvl;

        Exp2RefState_Direction
            = m_RefStatePos_WRF
            - m_GloTraj [ m_GloTrajExpertStateIdx - 1 ].position;
        
        ProjLvl = Exp2RefState_Direction.dot( ExpState_Direction );
    }


    // If the distance from the "expert state" found with above projection
    // to the "reference state" of the autopilot is more than 1 m,
    // find "expert state" as state on global trajectory that has min distance to "reference state".
    double DivergenceFromGlobalTraj 
        = (m_RefStatePos_WRF - m_GloTraj[m_GloTrajExpertStateIdx].position).norm();
    
    if (DivergenceFromGlobalTraj > p_EXPERT_PROJECTION_MAX_DIVERGENCE_FROM_GLOBAL_TRAJECTORY) {
        double _;
        findMinDistanceStateIdx(_, m_GloTrajExpertStateIdx);
    }
        
}



void ForgetfulDrone::findMinDistanceStateIdx (
    double& min_dist,
    size_t& min_dist_state_idx
) {
    // Find "expert state" as state on global trajectory that has min distance 
    // to "reference state" (~ position of the drone).
    min_dist = std::numeric_limits<double>::max();
    for (size_t i = 0; i < m_GloTraj.size(); i++) {
        //double dist = (m_GloTraj[i].position - m_T_WRF_DRF.getPosition()).norm();
        double dist = (m_GloTraj[i].position - m_RefStatePos_WRF).norm();

        if (dist < min_dist) {
            min_dist = dist;
            min_dist_state_idx = i;
        }
    }
}

void ForgetfulDrone::findDistance2GlobalTrajectory (
    const Eigen::Vector3d position,
    double& min_dist,
    size_t& min_dist_state_idx
) {
    // Find state on global trajectory that has min distance to given position
    min_dist = std::numeric_limits<double>::max();
    for (size_t i = 0; i < m_GloTraj.size(); i++) {
        double dist = (m_GloTraj[i].position - position).norm();
        if (dist < min_dist) {
            min_dist = dist;
            min_dist_state_idx = i;
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
    m_GT_HorizonState_i = m_GloTrajExpertStateIdx;
    
    do
    {
        ++m_GT_HorizonState_i %= m_GloTraj.size();

        
        Distance_Expert2HorizonState 
            = ( m_GloTraj[ m_GT_HorizonState_i ].position 
                - m_GloTraj[ m_GloTrajExpertStateIdx ].position ).norm();

    } while ( Distance_Expert2HorizonState < Horizon );
}


void 
ForgetfulDrone::findSpeedState
()
{
    double Distance_Expert2SpeedState;
    m_GT_SpeedState_i = m_GloTrajExpertStateIdx;
    
    do
    {
        ++m_GT_SpeedState_i %= m_GloTraj.size();

        
        Distance_Expert2SpeedState 
            = ( m_GloTraj[ m_GT_SpeedState_i ].position 
                - m_GloTraj[ m_GloTrajExpertStateIdx ].position ).norm();

    } while ( Distance_Expert2SpeedState < p_EXPERT_SPEED_HORIZON );
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
    kindr::minimal::QuatTransformation T_DRF_WRF = m_T_WRF_DRF.inverse();

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
    double X_IRF = Saturation( -Yaw2PosDRF / p_DRONE_CAMERA_HALF_YAW_AOV, -1.0, 1.0 );
    double Y_IRF = Saturation( Pitch2PosDRF / p_DRONE_CAMERA_HALF_PITCH_AOV, -1.0, 1.0 );

    // Pos_IRF: 
    return { X_IRF, Y_IRF };
}




void 
ForgetfulDrone::rvizExpertAndHorizonState
()
{
    rvizPosition(m_GloTraj[m_GloTrajExpertStateIdx].position, VisPosTypes::EXPERT, m_rosPUB_RVIZ_NAVIGATION_POINTS);
    rvizPosition(m_GloTraj[m_GT_HorizonState_i].position, VisPosTypes::HORIZON, m_rosPUB_RVIZ_NAVIGATION_POINTS);
}













//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////// TRAIN DATA SAVER //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


void ForgetfulDrone::runDataSaver (const bool& expert_intervened) {
    // Start saving data only after passing the first gate.
    if (!m_DataSavingENABLED && m_CurrWaypointIdx == 0) {
        m_DataSavingENABLED = true;
        ROSINFO("Enable data saving");
    }
            
    if (m_DataSavingENABLED) {
        saveTrainSample (expert_intervened);
    }
}



void ForgetfulDrone::saveTrainSample (const bool& expert_intervened) {
    if (isEmpty(ROS_LOG_PREFIX, m_RGBData)) return;

    // rgb
    std::ostringstream oss; oss << std::setw(5) << std::setfill('0') << m_RGBCnt++;
    std::string rgb_fpath = m_RunRGBDpath + "/" + oss.str() + ".jpg";
    saveCVMat(ROS_LOG_PREFIX, rgb_fpath, m_RGBData);

    // data
    std::ofstream ofs;
    const char delimiter = ',';
    ofs.open(m_RunDataFpath, std::ios_base::app); ofs
        << expert_intervened << delimiter
        << m_RGBTimeIncrement << delimiter
        << m_IMUTimeIncrement << delimiter
        << m_IMUData[0] << delimiter
        << m_IMUData[1] << delimiter
        << m_IMUData[2] << delimiter
        << m_IMUData[3] << delimiter
        << m_IMUData[4] << delimiter
        << m_IMUData[5] << delimiter
        << m_LocTrajMaxSpeed << delimiter
        << m_ExpertOutput.x() << delimiter
        << m_ExpertOutput.y() << delimiter
        << m_ExpertOutput.z() << delimiter
        << m_CtrlCmdData[0] << delimiter
        << m_CtrlCmdData[1] << delimiter
        << m_CtrlCmdData[2] << delimiter
        << m_CtrlCmdData[3] << delimiter
        << m_CtrlCmdData[4] << delimiter
        << m_CtrlCmdData[5] << delimiter
        << m_CtrlCmdData[6] << std::endl;
    ofs.close(); ofs.clear();
}


void ForgetfulDrone::updateFailedRunsFile () {
    std::string fpath = ExperimentDpath() + "/" + h_OUTPUT_DNAME + "/" + h_FAILEDRUNS_FNAME;
    std::ofstream file;
    file.open(fpath, std::ios_base::app);
    file << RunID() << std::endl;
    file.close();
}






//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////// NOT IMPLEMENTED YET //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



void ForgetfulDrone::performFlightMission_GlobalTrajectoryTracking () {
    ROSDEBUG_FUNCTION_ENTERED();
}




/*
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
*/



/*
std::vector<quadrotor_common::TrajectoryPoint> 
computeGlobalTrajectory(
    std::vector<
        Eigen::Vector3d, 
        Eigen::aligned_allocator<Eigen::Vector3d>
            > Waypoints
){
    ros::Duration(10.0).sleep();



    // --- Normalize waypoints around origin ---
    Eigen::Vector3d MeanWaypoint{0, 0, 0};
    for (const Eigen::Vector3d& Waypoint : Waypoints) MeanWaypoint += Waypoint /Waypoints.size();
    for (Eigen::Vector3d& Waypoint : Waypoints) Waypoint -= MeanWaypoint;

    //Eigen::Vector3d MinAxVals{0, 0, 0};
    //for (const Eigen::Vector3d& Waypoint : m_TrackWaypoints)
    //{
    //    MinAxVals.x() = std::min(MinAxVals.x(), Waypoint.x());
    //    MinAxVals.y() = std::min(MinAxVals.y(), Waypoint.y());
    //    MinAxVals.z() = std::min(MinAxVals.z(), Waypoint.z());
    //}
    //for (Eigen::Vector3d& Waypoint : m_TrackWaypoints) Waypoint -= MinAxVals;




    std::cout<<std::endl;
    for (const Eigen::Vector3d& Waypoint : Waypoints)
    {
        std::cout << "\t" << Waypoint.x() << "   \t" << Waypoint.y() << "   \t" << Waypoint.z() << std::endl;
    }
    std::cout<<std::endl;



    //// --- Set ´m_Expert_MaxHorizon´ <- max dist between waypoints ---
    //m_Expert_MaxHorizon = 0.0;
    //size_t LastWaypointIdx = m_TrackWaypoints.size() - 1;
    //for (size_t WaypointIdx = 0; WaypointIdx < m_TrackWaypoints.size(); WaypointIdx++)
    //{
    //    m_Expert_MaxHorizon = std::max( 
    //        m_Expert_MaxHorizon,
    //        (m_TrackWaypoints[ WaypointIdx ] - m_TrackWaypoints[ LastWaypointIdx ]).norm() );
    //    LastWaypointIdx = WaypointIdx;
    //}

    //    ROS_INFO(
    //    "[%s]\n  >> Computing circuit global trajectory...\n" 
    //        "\t#waypoints: %d\n"
    //        "\tpolynomial order: %d\n"
    //        "\tcontinuity order: %d\n"
    //        "\tminimization weights on\n"
    //        "\t\tvelocity: %f\n"
    //        "\t\tacceleration: %f\n"
    //        "\t\tjerk: %f\n"
    //        "\t\tsnap: %f\n"
    //        "\tmaximum speed: %f\n"
    //        "\tmaximum normalized thrust: %f\n"
    //        "\tmaximum roll-pitch rate: %f\n"
    //        "\tsampling time: %f\n",
    //    ros::this_node::getName().c_str(),
    //    static_cast<int>( m_TrackWaypoints.size() ),
    //    p_GLOBAL_TRAJECTORY_POLYNOMIAL_ORDER,
    //    p_GLOBAL_TRAJECTORY_CONTINUITY_ORDER,
    //    m_GTMinWeightVel,
    //    m_GTMinWeightAcc,
    //    m_GTMinWeightJerk,
    //    m_GTMinWeightSnap,
    //    p_GLOBAL_TRAJECTORY_MAX_SPEED,
    //    p_GLOBAL_TRAJECTORY_MAX_THRUST,
    //    p_GLOBAL_TRAJECTORY_MAX_ROLL_PITCH_RATE,
    //    1 / p_MAIN_LOOP_FREQUENCY
    //    );
    


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
    //m_CurrWaypointIdx = m_WUInitWaypointIdxForFIG8;
    //m_LastWaypointIdx 
    //    = (m_CurrWaypointIdx + m_TrackWaypoints.size() - 1) 
    //        % m_TrackWaypoints.size();

    return GlobalTrajectory;
}*/








//void ForgetfulDrone::precomputeGloTrajForExpert()
//{
//
//    //ros::Duration(5.0).sleep();
//
//
//
//    // --- Normalize waypoints around origin ---
//    Eigen::Vector3d MeanWaypoint{0, 0, 0};
//    for (const Eigen::Vector3d& Waypoint : m_TrackWaypoints) MeanWaypoint += Waypoint /m_TrackWaypoints.size();
//    for (Eigen::Vector3d& Waypoint : m_TrackWaypoints) Waypoint -= MeanWaypoint;
//
//    //Eigen::Vector3d MinAxVals{0, 0, 0};
//    //for (const Eigen::Vector3d& Waypoint : m_TrackWaypoints)
//    //{
//    //    MinAxVals.x() = std::min(MinAxVals.x(), Waypoint.x());
//    //    MinAxVals.y() = std::min(MinAxVals.y(), Waypoint.y());
//    //    MinAxVals.z() = std::min(MinAxVals.z(), Waypoint.z());
//    //}
//    //for (Eigen::Vector3d& Waypoint : m_TrackWaypoints) Waypoint -= MinAxVals;
//
//
//
//
//    std::cout<<std::endl;
//    for (const Eigen::Vector3d& Waypoint : m_TrackWaypoints)
//    {
//        std::cout << "\t" << Waypoint.x() << "   \t" << Waypoint.y() << "   \t" << Waypoint.z() << std::endl;
//    }
//    std::cout<<std::endl;
//
//
//
//    //// --- Set ´m_Expert_MaxHorizon´ <- max dist between waypoints ---
//    //m_Expert_MaxHorizon = 0.0;
//    //size_t LastWaypointIdx = m_TrackWaypoints.size() - 1;
//    //for (size_t WaypointIdx = 0; WaypointIdx < m_TrackWaypoints.size(); WaypointIdx++)
//    //{
//    //    m_Expert_MaxHorizon = std::max( 
//    //        m_Expert_MaxHorizon,
//    //        (m_TrackWaypoints[ WaypointIdx ] - m_TrackWaypoints[ LastWaypointIdx ]).norm() );
//    //    LastWaypointIdx = WaypointIdx;
//    //}
//
//    /*    ROS_INFO(
//        "[%s]\n  >> Computing circuit global trajectory...\n" 
//            "\t#waypoints: %d\n"
//            "\tpolynomial order: %d\n"
//            "\tcontinuity order: %d\n"
//            "\tminimization weights on\n"
//            "\t\tvelocity: %f\n"
//            "\t\tacceleration: %f\n"
//            "\t\tjerk: %f\n"
//            "\t\tsnap: %f\n"
//            "\tmaximum speed: %f\n"
//            "\tmaximum normalized thrust: %f\n"
//            "\tmaximum roll-pitch rate: %f\n"
//            "\tsampling time: %f\n",
//        ros::this_node::getName().c_str(),
//        static_cast<int>( m_TrackWaypoints.size() ),
//        p_GLOBAL_TRAJECTORY_POLYNOMIAL_ORDER,
//        p_GLOBAL_TRAJECTORY_CONTINUITY_ORDER,
//        m_GTMinWeightVel,
//        m_GTMinWeightAcc,
//        m_GTMinWeightJerk,
//        m_GTMinWeightSnap,
//        p_GLOBAL_TRAJECTORY_MAX_SPEED,
//        p_GLOBAL_TRAJECTORY_MAX_THRUST,
//        p_GLOBAL_TRAJECTORY_MAX_ROLL_PITCH_RATE,
//        1 / p_MAIN_LOOP_FREQUENCY
//        );
//    */
//
//
//    // --- Set up Settings of GT ---
//    polynomial_trajectories::PolynomialTrajectorySettings GTSettings;
//    GTSettings.way_points = {std::make_move_iterator(m_TrackWaypoints.begin()), std::make_move_iterator(m_TrackWaypoints.end())};
//    GTSettings.polynomial_order = p_GLOBAL_TRAJECTORY_POLYNOMIAL_ORDER;
//    GTSettings.continuity_order = p_GLOBAL_TRAJECTORY_CONTINUITY_ORDER;
//    GTSettings.minimization_weights = Eigen::Vector4d{m_GTMinWeightVel, m_GTMinWeightAcc, m_GTMinWeightJerk, m_GTMinWeightSnap};
//    
//
//    // --- Compute initial segment times (first element relates to segment from last to first waypoint) ---
//    Eigen::VectorXd GTInitSegmentTimes = Eigen::VectorXd::Ones(m_TrackWaypoints.size());
//    Eigen::Vector3d SegmentStart = m_TrackWaypoints.back();
//    for (size_t WaypointIdx = 0; WaypointIdx < m_TrackWaypoints.size(); WaypointIdx++) 
//    {
//        GTInitSegmentTimes[WaypointIdx] = (m_TrackWaypoints[ WaypointIdx ] - SegmentStart).norm() /p_GLOBAL_TRAJECTORY_MAX_SPEED;
//        SegmentStart = m_TrackWaypoints[WaypointIdx];
//    }
//
//
//    
//    quadrotor_common::Trajectory GlobalTraj 
//        = trajectory_generation_helper::polynomials::generateMinimumSnapRingTrajectoryWithSegmentRefinement(
//            GTInitSegmentTimes,
//            GTSettings,
//            p_GLOBAL_TRAJECTORY_MAX_SPEED,
//            p_GLOBAL_TRAJECTORY_MAX_THRUST,
//            p_GLOBAL_TRAJECTORY_MAX_ROLL_PITCH_RATE,
//            p_MAIN_LOOP_FREQUENCY
//            );
//        
//  
//    m_GloTraj.clear();
//    m_GloTraj = {std::make_move_iterator(GlobalTraj.points.begin()), std::make_move_iterator(GlobalTraj.points.end())};
//
//    
//    std::cout<<std::endl;
//    for (const quadrotor_common::TrajectoryPoint& TrajPoint : m_GloTraj)
//    {
//        std::cout << "\t" << TrajPoint.position.x() << "   \t" << TrajPoint.position.y() << "   \t" << TrajPoint.position.z() << std::endl;
//    }
//    std::cout<<std::endl;
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
//    if ( m_Expert_MaxSpeed > 1.1 * p_GLOBAL_TRAJECTORY_MAX_SPEED )
//        ROS_WARN(
//            "[%s]\n  >> Maximum Speed of computed global trajectory exceeds 110 %% of nominal value (%f/%f m/s)."
//            "\n     Re-computing global trajectory...",
//            ros::this_node::getName().c_str(), m_Expert_MaxSpeed, p_GLOBAL_TRAJECTORY_MAX_SPEED
//            );
//
//
//    if ( GlobalTraj.trajectory_type == quadrotor_common::Trajectory::TrajectoryType::UNDEFINED )
//        ROS_WARN(
//            "[%s]\n  >> Computation of global trajectory not successful."
//            "\n     Re-computing global trajectory....",
//            ros::this_node::getName().c_str()
//            );
//
//
//    for (Eigen::Vector3d& Waypoint : m_TrackWaypoints) Waypoint += MeanWaypoint; //+ MinAxVals;
//    for (quadrotor_common::TrajectoryPoint& TrajPoint : m_GloTraj) TrajPoint.position += MeanWaypoint;// + MinAxVals;
//
//    ROS_INFO( 
//        "[%s]\n  >> Global trajectory successfully computed.\n"
//            "\t#states: %d\n"
//            "\tduration: %1.1f s\n"
//            "\tmaximum speed: %1.1f m/s\n"
//            "\tminimum speed: %1.1f m/s\n",            
//        ros::this_node::getName().c_str(),
//        static_cast<int>( m_GloTraj.size() ),
//        m_GloTraj.size() * 1 / p_MAIN_LOOP_FREQUENCY,
//        m_Expert_MaxSpeed, m_Expert_MinSpeed
//        );
//
//
//
//    // --- Visualize global trajectory in RVIZ ---
//    //rvizGloTraj();
//
//    // --- Set Waypoint Idx for deterministic 8 RaceTrack 
//    //m_CurrWaypointIdx = m_WUInitWaypointIdxForFIG8;
//    //m_LastWaypointIdx 
//    //    = (m_CurrWaypointIdx + m_TrackWaypoints.size() - 1) 
//    //        % m_TrackWaypoints.size();
//}



//void ForgetfulDrone::flyDroneBetweenLastAndSecLastGate()
//{
//    ROSINFO("Fly drone between last and second last gate.");
//
//    ros::Rate Rate(100.0);
//
//    while (m_AutopilotState != quadrotor_msgs::AutopilotFeedback::HOVER)
//        { ros::spinOnce(); Rate.sleep(); }
//    
//
//        geometry_msgs::PoseStamped msg;
//        msg.pose.position = GMPoint__from__EV3d(
//                m_TrackWaypoints.end()[-2] + (m_TrackWaypoints.end()[-1] - m_TrackWaypoints.end()[-2]) / 2);
//        msg.pose.orientation = m_DroneInitPose.orientation;
//    m_rosPUB_AUTOPILOT_POSE_COMMAND.publish(msg);
//
//    while (m_AutopilotState == quadrotor_msgs::AutopilotFeedback::HOVER)
//        { ros::spinOnce(); Rate.sleep(); }
//    while (m_AutopilotState != quadrotor_msgs::AutopilotFeedback::HOVER)
//        { ros::spinOnce(); Rate.sleep(); }
//
//    ROSDEBUG("Drone reached target pose.");
//
//    ros::Duration(1.0).sleep();
//}
}