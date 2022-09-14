#include "forgetful_drones/forgetful_drone.hpp"
#include "forgetful_drones/forgetful_helpers.hpp"
#include "forgetful_drones/forgetful_global_trajectory.hpp"
#include "forgetful_drones/forgetful_simulator.hpp"

#include "forgetful_drones/forgetful_ann.hpp"








namespace forgetful_drone {

ForgetfulDrone::ForgetfulDrone (const ros::NodeHandle& rnh, const ros::NodeHandle& pnh)
    :
    
    m_rosRNH {rnh}, m_rosPNH {pnh},
    
    // Forgetful Simulator
    m_SCL__simBuild {m_rosRNH.serviceClient<fdBS>("simulator/build")},
    m_SCL__simStart {m_rosRNH.serviceClient<fdStartS>("simulator/start")},
    m_SCL__simStop {m_rosRNH.serviceClient<fdStopS>("simulator/stop")},
    m_SCL__simTeleport {m_rosRNH.serviceClient<std_srvs::Empty>("simulator/teleport_drone")},
    m_SCL__simLoad {m_rosRNH.serviceClient<fdLR>("simulator/load_racetrack")},
    
    
    // Forgetful Brain
    m_SUB__fbOutput         {m_rosRNH.subscribe("brain/output", 1, &ForgetfulDrone::CB__fbOutput, this)},
    m_PUB__fbTrigger   {m_rosRNH.advertise     <std_msgs::Empty>          ("brain/trigger_inference", 0)},
    m_SCL__fbExpment        {m_rosRNH.serviceClient <forgetful_drones::String> ("brain/init_experiment")},
    m_SCL__fbBuild       {m_rosRNH.serviceClient <forgetful_drones::String> ("brain/build_run")},
    m_SCL__fbTrain     {m_rosRNH.serviceClient <forgetful_drones::Int>    ("brain/start_training")},
    m_SCL__fbInfer     {m_rosRNH.serviceClient <forgetful_drones::Float>  ("brain/start_inference")},

    
    // Drone
    m_SUB__apFeedback {m_rosRNH.subscribe("autopilot/feedback", 1, &ForgetfulDrone::ROSCB_AUTOPILOT_FEEDBACK, this)},
    m_PUB__apOff {m_rosRNH.advertise<std_msgs::Empty>("autopilot/off", 1)},
    m_PUB__apStart {m_rosRNH.advertise<std_msgs::Empty>("autopilot/start", 1)},
    m_PUB__apLand {m_rosRNH.advertise<std_msgs::Empty>("autopilot/land", 1) },
    m_PUB__apRefState {m_rosRNH.advertise<quadrotor_msgs::TrajectoryPoint>("autopilot/reference_state", 1)},
    m_PUB__apPoseCmd {m_rosRNH.advertise<geometry_msgs::PoseStamped>("autopilot/pose_command", 1)},
    m_PUB__bridgeArm {m_rosRNH.advertise<std_msgs::Bool>("bridge/arm", 1)},
    m_SUB__gtOdometry {m_rosRNH.subscribe("ground_truth/odometry", 1, &ForgetfulDrone::ROSCB_GROUND_TRUTH_ODOMETRY, this)},
    m_SUB__gtIMU {m_rosRNH.subscribe("ground_truth/imu", 1, &ForgetfulDrone::ROSCB_GROUND_TRUTH_IMU, this)},
    m_SUB__RGB {m_rosRNH.subscribe("/flightmare/rgb", 1, &ForgetfulDrone::ROSCB_FLIGHTMARE_RGB, this)},
    m_SUB__CtrlCmd {m_rosRNH.subscribe("control_command", 1, &ForgetfulDrone::ROSCB_CONTROL_COMMAND, this)},

    // RViz
    m_PUB__rvNavPoints {m_rosRNH.advertise <visualization_msgs::Marker> ("rviz/navigation_points", 0)},
    m_PUB__rvLblRGB   {m_rosRNH.advertise <sensor_msgs::Image>         ("rviz/rgb/labeled", 1)},
    m_PUB__rvLocTraj    {m_rosRNH.advertise<visualization_msgs::Marker>("rviz/local_trajectory", 1)},
    m_PUB__rvGloTraj   {m_rosRNH.advertise <visualization_msgs::Marker> ("rviz/global_trajectory", 1)},
        //m_rosSVC_RVIZ_LOAD_CONFIG {m_rosRNH.serviceClient<rviz::SendFilePath>("rviz/load_config")},

    
    m_AutopilotState {quadrotor_msgs::AutopilotFeedback::OFF},
    m_ExpOut {0.0, 0.0, 0.0},
    m_BrainOutput {0.0, 0.0, 0.0},
    m_NavInput {0.0, 0.0, 0.0},
    m_GloTraj {},
    m_GloTrajWayps {},
    m_ExpStateIdx {0},
    m_Expert_MaxSpeed {std::numeric_limits<double>::min()},
    m_Expert_MinSpeed {std::numeric_limits<double>::max()},
    m_WRF_LRF {},
    m_ARF_LRF {},
    m_RefStatePos_WRF {},
    m_CurrGateIdx {0},
    m_LastGateIdx {0},
    m_CurrGateDist {0.0},
    m_LastGateDist {0.0},
    

    m_NavEnabled {false},
    m_SaveEnabled {false},
    m_LocTraj_SuccInfeasCnt {0},
    m_LocTraj_t0 {},
    m_LocTraj {Vec3(), Vec3(), Vec3(), Vec3()},
    m_MainIterCnt {0},
    m_LocTrajFeasibleCnt {0},
    m_RunIdx {0},
    m_DroneLaunched {false},
    m_RGBCnt {0},
    m_LapIdx {-1}
{
    if (initROSParameters () && initTrafoArf2Wrf ()) {
        ROSINFO("Node initialized");
        ros::Duration(3.0).sleep();
    } else {
        ROSERROR("Node initialization failed");
        ros::shutdown();
    }

    
    logMissionInfo (/*start*/true);
    switch (p_MISSION) {
        case 0: runMission_trackGloTraj(); break;
        case 1: runMission_testExp (); break;
        case 2: runMission_Racing (); break;
        case 3: runMission_DAGGER (); break;
        default: 
            ROSERROR("No implementation for " << GET_VAR_REP(p_MISSION));
            ros::shutdown(); break;
    }
    landDrone ();
    logMissionInfo (/*start*/false);
}



ForgetfulDrone::~ForgetfulDrone () {}



std::string ForgetfulDrone::RunID () {
    std::string runidx = p_MISSION == 3? asSeqNo (4, m_RunIdx) + "___" : "";
    std::string daggermargin = p_MISSION == 3? "_" + asFixedFloat (5, 2, m_DaggerMargin) : "";

    std::ostringstream oss; 
    oss
        << runidx
        << std::to_string (m_SceneIdx)          << '_'
        << std::to_string (m_SiteIdx)           << '_'
        << std::to_string (m_TrackTypeIdx)      << '_'
        << std::to_string (m_TrackGenIdx)       << '_'
        << std::to_string (m_TrackDirecIdx)     << '_'
        << std::to_string (m_GateIdx)           << '_'
        << asFixedFloat (5, 2, m_MaxSpeed)
        << daggermargin  << "___"
        << asSeqNo (3, m_RunRepIdx);
    return oss.str();


}

std::string ForgetfulDrone::_exp_ () {return ROS_PACKAGE_PATH + "/experiments/" + p_EXPERIMENT_ID;}
std::string ForgetfulDrone::_exp_cnf_ () {return _exp_ () + "/config";}
std::string ForgetfulDrone::_exp_cnf_YML (const bool& src) {if (src) return ROS_PACKAGE_PATH + "/parameters/forgetful_drone.yaml"; else return _exp_cnf_ () + "/forgetful_drone.yaml";}
std::string ForgetfulDrone::_exp_cnf_CNF () {return _exp_cnf_ () + "/config.json";}
std::string ForgetfulDrone::_exp_dat_ () {return _exp_ () + "/data";}
std::string ForgetfulDrone::_exp_dat_raw_ () {return _exp_dat_ () + "/raw";}
std::string ForgetfulDrone::_exp_dat_raw_rid_ () {return _exp_dat_raw_ () + '/' + RunID ();}
std::string ForgetfulDrone::_exp_dat_raw_rid_rgb_ () {return _exp_dat_raw_rid_ () + "/rgb";}
std::string ForgetfulDrone::_exp_dat_raw_rid_lbl_ () {return _exp_dat_raw_rid_ () + "/labeled_rgb";}
std::string ForgetfulDrone::_exp_dat_raw_rid_TXT () {return _exp_dat_raw_rid_ () + "/data.txt";}
std::string ForgetfulDrone::_exp_out_ () {return _exp_ () + "/output";}
std::string ForgetfulDrone::_exp_out_CPT () {return _exp_out_ () + "/checkpoint.pt";}
std::string ForgetfulDrone::_exp_out_ANT () {return _exp_out_ () + "/model_scripted_with_annotation.pt";}
std::string ForgetfulDrone::_exp_out_TRA () {return _exp_out_ () + "/model_scripted_with_tracing.pt";}
std::string ForgetfulDrone::_exp_out_BRC () {return _exp_out_ () + "/build_records.json";}
std::string ForgetfulDrone::_exp_out_TRC () {return _exp_out_ () + "/train_records.json";}
std::string ForgetfulDrone::_exp_rac_ () {return _exp_ () + "/racing";}
std::string ForgetfulDrone::_exp_rac_RRC () {return _exp_rac_ () + "/racing_records.json";}
std::string ForgetfulDrone::_exp_rac_RID () {return _exp_rac_ () + "/" + RunID () + ".txt";}












bool ForgetfulDrone::initANN () {
    switch (p_TORCH_INTERFACE_IDX) {
        case 0: return true;
        case 1: return initANN_Cpp ();
        default: 
            ROSERROR ("No implementation for" << GET_VAR_REP(p_TORCH_INTERFACE_IDX)); 
            return false;
    }
}


bool ForgetfulDrone::initANN_Python () {
    
}



bool ForgetfulDrone::initANN_Cpp () {
    int rgb_height;
    int rgb_width;
    int gru_nlayers;
    int gru_hsize;
    std::vector<std::string> opt_inputs;
    std::vector<bool> opt_mask;
    

    /* fetch params from json file */ 
    try {
        std::ifstream ifs (_exp_cnf_CNF ());
        nlohmann::json jf = nlohmann::json::parse (ifs);

        rgb_height = jf ["data"] ["processed"] ["rgb"] ["height"];
        rgb_width = jf["data"] ["processed"] ["rgb"] ["width"];
        
        nlohmann::json val = jf ["ann"] ["gru"] ["hidden_size"];
        if (val.is_null ()) gru_hsize = 1; else gru_hsize = val;

        val = jf ["ann"] ["gru"] ["num_layers"];
        if (val.is_null ()) gru_nlayers = 1; else gru_nlayers = val;

        opt_inputs = static_cast <std::vector <std::string>> (jf ["data"] ["input"] ["cat"]);
        
        for (const char* OI : h_OPTINPUTS) {
            bool found {false};
            for (const std::string& oi : opt_inputs) {
                if (std::string(OI) == oi) {
                    found = true;
                    break;
                }
            }
            opt_mask.push_back (found);
        }
    } catch(const std::exception& e) {
        ROSERROR(e.what());
        return false;
    }
    ROSINFO("From \"" << _exp_cnf_CNF () << "\":" << std::endl
        << "\t\t- RGB height:" << rgb_height << " width:" << rgb_width << std::endl
        << "\t\t- Opt input mask: " << opt_mask << std::endl
        << "\t\t- GRU # layers:" << gru_nlayers << " hidden size:" << gru_hsize << std::endl
    );

    return m_ANN.init (
        rgb_height,
        rgb_width,
        opt_mask,
        gru_nlayers,
        gru_hsize,
        1,
        _exp_out_ANT ()
    );    
}




bool ForgetfulDrone::initROSParameters () { 
    // ROS param keys and destinations of type bool
    std::vector <std::pair<const char*, const bool*>> kd_bool {
        //{"DYNAMIC_GATES_ON" , &m_DYNAMIC_GATES_ON},
        {"RVIZ_LOCTRAJ_ELAPSEDDISPLAYED", &p_RVIZ_LOCAL_TRAJECTORY_DISPLAY_ELAPSED_ENABLED},
        {"RVIZ_LABELEDRGB_SAVED", &p_RVIZ_LBLRGB_SAVE},
        {"RVIZ_LABELEDRGB_ENABLED", &p_RVIZ_LBLRGB_ENABLED},
    };

    // ROS param keys and destinations of type int
    std::vector <std::pair<const char*, const int*>> kd_int {
        {"RACING_NUMREPS" , &p_NRUNREPS},
        {"GLOTRAJ_POLYNOMIALORDER" , &p_GLOBAL_TRAJECTORY_POLYNOMIAL_ORDER},
        {"GLOTRAJ_CONTINUITYORDER", &p_GLOBAL_TRAJECTORY_CONTINUITY_ORDER},
        {"NAV_REPLANNING_MAINLOOPITERSCNT" , &p_NAV_REPLAN_ITER_CNT},
        {"NAV_LOCTRAJ_FEASIBILITY_MAXSUCCESSIVEFAILSCNT", &p_LOCTRAJ_MAXSUCCINFEASCNT},
        {"RUN_NUMLAPS", &p_RUN_NUMLAPS},
        {"WAYPARRIV_MAXDURATION", &p_WPARR_MAXDUR},
        {"MISSION", &p_MISSION},
        {"TORCH_INTERFACE", &p_TORCH_INTERFACE_IDX},
        {"TORCH_DEVICE", &p_TORCH_DEVICE},
        
        {"DAGGER_NUMLAPS_FIRSTRUN", &p_DAGGER_FIRSTRUN_NUMLAPS},
        {"DAGGER_NUMEPOCHS", &p_DAGGER_NEPOCHS},
    };

    // ROS param keys and destinations of type double
    std::vector <std::pair<const char*, const double*>> kd_dbl {
        {"DRONE_CAMERA_HALF_YAW_AOV", &p_CAM_HALFYAWAOV},
        {"DRONE_CAMERA_HALF_PITCH_AOV", &p_CAM_HALFPITCHAOV},
        {"GLOTRAJ_MAXTHRUST", &p_GLOBAL_TRAJECTORY_MAX_THRUST},
        {"GLOTRAJ_MAXROLLPITCHRATE", &p_GLOBAL_TRAJECTORY_MAX_ROLL_PITCH_RATE},
        {"GLOTRAJ_MAXSPEED", &p_GLOBAL_TRAJECTORY_MAX_SPEED}, 
        {"GLOTRAJ_NONDIMTEMPORALRANGE", &p_GLOBAL_TRAJECTORY_NON_DIMENSIONAL_TEMPORAL_RANGE},
        {"GLOTRAJ_NONDIMSPATIALRANGE", &p_GLOBAL_TRAJECTORY_NON_DIMENSIONAL_SPATIAL_RANGE},
        {"WAYPARRIV_THRESHOLDDIST", &p_GATE_THRESH_DIST},
        {"EXPERT_MINWAYPHORIZON", &p_EXPERT_MIN_HORIZON},
        {"EXPERT_SPEEDHORIZON", &p_EXPERT_SPEED_HORIZON},
        {"NAV_MAX_DIV", &p_NAV_MAX_DIV},
        {"EXPERT_PROJECTIONMAXDIVERGENCE", &p_EXP_THRESHOLD_DIV},
        {"MAIN_FREQ", &p_MAIN_FREQ},
        {"NAV_LOCTRAJ_MINSPEED", &p_LOCTRAJ_MINSPEED},
        {"NAV_LOCTRAJ_MAXSPEEDINCREMENT", &p_LOCAL_TRAJECTORY_MAX_SPEED_INCREMENT},
        {"NAV_LOCTRAJ_DURATION", &p_LOCTRAJ_DUR},
        {"NAV_LOCTRAJ_MINDIST", &p_LOCTRAJ_MINDIST},
        {"NAV_LOCTRAJ_MAXDIST", &p_LOCTRAJ_MAXDIST},
        {"NAV_LOCTRAJ_FEASIBILITY_POSITION_MAXALTITUDE", &p_LOCAL_TRAJECTORY_MAX_ALTITUDE},
        {"NAV_LOCTRAJ_FEASIBILITY_POSITION_MINALTITUDE", &p_LOCAL_TRAJECTORY_MIN_ALTITUDE},
        {"NAV_LOCTRAJ_FEASIBILITY_INPUT_MINTHRUST", &p_LOCAL_TRAJECTORY_MIN_THRUST},
        {"NAV_LOCTRAJ_FEASIBILITY_INPUT_MAXTHRUST", &p_LOCAL_TRAJECTORY_MAX_THRUST},
        {"NAV_LOCTRAJ_FEASIBILITY_INPUT_MAXBODYRATES", &p_LOCAL_TRAJECTORY_MAX_BODY_RATES},
        {"NAV_LOCTRAJ_FEASIBILITY_INPUT_MINSAMPLINGTIME", &p_LOCAL_TRAJECTORY_INPUT_FEASIBILITY_CHECK_MIN_SAMPLING_TIME},
        {"NAV_INPUTPERTURBATION_ELEMENTWISEAMP", &p_NAV_INPUTDISTURBAMP},
        {"RVIZ_LOCTRAJ_SAMPLINGDURATION", &p_RVIZ_LOCAL_TRAJECTORY_DURATION},
        {"RVIZ_LOCTRAJ_SAMPLINGFREQUENCY", &p_RVIZ_LOCAL_TRAJECTORY_SAMPLING_FREQUENCY},
        //{"DAGGER_EXPIVSHARE_THRESHOLD", &p_EIS_THRSH},
    };

    // String
    std::vector <std::pair <const char*, const std::string*>> kd_str {        
        {"EXPERIMENT_ID", &p_EXPERIMENT_ID},
    };
    
    
    return true
        && fetchROSArrayParameter ("SCENES",         p_SCENES,          m_rosRNH, false)
        && fetchROSArrayParameter ("SITES",          p_SITES,           m_rosRNH, false)
        && fetchROSArrayParameter ("TRACK_TYPES",    p_TRACK_TYPES,     m_rosRNH, false)
        && fetchROSArrayParameter ("TRACK_GENS",     p_TRACK_GENS,      m_rosRNH, false)
        && fetchROSArrayParameter ("TRACK_DIRECS",   p_TRACK_DIRECS,    m_rosRNH, false)
        && fetchROSArrayParameter ("GATES",          p_GATES,           m_rosRNH, false)
        && fetchROSArrayParameter ("ARF_POSE_WRF",           p_ARF_POSE_WRF,    m_rosRNH, false)
        && fetchROSArrayParameter ("MAX_SPEEDS",     p_MAXSPEEDS,       m_rosRNH, false)
        && fetchROSArrayParameter ("DAGGER_MARGINS",   p_DAGGERMARGINS,   m_rosRNH, false)
        && fetchROSArrayParameter ("DAGGER_EXPIVSHARE_THRESHOLD",   p_EXPINTVSHARES,   m_rosRNH, false)

        
        && fetchROSParameters (m_rosRNH, kd_bool, kd_int, kd_dbl, kd_str, /*log_enabled*/ false);
}
















bool ForgetfulDrone::initExperiment () {
    bool succ;
    
    if (p_EXPERIMENT_ID == "") {
        if (p_MISSION != 3) {
            ROSERROR ("Flight mission " << h_MISSIONS [p_MISSION] << " requires existing experiment ID but:  " 
                << GET_VAR_REP (p_EXPERIMENT_ID));
            return false;
        }

        const_cast <std::string&> (p_EXPERIMENT_ID) = UTCDateTime ();

        succ = true
            && createDir (ROS_LOG_PREFIX, _exp_ ())
            && createDir (ROS_LOG_PREFIX, _exp_cnf_ ())
            && createDir (ROS_LOG_PREFIX, _exp_dat_raw_ ())
            && copyFile (ROS_LOG_PREFIX, _exp_cnf_YML (true), _exp_cnf_YML (false))
            ;
        ROSINFO("Created experiment: " << p_EXPERIMENT_ID);
    
    } else {
        auto path_exists = [this] (const std::string& p, const bool& dir) {
            if (dir? isDir (p) : isFile(p)) return true; else {
                ROSERROR("\"" << p << "\" does not exist");
                return false;
            }
        };

        succ = true
            && path_exists (_exp_ (), true)
            && path_exists (_exp_cnf_ (), true)
            && path_exists (_exp_cnf_YML (false), false)
            && path_exists (_exp_cnf_CNF (), false)
            && path_exists (_exp_dat_ (), true)
            && path_exists (_exp_dat_raw_ (), true)
            && path_exists (_exp_out_ (), true)
            && path_exists (_exp_out_BRC (), false)
            && path_exists (_exp_out_TRC (), false)
            && path_exists (_exp_out_CPT (), false)
            && path_exists (_exp_out_ANT (), false)
            && path_exists (_exp_out_TRA (), false)
            ;
        ROSINFO("Loaded experiment: " << p_EXPERIMENT_ID);
    }

    if (!succ) return false;

    if (p_TORCH_INTERFACE_IDX == 0) {
        forgetful_drones::String srv;
        srv.request.data = p_EXPERIMENT_ID;
        while (!callRosSrv <forgetful_drones::String> (ROS_LOG_PREFIX, m_SCL__fbExpment, srv)) 
            ros::Duration(1.0).sleep();
    }
    
    return true;
}









bool ForgetfulDrone::initTrafoArf2Wrf () {
    // transformation from autopilot to world reference frame
    try {
        geometry_msgs::Pose p;
        p.position.x = p_ARF_POSE_WRF [0];
        p.position.y = p_ARF_POSE_WRF [1];
        p.position.z = p_ARF_POSE_WRF [2];
        p.orientation.w = p_ARF_POSE_WRF [3];
        p.orientation.x = p_ARF_POSE_WRF [4];
        p.orientation.y = p_ARF_POSE_WRF [5];
        p.orientation.z = p_ARF_POSE_WRF [6];
        tf::poseMsgToKindr (p, &const_cast <kindr::minimal::QuatTransformation&> (p_WRF_ARF));
    } 
    catch (const std::exception& e) {
        ROSERROR (e.what ());
        return false;
    }
    return true;
}










//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////// ROS CALLBACKS //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void ForgetfulDrone::ROSCB_CONTROL_COMMAND (const quadrotor_msgs::ControlCommand::ConstPtr& msg) { 
    m_CtrlCmdMtx.lock ();
    m_CtrlCmdPtr = msg;
    m_CtrlCmdMtx.unlock ();
}

void ForgetfulDrone::ROSCB_AUTOPILOT_FEEDBACK (const quadrotor_msgs::AutopilotFeedback::ConstPtr& msg) {
    m_AutopilotState = msg->autopilot_state;
}

void ForgetfulDrone::ROSCB_GROUND_TRUTH_ODOMETRY (const nav_msgs::OdometryConstPtr& msg) {
    tf::poseMsgToKindr (GMPose_From_NMO (*msg), &m_ARF_LRF);
    m_WRF_LRF = p_WRF_ARF * m_ARF_LRF; 
    m_ActualNormSpeed = EV3d_From_GMV3 (msg->twist.twist.linear).norm () / m_MaxSpeed;
}

void ForgetfulDrone::ROSCB_GROUND_TRUTH_IMU (const sensor_msgs::ImuConstPtr& msg) {
    m_IMUMtx.lock ();
    m_IMUPtr = msg;
    m_IMUMtx.unlock ();
}

void ForgetfulDrone::CB__fbOutput (const geometry_msgs::PointConstPtr& msg) {
    m_BrainOutput = {msg->x, msg->y, msg->z};
}

void ForgetfulDrone::ROSCB_FLIGHTMARE_RGB (const sensor_msgs::ImageConstPtr& msg) {
    //std::cout << *msg <<std::endl;
    m_RGBMtx.lock ();
    m_RGBPtr = cv_bridge::toCvShare (msg, sensor_msgs::image_encodings::BGR8);
    m_RGBMtx.unlock ();
}



void ForgetfulDrone::rvizLblRGB() {
    if (isEmpty (ROS_LOG_PREFIX, m_RGB)) return;

    
    const int& W = m_RGB.cols;
    const int& H = m_RGB.rows;
    int xPx, yPx;

    // Add vertical grid lines
    constexpr double angle_inc = 30.0 * M_PI/180.0;
    for (double yaw = 0.0; yaw <= p_CAM_HALFYAWAOV; yaw += angle_inc) {
        xPx = static_cast<int>((yaw/p_CAM_HALFYAWAOV + 1.0) * W/2.0);
        cv::line(m_RGB, cv::Point(xPx, 0), cv::Point(xPx, H), cv::Scalar(100, 100, 100, 0));

        xPx = static_cast<int>((-yaw/p_CAM_HALFYAWAOV + 1.0) * W/2.0);
        cv::line(m_RGB, cv::Point(xPx, 0), cv::Point(xPx, H), cv::Scalar(100, 100, 100, 0));
    }
    // Add horizontal grid lines
    for (double pitch = 0; pitch <= p_CAM_HALFPITCHAOV; pitch += angle_inc) {
        yPx = static_cast<int>((pitch/p_CAM_HALFPITCHAOV + 1.0) * H/2.0);
        cv::line(m_RGB, cv::Point(0, yPx), cv::Point(W, yPx), cv::Scalar(100, 100, 100, 0));

        yPx = static_cast<int>((-pitch/p_CAM_HALFPITCHAOV + 1.0) * H/2.0);
        cv::line(m_RGB, cv::Point(0, yPx), cv::Point(W, yPx), cv::Scalar(100, 100, 100, 0));
    }

    // Add expert output
    constexpr double alpha = 0.8;
    xPx = static_cast<int>(W * (1.0 + m_ExpOut.x()) / 2.0);
    yPx = static_cast<int>(H * (1.0 - m_ExpOut.y()) / 2.0);
    cv::circle(m_RGB, cv::Point(xPx, yPx), 0, cv::Scalar(255, 0, 0), 12, 8, 0);
    cv::Mat roi = m_RGB(cv::Rect(W - 200, H - 50, 190, 40));
    cv::Mat color = cv::Mat(roi.size(), CV_8UC3, cv::Scalar(125, 125, 125));
    cv::addWeighted(color, alpha, roi, 1.0 - alpha, 0.0, roi); // grey box
    cv::putText(m_RGB, 
        ("Expert " + std::to_string(m_ExpOut.z()).substr(0, 5)).c_str(), 
        cv::Point2f(W - 190, H - 23), 
        cv::FONT_ITALIC, 0.7, cv::Scalar(255, 0, 0, 255));

    // Add ANN output
    xPx = static_cast<int>(W * (1.0 + m_BrainOutput.x()) / 2.0);
    yPx = static_cast<int>(H * (1.0 - m_BrainOutput.y()) / 2.0);
    cv::circle(m_RGB, cv::Point(xPx, yPx), 0, cv::Scalar(0, 255, 0), 12, 8, 0);
    roi = m_RGB(cv::Rect(10, H - 50, 190, 40));
    color = cv::Mat(roi.size(), CV_8UC3, cv::Scalar(125, 125, 125));
    cv::addWeighted(color, alpha, roi, 1.0 - alpha, 0.0, roi);
    cv::putText(m_RGB, 
        ("Network " + std::to_string(m_BrainOutput.z()).substr(0, 5)).c_str(), 
        cv::Point2f(20, H - 23), 
        cv::FONT_ITALIC, 0.7, cv::Scalar(0, 255, 0, 255));

    // --- Add actual normalized speed
    roi = m_RGB(cv::Rect(static_cast<int>(W/2-95), H - 50, 190, 40));
    color = cv::Mat(roi.size(), CV_8UC3, cv::Scalar(125, 125, 125));
    cv::addWeighted(color, alpha, roi, 1.0 - alpha, 0.0, roi);
    cv::putText(m_RGB, 
        ("Estimate " + std::to_string(m_ActualNormSpeed).substr(0, 5)).c_str(), 
        cv::Point2f(static_cast<int>(W/2-85), H - 23), 
        cv::FONT_ITALIC, 0.7, cv::Scalar(0, 0, 255, 255));

    // --- Add brain decision / expert intervention bar
    int total_cnt = m_ExpDecCnt + m_ANNDecCnt;
    if (total_cnt != 0) {
        double exp_share = (double)m_ExpDecCnt / (double)total_cnt;
        double brain_share = 1.0 - exp_share;
        int total_width = W - 20;
            // - brain
            int brain_width = (int)(total_width * brain_share);
            roi = m_RGB(cv::Rect(10, 10, brain_width, 40));
            color = cv::Mat(roi.size(), CV_8UC3, cv::Scalar(0, 255, 0));
            cv::addWeighted(color, alpha, roi, 1.0 - alpha, 0.0, roi);
            // - expert
            int exp_width = total_width - brain_width;
            roi = m_RGB(cv::Rect(10 + brain_width + 1, 10, exp_width, 40));
            color = cv::Mat(roi.size(), CV_8UC3, cv::Scalar(255, 0, 0));
            cv::addWeighted(color, alpha, roi, 1.0 - alpha, 0.0, roi);

            cv::putText(m_RGB, 
                (std::to_string(100 * brain_share).substr(0, 5) + " %").c_str(), 
                cv::Point2f(20, 50 - 13), 
                cv::FONT_ITALIC, 0.7, cv::Scalar(125, 125, 125, 255));
                
            cv::putText(m_RGB, 
                (std::to_string(100 * exp_share).substr(0, 5) + " %").c_str(), 
                cv::Point2f(W - 130, 50 - 13), 
                cv::FONT_ITALIC, 0.7, cv::Scalar(125, 125, 125, 255));
    }
    

    

    // rviz labeled camera frame
    cv_bridge::CvImage msg;
    msg.header.stamp = ros::Time::now();
    msg.encoding = sensor_msgs::image_encodings::BGR8;
    msg.image = m_RGB;
    m_PUB__rvLblRGB.publish(msg.toImageMsg());

    // save labeled camera frame
    if (p_RVIZ_LBLRGB_SAVE) saveCVMat (
        ROS_LOG_PREFIX, 
        m_exp_dat_raw_rid_lbl_ + "/" + asSeqNo (5, m_RGBCnt) + ".jpg",
        m_RGB
    );
}



//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////// FLIGHT MISSIONS //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool ForgetfulDrone::wait4APState (const uint8_t& state, const bool& enter, const double& dur) const {
    
    auto cond = [this, state, enter] () {
        return enter? (m_AutopilotState != state) : (m_AutopilotState == state);
    };

    ros::Rate Rate (100.0);
    double t0 = ros::Time::now ().toSec ();
    while (cond ()) {
        ros::spinOnce (); 
        Rate.sleep ();
        
        if (ros::Time::now ().toSec () - t0 > dur) return false;
    }

    return true;
}
void ForgetfulDrone::waitForAutopilotState (const uint8_t& state, const bool& exit) const {
    //std::string aps_string;
    //switch (state)
    //{
    //    case quadrotor_msgs::AutopilotFeedback::BREAKING: aps_string = "BREAKING"; break;
    //    case quadrotor_msgs::AutopilotFeedback::HOVER: aps_string = "HOVER"; break;
    //    case quadrotor_msgs::AutopilotFeedback::OFF: aps_string = "OFF"; break;
    //    default:
    //        aps_string = "?"; break;
    //}
    
    ros::Rate Rate (100.0);
    if (!exit) {
        //ROSDEBUG("Wait for autopilot state " << aps_string);
        while (m_AutopilotState != state) {ros::spinOnce(); Rate.sleep();}
    }
    else {
        //ROSDEBUG("Wait for any autopilot state but " << aps_string);
        while (m_AutopilotState == state) {ros::spinOnce(); Rate.sleep();}
    }
}

bool ForgetfulDrone::launchDrone () {
    if (m_DroneLaunched) return true;

    std::string m = "Launch drone off ground";
    playAudio (m); ROSINFO (m);

    
    m_PUB__apOff.publish (std_msgs::Empty ());
    std_msgs::Bool msg; msg.data = true; m_PUB__bridgeArm.publish (msg);
    m_PUB__apStart.publish (std_msgs::Empty ());
    
    if (!wait4APState (quadrotor_msgs::AutopilotFeedback::BREAKING, true, 10.0)) return false;
    if (!wait4APState (quadrotor_msgs::AutopilotFeedback::HOVER, true, 5.0)) return false;
    //waitForAutopilotState (quadrotor_msgs::AutopilotFeedback::BREAKING);
    //waitForAutopilotState (quadrotor_msgs::AutopilotFeedback::HOVER);
    
    m_DroneLaunched = true;

    m = "Drone launched"; 
    playAudio (m); ROSINFO (m);
    ros::Duration(1.0).sleep();

    return true;
}

bool ForgetfulDrone::landDrone () {
    if (!m_DroneLaunched) return true;

    std::string m = "Land drone on ground";
    playAudio (m); ROSINFO (m);

    m_PUB__apLand.publish (std_msgs::Empty ());
    if (!wait4APState (quadrotor_msgs::AutopilotFeedback::OFF, true, 60.0)) return false;  
    
    
    m_DroneLaunched = false;

    m = "Drone landed"; 
    playAudio (m); ROSINFO (m);
    ros::Duration (1.0).sleep();

    return true;
}


bool ForgetfulDrone::flyDroneTo (const geometry_msgs::Pose& p, const double& max_dur) const {
    std::ostringstream m_p; m_p
        << std::setprecision (3)
        << " x:" << p.position.x 
        << " y:" << p.position.y 
        << " z:" << p.position.z 
        << " yaw:" << Yaw_From_EQd (EQd___GMQ (p.orientation));
    
    ROSINFO ("  - Fly drone to" << m_p.str ());
    

    if (!wait4APState (quadrotor_msgs::AutopilotFeedback::HOVER, true, 5.0)) return false;
    geometry_msgs::PoseStamped msg; msg.pose = p;
    m_PUB__apPoseCmd.publish (msg);
    if (!wait4APState (quadrotor_msgs::AutopilotFeedback::HOVER, false, 5.0)) return false;
    if (!wait4APState (quadrotor_msgs::AutopilotFeedback::HOVER, true, max_dur)) return false;

    //waitForAutopilotState (quadrotor_msgs::AutopilotFeedback::HOVER);
    //waitForAutopilotState (quadrotor_msgs::AutopilotFeedback::HOVER, true);
    //waitForAutopilotState (quadrotor_msgs::AutopilotFeedback::HOVER);
    
    ROSINFO ("  - Drone arrived at" << m_p.str ());
    return true;
}

bool ForgetfulDrone::carryDroneBack () {
    std::string m {"Carry drone back"};
    ROSINFO (m); playAudio (m);
    std::cout << std::endl;
    
    std_msgs::Bool msg; msg.data = false; m_PUB__bridgeArm.publish (msg);
    m_PUB__apOff.publish (std_msgs::Empty ());
    m_DroneLaunched = false;
    
    std_srvs::Empty srv;
    return callRosSrv (ROS_LOG_PREFIX, m_SCL__simTeleport, srv);
}

bool ForgetfulDrone::flyDroneAboveTrack () {
    std::string m {"Fly drone above racetrack"};
    ROSINFO (m); playAudio (m);

    // find indices of the two closest gates
    Eigen::Vector3d p = m_ARF_LRF.getPosition ();
    Eigen::Quaterniond q = m_ARF_LRF.getEigenQuaternion ();
    
    double dmin {1e10}; size_t imin {0};
    for (size_t i = 0; i < m_GloTrajWayps.size (); i++) {
        double d = (p - m_GloTrajWayps [i]).norm ();
        if (d < dmin) {imin = i; dmin = d;}
    }
    
    dmin = 1e10; size_t jmin {0};
    for (size_t j = 0; j < m_GloTrajWayps.size (); j++) {
        if (j == imin) continue;

        double d = (p - m_GloTrajWayps [j]).norm ();
        if (d < dmin) {jmin = j; dmin = d;}
    }

    // fly drone between the two closest gates
    p = (m_GloTrajWayps [imin] + m_GloTrajWayps [jmin]) / 2;
    if (!flyDroneTo (GMPose___EV3d_EQd (p, q), 120.0)) return false;

    // fly drone upwards above the track
    p.z () += 6.0;
    if (!flyDroneTo (GMPose___EV3d_EQd (p, q), 60.0)) return false;

    std::cout << std::endl;
    return true;
}




bool ForgetfulDrone::flyDroneToInitPose (const bool& from_above) {
    std::string m {"Fly drone to start"};
    ROSINFO (m); playAudio (m);

    if (from_above) {
        // fly drone above start position
        geometry_msgs::Pose p = m_DroneInitPose;
        p.position.z += 6.0;
        if (!flyDroneTo (p, 300.0)) return false;
    }


    // fly drone down to start position
    if (!flyDroneTo (m_DroneInitPose, 60.0)) return false;

    std::cout << std::endl;
    return true;
}








void ForgetfulDrone::initMainLoopTimer (void (forgetful_drone::ForgetfulDrone::*callback)(const ros::TimerEvent&)) {
    m_rosTMR_MAINLOOP = m_rosRNH.createTimer(
        /*rate*/ ros::Rate (p_MAIN_FREQ), 
        /*callback*/ callback, /*obj*/ this, 
        /*oneshot*/ false, /*autostart*/ false
    );
};






void ForgetfulDrone::initRacingPaths () {
    if (! isDir (_exp_rac_ ())) createDir (ROS_LOG_PREFIX, _exp_rac_ ());
    m_exp_rac_RID = _exp_rac_RID ();

    std::ofstream ofs;
    const char delimiter = ',';
    ofs.open (m_exp_rac_RID, std::ios_base::app); 
    ofs
        << "time" << delimiter
        << "pos_x" << delimiter
        << "pos_y" << delimiter
        << "pos_z" << delimiter
        << "vel_x" << delimiter
        << "vel_y" << delimiter
        << "vel_z" << delimiter
        << "acc_x" << delimiter
        << "acc_y" << delimiter
        << "acc_z" << delimiter
        << "jrk_x" << delimiter
        << "jrk_y" << delimiter
        << "jrk_z" << delimiter
        << "snp_x" << delimiter
        << "snp_y" << delimiter
        << "snp_z" << std::endl;
    ofs.close(); 
    ofs.clear();
}






void ForgetfulDrone::initRawRunPaths () {

    m_exp_dat_raw_rid_rgb_ = _exp_dat_raw_rid_rgb_ ();
    m_exp_dat_raw_rid_lbl_ = _exp_dat_raw_rid_lbl_ ();
    m_exp_dat_raw_rid_TXT = _exp_dat_raw_rid_TXT ();

    createDir (ROS_LOG_PREFIX, _exp_dat_raw_rid_ ());
    createDir (ROS_LOG_PREFIX, m_exp_dat_raw_rid_rgb_);
    createDir (ROS_LOG_PREFIX, m_exp_dat_raw_rid_lbl_);

    std::ofstream ofs;
    const char delimiter = ',';
    ofs.open (m_exp_dat_raw_rid_TXT, std::ios_base::app); 
    ofs
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
    ofs.close(); 
    ofs.clear();
}






void ForgetfulDrone::logRunInfo () {
    const char* scene = std::get <0> (ForgetfulSimulator::getUnityScenes ()->at (m_SceneIdx));
    const char* site = std::get <0> (ForgetfulSimulator::getSceneSites ()->at (m_SiteIdx));
    const char* track = std::get <0> (ForgetfulSimulator::getTrackTypes ()->at (m_TrackTypeIdx));
    const char* gen = std::get <0> (ForgetfulSimulator::getTrackGenerations ()->at (m_TrackGenIdx));
    const char* direc = std::get <0> (ForgetfulSimulator::getTrackDirections ()->at (m_TrackDirecIdx));
    const char* gate = std::get <0> (ForgetfulSimulator::getGateTypes ()->at (m_GateIdx));

    std::ostringstream margin; 
    if (p_MISSION == 3) margin << "\n\t\t- Iv. margin:   " << m_DaggerMargin << " m";

    std::ostringstream index;
    if (p_MISSION == 3) index << "\t\t- Index:        " << m_RunIdx << " {0, oo} (total),    " << m_RunRepIdx << " {0, oo} (repetition)";
    else                index << "\t\t- Index:        " << m_RunIdx << " {0, ?} (total),    " << m_RunRepIdx << " {0, " << p_NRUNREPS - 1 << "} (repetition)";

    std::cout << std::endl;
    ROSINFO("Start run" << std::endl
        << index.str () << std::endl
        << "\t\t- Location:     " << scene << ", " << site << std::endl
        << "\t\t- Track:        " << track << ", " << gen << ", " << direc << std::endl
        << "\t\t- Gate:         " << gate << std::endl
        << "\t\t- Max. speed:   " << m_MaxSpeed << " m/s"
        << margin.str ()
    );
    std::cout << std::endl;
};



void ForgetfulDrone::logMissionInfo (const bool& start) const {
    std::ostringstream msg; 
    msg << "Mission " << h_MISSIONS [p_MISSION] << (start? " started" : " completed");
    
    playAudio (msg.str ());

    std::cout << std::endl << std::endl;
    ROSINFO (msg.str ());
    std::cout << std::endl << std::endl;
}


void ForgetfulDrone::newRunReset () {
    m_ExpDecCnt = 0;
    m_ANNDecCnt = 0;
    m_LapIdx = -1;
    m_MainIterCnt = 0;
    m_LocTraj_SuccInfeasCnt = 0;
    m_LocTrajFeasibleCnt = 0;
    m_LocTraj = {Vec3(), Vec3(), Vec3(), Vec3()};
    m_LocTraj_t0 = {};
    m_RGBCnt = 0;
    m_SaveEnabled = false;
}



void ForgetfulDrone::buildSimulation (const int& track_idx) {
    fdLR srv;   srv.request.unity_scene = m_SceneIdx;
                srv.request.scene_site = m_SiteIdx;
                srv.request.track_type = m_TrackTypeIdx;
                srv.request.track_generation = m_TrackGenIdx;
                srv.request.track_direction = m_TrackDirecIdx;
                srv.request.gate_type = m_GateIdx;

                srv.request.track_idx = track_idx;

    while (!callRosSrv <fdLR> (ROS_LOG_PREFIX, m_SCL__simLoad, srv)) {
        ros::Duration (1.0).sleep ();
    }

    m_GateInitPoses = srv.response.gate_init_poses;
    m_DroneInitPose = srv.response.drone_init_pose;
}




void ForgetfulDrone::buildSimulation () {
    fdBS srv;   srv.request.unity_scene = m_SceneIdx;
                srv.request.scene_site = m_SiteIdx;
                srv.request.track_type = m_TrackTypeIdx;
                srv.request.track_generation = m_TrackGenIdx;
                srv.request.track_direction = m_TrackDirecIdx;
                srv.request.gate_type = m_GateIdx;

    while (!callRosSrv<fdBS>(ROS_LOG_PREFIX, m_SCL__simBuild, srv)) {
        ros::Duration(1.0).sleep();
    }

    m_GateInitPoses = srv.response.gate_init_poses;
    m_DroneInitPose = srv.response.drone_init_pose;
}

void ForgetfulDrone::setTrackWaypoints () {

    // Set waypoints of global trajectory from gate poses
    m_GloTrajWayps.clear(); 
    m_GloTrajWayps.reserve(m_GateInitPoses.size());
    for (const geometry_msgs::Pose& pose : m_GateInitPoses) {
        m_GloTrajWayps.push_back(EV3d___GMP(pose.position));
    }

    if (m_TrackTypeIdx == 1 && p_MISSION == 3) // Gap
        insertGapWaypoint();
}

void ForgetfulDrone::insertGapWaypoint () {
    int i0 = static_cast<int>(m_GloTrajWayps.size() / 2) - 1;
    int i1 = i0 + 1;

    const Eigen::Vector3d& wp_0 = m_GloTrajWayps.back();
    const Eigen::Vector3d& wp_gap_0 = m_GloTrajWayps[i0];
    const Eigen::Vector3d& wp_gap_1 = m_GloTrajWayps[i1];

    Eigen::Vector3d wp_gap_c = (wp_gap_0 + wp_gap_1) / 2;
    

    Eigen::Vector3d direction = wp_gap_c - wp_0; 
    direction.normalize();
    double distance = (wp_gap_0 -wp_gap_1).norm();

    Eigen::Vector3d wp_gap = wp_gap_c + direction * distance / 2;

    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>::iterator it = m_GloTrajWayps.begin();
    m_GloTrajWayps.insert(it + i1, wp_gap);
}


void ForgetfulDrone::initGateIdc () {
    m_CurrGateIdx = m_GloTrajWayps.size() - 1 - *ForgetfulSimulator::getDroneInitPoseNumGatesSetback();
    m_LastGateIdx = (m_CurrGateIdx + m_GloTrajWayps.size() - 1) % m_GloTrajWayps.size();
}



void ForgetfulDrone::findExpMaxHor () {
    const std::vector <Eigen::Vector3d, 
        Eigen::aligned_allocator <Eigen::Vector3d>>& wayps = m_GloTrajWayps;

    m_ExpMaxHor = 0.0;
    for (size_t i = 0; i < wayps.size (); i++) {
        size_t j = (i - 1 + wayps.size()) % wayps.size();
        double dist = (wayps[i] - wayps[j]).norm ();
        m_ExpMaxHor = std::max (m_ExpMaxHor, dist);
    }
}





void ForgetfulDrone::startSim () {
    fdStartS srv; 
    while (!callRosSrv <fdStartS> (ROS_LOG_PREFIX, m_SCL__simStart, srv)) 
        ros::Duration (1.0).sleep();
}


void ForgetfulDrone::stopSimulation () {
    fdStopS srv;
    while (!callRosSrv <fdStopS> (ROS_LOG_PREFIX, m_SCL__simStop, srv))
        ros::Duration (1.0).sleep();
}


bool ForgetfulDrone::startNavigation () {
    std::string m {"Start navigation"};
    playAudio (m); ROSINFO (m);
    
    switchNav (/*enabled*/true);

    m_rosTMR_MAINLOOP.start ();
    std::thread t1 (runMultiThreadedSpinner);
    m_LastGateTime = ros::Time::now ();  
    ros::Rate rate (10);
    
    while (m_LapIdx < m_RunNumLaps && m_NavEnabled) {
        rate.sleep ();

        ros::Duration T = ros::Time::now () - m_LastGateTime;
        int t_left = p_WPARR_MAXDUR - static_cast <int> (T.toSec ());
        
        ROS_INFO_STREAM_THROTTLE (1.0, 
            ROS_LOG_PREFIX << "   - " << t_left << " s left to pass target gate");
        
        if (t_left < 0) {
            std::ostringstream oss; oss << "No waypoint passed in " << p_WPARR_MAXDUR << " s -> abort run";
            ROSERROR (oss.str()); playAudio (oss.str ());
            switchNav (false);
            break;
        }
    }

    m_SaveEnabled = false;

    
    
    m_rosTMR_MAINLOOP.stop ();
    t1.detach ();
    t1.~thread ();

    if (m_NavEnabled) {
        std::ostringstream oss; oss << "Run " << m_RunIdx << " succeeded";
        playAudio (oss.str ()); ROSINFO (oss.str ());
        
        switchNav (false);
        return true;
    } else {
        std::ostringstream oss; oss << "Run " << m_RunIdx << " aborted";
        playAudio (oss.str ()); ROSINFO (oss.str ());

        return false;
    }
}




void ForgetfulDrone::runMission_trackGloTraj () {
    throw std::logic_error ("Function not implemented yet");
}

void ForgetfulDrone::runMission_testExp () {

    m_RunIdx = 0;
    initMainLoopTimer (&ForgetfulDrone::ROSCB_testExp);

    m_RunNumLaps = p_RUN_NUMLAPS;

    for (const int& unity_scene : p_SCENES)
    for (const int& scene_site : p_SITES)
    for (const int& track_type : p_TRACK_TYPES)
    for (const int& track_generation : p_TRACK_GENS)
    for (const int& track_direction : p_TRACK_DIRECS)
    for (const int& gate_type : p_GATES) 
    for (const double& loctraj_maxspeed: p_MAXSPEEDS) {
        

        m_SceneIdx = static_cast<uint8_t>(unity_scene);
        m_SiteIdx = static_cast<uint8_t>(scene_site);
        m_TrackTypeIdx = static_cast<uint8_t>(track_type);
        m_TrackGenIdx = static_cast<uint8_t>(track_generation);
        m_TrackDirecIdx = static_cast<uint8_t>(track_direction);
        m_GateIdx = static_cast<uint8_t>(gate_type);
        m_MaxSpeed = loctraj_maxspeed;
        
        
        
        newRunReset();

        do {
            buildSimulation ();
            setTrackWaypoints ();
            ros::Duration(1.0).sleep ();
        } while (!compGloTraj ());

        initGateIdc ();
        findExpMaxHor ();
        startSim ();
        rvizGloTraj ();

        
        if (!launchDrone ()) {
            ROS_WARN ("Drone failed to launch -> skip run");
        }
        if (!flyDroneToInitPose (true)) {
            ROS_WARN ("Drone failed to arrive at start -> skip run");
        }

        bool track_completed = startNavigation();
        stopSimulation ();
        flyDroneAboveTrack ();

        m_RunIdx++;
    }
}

void ForgetfulDrone::runMission_Racing () {
    if (!initExperiment ()) return;
    if (!initANN ()) return;

    initMainLoopTimer (std::get<1>(h_TORCH_INTERFACES[p_TORCH_INTERFACE_IDX]) /*&ForgetfulDrone::CB_DAGGER_Python*/);

    m_RunIdx = 0;
    m_RunNumLaps = p_RUN_NUMLAPS; //+

    for (const int& unity_scene : p_SCENES)
    for (const int& scene_site : p_SITES)
    for (const int& track_type : p_TRACK_TYPES)
    for (const int& track_generation : p_TRACK_GENS)
    for (const int& track_direction : p_TRACK_DIRECS)
    for (const int& gate_type : p_GATES) 
    for (const double& max_speed: p_MAXSPEEDS) 
    /*for (const double& dagger_margin : p_DAGGERMARGINS)*/ {

        m_SceneIdx = static_cast<uint8_t>(unity_scene);
        m_SiteIdx = static_cast<uint8_t>(scene_site);
        m_TrackTypeIdx = static_cast<uint8_t>(track_type);
        m_TrackGenIdx = static_cast<uint8_t>(track_generation);
        m_TrackDirecIdx = static_cast<uint8_t>(track_direction);
        m_GateIdx = static_cast<uint8_t>(gate_type);
        m_MaxSpeed = max_speed;
        //m_DaggerMargin  = dagger_margin;
        
        m_RunRepIdx = -1; //0 
        m_RunRepSuccCnt = 0; //bool repeat_run {true};

        while (++m_RunRepIdx < p_NRUNREPS /*repeat_run*/) {
            playAudio ("Start run " + std::to_string (m_RunIdx));

            //if (m_RunIdx == 0) {
            //    m_DaggerMargin = 0;
            //    m_RunNumLaps = p_DAGGER_FIRSTRUN_NUMLAPS;
            //    m_NavInputDisturbed = true;
            //}
            //else {
            //    m_DaggerMargin = dagger_margin;
            //    m_RunNumLaps = p_RUN_NUMLAPS;
            //    m_NavInputDisturbed = false;
            //}
            
            logRunInfo ();
            newRunReset ();

            if (RacingRecorded (RunID ())) {
                ROSINFO ("Run \"" << RunID () << " already recorded");
                
                //double eis = readExpIvShare (m_RunIdx);
                //ROSINFO ("Expert intervention share: " << 100 * eis 
                //    << " %   (threshold " << 100 * p_EIS_THRSH << " %)");
                //if (eis > p_EIS_THRSH) m_RunRepIdx ++;
                //else repeat_run = false;
                
                m_RunIdx++;
                continue;
            }

            initRacingPaths (); //initRawRunPaths
            
            //do {
                buildSimulation (m_RunRepIdx);
                setTrackWaypoints ();
                //ros::Duration(1.0).sleep ();
            //} while (!compGloTraj ());

            initGateIdc ();
            //initANN (); //findExpMaxHor ();
            startSim ();
            // rvizGloTraj ();
            
            if (!carryDroneBack ()) {
                ROS_ERROR ("Failed to carry drone back");
                ros::shutdown ();
            }

            if (!launchDrone ()) {
                ROS_WARN ("Drone failed to launch -> repeat run");
                stopSimulation ();
                m_RunRepIdx--;
                continue;
            }
            if (!flyDroneToInitPose (false)) {
                ROS_WARN ("Drone failed to arrive at start -> repeat run");
                stopSimulation ();
                m_RunRepIdx--;
                continue;
            }

            
            if (p_TORCH_INTERFACE_IDX == 0) {
                
                playAudio ("Start inference");
                forgetful_drones::Float srv; 
                srv.request.data = m_MaxSpeed;
                callRosSrv (ROS_LOG_PREFIX, m_SCL__fbInfer, srv);
            }
            else if (p_TORCH_INTERFACE_IDX == 1) {
                m_ANN.initHiddenState ();
            }

            bool track_completed = startNavigation ();
            stopSimulation ();

            
            
            if (track_completed) {
                playAudio ("Run successful");
                m_RunRepSuccCnt++;
            }
            
            addRacingRecord (RunID (), track_completed);
            m_RunIdx++;
        }

        std::cout << std::endl;
        ROSINFO("Run repetitions completed:  " << (float) m_RunRepSuccCnt / (m_RunRepIdx + 1) << " % successful");
        std::cout << std::endl;
    }
}

void ForgetfulDrone::runMission_DAGGER () {
    if (!initExperiment ()) return;
    if (!initANN ()) return;

    initMainLoopTimer (&ForgetfulDrone::CB_DAGGER_Python);

    m_RunIdx = 0;
    

    for (size_t ii = 0; ii < p_DAGGERMARGINS.size (); ii++)
    //for (const double& dagger_margin : p_DAGGERMARGINS)
    for (const int& unity_scene      : p_SCENES)
    for (const int& scene_site       : p_SITES)
    for (const int& track_type       : p_TRACK_TYPES)
    for (const int& track_gen        : p_TRACK_GENS)
    for (const int& track_direc      : p_TRACK_DIRECS)
    for (const int& gate_type        : p_GATES) 
    for (const double& max_speed     : p_MAXSPEEDS)
    {
        
        m_SceneIdx      = static_cast <uint8_t> (unity_scene);
        m_SiteIdx       = static_cast <uint8_t> (scene_site);
        m_TrackTypeIdx  = static_cast <uint8_t> (track_type);
        m_TrackGenIdx   = static_cast <uint8_t> (track_gen);
        m_TrackDirecIdx = static_cast <uint8_t> (track_direc);
        m_GateIdx       = static_cast <uint8_t> (gate_type);
        m_MaxSpeed      = max_speed;
        m_DaggerMargin  = p_DAGGERMARGINS [ii];

        m_RunRepIdx = 0;
        bool repeat_run {true};

        bool track_completed {false};
        while (repeat_run) {
            playAudio ("Start run " + std::to_string (m_RunIdx));

            if (m_RunIdx == 0) {
                m_DaggerMargin = 0;
                m_RunNumLaps = p_DAGGER_FIRSTRUN_NUMLAPS;
                m_NavInputDisturbed = true;
            }
            else {
                m_DaggerMargin = p_DAGGERMARGINS [ii];
                m_RunNumLaps = p_RUN_NUMLAPS;
                m_NavInputDisturbed = false;
            }

            logRunInfo ();
            newRunReset ();

            
            if (BuildRecorded (RunID ())) {
                ROSINFO ("Run \"" << RunID () << "already recorded");

                double eis = readExpIvShare (m_RunIdx);
                ROSINFO ("Expert intervention share: " << 100 * eis 
                    << " %   (threshold " << 100 * p_EXPINTVSHARES [ii] << " %)");
                if (eis > p_EXPINTVSHARES [ii]) m_RunRepIdx ++;
                else repeat_run = false;
                
                m_RunIdx ++;
                continue;
            }

            initRawRunPaths ();
    
            do {
                buildSimulation ();
                setTrackWaypoints ();
                ros::Duration(1.0).sleep ();
            } while (!compGloTraj ());

            initGateIdc ();
            findExpMaxHor ();
            startSim ();
            rvizGloTraj ();

            if (!carryDroneBack ()) {
                ROS_ERROR ("Failed to carry drone back");
                ros::shutdown ();
            }

            if (!launchDrone ()) {
                ROS_WARN ("Drone failed to launch -> repeat run");
                stopSimulation ();
                continue;
            }
            if (!flyDroneToInitPose (false)) {
                ROS_WARN ("Drone failed to arrive at start -> repeat run");
                stopSimulation ();
                continue;
            }

            if (m_RunIdx > 0) {
                if (p_TORCH_INTERFACE_IDX == 0) {
                    
                    playAudio ("Start inference");
                    forgetful_drones::Float srv; 
                    srv.request.data = m_MaxSpeed;
                    callRosSrv (ROS_LOG_PREFIX, m_SCL__fbInfer, srv);
                }
                if (p_TORCH_INTERFACE_IDX == 1) {
                    m_ANN.initHiddenState();
                }
            }

            track_completed = startNavigation ();
            stopSimulation ();

            //if (track_completed) {
                
                playAudio ("Build run");
                forgetful_drones::String srv0;
                srv0.request.data = RunID ();
                callRosSrv (ROS_LOG_PREFIX, m_SCL__fbBuild, srv0);

                playAudio ("Start training");
                forgetful_drones::Int srv1;
                srv1.request.data = p_DAGGER_NEPOCHS;
                callRosSrv(ROS_LOG_PREFIX, m_SCL__fbTrain, srv1);

                
                double eis = readExpIvShare (m_RunIdx);
                ROSINFO ("Last run's expert intervention share: " << 100 * eis 
                    << " %   (threshold " << 100 * p_EXPINTVSHARES [ii] << " %)");                
                

                repeat_run = (eis > p_EXPINTVSHARES [ii]);
                if (!track_completed) repeat_run = true;
                m_RunRepIdx ++;
                m_RunIdx ++;
            //} else {
            //    playAudio ("Race track not completed. Delete data and repeat run.");
            //    ROSINFO ("Race track not completed. Delete data and repeat run.");
            //    std::experimental::filesystem::remove_all (_exp_dat_raw_rid_ ());
            //}
        }
    }
}





double ForgetfulDrone::readExpIvShare (const int& run_idx) {
    return openJSON (_exp_out_BRC ()) [run_idx] ["eis"];
}

bool ForgetfulDrone::BuildRecorded (const std::string& run_id) {
    if (! isFile (_exp_out_BRC ())) return false;
    
    for (auto rec : openJSON (_exp_out_BRC ())) 
        if (rec ["id"] == run_id) return true;
    return false;
}

json ForgetfulDrone::openJSON (const std::string& path) {
    std::ifstream ifs (path);
    return nlohmann::json::parse (ifs);
}

bool ForgetfulDrone::RacingRecorded (const std::string& run_id) {
    if (! isFile (_exp_rac_RRC ())) return false;
    
    for (auto rec : openJSON (_exp_rac_RRC ()))
        if (rec ["id"] == run_id) return true;
    return false;
}

void ForgetfulDrone::addRacingRecord (const std::string& run_id, const bool& completed) {
    json j;
    j ["id"] = run_id;
    j ["completed"] = completed;

    if (! isFile (_exp_rac_RRC ())) {
        std::ofstream f (_exp_rac_RRC ());
        f << "[]";
    }
   
    json J = openJSON (_exp_rac_RRC ());
    J.push_back (j);

    std::ofstream f (_exp_rac_RRC ());
    f << J.dump (4);
}







//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////// ROS TIMER FUNCTIONS //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



void ForgetfulDrone::ROSCB_testExp (const ros::TimerEvent& te) {
    checkTimerPeriod (ROS_LOG_PREFIX, te, 1 / p_MAIN_FREQ);
    
    runStatusTracker ();
    fetchRGBData ();
    
    runExp ();
    m_NavInput = m_ExpOut;
    
    bool _;
    runNav (_, &ForgetfulDrone::noIntervention);

    if (p_RVIZ_LBLRGB_ENABLED) rvizLblRGB();
}

void ForgetfulDrone::CB_Racing_Cpp (const ros::TimerEvent& te) {
    checkTimerPeriod (ROS_LOG_PREFIX, te, 1 / p_MAIN_FREQ);
    runStatusTracker ();

    fetchRGBData ();
    fetchIMUData ();
    //fetchCtrlCmdData ();
    
    //runBrain ();
    float opt_input[9] = {
        (float) m_RGB_dt, 
        (float) m_IMU_dt,
        (float) m_IMU[0],
        (float) m_IMU[1],
        (float) m_IMU[2],
        (float) m_IMU[3],
        (float) m_IMU[4],
        (float) m_IMU[5],
        (float) m_MaxSpeed,
    };

    if (!isEmpty (ROS_LOG_PREFIX, m_RGB))
        m_BrainOutput = m_ANN.forward (m_RGB, opt_input); //triggerBrain ();
    m_NavInput = m_BrainOutput;

    bool intervened {false};
    //runExp ();
    runNav(intervened, &ForgetfulDrone::noIntervention/*expIntervention*/);

    saveOdometry (); //runDataSaver (intervened);

    if (p_RVIZ_LBLRGB_ENABLED) rvizLblRGB();
}

void ForgetfulDrone::CB_Racing_Python (const ros::TimerEvent& te) {
    checkTimerPeriod (ROS_LOG_PREFIX, te, 1 / p_MAIN_FREQ);
    runStatusTracker ();
    
    fetchRGBData ();
    fetchIMUData ();
    //fetchCtrlCmdData ();
    
    triggerBrain ();
    m_NavInput = m_BrainOutput;

    bool intervened {false};
    //runExp ();
    runNav (intervened, &ForgetfulDrone::noIntervention/*expIntervention*/);
    
    saveOdometry (); //runDataSaver (intervened);

    if (p_RVIZ_LBLRGB_ENABLED) rvizLblRGB();
}

void ForgetfulDrone::ROSCB_TRAINING_DATA_GENERATION (const ros::TimerEvent& te) {
    checkTimerPeriod(ROS_LOG_PREFIX, te, 1 / p_MAIN_FREQ);
    
    runStatusTracker();
    runExp();
    m_NavInput = m_ExpOut;
    runDataSaver(true);
    bool _;
    runNav(_, &ForgetfulDrone::noIntervention);
}

void ForgetfulDrone::fetchRGBData () {
    m_RGBMtx.lock ();
        m_RGB = m_RGBPtr->image.clone ();
        m_RGB_dt = m_RGBPtr->header.stamp.toSec () - m_RGBLastStampTime;
        m_RGBLastStampTime = m_RGBPtr->header.stamp.toSec ();
    m_RGBMtx.unlock ();
}

void ForgetfulDrone::fetchIMUData () {
    m_IMUMtx.lock ();
        m_IMU = {
            m_IMUPtr->linear_acceleration.x,
            m_IMUPtr->linear_acceleration.y,
            m_IMUPtr->linear_acceleration.z,
            m_IMUPtr->angular_velocity.x,
            m_IMUPtr->angular_velocity.y,
            m_IMUPtr->angular_velocity.z,
        };
        m_IMU_dt = m_IMUPtr->header.stamp.toSec () - m_IMULastStampTime;
        m_IMULastStampTime = m_IMUPtr->header.stamp.toSec ();
    m_IMUMtx.unlock ();
}

void ForgetfulDrone::fetchCtrlCmdData () {
    m_CtrlCmdMtx.lock ();
    m_CtrlCmd = {
        m_CtrlCmdPtr->bodyrates.x,
        m_CtrlCmdPtr->bodyrates.y,
        m_CtrlCmdPtr->bodyrates.z,
        m_CtrlCmdPtr->angular_accelerations.x,
        m_CtrlCmdPtr->angular_accelerations.y,
        m_CtrlCmdPtr->angular_accelerations.z,
        m_CtrlCmdPtr->collective_thrust
    };
    m_CtrlCmdMtx.unlock ();
}



void ForgetfulDrone::CB_DAGGER_Python (const ros::TimerEvent& te) {
    checkTimerPeriod (ROS_LOG_PREFIX, te, 1 / p_MAIN_FREQ); 
    runStatusTracker ();

    fetchRGBData ();
    fetchIMUData ();
    fetchCtrlCmdData ();

    triggerBrain ();
    m_NavInput = m_BrainOutput;
    
    bool intervened {false};
    runExp ();
    runNav (intervened, &ForgetfulDrone::expIntervention);

    runDataSaver (intervened);
    
    if (p_RVIZ_LBLRGB_ENABLED) rvizLblRGB ();
}




//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////// GLOBAL TRAJECTORY //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////





bool ForgetfulDrone::compGloTraj () {

    std::cout << std::endl;
    ROSINFO("Compute Global Trajectory" << std::endl
        << "\t\t- # WAYPOINTS:        " << static_cast<int>(m_GloTrajWayps.size()) << std::endl
        << "\t\t- POLYNOMIAL ORDER:   " << p_GLOBAL_TRAJECTORY_POLYNOMIAL_ORDER << std::endl
        << "\t\t- CONTINUITY ORDER:   " << p_GLOBAL_TRAJECTORY_CONTINUITY_ORDER << std::endl
        << "\t\t- MAX. SPEED:         " << p_GLOBAL_TRAJECTORY_MAX_SPEED << " m/s" << std::endl
        << "\t\t- MAX. THRUST:        " << p_GLOBAL_TRAJECTORY_MAX_THRUST << " m/s^2" << std::endl
        << "\t\t- MAX. RP-RATE:       " << p_GLOBAL_TRAJECTORY_MAX_ROLL_PITCH_RATE << " Hz" << std::endl
        << "\t\t- SAMPLING TIME:      " << 1 / p_MAIN_FREQ << " s"
    );
    std::cout << std::endl;

    std::shared_ptr<ForgetfulGlobalTrajectory<long double>> gt_ptr
        = std::make_shared<ForgetfulGlobalTrajectory<long double>>(
            m_GloTrajWayps, 
            p_GLOBAL_TRAJECTORY_POLYNOMIAL_ORDER, 
            p_GLOBAL_TRAJECTORY_CONTINUITY_ORDER, 
            p_GLOBAL_TRAJECTORY_MAX_SPEED, 
            p_GLOBAL_TRAJECTORY_MAX_THRUST, 
            p_GLOBAL_TRAJECTORY_MAX_ROLL_PITCH_RATE, 
            1 / p_MAIN_FREQ,
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

    std::cout << std::endl;
    ROSINFO("Computed Global Trajectory" << std::endl
        << "\t\t- # STATES:      " << static_cast<int>(m_GloTraj.size()) << std::endl
        << "\t\t- DURATION:      " << static_cast<int>(m_GloTraj.size()) / p_MAIN_FREQ << " s" << std::endl
        << "\t\t- SPEED RANGE:   [" << static_cast<double>(gt_ptr->m_Traj_MinSpeed) << ", " << static_cast<double>(gt_ptr->m_Traj_MaxSpeed) << "] m/s" << std::endl
        << "\t\t- THRUST RANGE:  [" << static_cast<double>(gt_ptr->m_Traj_MinThrust) << ", " << static_cast<double>(gt_ptr->m_Traj_MaxThrust) << "] m/s^2" << std::endl
        << "\t\t- RP-RATE RANGE: [" << static_cast<double>(gt_ptr->m_Traj_MinRollPitchRate) << ", " << static_cast<double>(gt_ptr->m_Traj_MaxRollPitchRate) << "] 1/s"
    );
    std::cout << std::endl;

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

    m_PUB__rvGloTraj.publish(marker);
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////// WAYPOINT UPDATER //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


void ForgetfulDrone::runStatusTracker () {

    // Distance from reference state to current and last gate waypoint.
    m_CurrGateDist = (m_RefStatePos_WRF - m_GloTrajWayps [m_CurrGateIdx]).norm();
    m_LastGateDist = (m_RefStatePos_WRF - m_GloTrajWayps [m_LastGateIdx]).norm();
    
    if (m_CurrGateDist < p_GATE_THRESH_DIST) updStatus();
    
    rvizCurrGate();
}


void ForgetfulDrone::updStatus () {
    ROSINFO ("Gate " << m_CurrGateIdx << " {0, ..., " << m_GloTrajWayps.size() - 1 << "} passed");
    m_LastGateTime = ros::Time::now();

    ++m_CurrGateIdx %= m_GloTrajWayps.size();
    m_LastGateIdx = (m_CurrGateIdx - 1 + m_GloTrajWayps.size()) % m_GloTrajWayps.size();
    
    if (m_CurrGateIdx == 0) 
        ROSINFO("Lap " << ++m_LapIdx << " {0, ..., " << m_RunNumLaps - 1 << "} started");
}

void ForgetfulDrone::rvizCurrGate () {
    rvizPosition (m_GloTrajWayps [m_CurrGateIdx], VisPosTypes::CURRGATE, m_PUB__rvNavPoints);
}





//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////// NAVIGATOR //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


void ForgetfulDrone::noIntervention (Eigen::Vector3d& _0, double& _1, bool& _2) {}

void ForgetfulDrone::expIntervention (Eigen::Vector3d& wayp_ARF, double& speed, bool& intervened) {
    
    // waypoint in world ref sys
    kindr::minimal::QuatTransformation ARF_PRF; 
    tf::poseMsgToKindr (GMPose___EV3d (wayp_ARF), &ARF_PRF);
    Eigen::Vector3d wayp_WRF = (p_WRF_ARF * ARF_PRF).getPosition();
    
    // distance to glo traj
    double dist; size_t _;
    minDist2GloTraj (wayp_WRF, dist, _);
    
    // If waypoint not within margin, intervene with expert
    if (dist > m_DaggerMargin * (m_MaxSpeed / 5.0 + 1.0 / 5.0)) {
        
        m_NavInput = m_ExpOut;
        if (m_NavInputDisturbed) m_NavInput
            += Eigen::Vector3d::Ones() * p_NAV_INPUTDISTURBAMP * std::sin (0.5 * ros::WallTime::now ().toSec ());    
        
        procNavInput (wayp_ARF, speed);
        if (m_LapIdx >= 0) m_ExpDecCnt++;
        intervened = true;

    } else {
        if (m_LapIdx >= 0) m_ANNDecCnt++;
        intervened = false;
    }
}




void ForgetfulDrone::runNav (
    bool& intervened, 
    void (forgetful_drone::ForgetfulDrone::*iv_fct)(Eigen::Vector3d& _0, double& _1, bool& _2)
) {

    if (!m_NavEnabled) {
        ROS_INFO_STREAM_THROTTLE (1.0, ROS_LOG_PREFIX << "Navigator disabled");
        return;
    }

    double div = (m_RefState_ARF.position - m_ARF_LRF.getPosition ()).norm ();
    if (div > p_NAV_MAX_DIV) {
        std::string m {"Drone diverged from reference state"};
        ROSERROR (m); playAudio (m);
        switchNav (false);
        return;
    }

    if (m_MainIterCnt % p_NAV_REPLAN_ITER_CNT == 0) {

        // Compute goal position and speed to reach goal position from navigator input
        Eigen::Vector3d wayp_ARF; 
        double speed;
        procNavInput (wayp_ARF, speed);


        (this->*iv_fct) (wayp_ARF, speed, intervened);


        if (compLocTraj (wayp_ARF, speed)) {
            m_LocTraj_SuccInfeasCnt = 0;
            m_LocTraj_t0 = ros::Time::now ();
            rvizLocTraj ();
        } else {
            if (m_LocTraj_SuccInfeasCnt < p_LOCTRAJ_MAXSUCCINFEASCNT) {
                ROSWARN ("# successive infeasible local trajectories: " << ++m_LocTraj_SuccInfeasCnt 
                    << " {1, " << p_LOCTRAJ_MAXSUCCINFEASCNT << "}");
            } else {
                std::string m {"Max. number of successively infeasible local trajectories reached"};
                ROSERROR (m); playAudio (m);
                switchNav (false);
                return;
            }
        }
    }
    
    pubRefState ();
    rvizRefState ();
    
    m_MainIterCnt++;
}












void ForgetfulDrone::rvizRefState () {
    rvizState (
        m_RefStatePos_WRF, 
        m_RefState_ARF.acceleration, 
        VisPosTypes::REFERENCE, 
        m_PUB__rvNavPoints
    );
}

void ForgetfulDrone::switchNav (const bool& enabled) {
    std::string prv = m_NavEnabled? "ENABLED" : "DISABLED";

    auto pubInitRefState = [this] () {
        // To use autopilot in REFERENCE_CONTROL state,
        // the first published reference state must be close to current pose.
        m_RefState_ARF = quadrotor_common::TrajectoryPoint ();
        m_RefState_ARF.position = m_ARF_LRF.getPosition ();
        m_RefState_ARF.heading = Yaw_From_EQd (m_ARF_LRF.getEigenQuaternion ().normalized ());
        m_PUB__apRefState.publish( m_RefState_ARF.toRosMessage() );
        ros::Duration (2.0).sleep();
    };

    auto enable = [prv, pubInitRefState, this] () {
        pubInitRefState ();
        m_NavEnabled = true;
        ROSINFO ("Navigator: " << prv << " -> " << "ENABLED");
        
    };

    auto disable = [prv, pubInitRefState, this] () {
        m_NavEnabled = false;
        pubInitRefState ();
        ROSINFO ("Navigator: " << prv << " -> " << "DISABLED");
    };

    enabled? enable () : disable ();
    ros::Duration (1.0).sleep ();
}


void ForgetfulDrone::pubRefState()
{
    double t = (ros::Time::now() - m_LocTraj_t0).toSec() + 1 / p_MAIN_FREQ;
    
    // Set reference state from local trajectory
    m_RefState_ARF = quadrotor_common::TrajectoryPoint();
    m_RefState_ARF.position        = EV3d___Vec3 (m_LocTraj.GetPosition (t));
    m_RefState_ARF.velocity        = EV3d___Vec3 (m_LocTraj.GetVelocity (t));
    m_RefState_ARF.acceleration    = EV3d___Vec3 (m_LocTraj.GetAcceleration (t));
    m_RefState_ARF.jerk            = EV3d___Vec3 (m_LocTraj.GetJerk (t));
    m_RefState_ARF.snap            = {};
    m_RefState_ARF.heading = std::atan2 (m_RefState_ARF.velocity.y (), m_RefState_ARF.velocity.x ());

    // Set position of reference state in world RF
    kindr::minimal::QuatTransformation ARF_RRF; 
    tf::poseMsgToKindr (GMPose___EV3d (m_RefState_ARF.position), &ARF_RRF);
    m_RefStatePos_WRF = (p_WRF_ARF * ARF_RRF).getPosition();
    
    // Publish reference state to autopilot
    m_PUB__apRefState.publish (m_RefState_ARF.toRosMessage());
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

    
    const Eigen::Quaterniond Q = p_WRF_ARF.getEigenQuaternion();
    const Eigen::Vector3d T = p_WRF_ARF.getPosition();

    for (double t = 0.0; t <= VisDuration; t += 1/p_RVIZ_LOCAL_TRAJECTORY_SAMPLING_FREQUENCY)
        {
            Eigen::Vector3d Pos = Q * (EV3d___Vec3(m_LocTraj.GetPosition(t))) + T;
            Eigen::Vector3d Vel = Q * EV3d___Vec3(m_LocTraj.GetVelocity(t));

            msg.points.push_back(GMPoint__from__EV3d(Pos));
            msg.points.push_back(GMPoint__from__EV3d(Pos + Vel/20.0));  
        }

    m_PUB__rvLocTraj.publish(msg);
}


bool ForgetfulDrone::compLocTraj (
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
        m_MaxSpeed
    });

    // Duration of the trajectory
    const double lin_duration = lin_dist / act_speed_to_target;

    // Compute trajectory
    RQTG::RapidTrajectoryGenerator loc_traj {pos_0_ARF, vel_0_ARF, acc_0_ARF, Gravity};
    loc_traj.SetGoalPosition (pos_1_ARF);
    loc_traj.Generate (lin_duration);
    
    // Take/discard computed trajectory if feasible/infeasible
    if (checkFeasibility (loc_traj)) {
        m_LocTraj = loc_traj;
        return true;
    } else {
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
        ROSWARN ("Local trajectory violates thrust range:[" 
            << p_LOCAL_TRAJECTORY_MIN_THRUST << ", " << p_LOCAL_TRAJECTORY_MAX_THRUST << "] m/s^2 and/or max. body rates:"
            << p_LOCAL_TRAJECTORY_MAX_BODY_RATES << " 1/s (checked with sampling time:"
            << p_LOCAL_TRAJECTORY_INPUT_FEASIBILITY_CHECK_MIN_SAMPLING_TIME << " s)");
        return false;
    }
    if (sfr_floor != RTG::StateFeasible) {
        ROSWARN ("Local trajectory violates min. altitude: "
            << p_LOCAL_TRAJECTORY_MIN_ALTITUDE << " m");
        return false;
    }
    if (sfr_ceiling != RTG::StateFeasible) {
        ROSWARN ("Local trajectory violates max. altitude: "
            << p_LOCAL_TRAJECTORY_MAX_ALTITUDE << " m");
        return false;
    }

    return true;
}



void ForgetfulDrone::procNavInput (Eigen::Vector3d& wayp_ARF, double& speed) {
    const double& x_IRF = m_NavInput.x();
    const double& y_IRF = m_NavInput.y();
    const double& normSpeed = m_NavInput.z();

    speed = std::max (p_LOCTRAJ_MINSPEED, m_MaxSpeed * normSpeed);

    // Compute linear distance from drone to goal position
    double dist = std::max (
        p_LOCTRAJ_MINDIST,
        std::min (p_LOCTRAJ_MAXDIST, p_LOCTRAJ_DUR * speed)
    );

    // Compute goal position
    wayp_ARF = ARF___LRF (LRF____IRF (x_IRF, y_IRF, dist));
}





Eigen::Vector3d ForgetfulDrone::ARF___LRF (const Eigen::Vector3d& pos)
{
    kindr::minimal::QuatTransformation T_LRF_PRF;
    tf::poseMsgToKindr (GMPose___EV3d (pos), &T_LRF_PRF);
    
    return (m_ARF_LRF * T_LRF_PRF).getPosition();
}

Eigen::Vector3d ForgetfulDrone::LRF___WRF (const Eigen::Vector3d& pos) {
    
    kindr::minimal::QuatTransformation T_WRF_PRF;
    tf::poseMsgToKindr (GMPose___EV3d (pos), &T_WRF_PRF);
    
    return (m_WRF_LRF.inverse () * T_WRF_PRF).getPosition ();
}

Eigen::Vector2d ForgetfulDrone::IRF___LRF (const Eigen::Vector3d& pos) {
    
    double yaw = std::atan2 (pos.y(), pos.x());
    double pitch = std::atan2 (pos.z(), pos.norm());

    double x_IRF = capMinMax <double> (-yaw / p_CAM_HALFYAWAOV, -1.0, 1.0);
    double y_IRF = capMinMax <double> (pitch / p_CAM_HALFPITCHAOV, -1.0, 1.0 );

    return {x_IRF, y_IRF};
}

Eigen::Vector3d ForgetfulDrone::LRF____IRF (const double& x, const double& y, const double& dist) {
    double yaw = - p_CAM_HALFYAWAOV * x;
    double pitch = p_CAM_HALFPITCHAOV * y;
    
    return {
        dist * cos (pitch) * cos (yaw),
        dist * cos (pitch) * sin (yaw),
        dist * sin (pitch)
    };
}









void ForgetfulDrone::runExp () {
    double horizon = capMinMax <double> (
        std::min (m_CurrGateDist, m_LastGateDist), 
        p_EXPERT_MIN_HORIZON, m_ExpMaxHor
    );

    findExpState (); 
    findWaypState (horizon);
    rvizExpAndWaypState ();

    Eigen::Vector2d target_pos_IRF = IRF___LRF (
        LRF___WRF (m_GloTraj [m_WaypStateIdx].position));

    findSpeedState ();
    double norm_speed = m_GloTraj [m_SpeedStateIdx].velocity.norm () / m_Expert_MaxSpeed;

    m_ExpOut = {
        target_pos_IRF.x(), target_pos_IRF.y(), norm_speed
    };
}



void ForgetfulDrone::triggerBrain () {
    m_PUB__fbTrigger.publish (std_msgs::Empty ());
}






/*void ForgetfulDrone::runBrain () {
    if (isEmpty(ROS_LOG_PREFIX, m_RGB)) return;
    
    //ROSDEBUG("Resize camera frame.");
    cv::Mat bgr_resized;
    cv::resize(
        m_RGB, 
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
        m_RGB_dt,
        m_IMU_dt,
        m_IMU[0],
        m_IMU[1],
        m_IMU[2],
        m_IMU[3],
        m_IMU[4],
        m_IMU[5],
        m_MaxSpeed,
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
*/






void ForgetfulDrone::findExpState () {
    // find with projection
    auto criterion = [this] () {
        const Eigen::Vector3d& anc = m_GloTraj [m_ExpStateIdx - 1].position;
        const Eigen::Vector3d& exp = m_GloTraj [m_ExpStateIdx].position;
        const Eigen::Vector3d& ref = m_RefStatePos_WRF;
        Eigen::Vector3d anc2exp = exp - anc;
        Eigen::Vector3d anc2ref = ref - anc;
        return anc2ref.dot (anc2exp) > anc2exp.squaredNorm ();
    };
    while (criterion ()) {
        m_ExpStateIdx++;
        m_ExpStateIdx %= m_GloTraj.size ();
    }

    // if too far away, find with min distance
    double div = (m_RefStatePos_WRF - m_GloTraj [m_ExpStateIdx].position).norm ();
    if (div > p_EXP_THRESHOLD_DIV) {
        double _; minDist2GloTraj (m_RefStatePos_WRF, _, m_ExpStateIdx);
    }
}




void ForgetfulDrone::minDist2GloTraj (const Eigen::Vector3d pos, double& min_dist, size_t& state_idx) {
    
    min_dist = std::numeric_limits<double>::max ();
    for (size_t i = 0; i < m_GloTraj.size (); i++) {
        
        double dist = (m_GloTraj [i].position - pos).norm ();
        if (dist < min_dist) {
            min_dist = dist;
            state_idx = i;
        }
    }
}





void ForgetfulDrone::findWaypState (const double& Horizon) {

    double dist;
    m_WaypStateIdx = m_ExpStateIdx;
    do {
        ++m_WaypStateIdx %= m_GloTraj.size();
        dist = (m_GloTraj [m_WaypStateIdx].position - m_GloTraj[m_ExpStateIdx].position).norm();
    } while ( dist < Horizon );
}


void ForgetfulDrone::findSpeedState () {

    m_SpeedStateIdx = m_ExpStateIdx;
    double dist;
    do {
        ++m_SpeedStateIdx %= m_GloTraj.size ();
        dist = ( m_GloTraj [m_SpeedStateIdx].position - m_GloTraj [m_ExpStateIdx].position).norm();
    } while ( dist < p_EXPERT_SPEED_HORIZON );
}








void ForgetfulDrone::rvizExpAndWaypState () {
    rvizPosition (m_GloTraj [m_ExpStateIdx].position,  VisPosTypes::EXPERT,  m_PUB__rvNavPoints);
    rvizPosition (m_GloTraj [m_WaypStateIdx].position, VisPosTypes::HORIZON, m_PUB__rvNavPoints);
}




void ForgetfulDrone::saveOdometry () {
    // start saving data after passing the first gate
    if (!m_SaveEnabled && m_CurrGateIdx == 0) {
        m_SaveEnabled = true;
        ROSINFO("Data saving enabled");
    } if (!m_SaveEnabled) return;

    std::ofstream ofs;
    const char delimiter = ',';
    ofs.open (m_exp_rac_RID, std::ios_base::app); 
    ofs
        << ros::Time::now ().toSec () << delimiter
        << m_WRF_LRF.getPosition ().x () << delimiter
        << m_WRF_LRF.getPosition ().y () << delimiter
        << m_WRF_LRF.getPosition ().z () << delimiter
        << "" << delimiter
        << "" << delimiter
        << "" << delimiter
        << "" << delimiter
        << "" << delimiter
        << "" << delimiter
        << "" << delimiter
        << "" << delimiter
        << "" << delimiter
        << "" << delimiter
        << "" << delimiter
        << "" << std::endl;
    ofs.close(); 
    ofs.clear();
}



void ForgetfulDrone::runDataSaver (const bool& expert_intervened) {
    
    // start saving data after passing the first gate
    if (!m_SaveEnabled && m_CurrGateIdx == 0) {
        m_SaveEnabled = true;
        ROSINFO("Data saving enabled");
    } if (!m_SaveEnabled) return;

    // save rgb
    if (isEmpty(ROS_LOG_PREFIX, m_RGB)) return;
    saveCVMat (
        ROS_LOG_PREFIX, 
        m_exp_dat_raw_rid_rgb_ + "/" + asSeqNo (5, m_RGBCnt++) + ".jpg", 
        m_RGB
    );

    // save txt
    std::ofstream ofs;
    const char delimiter = ',';
    ofs.open (m_exp_dat_raw_rid_TXT, std::ios_base::app); 
    ofs
        << expert_intervened    << delimiter
        << m_RGB_dt             << delimiter
        << m_IMU_dt             << delimiter
        << m_IMU [0]            << delimiter
        << m_IMU [1]            << delimiter
        << m_IMU [2]            << delimiter
        << m_IMU [3]            << delimiter
        << m_IMU [4]            << delimiter
        << m_IMU [5]            << delimiter
        << m_MaxSpeed           << delimiter
        << m_ExpOut.x ()        << delimiter
        << m_ExpOut.y ()        << delimiter
        << m_ExpOut.z ()        << delimiter
        << m_CtrlCmd [0]        << delimiter
        << m_CtrlCmd [1]        << delimiter
        << m_CtrlCmd [2]        << delimiter
        << m_CtrlCmd [3]        << delimiter
        << m_CtrlCmd [4]        << delimiter
        << m_CtrlCmd [5]        << delimiter
        << m_CtrlCmd [6]        << std::endl;
    ofs.close(); 
    ofs.clear();
}








//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////// NOT IMPLEMENTED YET //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


















/*
std::vector<quadrotor_common::TrajectoryPoint>
compGloTraj(
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
    //for (const Eigen::Vector3d& Waypoint : m_GloTrajWayps)
    //{
    //    MinAxVals.x() = std::min(MinAxVals.x(), Waypoint.x());
    //    MinAxVals.y() = std::min(MinAxVals.y(), Waypoint.y());
    //    MinAxVals.z() = std::min(MinAxVals.z(), Waypoint.z());
    //}
    //for (Eigen::Vector3d& Waypoint : m_GloTrajWayps) Waypoint -= MinAxVals;




    std::cout<<std::endl;
    for (const Eigen::Vector3d& Waypoint : Waypoints)
    {
        std::cout << "\t" << Waypoint.x() << "   \t" << Waypoint.y() << "   \t" << Waypoint.z() << std::endl;
    }
    std::cout<<std::endl;



    //// --- Set ´m_ExpMaxHor´ <- max dist between waypoints ---
    //m_ExpMaxHor = 0.0;
    //size_t LastWaypointIdx = m_GloTrajWayps.size() - 1;
    //for (size_t WaypointIdx = 0; WaypointIdx < m_GloTrajWayps.size(); WaypointIdx++)
    //{
    //    m_ExpMaxHor = std::max( 
    //        m_ExpMaxHor,
    //        (m_GloTrajWayps[ WaypointIdx ] - m_GloTrajWayps[ LastWaypointIdx ]).norm() );
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
    //    static_cast<int>( m_GloTrajWayps.size() ),
    //    p_GLOBAL_TRAJECTORY_POLYNOMIAL_ORDER,
    //    p_GLOBAL_TRAJECTORY_CONTINUITY_ORDER,
    //    m_GTMinWeightVel,
    //    m_GTMinWeightAcc,
    //    m_GTMinWeightJerk,
    //    m_GTMinWeightSnap,
    //    p_GLOBAL_TRAJECTORY_MAX_SPEED,
    //    p_GLOBAL_TRAJECTORY_MAX_THRUST,
    //    p_GLOBAL_TRAJECTORY_MAX_ROLL_PITCH_RATE,
    //    1 / p_MAIN_FREQ
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
    //m_CurrGateIdx = m_WUInitWaypointIdxForFIG8;

    return GlobalTrajectory;
}*/


}