#pragma once

// forgetful_drone
#include "forgetful_drones/forgetful_helpers.hpp"
#include "forgetful_drones/BuildDroneRacingSimulation.h"
#include "forgetful_drones/StartDroneRacingSimulation.h"
#include "forgetful_drones/StopDroneRacingSimulation.h"
#include "forgetful_drones/BuildSimulation.h"
#include <minkindr_conversions/kindr_msg.h>
#include <minkindr_conversions/kindr_tf.h>

// ros
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/Empty.h>
#include <rviz/SendFilePath.h>

// gazebo_ros
#include <gazebo_msgs/SpawnModel.h>
#include <gazebo_msgs/ModelStates.h>
#include <gazebo_msgs/DeleteModel.h>

// standard libraries
#include <assert.h>
#include <Eigen/Dense>
#include <chrono>
#include <cmath>
#include <cstring>
#include <fstream>
#include <iostream>
#include <iterator>
#include <sstream>
#include <thread>
#include <vector>
#include <memory>
#include <random>

// flightlib
#include "flightlib/bridges/unity_bridge.hpp"
#include "flightlib/bridges/unity_message_types.hpp"
#include "flightlib/common/quad_state.hpp"
#include "flightlib/common/types.hpp"
#include "flightlib/objects/quadrotor.hpp"
#include "flightlib/objects/static_gate.hpp"
#include "flightlib/sensors/rgb_camera.hpp"

// trajectory
#include <polynomial_trajectories/minimum_snap_trajectories.h>
#include <polynomial_trajectories/polynomial_trajectories_common.h>
#include <polynomial_trajectories/polynomial_trajectory.h>
#include <polynomial_trajectories/polynomial_trajectory_settings.h>
#include <quadrotor_common/trajectory_point.h>

// rpg quadrotor
#include <autopilot/autopilot_helper.h>
#include <autopilot/autopilot_states.h>
#include <quadrotor_common/parameter_helper.h>
#include <quadrotor_msgs/AutopilotFeedback.h>

#include "matplotlibcpp/matplotlibcpp.h"









namespace forgetful_drone { 
    
class ForgetfulSimulator {

public:

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    //ForgetfulSimulator () : ForgetfulSimulator (ros::NodeHandle(), ros::NodeHandle("~")) {}
    ForgetfulSimulator (const bool& as_ros_node=true) : ForgetfulSimulator (ros::NodeHandle(), ros::NodeHandle("~"), as_ros_node) {}

    ForgetfulSimulator (const ros::NodeHandle &nh, const ros::NodeHandle &pnh, const bool& as_ros_node=true);
    virtual ~ForgetfulSimulator();


public: 

    bool init (fdBS& srv);
    bool start (fdStartS& srv);
    bool stop (fdStopS& srv);
    void spinOnce (const ros::TimerEvent& te);



/*ROS*/ private:
    const bool m_runAsROSNode {};

    ros::NodeHandle m_rosRNH; // resolved to the node's namespace.
    ros::NodeHandle m_rosPNH; // resolved to: <node's namespace>/<node's name>.

    ros::Publisher m_rosPUB_RVIZ_GATES;
    ros::Publisher m_rosPUB_RVIZ_DRONE;
    image_transport::Publisher m_rosPUB_FLIGHTMARE_RGB;
    //image_transport::Publisher m_ITPub_DepthImg;
    //image_transport::Publisher m_ITPub_SegmentationImg;
    //image_transport::Publisher m_ITPub_OpticalFlowImg;
    //ros::Publisher m_ROSPub_GazeboSetModelState; // gazebo_msgs::ModelState >> "/gazebo/set_model_state"

    ros::Subscriber m_ROS_SUB_GROUND_TRUTH_ODOMETRY;
    //ros::Subscriber m_ROSSub_DynamicGatesSwitch;
    //ros::Subscriber m_ROSSub_GazeboModelStates; // msg << "/gazebo/model_states"

    ros::ServiceClient m_rosSVC_GAZEBO_SPAWN_URDF_MODEL;
    ros::ServiceClient m_rosSVC_GAZEBO_DELETE_MODEL; // gazebo_msgs::DeleteModel -> "/gazebo/delete_model"
    ros::ServiceClient m_rosSVC_GAZEBO_SPAWN_SDF_MODEL; // gazebo_msgs::SpawnModel -> "/gazebo/spawn_sdf_model"
    ros::ServiceClient m_rosSVC_RVIZ_LOAD_CONFIG;
    //ros::ServiceClient m_ROSSrvCl_GazeboResetSimulation; // std_srvs::Empty -> "/gazebo/reset_simulation"

    ros::ServiceServer m_rosSVS_SIMULATOR_BUILD;
    ros::ServiceServer m_rosSVS_SIMULATOR_START;
    ros::ServiceServer m_rosSVS_SIMULATOR_STOP;
    
    //ros::Timer m_ROSTimer_SimulatorLoop;
    ros::Timer m_rosTMR_MAIN_LOOP;

    
    
    
    
    
    
    
/*Members*/ private:
    uint8_t m_UnitySceneIdx;
    uint8_t m_SceneSiteIdx;
    uint8_t m_TrackTypeIdx;
    uint8_t m_TrackGenerationIdx;
    uint8_t m_TrackDirectionIdx;
    uint8_t m_GateTypeIdx;
    unsigned int m_CamFrameCnt;
    unsigned int m_CamFrameFailsCnt;
    bool m_GazeboDroneSpawned;
    bool m_RVizDroneSpawned;
    bool m_UnityLaunched;
    bool m_UnityConnected;
    bool m_SimulationRunning;
    std::shared_ptr<flightlib::UnityBridge> m_UnityBridgePtr;
    

    std::default_random_engine m_RandEngine;
    std::shared_ptr<flightlib::Quadrotor> m_UnityDronePtr;
    std::shared_ptr<flightlib::RGBCamera> m_UnityRGBCamPtr;
    flightlib::QuadState m_UnityDroneState;
    forgetful_drone::Pose m_UnityTrackPose;
    forgetful_drone::Pose m_GazeboTrackPose;
    std::vector<forgetful_drone::Pose> m_UnityGateInitPoses;
    std::vector<forgetful_drone::Pose> m_GazeboGateInitPoses;
    forgetful_drone::Pose m_UnityDroneInitPose;
    forgetful_drone::Pose m_GazeboDroneInitPose;
    Eigen::Vector3f m_GazeboGroundPlaneOrigin;
    Eigen::Vector2f m_GazeboGroundPlaneSize;
    std::vector<std::shared_ptr<flightlib::StaticObject>> m_UnityGates;
    std::array<std::string, 6> m_ReqLogRand;


    
    

    



/*ROS Parameters*/ private:

    const bool p_UNITY_ENABLED {};
    const bool p_RVIZ_ENABLED {};
    const bool p_TEST_ENABLED {};
    
    
    
    const int p_DRONE_CAM_WIDTH {};
    const int p_DRONE_CAM_HEIGHT {};
    const int p_RAND_SPRINT_GATE_N {};
    const int p_RAND_SPRINT_GATE_MIN_N {};
    const int p_RAND_SPRINT_GATE_MAX_N {};
    const int p_RAND_SPRINT_SECTION_GATE_MIN_N {};
    const int p_RAND_SPRINT_SECTION_GATE_MAX_N {};
    const int p_RVIZ_DRONE_ROTOR_N {};

    const double p_MAIN_LOOP_FREQ {};
    const double p_DRONE_CAM_FOV {};
    const double p_RAND_FIG8_AXIAL_SHIFT_MAX {};
    const double p_RAND_FIG8_YAW_TWIST_MAX {};
    const double p_RAND_FIG8_SCALE_MIN {};
    const double p_RAND_FIG8_SCALE_MAX {};
    const double p_RAND_SPRINT_SPACING_MIN {};
    const double p_RAND_SPRINT_SPACING_UNI_DIRECT_STD_DEV {};
    const double p_RAND_SPRINT_LR_YAW_MEAN {};
    const double p_RAND_SPRINT_LR_YAW_STD_DEV {};
    const double p_RAND_SPRINT_PITCH_MEAN {};
    const double p_RAND_SPRINT_PITCH_STD_DEV {};
    const double p_DYN_GATES_AXIAL_AMP_MAX {};
    const double p_DYN_GATES_AXIAL_FREQ_MAX {};
    const double p_RVIZ_DRONE_ARM_LENGTH {};
    const double p_RVIZ_DRONE_BODY_WIDTH {};
    const double p_RVIZ_DRONE_BODY_HEIGHT {};
    const double p_RVIZ_DRONE_SCALE {};

    const std::string p_DRONE_MODEL_DESCRIPTION {};


/*Hard Coded*/ private:
    static constexpr const char* h_DRONE_MODEL_NAME {"hummingbird"};
    static constexpr const char* h_DRONE_FRAME_ID {"hummingbird/base_link"};

    using fUS = flightlib::UnityScene;
    static constexpr std::array<std::tuple<const char*, flightlib::SceneID>, 5> h_UNITY_SCENES {
        std::make_tuple(/*NAME*/"spaceship_interior", /*UNITY_ID*/fUS::SPACESHIP_INTERIOR),
        std::make_tuple(/*NAME*/"destroyed_city",     /*UNITY_ID*/fUS::DESTROYED_CITY),
        std::make_tuple(/*NAME*/"industrial_park",    /*UNITY_ID*/fUS::INDUSTRIAL_PARK),
        std::make_tuple(/*NAME*/"polygon_city",       /*UNITY_ID*/fUS::POLYGON_CITY),
        std::make_tuple(/*NAME*/"desert_mountain",    /*UNITY_ID*/fUS::DESERT_MOUNTAIN),
    };
    static constexpr std::array<std::tuple<const char*>, 3> h_SCENE_SITES {
        std::make_tuple("a"),
        std::make_tuple("b"),
        std::make_tuple("c"),
    };
    static constexpr std::array<std::tuple<const char*>, 4> h_TRACK_TYPES {
        std::make_tuple("figure8"),
        std::make_tuple("gap"),
        std::make_tuple("random_circuit"),
        std::make_tuple("random_sprint"),
    };
    static constexpr std::array<std::tuple<const char*>, 2> h_TRACK_GENERATIONS {
        std::make_tuple("deterministic"),
        std::make_tuple("randomized"),
    };
    static constexpr std::array<std::tuple<const char*>, 2> h_TRACK_DIRECTIONS {
        std::make_tuple("clockwise"),
        std::make_tuple("counterclockwise"),
    };
    static constexpr std::array<std::tuple<const char*, float, const char*, const char*>, 3> h_GATE_TYPES {
        std::make_tuple(/*NAME*/"rpg",     /*WP_H*/2.0, /*UNITY_PREFAB_ID*/"rpg_gate",     /*GAZEBO_MODEL_DPATH*/"/gazebo/rpg_gate"),
        std::make_tuple(/*NAME*/"tub_dai", /*WP_H*/2.0, /*UNITY_PREFAB_ID*/"tub_dai_gate", /*GAZEBO_MODEL_DPATH*/"/gazebo/tub_dai_gate"),
        std::make_tuple(/*NAME*/"thu_dme", /*WP_H*/2.0, /*UNITY_PREFAB_ID*/"thu_dme_gate", /*GAZEBO_MODEL_DPATH*/"/gazebo/thu_dme_gate"),
    };
    static constexpr std::array<std::tuple<const char*, uint8_t>, 6> h_REQUEST {
        std::make_tuple(/*NAME*/"UNITY_SCENE",      /*MAX*/static_cast<uint8_t>(h_UNITY_SCENES.size() - 1)),
        std::make_tuple(/*NAME*/"SCENE_SITE",       /*MAX*/static_cast<uint8_t>(h_SCENE_SITES.size() - 1)),
        std::make_tuple(/*NAME*/"TRACK_TYPE",       /*MAX*/static_cast<uint8_t>(h_TRACK_TYPES.size() - 1)),
        std::make_tuple(/*NAME*/"TRACK_GENERATION", /*MAX*/static_cast<uint8_t>(h_TRACK_GENERATIONS.size() - 1)),
        std::make_tuple(/*NAME*/"TRACK_DIRECTION",  /*MAX*/static_cast<uint8_t>(h_TRACK_DIRECTIONS.size() - 1)),
        std::make_tuple(/*NAME*/"GATE_TYPE",        /*MAX*/static_cast<uint8_t>(h_GATE_TYPES.size() - 1)),
    };
    std::array<const uint8_t*, 6> parseRequest (const fdBSReq& req) {
        return {
            &req.unity_scene,
            &req.scene_site,
            &req.track_type,
            &req.track_generation,
            &req.track_direction,
            &req.gate_type,
        };
    }



private:
    static constexpr std::array<std::array<std::array<float, 4>, 3>, 5> h_UNITY_TRACK_XYZYAW_WRF {
        /*SPACESHIP_INTERIOR*/ std::array<std::array<float, 4>, 3> {
            /*A*/ std::array<float, 4> {/*X*/  -96.3, /*Y*/  3.4, /*Z*/0.0, /*YAW*/100.0}, 
            /*B*/ std::array<float, 4> {/*X*/ -220.5, /*Y*/-10.9, /*Z*/0.0, /*YAW*/ -9.0}, 
            /*C*/ std::array<float, 4> {/*X*/-158.94, /*Y*/36.83, /*Z*/0.0, /*YAW*/-20.0},
        },
        /*DESTROYED_CITY*/ std::array<std::array<float, 4>, 3> {
            /*A*/ std::array<float, 4> {/*X*/811.8, /*Y*/373.1, /*Z*/-5.0, /*YAW*/-27.5},
            /*B*/ std::array<float, 4> {/*X*/255.9, /*Y*/349.8, /*Z*/-5.0, /*YAW*/-69.0},
            /*C*/ std::array<float, 4> {/*X*/549.3, /*Y*/314.6, /*Z*/-5.0, /*YAW*/-38.6},
        },
        /*INDUSTRIAL_PARK*/ std::array<std::array<float, 4>, 3> {
            /*A*/ std::array<float, 4> {/*X*/ 103.5, /*Y*/  -35.7, /*Z*/-2.27, /*YAW*/-90.0},
            /*B*/ std::array<float, 4> {/*X*/-10.64, /*Y*/ -35.39, /*Z*/-2.92, /*YAW*/-90.0},
            /*C*/ std::array<float, 4> {/*X*/ 36.31, /*Y*/-111.05, /*Z*/-2.92, /*YAW*/-20.7},
        },
        /*POLYGON_CITY*/ std::array<std::array<float, 4>, 3> {
            /*A*/ std::array<float, 4> {/*X*/ -702.5, /*Y*/  65.5, /*Z*/139.73, /*YAW*/   0.0},
            /*B*/ std::array<float, 4> {/*X*/-695.02, /*Y*/-37.39, /*Z*/139.53, /*YAW*/ -90.0},
            /*C*/ std::array<float, 4> {/*X*/-455.81, /*Y*/ 76.03, /*Z*/139.53, /*YAW*/-110.0},
        },
        /*DESERT_MOUNTAIN*/ std::array<std::array<float, 4>, 3> {
            /*A*/ std::array<float, 4> {/*X*/1581.43, /*Y*/1063.43, /*Z*/ 42.0, /*YAW*/  0.0},
            /*B*/ std::array<float, 4> {/*X*/ 1662.6, /*Y*/1210.30, /*Z*/ 41.3, /*YAW*/ 70.0},
            /*C*/ std::array<float, 4> {/*X*/ 1775.5, /*Y*/1412.30, /*Z*/42.35, /*YAW*/-70.0},
        },
    };
    static constexpr std::array<std::array<float, 3>, 14> h_FIGURE8_GATE_INIT_XYYAW_TRF {
        /*#00*/ std::array<float, 3> {/*X*/-20.45, /*Y*/-08.65, /*YAW*/01.13},
        /*#01*/ std::array<float, 3> {/*X*/-12.55, /*Y*/-11.15, /*YAW*/-1.57},
        /*#02*/ std::array<float, 3> {/*X*/-04.15, /*Y*/-05.35, /*YAW*/-0.60},
        /*#03*/ std::array<float, 3> {/*X*/003.45, /*Y*/004.25, /*YAW*/-0.63},
        /*#04*/ std::array<float, 3> {/*X*/011.95, /*Y*/011.15, /*YAW*/-1.21},
        /*#05*/ std::array<float, 3> {/*X*/021.85, /*Y*/006.85, /*YAW*/00.99},
        /*#06*/ std::array<float, 3> {/*X*/024.25, /*Y*/-01.75, /*YAW*/00.00},
        /*#07*/ std::array<float, 3> {/*X*/019.25, /*Y*/-09.55, /*YAW*/-1.03},
        /*#08*/ std::array<float, 3> {/*X*/010.55, /*Y*/-10.65, /*YAW*/01.53},
        /*#09*/ std::array<float, 3> {/*X*/002.85, /*Y*/-05.95, /*YAW*/00.57},
        /*#10*/ std::array<float, 3> {/*X*/-04.95, /*Y*/004.65, /*YAW*/00.67},
        /*#11*/ std::array<float, 3> {/*X*/-12.95, /*Y*/009.65, /*YAW*/-1.53},
        /*#12*/ std::array<float, 3> {/*X*/-21.05, /*Y*/006.65, /*YAW*/-0.77},
        /*#13*/ std::array<float, 3> {/*X*/-24.25, /*Y*/-01.00, /*YAW*/00.07},
    };
    static constexpr std::array<std::array<std::array<float, 3>, 2>, 11> h_GAP_GATE_INIT_XYYAW_TRF {
    /*#00*/ std::array<std::array<float, 3>, 2> {
        /*NARROW*/std::array<float, 3> {/*X*/-20.45, /*Y*/-08.65, /*YAW*/01.13},
        /*WIDE*/  std::array<float, 3> {/*X*/-20.45, /*Y*/-08.65, /*YAW*/01.13},
    },
    /*#01*/ std::array<std::array<float, 3>, 2> {
        /*NARROW*/std::array<float, 3> {/*X*/-12.55, /*Y*/-11.15, /*YAW*/-1.57},
        /*WIDE*/  std::array<float, 3> {/*X*/-12.55, /*Y*/-11.15, /*YAW*/-1.57},
    },
    /*#02*/ std::array<std::array<float, 3>, 2> {
        /*NARROW*/std::array<float, 3> {/*X*/-04.15, /*Y*/-09.35, /*YAW*/-1.10},
        /*WIDE*/  std::array<float, 3> {/*X*/-04.15, /*Y*/-09.35, /*YAW*/-1.10},
    },
    /*#03*/ std::array<std::array<float, 3>, 2> {
        /*NARROW*/std::array<float, 3> {/*X*/004.85, /*Y*/-04.95, /*YAW*/-1.4},
        /*WIDE*/  std::array<float, 3> {/*X*/004.85, /*Y*/-05.95, /*YAW*/-1.4},
    },
    /***/ /*#04*/ std::array<std::array<float, 3>, 2> {
    /***/     /*NARROW*/std::array<float, 3> {/*X*/016.95, /*Y*/-02.25, /*YAW*/01.57},
    /***/     /*WIDE*/  std::array<float, 3> {/*X*/016.95, /*Y*/-05.25, /*YAW*/01.57},
    /*G*/ },
    /*A*/ /*#05*/ std::array<std::array<float, 3>, 2> {
    /*P*/     /*NARROW*/std::array<float, 3> {/*X*/016.95, /*Y*/002.25, /*YAW*/01.57},
    /***/     /*WIDE*/  std::array<float, 3> {/*X*/016.95, /*Y*/005.25, /*YAW*/01.57},
    /***/ },
    /*#06*/ std::array<std::array<float, 3>, 2> {
        /*NARROW*/std::array<float, 3> {/*X*/005.45, /*Y*/004.45, /*YAW*/01.40},
        /*WIDE*/  std::array<float, 3> {/*X*/005.45, /*Y*/005.45, /*YAW*/01.40},
    },
    /*#07*/ std::array<std::array<float, 3>, 2> {
        /*NARROW*/std::array<float, 3> {/*X*/-04.95, /*Y*/007.95, /*YAW*/01.20},
        /*WIDE*/  std::array<float, 3> {/*X*/-04.95, /*Y*/007.95, /*YAW*/01.20},
    },
    /*#08*/ std::array<std::array<float, 3>, 2> {
        /*NARROW*/std::array<float, 3> {/*X*/-12.95, /*Y*/009.65, /*YAW*/-1.53},
        /*WIDE*/  std::array<float, 3> {/*X*/-12.95, /*Y*/009.65, /*YAW*/-1.53},
    },
    /*#09*/ std::array<std::array<float, 3>, 2> {
        /*NARROW*/std::array<float, 3> {/*X*/-21.05, /*Y*/006.65, /*YAW*/-0.77},
        /*WIDE*/  std::array<float, 3> {/*X*/-21.05, /*Y*/006.65, /*YAW*/-0.77},
    },
    /*#10*/ std::array<std::array<float, 3>, 2> {
        /*NARROW*/std::array<float, 3> {/*X*/-24.25, /*Y*/-01.00, /*YAW*/00.07},
        /*WIDE*/  std::array<float, 3> {/*X*/-24.25, /*Y*/-01.00, /*YAW*/00.07},
    },
};
    static constexpr float h_GATE_GAZEBO_MODEL_YAW_TWIST {M_PI/2}; // because y and not x axis goes through gate
    static constexpr uint8_t h_DRONE_INIT_POSE_NUM_GATES_SETBACK {1};
    static constexpr float h_DRONE_INIT_POSE_BACK2FRONT_SLIDER {0.5F};
    //static constexpr std::array<float, 4> h_GAP_DRONE_INIT_XYZYAW_TRF {
    //    /*X*/-14.00, /*Y*/6.65, /*Z*/0.2, /*YAW*/-M_PI/2
    //};
    static constexpr float h_GAZEBO_GROUND_PLANE_BUFFER {10.0};

public:
    static auto getUnityScenes () {return &h_UNITY_SCENES;}
    static auto getSceneSites () {return &h_SCENE_SITES;}
    static auto getTrackTypes () {return &h_TRACK_TYPES;}
    static auto getTrackGenerations () {return &h_TRACK_GENERATIONS;}
    static auto getTrackDirections () {return &h_TRACK_DIRECTIONS;}
    static auto getGateTypes () {return &h_GATE_TYPES;}
    static auto getDroneInitPoseNumGatesSetback() {return &h_DRONE_INIT_POSE_NUM_GATES_SETBACK;}

private:




/*ROS Callbacks*/ private:

    bool ROSCB_SIMULATOR_BUILD (fdBSReq& req, fdBSRes& res);
    bool ROSCB_SIMULATOR_START (fdStartSReq& req, fdStartSRes& res);
    bool ROSCB_SIMULATOR_STOP (fdStopSReq& req, fdStopSRes& res);
    void ROSCB_GROUND_TRUTH_ODOMETRY (const nav_msgs::Odometry::ConstPtr& msg);
    void ROSCB_MAIN_LOOP (const ros::TimerEvent& te);



private:
    bool initROSParameters ();
    void resetRVIZ();
    
    bool startSimulation();
    bool stopSimulation();

    
    //bool buildSimulation_OLD(
    //    const forgetful_drones::BuildDroneRacingSimulation::Request::_UnityScene_type UnityScene,
    //    const forgetful_drones::BuildDroneRacingSimulation::Request::_RacetrackSite_type& SceneSite,
    //    const forgetful_drones::BuildDroneRacingSimulation::Request::_RacetrackType_type& RacetrackType,
    //    const forgetful_drones::BuildDroneRacingSimulation::Request::_RaceGateType_type& RaceGateType,
    //    const forgetful_drones::BuildDroneRacingSimulation::Request::_RacetrackClockwise_type& RacetrackClockwise,
    //const forgetful_drones::BuildDroneRacingSimulation::Request::_RacetrackMode_type& RacetrackMode);

    void plotTracks (
        const std::string& title,
        const std::vector<std::vector<Pose>>& vec_gp, 
        const std::vector<Pose>& vec_dp,
        const std::vector<std::string>& vec_label,
        const std::vector<std::string>& vec_color
    );

    bool buildSimulation (const fdBSReq& req);
    void buildSimulation_resetMembers();
    void buildSimulation_handleRandRequest (const fdBSReq& req);
    bool buildSimulation_RequestValid (const fdBSReq& req);
    void buildSimulation_logRequest();
    bool buildSimulation_generateTrack();
    Pose buildSimulation_generateTrack_getInitDronePoseTRF (
        std::vector<forgetful_drone::Pose> gps
    );
    
    void buildSimulation_generateTrack_FIGURE8 ();
    
    
    std::vector<Pose> buildSimulation_generateTrack_FIGURE8_getInitGatePosesTRF ();
    
    void buildSimulation_generateTrack_GAP ();
    std::tuple<std::vector<Pose>, std::vector<Pose>>
    buildSimulation_generateTrack_GAP_getInitGatePosesTRF ();

    std::vector<Pose>
    buildSimulation_generateTrack_randomizeInitGatePosesTRF (
        const std::vector<Pose>& gps
    );

    std::vector<Pose>
    buildSimulation_generateTrack_redirectInitGatePosesTRF (
        const std::vector<Pose>& gps
    );
    
    void 
    buildSimulation_generateTrack_transformInitPosesTRFtoWRF (
        const std::vector<Pose>& gate_init_poses_TRF,
        const Pose& drone_init_pose_TRF
    );



    //void computeRaceTrack_Figure8(const forgetful_drones::BuildDroneRacingSimulation::Request::_RacetrackType_type& RacetrackType);
    void computeRaceTrack_IntermediateTargetLoss(const bool& IsClockwise, const int& Mode);
    
    void startSimulation_launchUnity ();
    void startSimulation_addUnityGates();
    bool startSimulation_configUnityGates();
    
    void spawnGazeboModel (
        gazebo_msgs::SpawnModel& srv,
        const std::string& model_type
    );
    void _spawnGazeboModel (
        gazebo_msgs::SpawnModel& srv,
        ros::ServiceClient& srv_cl
    );

    void startSimulation_spawnGazeboGates();
    void startSimulation_spawnRVizGates();
    void startSimulation_spawnGazeboDrone();
    void startSimulation_addUnityDrone();
    void test();
    void startSimulation_spawnRVizDrone();
    void startSimulation_deleteGazeboModelsExceptDrone();
    void buildSimulation_generateGazeboGroundPlane();
    std::string getGroundPlaneSDF(
        const std::string& id,
        const Eigen::Vector2f& size
    );
    void startSimulation_spawnGazeboGroundPlane();
    void buildSimulation_setTrackPoses();


/*friends*/ private:

    friend bool fetchROSParameters (
        const ros::NodeHandle& rosNH,
        const std::vector<std::pair<const char*, const bool*>>& keys_bools,
        const std::vector<std::pair<const char*, const int*>>& keys_ints,
        const std::vector<std::pair<const char*, const double*>>& keys_doubles,
        const std::vector<std::pair<const char*, const std::string*>>& keys_strings,
        const bool& log_enabled
    );


};
}