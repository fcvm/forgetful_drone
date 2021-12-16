#include "forgetful_drones/forgetful_drones.old.h"
#include "forgetful_drones/forgetful_helpers.h"

//#include "forgetful_drones/magic_enum.hpp" //Header-only C++17 library provides static reflection for enums.
// installed with: 
// cd /home/fm/drone_racing_ws/catkin_ddr/src/forgetful_drones/include/forgetful_drones
// wget https://raw.githubusercontent.com/Neargye/magic_enum/master/include/magic_enum.hpp

#include <boost/range/adaptor/indexed.hpp>
#include <limits>
#include <random>

#include <initializer_list>
#include "geometry_msgs/Point.h"

#include "trajectory_generation_helper/polynomial_trajectory_helper.h"







//#include <filesystem>
#include "dirent.h"
//#include "boost/filesystem.hpp"
#include <ros/package.h>
#include <stdlib.h>
//#include <iomanip>
//#include <ctime>

#include <chrono>
#include <gazebo_msgs/SpawnModel.h> 
#include <tf/transform_datatypes.h>



// sudo apt-get install libcgal-dev
#include <CGAL/Cartesian.h>
#include <CGAL/Random.h>
#include <CGAL/Exact_rational.h>
#include <CGAL/Min_sphere_of_spheres_d.h>


namespace forgetful_drone 
{



// public: Con- and destructors

    ForgetfulDrones::ForgetfulDrones(
        const ros::NodeHandle& nh, 
        const ros::NodeHandle& pnh
    ) 
        :
        m_ROSRNH(nh), 
        ROS_PNH(pnh),
        m_ReferenceState_WRF(),
        m_AutopilotState(AutopilotStates::OFF), 
        m_LT_SubsequentlyFailedComputations_Count(0),
        m_Run_CameraFrame_Count(0),
        m_Gates_Last_i(0),
        m_ORCFlightMission(FlightMissions::DATACOLLECTION),
        m_Run_Count(5000),
        m_Run_DirectoryExists(false),
        m_Expert_Output(Eigen::Vector3d::Zero()), 
        m_CNN_Output(Eigen::Vector3d::Zero()),
        m_RVIZ_LT_Count(0),
        m_Run_SimEnvIsReady(false),
        m_LT_StartTime(),
        m_GT_ProjectionState_i(1),
        m_GT_MaxSpeed(0.0),
        m_GT_MinSpeed(std::numeric_limits< double >::max()),
        m_LocTraj(Vec3(), Vec3(), Vec3(), Vec3()),
        m_ROS_Param_MainLoop_NominalDuration(0.02),
        p_Gates_Respawn_MinZ(1.2),
        m_Gates_COLLADA_Filename("package://drone_racing/resources/race_track/real_world/gate/meshes/gate.dae")
    {
        updateTransformationsBetweenReferenceFrames( nav_msgs::Odometry() );
                 
        quadrotor_common::getParam<int>      ("plan_every_nth_iteration",        m_ROS_Param_MainLoop_Its2LTCompsRatio,               1,      ROS_PNH);
        quadrotor_common::getParam<double>      ("max_velocity",                    p_LT_MaxSpeed,                    1.0,    ROS_PNH);
        quadrotor_common::getParam<double>      ("max_divergence",                  p_MainLoop_DivergenceThreshold2Abort,                  0.2,    ROS_PNH);
        quadrotor_common::getParam<double>      ("d_replan",                        p_Gates_DistThreshold2UpdateIdx,                        1.0,    ROS_PNH);
        quadrotor_common::getParam<double>      ("camera_fov_yaw",                  p_Drone_Cam_HalfYawAOV_Deg,              45.0,   ROS_PNH);
        quadrotor_common::getParam<double>      ("camera_fov_pitch",                p_Drone_Cam_HalfPitchAOV_Deg,            45.0,   ROS_PNH);
            p_Drone_Cam_HalfYawAOV_Rad        =    M_PI / 180.0   * p_Drone_Cam_HalfYawAOV_Deg;
            p_Drone_Cam_HalfPitchAOV_Rad      =    M_PI / 180.0   * p_Drone_Cam_HalfPitchAOV_Deg;
        quadrotor_common::getParam<bool>        ("moving_gates",                    p_Gates_Dynamic_On,                    false,  ROS_PNH);
        quadrotor_common::getParam<double>      ("gates_static_amplitude",          p_Gates_Respawn_MaxAxialShift,                0.0,    ROS_PNH);
        quadrotor_common::getParam<int>         ("max_failed_trials",               p_LT_SubsequentlyFailedComputations_MaxCount,               5,      ROS_PNH);
        quadrotor_common::getParam<double>      ("wait_n_sec_till_record",          p_Run_DataRecording_StartTime,          0.0,    ROS_PNH);
        quadrotor_common::getParam<bool>        ("record_data",                     p_DataRecording_On,                     false,  ROS_PNH);
        quadrotor_common::getParam<bool>        ("perturb_actions",                 p_Expert_Perturbation_On,                 false,  ROS_PNH);
        quadrotor_common::getParam<int>         ("curr_goal_idx",                   p_Gates_Start_i,                 0,      ROS_PNH);
            m_Gates_Curr_i = p_Gates_Start_i;
        quadrotor_common::getParam<double>      ("horizon_min",                     p_Expert_MinHorizon,                     1.0,    ROS_PNH);   
        quadrotor_common::getParam<std::string> ("root_dir",                        p_Run_RootDirPath,                        "",     ROS_PNH);
        quadrotor_common::getParam<std::string> ("quad_frame",                      p_RVIZ_Drone_Childs_FrameID,                  "",     ROS_PNH);
        quadrotor_common::getParam<bool>        ("show_all_trajectories",           p_RVIZ_AllLTs_On,           true,   ROS_PNH);
        quadrotor_common::getParam<double>      ("min_velocity",                    p_LT_MinSpeed,                    0.5,    ROS_PNH);
        quadrotor_common::getParam<double>      ("planning_length",                 p_LT_Duration,                 2.0,    ROS_PNH);
        quadrotor_common::getParam<double>      ("planning_length_min",             p_LT_MinDist,             2.0,    ROS_PNH);
        quadrotor_common::getParam<double>      ("planning_length_max",             p_LT_MaxDist,             2.0,    ROS_PNH);
        quadrotor_common::getParam<double>      ("trajectory_viz_freq",             p_RVIZ_LT_SamplingFreq,             10.0,   ROS_PNH);
        quadrotor_common::getParam<bool>        ("show_full_trajectory",            p_RVIZ_CompleteLT_On,            false,  ROS_PNH);
        quadrotor_common::getParam<double>      ("viz_horizon",                     p_RVIZ_LT_Duration,                     1.0,    ROS_PNH);
            p_RVIZ_LT_Duration = std::max(p_RVIZ_LT_Duration, 1.0 / p_RVIZ_LT_SamplingFreq);
        quadrotor_common::getParam<double>      ("global_traj_max_v",               p_GT_MaxVel,               1.0,    ROS_PNH);
        quadrotor_common::getParam<bool>        ("load_existing_trajectory",        p_GT_UseExisting_On,        false,  ROS_PNH);
        quadrotor_common::getParam<std::string> ("trajectory_path",                 p_GT_DirPath,                        "",     ROS_PNH);
        quadrotor_common::getParam<double>      ("gates_dyn_amplitude",             p_Gates_Dynamic_AxialAmp,                 10,     ROS_PNH);
        quadrotor_common::getParam<double>      ("speed_moving_gates",              p_Gates_Dynamic_RadFreq,                           1.0,    ROS_PNH);
        quadrotor_common::getParam<double>      ("gate_height",                     p_Gates_HalfHeight,                     1.0,    ROS_PNH);
        quadrotor_common::getParam<double>      ("trajectory_max_z",                p_LT_MaxAltitude,                      2.5,    ROS_PNH);
        quadrotor_common::getParam<double>      ("trajectory_min_z",                p_LT_MinAltitude,                      0.0,    ROS_PNH);
        quadrotor_common::getParam<double>      ("min_normalized_thrust",           p_LT_MinThrust,           5.0,    ROS_PNH);
        quadrotor_common::getParam<double>      ("max_normalized_thrust",           p_LT_MaxThrust,           15.0,   ROS_PNH);
        quadrotor_common::getParam<double>      ("max_roll_pitch_rate",             p_LT_MaxBodyRates,             0.5,    ROS_PNH);
        quadrotor_common::getParam<double>      ("min_trajectory_sampling_time",    p_LT_InputFeasibilityCheck_MinSamplingTime,          0.02,   ROS_PNH);

        ROS_Sub_StateEstimate      = m_ROSRNH.subscribe( "state_estimate",          1,  &ForgetfulDrones::ROSCallback_StateEstimate,    this );
        ROS_Sub_StartNavigation    = m_ROSRNH.subscribe( "start_navigation",        1,  &ForgetfulDrones::ROSCallback_StartNavigation,  this );
        ROS_Sub_HardStop           = m_ROSRNH.subscribe( "hard_stop",               1,  &ForgetfulDrones::ROSCallback_HardStop,         this );
        ROS_Sub_CNNOutTraj         = m_ROSRNH.subscribe( "/cnn_out/traj",           1,  &ForgetfulDrones::ROSCallback_CNNOutTraj,          this );
        ROS_Sub_OnlyNetwork        = m_ROSRNH.subscribe( "only_network",            1,  &ForgetfulDrones::ROSCallback_OnlyNetwork,   this );
        ROS_Sub_RunIdx             = m_ROSRNH.subscribe( "run_idx",                 1,  &ForgetfulDrones::ROSCallback_RunIdx,        this );
        ROS_Sub_SetupEnvironment   = m_ROSRNH.subscribe( "setup_environment",       1,  &ForgetfulDrones::ROSCallback_SetupEnvironment, this );
        ROS_Sub_ImageRGB           = m_ROSRNH.subscribe( "image_rgb",               1,  &ForgetfulDrones::ROSCallback_ImageRGB,            this );
        ROS_Sub_GazeboModelStates  = m_ROSRNH.subscribe( "/gazebo/model_states",    1,  &ForgetfulDrones::ROSCallback_GazeboModelStates,     this );
        ROS_Sub_ReplaceGates       = m_ROSRNH.subscribe( "/replace_gates",          1,  &ForgetfulDrones::ROSCallback_ReplaceGates,     this );

        ROS_Pub_Crashed                = m_ROSRNH.advertise<std_msgs::Empty>                    ( "/crashed",                   1 );
        ROS_Pub_PassedGate            = m_ROSRNH.advertise<std_msgs::Empty>                    ( "/passed_gate",               1 );
        ROS_Pub_CopilotFeedthrough            = m_ROSRNH.advertise<std_msgs::Bool>                     ( "copilot/feedthrough",        1 );
        ROS_Pub_AutopilotReferenceState          = m_ROSRNH.advertise<quadrotor_msgs::TrajectoryPoint>    ( "autopilot/reference_state",  1 );
        ROS_Pub_AutopilotOff = m_ROSRNH.advertise<std_msgs::Empty> ( "autopilot/off",  1 );
        m_ROSPub_AutopilotStart = m_ROSRNH.advertise<std_msgs::Empty> ( "autopilot/start",  1 );
        m_ROSPub_BridgeArm = m_ROSRNH.advertise<std_msgs::Bool> ( "bridge/arm",  1 );
        ROS_Pub_Divergence             = m_ROSRNH.advertise<std_msgs::Float64>                  ( "divergence",                 1 );
        ROS_Pub_ImageWithPrediction                  = m_ROSRNH.advertise<sensor_msgs::Image>                 ( "image_with_prediction",      1 );
        ROS_Pub_GoalMarker            = m_ROSRNH.advertise<visualization_msgs::Marker>         ( "goal_marker",                0 );
        ROS_Pub_Debug           = m_ROSRNH.advertise<visualization_msgs::Marker>         ( "debug",                      0 );
        ROS_Pub_VehicleMarker         = m_ROSRNH.advertise<visualization_msgs::MarkerArray>    ( "vehicle_marker",             10 );
        ROS_Pub_TrajectoriesCNN     = m_ROSRNH.advertise<visualization_msgs::Marker>         ( "trajectories_cnn",           1 );
        ROS_Pub_GlobalTrajectory  = m_ROSRNH.advertise<visualization_msgs::Marker>         ( "global_trajectory",          1 );
        ROS_Pub_GazeboSetModelState = m_ROSRNH.advertise<gazebo_msgs::ModelState>( "/gazebo/set_model_state", 0 );
        ROS_Pub_RvizGates = m_ROSRNH.advertise<visualization_msgs::MarkerArray>( "rviz_gates", 1 );

        //ROS_SrvClient_GazeboSetModelState = m_ROSRNH.serviceClient<gazebo_msgs::ModelState>("/gazebo/set_model_state");
        //ROS_SrvClient_GazeboSpawnGazeboModel = m_ROSRNH.serviceClient<gazebo_msgs::SpawnModel>("/gazebo/spawn_gazebo_model");

        initializeVisMarker4Drone();
        loadGatesXYZYawFromROSParameterServer();
        for ( int i = 0; i < m_Gates_n; i++ ) 
        {
            double random_phase_x = M_PI * ( ( (double) rand() / RAND_MAX ) + 0.5 );
            double random_phase_y = M_PI * ( ( (double) rand() / RAND_MAX ) );
            double random_phase_z = M_PI * ( ( (double) rand() / RAND_MAX ) - 0.5 );

            m_Gates_Dynamic_RandomPhases.emplace_back ( 
                Eigen::Vector3d (
                    random_phase_x,
                    random_phase_y,
                    random_phase_z
                )
            );
        }

        m_GT_Settings.minimization_weights    = Eigen::Vector4d{ 0.1, 10.0, 100.0, 100.0 };
        m_GT_Settings.polynomial_order        = 11;
        m_GT_Settings.continuity_order        = 4;
        m_GT_SamplingTime = std::min(
            m_ROS_Param_MainLoop_NominalDuration * m_ROS_Param_MainLoop_Its2LTCompsRatio,
            0.05
            );


        geometry_msgs::Pose pose_W_S;
        pose_W_S.position.x     = 0.0;
        pose_W_S.position.y     = 0.0;
        pose_W_S.position.z     = 0.0;
        pose_W_S.orientation.w  = 1.0;
        pose_W_S.orientation.x  = 0.0;
        pose_W_S.orientation.y  = 0.0;
        pose_W_S.orientation.z  = 0.0;
        tf::poseMsgToKindr (
            pose_W_S, 
            &m_T_WRF_ARF
            );

        m_MainLoop_Iterations_Count = 0;
        ROS_Timer_runMainLoop = m_ROSRNH.createTimer(
            ros::Duration( m_ROS_Param_MainLoop_NominalDuration ), 
            &ForgetfulDrones::runMainLoop, 
            this
            );
    }


    ForgetfulDrones::~ForgetfulDrones() = default;



// private: Callback methods

    void ForgetfulDrones::ROSCallback_SetupEnvironment( const std_msgs::EmptyConstPtr& msg )
    {
        ROS_INFO(
            "[%s] Resetting environment...",
            ros::this_node::getName().c_str()
            );
        
        m_Run_SimEnvIsReady = false;
        p_DataRecording_On = false;

        m_Gates_Curr_i = p_Gates_Start_i;
        m_Gates_Last_i = ( m_Gates_Curr_i + m_Gates_n - 1 ) % m_Gates_n;

        respawnGazeboGateModelsWithRandomAxialShifts();
        ros::Duration(2.0).sleep();
    
        visualizeGazeboGateModelsInRVIZ();

        initializeGlobalTrajectory();
        ros::Duration(1.0).sleep();

        Eigen::Vector3d DronePos_WRF = m_T_WRF_DRF.getPosition();
        displayCubeInRVIZ ( DronePos_WRF.x(), DronePos_WRF.y(), DronePos_WRF.z(),
            4, VisualizationColors::GREEN );
        
        if ( m_ORCFlightMission == FlightMissions::DATACOLLECTION )
        {
            setMinAndMaxSpeedAfterScanningGlobalTrajectory();
            setIndexOfGTProjStateToGTStateWithMinDist2( DronePos_WRF );
        }
                
        m_Run_SimEnvIsReady = true;

        updateAutopilotStateTo( AutopilotStates::RACING );

        m_Run_StartTime = ros::WallTime::now();
        setReferenceStateInWRFToDronePosYawInWRF();

        // Publish the current state estimation as reference state to the autopilot in order that the drone hovers.
        ROS_Pub_AutopilotReferenceState.publish( 
            QCTrajectoryPoint_From_KMQuatTransformation( 
                m_T_ARF_DRF 
                ).toRosMessage()
            );

        std_msgs::Bool feedthrough_msg;
        feedthrough_msg.data = true;
        ROS_Pub_CopilotFeedthrough.publish( feedthrough_msg );

        ROS_INFO(
            "[%s] Environment successfully reset.",
            ros::this_node::getName().c_str()
            );
    }


    void ForgetfulDrones::ROSCallback_RunIdx( const std_msgs::Int16ConstPtr& msg ) 
    {
        m_Run_Count = msg->data;
        m_Run_DirectoryExists = false;
        ROS_INFO(
            "[%s] Start Run #%d.", 
            ros::this_node::getName().c_str(), 
            m_Run_Count
            );
    }


    void ForgetfulDrones::ROSCallback_StartNavigation( const std_msgs::EmptyConstPtr& msg )
    {
        updateAutopilotStateTo( AutopilotStates::RACING );

        m_Run_StartTime = ros::WallTime::now();
        
        setReferenceStateInWRFToDronePosYawInWRF();

        // to ROS topic "autopilot/reference_state"
        // Publish the current state estimation as reference state to the autopilot in order that the drone hovers.
        ROS_Pub_AutopilotReferenceState.publish( 
            QCTrajectoryPoint_From_KMQuatTransformation( 
                m_T_ARF_DRF 
                ).toRosMessage()
            );
        
        // Publish true to ROS topic "copilot/feedthrough"
        std_msgs::Bool feedthrough_msg;
        feedthrough_msg.data = true;
        ROS_Pub_CopilotFeedthrough.publish( feedthrough_msg );
    }

    void ForgetfulDrones::ROSCallback_HardStop(const std_msgs::EmptyConstPtr& msg) 
    {
        m_Run_CameraFrame_Count = 0;

        updateAutopilotStateTo( AutopilotStates::HOVER );
    }



    void ForgetfulDrones::ROSCallback_StateEstimate( const nav_msgs::OdometryConstPtr& msg ) 
    {
        updateTransformationsBetweenReferenceFrames( *msg );
    }


    void ForgetfulDrones::ROSCallback_CNNOutTraj( const geometry_msgs::TwistStampedConstPtr& msg ) 
    {
        m_CNN_Output = 
        Eigen::Vector3d ( msg->twist.linear.x, msg->twist.linear.y, msg->twist.linear.z );
    }


// ???default cases for all switchs for robustness???

    void ForgetfulDrones::ROSCallback_OnlyNetwork( const std_msgs::BoolConstPtr& msg ) 
    {
        std::string NameOfLastNavigationMode;
        switch ( m_ORCFlightMission )
        {
            case FlightMissions::DATACOLLECTION:
                NameOfLastNavigationMode = "DATACOLLECTION";
                break;
            case FlightMissions::TESTING:
                NameOfLastNavigationMode = "TESTING";
                break;
        }
        
        if ( msg->data )
        {
            m_ORCFlightMission = FlightMissions::TESTING;
            ROS_INFO (
                "[%s] Navigation mode updated: %s -> %s.", 
                ros::this_node::getName().c_str(),
                NameOfLastNavigationMode.c_str(),
                "TESTING"
                );
        }
        else 
        {
            m_ORCFlightMission = FlightMissions::DATACOLLECTION;
            ROS_INFO (
                "[%s] Navigation mode updated: %s -> %s.", 
                ros::this_node::getName().c_str(),
                NameOfLastNavigationMode.c_str(),
                "DATACOLLECTION"
                );
        }
    }


    void ForgetfulDrones::ROSCallback_ImageRGB(const sensor_msgs::ImageConstPtr& msg) 
    {
        cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare( msg, sensor_msgs::image_encodings::TYPE_8UC3 );

        m_CloneImgMtx.lock();
            
            m_Drone_Cam_Frame = cv_ptr->image.clone();

            cv::Mat imageRaw = m_Drone_Cam_Frame.clone();
            cv::Mat image;
            // shrink image
            // get image dimensions
            int border_top_bottom = (int) (imageRaw.rows / 2.0 * (p_Drone_Cam_HalfPitchAOV_Deg / 35.0 - 1.0));
            int border_left_right = (int) (imageRaw.cols / 2.0 * (p_Drone_Cam_HalfYawAOV_Deg / 50.0 - 1.0));
            int borderType = cv::BORDER_CONSTANT;
            cv::Scalar value = cv::Scalar(50.0, 50.0, 50.0);

            cv::copyMakeBorder(imageRaw, image, border_top_bottom, border_top_bottom, border_left_right, border_left_right,
                            borderType, value);
            // draw some grid lines
            for (int i = 0; i <= p_Drone_Cam_HalfYawAOV_Deg; i += 30) 
            {
                int yaw_px = (int) (i / ((float) p_Drone_Cam_HalfYawAOV_Deg) * image.cols / 2.0 + image.cols / 2.0);
                cv::line(image, cv::Point(yaw_px, 0), cv::Point(yaw_px, image.rows), cv::Scalar(100, 100, 100, 0));
                yaw_px = (int) (-i / ((float) p_Drone_Cam_HalfYawAOV_Deg) * image.cols / 2.0 + image.cols / 2.0);
                cv::line(image, cv::Point(yaw_px, 0), cv::Point(yaw_px, image.rows), cv::Scalar(100, 100, 100, 0));
            }

            for (int i = 0; i <= p_Drone_Cam_HalfPitchAOV_Deg; i += 30) 
            {
                int pitch_px = (int) (i / ((float) p_Drone_Cam_HalfPitchAOV_Deg) * image.rows / 2.0 + image.rows / 2.0);
                cv::line(image, cv::Point(0, pitch_px), cv::Point(image.cols, pitch_px), cv::Scalar(100, 100, 100, 0));
                pitch_px = (int) (-i / ((float) p_Drone_Cam_HalfPitchAOV_Deg) * image.rows / 2.0 + image.rows / 2.0);
                cv::line(image, cv::Point(0, pitch_px), cv::Point(image.cols, pitch_px), cv::Scalar(100, 100, 100, 0));
            }

            int rows = image.rows;
            int cols = image.cols;
            int up_coor_mb = static_cast<int>(rows * (1.0 / 2.0 - m_Expert_Output.y() / 2.0));
            int right_coor_mb = static_cast<int>(cols * (1.0 / 2.0 + m_Expert_Output.x() / 2.0));
            int up_coor_nw = static_cast<int>(rows * (1.0 / 2.0 - m_CNN_Output.y() / 2.0));
            int right_coor_nw = static_cast<int>(cols * (1.0 / 2.0 + m_CNN_Output.x() / 2.0));

            // draw boxes behind velocity info to make it better readable
            cv::Mat roi = image(cv::Rect(3, image.rows - 25, 85, 20));
            cv::Mat color(roi.size(), CV_8UC3, cv::Scalar(125, 125, 125));
            double alpha = 0.8;
            cv::addWeighted(color, alpha, roi, 1.0 - alpha, 0.0, roi);

            cv::Mat roi2 = image(cv::Rect(image.cols - 88, image.rows - 25, 85, 20));
            cv::addWeighted(color, alpha, roi2, 1.0 - alpha, 0.0, roi2);

            char v_nw[200];
            sprintf(v_nw, "%f ", m_CNN_Output.z());
            cv::putText(image, v_nw, cv::Point2f(5, image.rows - 10), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0, 255, 0, 255));

            char v_mb[200];
            sprintf(v_mb, "%f ", m_Expert_Output.z());
            cv::putText(image, v_mb, cv::Point2f(image.cols - 80, image.rows - 10), cv::FONT_HERSHEY_PLAIN, 1,
                        cv::Scalar(255, 0, 0, 255));

            cv::circle(image, cv::Point(right_coor_mb, up_coor_mb), 0, cv::Scalar(255, 0, 0), 5, 8, 0);
            cv::circle(image, cv::Point(right_coor_nw, up_coor_nw), 0, cv::Scalar(0, 255, 0), 5, 8, 0);

            cv_bridge::CvImage out_msg;
            out_msg.header.stamp = ros::Time::now(); // Same timestamp and tf frame as input image
            out_msg.encoding = "rgb8";
            out_msg.image = image;

            ROS_Pub_ImageWithPrediction.publish(out_msg.toImageMsg());
        m_CloneImgMtx.unlock();
    }


    void ForgetfulDrones::ROSCallback_GazeboModelStates( const gazebo_msgs::ModelStates::ConstPtr &msg ) 
    { m_Gazebo_ModelStates = *msg; }

    void ForgetfulDrones::ROSCallback_ReplaceGates( const std_msgs::EmptyConstPtr &msg ) 
    { respawnGazeboGateModelsWithRandomAxialShifts(); }







// private: Methods

    void ForgetfulDrones::runMainLoop( const ros::TimerEvent& MainLoop_TimerEvent ) 
    {
        if ( MainLoop_TimerEvent.profile.last_duration.toSec() > m_ROS_Param_MainLoop_NominalDuration )
                ROS_ERROR(
                    "[%s] Last main loop iteration took %f s but is required to take less than %f s.",
                    ros::this_node::getName().c_str(), 
                    MainLoop_TimerEvent.profile.last_duration.toSec(),
                    m_ROS_Param_MainLoop_NominalDuration
                    );


        ROS_Pub_VehicleMarker.publish( m_RVIZ_Drone );
        

        if ( m_Run_SimEnvIsReady ) 
        {

            if ( p_Gates_Dynamic_On && ( ros::WallTime::now() - m_Run_StartTime ).toSec() > p_Run_DataRecording_StartTime ) 
                moveGazeboGateModels();


            visualizeGazeboGateModelsInRVIZ(); /// ???If marker array really has to be published in main loop at least optimize that not each time have to compile gate positions???


            if ( m_AutopilotState != AutopilotStates::RACING ) 
                visualizeGateWaypointsInRVIZ();
            else
            {
                performNavigation();
                
                setReferenceStateFromLocalTrajectory();

                ROS_Pub_AutopilotReferenceState.publish( m_ReferenceState_ARF.toRosMessage() );
            }




        }

        m_MainLoop_Iterations_Count++;
    }



    





    const quadrotor_common::TrajectoryPoint& 
    ForgetfulDrones::getGTStateAtHorizonOf( const double& Horizon ) const
    {
        size_t GT_HorizonState_i = m_GT_ProjectionState_i;
        double Dist_ProjectionState2HorizonState;

        do
        {
            GT_HorizonState_i++;
            GT_HorizonState_i %= m_GT_States.size();
            
            Dist_ProjectionState2HorizonState = ( 
                  m_GT_States[ GT_HorizonState_i ].position 
                - m_GT_States[ m_GT_ProjectionState_i ].position
                ).norm();

        } while ( Dist_ProjectionState2HorizonState < Horizon );

        return m_GT_States[ GT_HorizonState_i ];
    }




    void ForgetfulDrones::performNavigation() 
    {
        if ( checkDivergence_Drone2RefState() ) 
            abortRunDueToDivergence_Drone2RefState();
        else if ( m_MainLoop_Iterations_Count 
            % m_ROS_Param_MainLoop_Its2LTCompsRatio == 0 
            )
            switch ( m_ORCFlightMission )
            {
                case FlightMissions::DATACOLLECTION: executeDataCollection(); break;
                case FlightMissions::TESTING: executeTesting(); break;
                default: 
                    ROS_WARN(
                        "[%s] No implementation for current navigation mode. No navigation performed.", 
                        ros::this_node::getName().c_str()
                        );
                    break;
            }
    }

    bool ForgetfulDrones::checkDivergence_Drone2RefState()  /// ??? WRF ->DRF for testing ???
    {        
        std_msgs::Float64 Dist_Drone2RefState;            
            Dist_Drone2RefState.data 
                = ( m_ReferenceState_WRF.position - m_T_WRF_DRF.getPosition() ).norm();

        ROS_Pub_Divergence.publish( Dist_Drone2RefState ); //??? check if the publishing is necessary???

        return Dist_Drone2RefState.data > p_MainLoop_DivergenceThreshold2Abort;
    }

    void ForgetfulDrones::abortRunDueToDivergence_Drone2RefState() 
    {        
        ROS_WARN(
            "[%s] Goal position and position estimate diverged.", 
            ros::this_node::getName().c_str()
            );
        ROS_WARN(
            "Goal position: [%f, %f, %f]", 
            m_ReferenceState_WRF.position.x(),
            m_ReferenceState_WRF.position.y(), 
            m_ReferenceState_WRF.position.z()
            );
        ROS_WARN(
            "Position estimate: [%f, %f, %f]", 
            m_T_WRF_DRF.getPosition().x(),
            m_T_WRF_DRF.getPosition().y(), 
            m_T_WRF_DRF.getPosition().z()
            );

        

        updateAutopilotStateTo( AutopilotStates::HOVER );
        
        std::ofstream outfile;
        std::string filename_fails = p_Run_RootDirPath + "/fails.txt";
        outfile.open(filename_fails, std::ios_base::app);
        outfile << std::to_string(m_Run_Count) + "\n";
        outfile.close();
        
        ROS_Pub_Crashed.publish( std_msgs::Empty() ); 
    }






    void ForgetfulDrones::setExpertOutput()
    {
        Eigen::Vector2d Expert_GoalPos_IRF = getExpertGoalPosInIRF();
        double Expert_NormalizedSpeed = getExpertNormalizedSpeed();

        m_Expert_Output = {
            Expert_GoalPos_IRF.x(),
            Expert_GoalPos_IRF.y(),
            Expert_NormalizedSpeed
            };
    }


    double ForgetfulDrones::getExpertNormalizedSpeed() 
    {        
        return getGTStateAtHorizonOf( m_Expert_Speed_Horizon ).velocity.norm() / m_GT_MaxSpeed;
    }


    Eigen::Vector2d ForgetfulDrones::getExpertGoalPosInIRF()
    {
        Eigen::Vector3d Expert_GoalPos_WRF = getExpertGoalPosInWRF();

        displayCubeInRVIZ( 
            Expert_GoalPos_WRF.x(), Expert_GoalPos_WRF.y(), Expert_GoalPos_WRF.z(),
            0, VisualizationColors::YELLOW );

        Eigen::Vector3d Expert_GoalPos_DRF = transformPos_WRF2DRF( Expert_GoalPos_WRF );
        Eigen::Vector2d Expert_GoalPos_IRF = transformPos_DRF2IRF( Expert_GoalPos_DRF );
        
        return Expert_GoalPos_IRF;
    }

    Eigen::Vector3d ForgetfulDrones::getExpertGoalPosInWRF()
    {
        Eigen::Vector3d Expert_GoalPos_WRF
            = ( p_Gates_Dynamic_On || p_Gates_Respawn_MaxAxialShift > 0.0 ) ?
                m_Gates_WaypointPos_Temp[ m_Gates_Curr_i ] :
                getExpertGoalPosInWRFFromGlobalTrajectory();

        return Expert_GoalPos_WRF;
    }


    Eigen::Vector3d ForgetfulDrones::getExpertGoalPosInWRFFromGlobalTrajectory() 
    {
        // Distance from reference state to current gate waypoint.
        double Dist_RefState2CurrGateWaypoint = ( m_ReferenceState_WRF.position 
            - m_Gates_WaypointPos_Temp[ m_Gates_Curr_i ] ).norm();
        // Distance from reference state to last gate waypoint.
        double Dist_ReferenceState2LastGate = ( m_ReferenceState_WRF.position 
            - m_Gates_WaypointPos_Temp[ m_Gates_Last_i ] ).norm();
        
        // Horizon: Distance 2 current/last gate if current/last gate is closer
        // but at least p_Expert_MinHorizon
        double Horizon = std::max( p_Expert_MinHorizon, std::min(
                Dist_RefState2CurrGateWaypoint, Dist_ReferenceState2LastGate ) );
        
        // Position of state of GT with above horizon
        return getGTStateAtHorizonOf( Horizon ).position;
    }




    Eigen::Vector3d 
    ForgetfulDrones::transformPos_WRF2DRF( const Eigen::Vector3d& IN_Pos_WRF )
    {
        // Transformation from reference frame originating in IN_Pos_WRF
        // (with neutral orientation with respect to world reference frame)
        // to world reference frame
        kindr::minimal::QuatTransformation T_WRF_PosRF;
        geometry_msgs::Pose Pose_WRF;
            Pose_WRF.position.x = IN_Pos_WRF.x();
            Pose_WRF.position.y = IN_Pos_WRF.y();
            Pose_WRF.position.z = IN_Pos_WRF.z();
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

    Eigen::Vector2d ForgetfulDrones::transformPos_DRF2IRF( const Eigen::Vector3d& IN_Pos_DRF )
    {
        // Yaw and pitch angle from drone facing direction to goal position
        double Yaw2Pos_DRF = std::atan2( IN_Pos_DRF.y(), IN_Pos_DRF.x() );
        double Pitch2Pos_DRF = std::atan2( IN_Pos_DRF.z(), IN_Pos_DRF.norm() );

        // Image coordinates: above angles related to camera angles of view but in [-1, 1].
        double X_IRF = std::min( 1.0, std::max( -1.0, 
            -Yaw2Pos_DRF / p_Drone_Cam_HalfYawAOV_Rad ) );
        double Y_IRF = std::min( 1.0, std::max( -1.0, 
            Pitch2Pos_DRF / p_Drone_Cam_HalfPitchAOV_Rad ) );

        // Pos_IRF: 
        return { X_IRF, Y_IRF };
    }








    void ForgetfulDrones::executeDataCollection() 
    {   
        setIndexOfGlobalTrajectoryProjectionState();

        /// Displays [red/blue/black] cube for 
        /// [reference state/projection state/waypoint of current gate] in RVIZ.
        displayCubeInRVIZ( m_ReferenceState_WRF.position.x(), m_ReferenceState_WRF.position.y(), m_ReferenceState_WRF.position.z(),
            3, VisualizationColors::RED );
        
        displayCubeInRVIZ( m_GT_States[ m_GT_ProjectionState_i ].position.x(), m_GT_States[ m_GT_ProjectionState_i ].position.y(), m_GT_States[ m_GT_ProjectionState_i ].position.z(),
            1, VisualizationColors::BLUE );

        displayCubeInRVIZ ( m_Gates_WaypointPos_Temp[ m_Gates_Curr_i ].x(), m_Gates_WaypointPos_Temp[ m_Gates_Curr_i ].y(), m_Gates_WaypointPos_Temp[ m_Gates_Curr_i ].z(),
            27, VisualizationColors::BLACK );

        // If distance from reference state to waypoint of current gate 
        // shorter than "p_Gates_DistThreshold2UpdateIdx"
        if ( 
            ( m_ReferenceState_WRF.position 
            - m_Gates_WaypointPos_Temp[ m_Gates_Curr_i ] ).norm() 
            < p_Gates_DistThreshold2UpdateIdx 
            ) 
            updateIdxOfCurrGate();


        setExpertOutput();


        Eigen::Vector3d Expert_Output = m_Expert_Output;
        if ( p_Expert_Perturbation_On ) 
            Expert_Output += getRandomPerturbation4ExpertOutput();


        Eigen::Vector3d Expert_GoalPos_WRF;
        double Expert_Speed2Goal;
        processCNNOutput(
            Expert_Output.x(),
            Expert_Output.y(),
            Expert_Output.z(),
            Expert_GoalPos_WRF,
            Expert_Speed2Goal
            );
        
        bool LT_Expert_SuccessfullyComputed = computeLocalTrajectory(
            m_ReferenceState_WRF,
            Expert_GoalPos_WRF,
            Expert_Speed2Goal
            );


        double RunElapsedTime = ( ros::WallTime::now() - m_Run_StartTime ).toSec();
        if ( RunElapsedTime > p_Run_DataRecording_StartTime && p_DataRecording_On ) 
            saveTrainingData();

        m_Run_CameraFrame_Count++;

        if ( LT_Expert_SuccessfullyComputed ) 
        {
            // visualization
            visualizeTrajectory( TrajectoryTypes::LOCAL );
                
            m_LT_SubsequentlyFailedComputations_Count = 0;
            m_LT_StartTime = ros::Time::now();
        } 
        else 
        {
            if ( m_LT_SubsequentlyFailedComputations_Count < p_LT_SubsequentlyFailedComputations_MaxCount ) 
            {
                m_LT_SubsequentlyFailedComputations_Count++;
                ROS_WARN (
                    "[%s] Computation of local trajectory subsequently failed %d of maximum %d times before going to HOVER.", 
                    ros::this_node::getName().c_str(), 
                    m_LT_SubsequentlyFailedComputations_Count,
                    p_LT_SubsequentlyFailedComputations_MaxCount
                    );
            } 
            else 
            {
                updateAutopilotStateTo( AutopilotStates::HOVER );
            }
        }
    
    }

    Eigen::Vector3d ForgetfulDrones::getRandomPerturbation4ExpertOutput()
    {
        return {    0.1  * std::sin( 0.5 * ros::WallTime::now().toSec() ),
                    0.1  * std::sin( 0.5 * ros::WallTime::now().toSec() ),
                    0.05 * std::sin( 0.5 * ros::WallTime::now().toSec() )     };
    }

    void ForgetfulDrones::saveTrainingData()
    {
        std::string RunCountString = getStringRepresentation4RunCount();
        std::string CameraFrameCountString = getStringRepresentation4CameraFrameCount(); //???Camera->Cam???

        if ( !m_Run_DirectoryExists ) // ??? Avoid???
        {
            createDirectory4TrainingDataOfRun( RunCountString );
            m_Run_CameraFrame_Count = 0;
            m_Run_DirectoryExists = true;
        }

        saveTrainingData_Image( RunCountString, CameraFrameCountString );
        saveTrainingData_Label( RunCountString );
    }


    std::string ForgetfulDrones::getStringRepresentation4RunCount()
    {
        std::ostringstream RunCountStream;
        RunCountStream << std::setw( 4 ) << std::setfill( '0' ) << m_Run_Count;
        return std::string( RunCountStream.str() );
    };

    std::string ForgetfulDrones::getStringRepresentation4CameraFrameCount()
    {   
        std::ostringstream CameraFrameCountStream;
        CameraFrameCountStream << std::setw(5) << std::setfill('0') << m_Run_CameraFrame_Count;
        return std::string( CameraFrameCountStream.str() );
    };

    void ForgetfulDrones::createDirectory4TrainingDataOfRun( const std::string& RunCountString )
    {
        ROS_INFO(
            "[%s] Creating directory for run #%d...",
            ros::this_node::getName().c_str(),
            m_Run_Count
            );

        // generate new directory for experimental data
        std::string RunDirectoryName = p_Run_RootDirPath + "/Run_" + RunCountString + "/images";
        std::string MakeRunDirectoryCommand = "mkdir -p " + RunDirectoryName;

        if ( -1 == system( MakeRunDirectoryCommand.c_str() ) ) 
            ROS_ERROR(
                "[%s] Failed to create directory %s.", 
                ros::this_node::getName().c_str(),
                RunDirectoryName.c_str()
                );
        else
            ROS_INFO(
                "[%s] Directory %s succesfully created.", 
                ros::this_node::getName().c_str(),
                RunDirectoryName.c_str()
                );
    }

    void ForgetfulDrones::saveTrainingData_Image( const std::string& RunCountString, const std::string& CameraFrameCountString )
    {
        ROS_DEBUG(
            "[%s] Saving current camera frame to image...",
            ros::this_node::getName().c_str()
            );

        std::string ImageFilename 
            = p_Run_RootDirPath + "/Run_" + RunCountString 
            + "/images/frame_center_" + CameraFrameCountString + ".jpg";
        
        m_CloneImgMtx.lock();
            //convert from bgr to rgb
            cv::Mat SavedImage = m_Drone_Cam_Frame.clone();
            cv::cvtColor(
                m_Drone_Cam_Frame, 
                SavedImage, 
                CV_BGR2RGB
                );

            if ( !cv::imwrite( ImageFilename, SavedImage ) ) 
                ROS_FATAL(
                    "Failed to save current camera frame to %s.", 
                    ImageFilename.c_str()
                    );
            else 
                ROS_DEBUG(
                "Current camera frame successfully saved to %s.", 
                ImageFilename.c_str()
                );

        m_CloneImgMtx.unlock();
    }

    void ForgetfulDrones::saveTrainingData_Label( const std::string& RunCountString )
    {
        ROS_DEBUG(
            "[%s] Saving current expert output as label...",
            ros::this_node::getName().c_str()
            );

        // save label of best trajectory
        std::ofstream RunLabelsOutfile;
        std::string RunLablesFilename = p_Run_RootDirPath + "/Run_" + RunCountString + "/labels.txt";
        RunLabelsOutfile.open(
            RunLablesFilename, 
            std::ios_base::app
            );
        RunLabelsOutfile 
            << std::to_string( m_Expert_Output.x() ) + ";" 
            << std::to_string( m_Expert_Output.y() ) + ";"
            << std::to_string( m_Expert_Output.z() ) + ";" 
            << std::to_string( m_GT_MaxSpeed    ) + ";"
            << std::to_string( m_Run_CameraFrame_Count == 0? 1 : 0 ) + "\n"; // ???new dagger batch???
        RunLabelsOutfile.close();

        ROS_DEBUG(
            "[%s] Label successfully saved.",
            ros::this_node::getName().c_str()
            );
    }



    void ForgetfulDrones::setReferenceStateInWRFToDronePosYawInWRF()
    {
        m_ReferenceState_WRF = quadrotor_common::TrajectoryPoint();

            m_ReferenceState_WRF.position 
                = m_T_WRF_DRF.getPosition();
            
            m_ReferenceState_WRF.heading 
                = quadrotor_common::quaternionToEulerAnglesZYX(
                    m_T_WRF_DRF.getEigenQuaternion()
                    ).z();
    }





    












    void ForgetfulDrones::updateIdxOfCurrGate() 
    {
        m_Gates_Curr_i++;
        m_Gates_Curr_i =  m_Gates_Curr_i % m_Gates_n;
        m_Gates_Last_i = (m_Gates_Curr_i + m_Gates_n - 1) % m_Gates_n;
        
        ROS_Pub_PassedGate.publish( std_msgs::Empty() );
    }



    void ForgetfulDrones::executeTesting() 
    {

        
        setIndexOfGlobalTrajectoryProjectionState();

        /// Displays [red/blue/black] cube for 
        /// [reference state/projection state/waypoint of current gate] in RVIZ.
        displayCubeInRVIZ( m_ReferenceState_WRF.position.x(), m_ReferenceState_WRF.position.y(), m_ReferenceState_WRF.position.z(),
            3, VisualizationColors::RED );
        
        displayCubeInRVIZ( m_GT_States[ m_GT_ProjectionState_i ].position.x(), m_GT_States[ m_GT_ProjectionState_i ].position.y(), m_GT_States[ m_GT_ProjectionState_i ].position.z(),
            1, VisualizationColors::BLUE );

        displayCubeInRVIZ ( m_Gates_WaypointPos_Temp[ m_Gates_Curr_i ].x(), m_Gates_WaypointPos_Temp[ m_Gates_Curr_i ].y(), m_Gates_WaypointPos_Temp[ m_Gates_Curr_i ].z(),
            27, VisualizationColors::BLACK );

        // If distance from reference state to waypoint of current gate 
        // shorter than "p_Gates_DistThreshold2UpdateIdx"
        if ( 
            ( m_ReferenceState_WRF.position 
            - m_Gates_WaypointPos_Temp[ m_Gates_Curr_i ] ).norm() 
            < p_Gates_DistThreshold2UpdateIdx 
            ) 
            updateIdxOfCurrGate();
        



        // ??? actually should be _DRF instead of _WRF during test time.
        // ??? I assume that this only works because 
        // ??? either nothing or zero estimate is published to "state_estimate"
        // ??? during test time. TODO<check if that is the case>

        // Generate network trajectory
        
        Eigen::Vector3d CNN_GoalPos_WRF; /// ??? _DRF
        double CNN_Speed2Goal;
        processCNNOutput( m_CNN_Output.x(), m_CNN_Output.y(), m_CNN_Output.z(),
            CNN_GoalPos_WRF, CNN_Speed2Goal );
        
        bool LT_CNN_SuccessfullyComputed = computeLocalTrajectory(
            m_ReferenceState_WRF, /// ??? _DRF
            CNN_GoalPos_WRF, /// ??? _DRF
            CNN_Speed2Goal
            );

            
        if ( LT_CNN_SuccessfullyComputed ) 
        {
            m_Run_CameraFrame_Count = 0;

            // visualization
            visualizeTrajectory( TrajectoryTypes::LOCAL );
            m_LT_SubsequentlyFailedComputations_Count = 0;
            m_LT_StartTime = ros::Time::now();
        } 
        else 
        {
            ROS_ERROR(
                "[%s] Failed to compute CNN trajectory.", 
                ros::this_node::getName().c_str()
                );
            if ( m_LT_SubsequentlyFailedComputations_Count < p_LT_SubsequentlyFailedComputations_MaxCount ) 
            {
                m_LT_SubsequentlyFailedComputations_Count++;
                ROS_WARN (
                    "[%s] Failed trials: [%d].", 
                    ros::this_node::getName().c_str(), 
                    m_LT_SubsequentlyFailedComputations_Count
                );
            } 
            else 
            {
                updateAutopilotStateTo( AutopilotStates::HOVER );

                ROS_Pub_Crashed.publish( std_msgs::Empty() );
            }
        }
        
    }







    void ForgetfulDrones::displayCubeInRVIZ(
        const double& RVIZ_Cube_X,
        const double& RVIZ_Cube_Y,
        const double& RVIZ_Cube_Z,
        const int& RVIZ_Cube_ID,
        const VisualizationColors& RVIZ_Cube_VisColor
        )
    {
        visualization_msgs::Marker RVIZ_Cube;
            RVIZ_Cube.header.frame_id = "world";
            RVIZ_Cube.header.stamp = ros::Time();
            RVIZ_Cube.ns = "my_namespace";
            RVIZ_Cube.type = visualization_msgs::Marker::CUBE;
            RVIZ_Cube.action = visualization_msgs::Marker::ADD;
            RVIZ_Cube.pose.orientation.x = 0.0;
            RVIZ_Cube.pose.orientation.y = 1.0;
            RVIZ_Cube.pose.orientation.z = 0.0;
            RVIZ_Cube.pose.orientation.w = 1.0;
            RVIZ_Cube.scale.x = 0.2;
            RVIZ_Cube.scale.y = 0.2;
            RVIZ_Cube.scale.z = 0.2;
            RVIZ_Cube.color.a = 1.0;

            RVIZ_Cube.id = RVIZ_Cube_ID;
            RVIZ_Cube.pose.position.x = RVIZ_Cube_X;
            RVIZ_Cube.pose.position.y = RVIZ_Cube_Y;
            RVIZ_Cube.pose.position.z = RVIZ_Cube_Z;
        setColorRGBOfVisMarker( RVIZ_Cube_VisColor, RVIZ_Cube );

        ROS_Pub_Debug.publish( RVIZ_Cube );
    }

    

    
    void ForgetfulDrones::visualizeGateWaypointsInRVIZ()
    {
        // Construct GateWaypointMarker and 
        // set members that all GateWaypointMarkers have in common.
        visualization_msgs::Marker GateWaypointMarker;
            GateWaypointMarker.header.frame_id = "world"; // ???used???
            GateWaypointMarker.ns = "my_namespace"; // ???used???
            GateWaypointMarker.type = visualization_msgs::Marker::SPHERE;
            GateWaypointMarker.action = visualization_msgs::Marker::ADD;
            GateWaypointMarker.pose.orientation.x = 0.0; // ???used???
            GateWaypointMarker.pose.orientation.y = 1.0; // ???used???
            GateWaypointMarker.pose.orientation.z = 0.0; // ???used???
            GateWaypointMarker.pose.orientation.w = 1.0; // ???used???
            GateWaypointMarker.scale.x = 0.2;
            GateWaypointMarker.scale.y = 0.2;
            GateWaypointMarker.scale.z = 0.2;
            GateWaypointMarker.color.a = 1.0; // ???write fct that sets color of marker???    
            setColorRGBOfVisMarker( VisualizationColors::YELLOW, GateWaypointMarker );
    
        
        // Set members that are individual to each GateWaypointMarker 
        // (stamp, id, position) and publish the Gatemarker.
        for ( auto const& GateWaypointPos : m_Gates_WaypointPos_Temp | boost::adaptors::indexed(0) )
        {
            GateWaypointMarker.header.stamp = ros::Time();
            GateWaypointMarker.id = GateWaypointPos.index();
            GateWaypointMarker.pose.position.x = GateWaypointPos.value().x();
            GateWaypointMarker.pose.position.y = GateWaypointPos.value().y();
            GateWaypointMarker.pose.position.z = GateWaypointPos.value().z();
        
            ROS_Pub_GoalMarker.publish( GateWaypointMarker );
        }
    }

    
    void ForgetfulDrones::updateTransformationsBetweenReferenceFrames( const nav_msgs::Odometry& StateEstimate_ARF )
    {
        // Transformation: drone reference frame -> state estimate reference frame.
        tf::poseMsgToKindr(
            GeometryMsgsPose_From_NavMsgsOdometry( StateEstimate_ARF ), 
            &m_T_ARF_DRF
            );

        // Transformation: drone reference frame -> world reference frame.
        m_T_WRF_DRF = m_T_WRF_ARF * m_T_ARF_DRF;
    }






////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////



    void ForgetfulDrones::initializeVisMarker4Drone()
    {
        if (    ( ! p_Drone_Rotors_n      > 0 )     ||
                ( ! p_Drone_ArmLength     > 0 )     ||
                ( ! p_Drone_BodyWidth     > 0 )     ||
                ( ! p_Drone_BodyHeight    > 0 )         )
        {
            ROS_FATAL(
                "[%s] Failed to initialize visualization marker for drone since parameters have invalid values."
                "\t\tNumber of rotors: %d"
                "\t\tArm length: %f"
                "\t\tBody width: %f"
                "\t\tBody Height: %f",
                ros::this_node::getName().c_str(),
                p_Drone_Rotors_n, p_Drone_ArmLength, p_Drone_BodyWidth, p_Drone_BodyHeight
                );
            ros::shutdown();
        }
        

        if ( ! m_RVIZ_Drone.markers.empty() )
            ROS_WARN(
                "[%s] Re-initializing visualization marker for drone.",
                ros::this_node::getName().c_str()
                );


        m_RVIZ_Drone.markers.clear();
        m_RVIZ_Drone.markers.reserve( 2 * p_Drone_Rotors_n + 1 );
        
        visualization_msgs::Marker RVIZ_Drone_Rotor;
            RVIZ_Drone_Rotor.header.stamp = ros::Time();
            RVIZ_Drone_Rotor.header.frame_id = p_RVIZ_Drone_Childs_FrameID;
            RVIZ_Drone_Rotor.ns = "vehicle_rotor";
            RVIZ_Drone_Rotor.action = visualization_msgs::Marker::ADD;
            RVIZ_Drone_Rotor.type = visualization_msgs::Marker::CYLINDER;
            RVIZ_Drone_Rotor.scale.x = 0.2 * p_RVIZ_Drone_Scale;
            RVIZ_Drone_Rotor.scale.y = 0.2 * p_RVIZ_Drone_Scale;
            RVIZ_Drone_Rotor.scale.z = 0.01 * p_RVIZ_Drone_Scale;
            RVIZ_Drone_Rotor.color.r = 0.4;
            RVIZ_Drone_Rotor.color.g = 0.4;
            RVIZ_Drone_Rotor.color.b = 0.4;
            RVIZ_Drone_Rotor.color.a = 0.8;
            RVIZ_Drone_Rotor.pose.position.z = 0;

        visualization_msgs::Marker RVIZ_Drone_Arm;
            RVIZ_Drone_Arm.header.stamp = ros::Time();
            RVIZ_Drone_Arm.header.frame_id = p_RVIZ_Drone_Childs_FrameID;
            RVIZ_Drone_Arm.ns = "vehicle_arm";
            RVIZ_Drone_Arm.action = visualization_msgs::Marker::ADD;
            RVIZ_Drone_Arm.type = visualization_msgs::Marker::CUBE;
            RVIZ_Drone_Arm.scale.x = p_Drone_ArmLength * p_RVIZ_Drone_Scale;
            RVIZ_Drone_Arm.scale.y = 0.02 * p_RVIZ_Drone_Scale;
            RVIZ_Drone_Arm.scale.z = 0.01 * p_RVIZ_Drone_Scale;
            RVIZ_Drone_Arm.color.r = 0.0;
            RVIZ_Drone_Arm.color.g = 0.0;
            RVIZ_Drone_Arm.color.b = 1.0;
            RVIZ_Drone_Arm.color.a = 1.0;
            RVIZ_Drone_Arm.pose.position.z = -0.015 * p_RVIZ_Drone_Scale;

            const float RVIZ_Drone_RotorAngleIncrement = 2 * M_PI / p_Drone_Rotors_n;
            for 
            (
                float Angle = RVIZ_Drone_RotorAngleIncrement / 2; 
                Angle <= 2 * M_PI; 
                Angle += RVIZ_Drone_RotorAngleIncrement
            ) 
            {
                RVIZ_Drone_Rotor.pose.position.x 
                    = p_Drone_ArmLength * cos( Angle ) * p_RVIZ_Drone_Scale;
                RVIZ_Drone_Rotor.pose.position.y 
                    = p_Drone_ArmLength * sin( Angle ) * p_RVIZ_Drone_Scale;
                RVIZ_Drone_Rotor.id++;

                RVIZ_Drone_Arm.pose.position.x = RVIZ_Drone_Rotor.pose.position.x / 2;
                RVIZ_Drone_Arm.pose.position.y = RVIZ_Drone_Rotor.pose.position.y / 2;
                RVIZ_Drone_Arm.pose.orientation = tf::createQuaternionMsgFromYaw( Angle );
                RVIZ_Drone_Arm.id++;

                m_RVIZ_Drone.markers.push_back( RVIZ_Drone_Rotor );
                m_RVIZ_Drone.markers.push_back( RVIZ_Drone_Arm );
            }

        visualization_msgs::Marker RVIZ_Drone_Body;
            RVIZ_Drone_Body.header.stamp = ros::Time();
            RVIZ_Drone_Body.header.frame_id = p_RVIZ_Drone_Childs_FrameID;
            RVIZ_Drone_Body.ns = "vehicle_body";
            RVIZ_Drone_Body.action = visualization_msgs::Marker::ADD;
            RVIZ_Drone_Body.type = visualization_msgs::Marker::CUBE;
            RVIZ_Drone_Body.scale.x = p_Drone_BodyWidth * p_RVIZ_Drone_Scale;
            RVIZ_Drone_Body.scale.y = p_Drone_BodyWidth * p_RVIZ_Drone_Scale;
            RVIZ_Drone_Body.scale.z = p_Drone_BodyHeight * p_RVIZ_Drone_Scale;
            RVIZ_Drone_Body.color.r = 0.0;
            RVIZ_Drone_Body.color.g = 1.0;
            RVIZ_Drone_Body.color.b = 0.0;
            RVIZ_Drone_Body.color.a = 0.8;

        m_RVIZ_Drone.markers.push_back( RVIZ_Drone_Body );
    }



    void ForgetfulDrones::processCNNOutput(
        const double& IN_GoalX_IRF, 
        const double& IN_GoalY_IRF,
        const double& IN_Speed2Goal_Normalized,
        Eigen::Vector3d& OUT_GoalPos_WRF,
        double& OUT_Speed2Goal
        )
    {
        // Denormalize speed prediction of CNN with p_LT_MaxSpeed. 
        // p_LT_MinSpeed <= OUT_Speed2Goal <= p_LT_MaxSpeed.
        OUT_Speed2Goal = std::max( p_LT_MinSpeed, p_LT_MaxSpeed * IN_Speed2Goal_Normalized );

        // Compute distance from drone to goal position:
        // p_LT_MinDist <= p_LT_Duration * OUT_Speed2Goal <= p_LT_MaxDist
        double LT_Dist = std::max( p_LT_MinDist,
            std::min( p_LT_MaxDist, p_LT_Duration * OUT_Speed2Goal ) );

        // Compute T_DRF_GRF: transformation from goal to drone reference frame. 
        geometry_msgs::Pose GoalPose_DRF;
            GoalPose_DRF.position 
                = transform_IRF2DRF( IN_GoalX_IRF, IN_GoalY_IRF, LT_Dist );
            GoalPose_DRF.orientation.w = 1.0;
            GoalPose_DRF.orientation.x = 0.0;
            GoalPose_DRF.orientation.y = 0.0;
            GoalPose_DRF.orientation.z = 0.0;
        kindr::minimal::QuatTransformation T_DRF_GRF;
        tf::poseMsgToKindr(
            GoalPose_DRF, 
            &T_DRF_GRF
            );

        // Compute T_WRF_GRF: transformation from goal to world reference frame
        kindr::minimal::QuatTransformation T_WRF_GRF = m_T_WRF_DRF * T_DRF_GRF;

        // Compute the goal position in world reference frame
        OUT_GoalPos_WRF = T_WRF_GRF.getPosition();
    }



    geometry_msgs::Point ForgetfulDrones::transform_IRF2DRF(
        const double& IN_PointX_IRF, 
        const double& IN_PointY_IRF, 
        const double& IN_Dist2Point
        )
    {
        // Compute yaw and pitch angle to position in reference frame of drone.
        const double Yaw_DRF = -p_Drone_Cam_HalfYawAOV_Rad * IN_PointX_IRF;
        const double Pitch_DRF = p_Drone_Cam_HalfPitchAOV_Rad * IN_PointY_IRF;

        // Compute vector in drone reference frame.
        geometry_msgs::Point OUT_PointPos_DRF;
            OUT_PointPos_DRF.x = IN_Dist2Point * cos( Pitch_DRF ) * cos( Yaw_DRF );
            OUT_PointPos_DRF.y = IN_Dist2Point * cos( Pitch_DRF ) * sin( Yaw_DRF );
            OUT_PointPos_DRF.z = IN_Dist2Point * sin( Pitch_DRF );
        
        return OUT_PointPos_DRF;
    }




    bool ForgetfulDrones::computeLocalTrajectory(
        const quadrotor_common::TrajectoryPoint& IN_StartState,
        const Eigen::Vector3d& IN_EndPosition,
        const double& IN_Velocity
        ) 
    {   
        bool LT_SuccessfullyComputed;

        const double LT_StraightDist 
            = ( IN_EndPosition - IN_StartState.position ).norm();

        const double LT_Duration 
            = LT_StraightDist / std::min({
                IN_Velocity,
                IN_StartState.velocity.norm() + m_LT_MaxVelocityIncrement,
                p_LT_MaxSpeed
                });

        RQTG::RapidTrajectoryGenerator LocalTrajectory = {
            Vec3_From_EigenVector3d( IN_StartState.position ), 
            Vec3_From_EigenVector3d( IN_StartState.velocity ), 
            Vec3_From_EigenVector3d( IN_StartState.acceleration ), 
            m_LT_GravityVector
            };
        LocalTrajectory.SetGoalPosition( Vec3_From_EigenVector3d( IN_EndPosition ) );
        LocalTrajectory.Generate( LT_Duration );

        
        if ( LocalTrajectoryIsInputAndPositionFeasible( LocalTrajectory ) )
        {
            m_LocTraj = LocalTrajectory;
            LT_SuccessfullyComputed = true;
        }
        else
        {
            LT_SuccessfullyComputed = false;

            ROS_WARN(
                "[%s] Computed local trajectory failed input and/or position feasibility check and was therefore discarded."
                "\t\tStart position: [%f, %f, %f]"
                "\t\tEnd position: [%f, %f, %f]"
                "\t\tSet Velocity: [%f]"
                "\t\tDuration: [%f]",
                ros::this_node::getName().c_str(),
                IN_StartState.position.x(), IN_StartState.position.y(), IN_StartState.position.z(),
                IN_EndPosition.x(), IN_EndPosition.y(), IN_EndPosition.z(),
                IN_Velocity,
                LT_Duration
                );
        }

        return LT_SuccessfullyComputed;
    }




    bool ForgetfulDrones::LocalTrajectoryIsInputAndPositionFeasible( RQTG::RapidTrajectoryGenerator& LocalTrajectory )
    {
        RQTG::RapidTrajectoryGenerator::InputFeasibilityResult InputFeasibility 
            = LocalTrajectory.CheckInputFeasibility(
                p_LT_MinThrust,
                p_LT_MaxThrust,
                p_LT_MaxBodyRates,
                p_LT_InputFeasibilityCheck_MinSamplingTime
                );

        Vec3 BoundaryPoint = { 0.0, 0.0, p_LT_MinAltitude };  // a point on the floor
        Vec3 BoundaryVector = { 0.0, 0.0, 1.0 };  // we want to be above the point
        RQTG::RapidTrajectoryGenerator::StateFeasibilityResult PositionFeasibility_Floor 
            = LocalTrajectory.CheckPositionFeasibility(
                BoundaryPoint, 
                BoundaryVector
                );

        BoundaryPoint[2] = p_LT_MaxAltitude;  // a point on the ceiling
        BoundaryVector[2] = -1.0;  // we want to be below the point
        RQTG::RapidTrajectoryGenerator::StateFeasibilityResult PositionFeasibility_Ceiling 
            = LocalTrajectory.CheckPositionFeasibility(
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



    void ForgetfulDrones::visualizeTrajectory( const TrajectoryTypes& TrajType )
    {
        visualization_msgs::Marker TrajectoryMarker;
            TrajectoryMarker.header.frame_id = "world";
            TrajectoryMarker.pose.orientation.w = 1.0;
            TrajectoryMarker.scale.x = 0.05;
            TrajectoryMarker.color.a = 1.0;
            TrajectoryMarker.type = visualization_msgs::Marker::LINE_LIST;

        switch ( TrajType )
        {
            case TrajectoryTypes::GLOBAL:
            {
                visualizeGlobalTrajectory( TrajectoryMarker );
                break;
            }
            case TrajectoryTypes::LOCAL:
            {
                visualizeLocalTrajectory( TrajectoryMarker );
                break;
            }
        }
    }


    void ForgetfulDrones::visualizeGlobalTrajectory( visualization_msgs::Marker& Vis_GT_Marker )
    {
        Vis_GT_Marker.id = 0;

        setColorRGBOfVisMarker( VisualizationColors::PURPLE, Vis_GT_Marker );
        
        setPointsOfTrajectoryVisMarker(
            m_GT_States,
            Vis_GT_Marker
            );

        ROS_Pub_GlobalTrajectory.publish( Vis_GT_Marker );
    }


    void ForgetfulDrones::setColorRGBOfVisMarker(
        const VisualizationColors& IN_VisColor,
        visualization_msgs::Marker& OUT_VisMarker
        )
    {
        switch ( IN_VisColor ) 
        {
            case VisualizationColors::RED: 
                OUT_VisMarker.color.r = 1.0;
                OUT_VisMarker.color.g = 0.0;
                OUT_VisMarker.color.b = 0.0;
                break;

            case VisualizationColors::BLUE: 
                OUT_VisMarker.color.r = 0.0;
                OUT_VisMarker.color.g = 0.0;
                OUT_VisMarker.color.b = 1.0;
                break;
            
            case VisualizationColors::BLACK: 
                OUT_VisMarker.color.r = 0.0;
                OUT_VisMarker.color.g = 0.0;
                OUT_VisMarker.color.b = 0.0;
                break;
            
            case VisualizationColors::GREEN: 
                OUT_VisMarker.color.r = 0.0;
                OUT_VisMarker.color.g = 1.0;
                OUT_VisMarker.color.b = 0.0;
                break;
            
            case VisualizationColors::PURPLE: 
                OUT_VisMarker.color.r = 1.0;
                OUT_VisMarker.color.g = 0.0;
                OUT_VisMarker.color.b = 1.0;
                break;
            
            case VisualizationColors::WHITE: 
                OUT_VisMarker.color.r = 1.0;
                OUT_VisMarker.color.g = 1.0;
                OUT_VisMarker.color.b = 1.0;
                break;
            
            case VisualizationColors::YELLOW: 
                OUT_VisMarker.color.r = 1.0;
                OUT_VisMarker.color.g = 1.0;
                OUT_VisMarker.color.b = 0.0;
                break;            
        }
    }


    void ForgetfulDrones::setPointsOfTrajectoryVisMarker(
        const std::vector<quadrotor_common::TrajectoryPoint>& IN_Trajectory,
        visualization_msgs::Marker& OUT_TrajectoryVisMarker
        )
    {
        for ( const quadrotor_common::TrajectoryPoint& state : IN_Trajectory )
        {
            OUT_TrajectoryVisMarker.points.push_back(
                quadrotor_common::vectorToPoint( quadrotor_common::eigenToGeometry(state.position) )
                );
            OUT_TrajectoryVisMarker.points.push_back(
                quadrotor_common::vectorToPoint( quadrotor_common::eigenToGeometry(state.position + state.velocity / 20.0) )
                );
        }
    }




    void ForgetfulDrones::initializeGlobalTrajectory() 
    {
        if ( p_GT_UseExisting_On ) 
            loadGlobalTrajectory();
        else
        {
            computeGloTrajForExpert();
            saveGlobalTrajectory();
        } 

        visualizeTrajectory( TrajectoryTypes::GLOBAL );
    }



    void ForgetfulDrones::loadGlobalTrajectory()
    {
        ROS_INFO(
            "[%s] Loading global trajectory...",
            ros::this_node::getName().c_str()
            );
        
        m_GT_States.clear();

        std::string GT_Filename = p_GT_DirPath + "/global_trajectory.txt";
        std::ifstream GT_InputFile( GT_Filename );
        
        if ( GT_InputFile.is_open() ) 
        {
            std::string String;
            quadrotor_common::TrajectoryPoint State;

            while ( getline ( GT_InputFile, String, ';' ) ) 
            {
                // Position, velocity, acceleration, jerk, snap
                                                          State.position[0]               = atof(String.c_str());
                getline(GT_InputFile, String, ';');       State.position[1]               = atof(String.c_str());
                getline(GT_InputFile, String, ';');       State.position[2]               = atof(String.c_str());
                getline(GT_InputFile, String, ';');       State.velocity[0]               = atof(String.c_str());
                getline(GT_InputFile, String, ';');       State.velocity[1]               = atof(String.c_str());
                getline(GT_InputFile, String, ';');       State.velocity[2]               = atof(String.c_str());
                getline(GT_InputFile, String, ';');       State.acceleration[0]           = atof(String.c_str());
                getline(GT_InputFile, String, ';');       State.acceleration[1]           = atof(String.c_str());
                getline(GT_InputFile, String, ';');       State.acceleration[2]           = atof(String.c_str());
                getline(GT_InputFile, String, ';');       State.jerk[0]                   = atof(String.c_str());
                getline(GT_InputFile, String, ';');       State.jerk[1]                   = atof(String.c_str());
                getline(GT_InputFile, String, ';');       State.jerk[2]                   = atof(String.c_str());
                getline(GT_InputFile, String, ';');       State.snap[0]                   = atof(String.c_str());
                getline(GT_InputFile, String, ';');       State.snap[1]                   = atof(String.c_str());
                getline(GT_InputFile, String, ';');       State.snap[2]                   = atof(String.c_str());
                // Heading, heading rate, heading acceleration
                getline(GT_InputFile, String, ';');       State.heading                   = atof(String.c_str());
                getline(GT_InputFile, String, ';');       State.heading_rate              = atof(String.c_str());
                getline(GT_InputFile, String, ';');       State.heading_acceleration      = atof(String.c_str());
                // Orientation, bodyrates, angular_acceleration, angular_jerk, angular_snap
                getline(GT_InputFile, String, ';');       double qw                       = atof(String.c_str());
                getline(GT_InputFile, String, ';');       double qx                       = atof(String.c_str());
                getline(GT_InputFile, String, ';');       double qy                       = atof(String.c_str());
                getline(GT_InputFile, String, ';');       double qz                       = atof(String.c_str());
                                                          State.orientation = Eigen::Quaternion<double>(qw, qx, qy, qz);
                getline(GT_InputFile, String, ';');       State.bodyrates[0]              = atof(String.c_str());
                getline(GT_InputFile, String, ';');       State.bodyrates[1]              = atof(String.c_str());
                getline(GT_InputFile, String, ';');       State.bodyrates[2]              = atof(String.c_str());
                getline(GT_InputFile, String, ';');       State.angular_acceleration[0]   = atof(String.c_str());
                getline(GT_InputFile, String, ';');       State.angular_acceleration[1]   = atof(String.c_str());
                getline(GT_InputFile, String, ';');       State.angular_acceleration[2]   = atof(String.c_str());
                getline(GT_InputFile, String, ';');       State.angular_jerk[0]           = atof(String.c_str());
                getline(GT_InputFile, String, ';');       State.angular_jerk[1]           = atof(String.c_str());
                getline(GT_InputFile, String, ';');       State.angular_jerk[2]           = atof(String.c_str());
                getline(GT_InputFile, String, ';');       State.angular_snap[0]           = atof(String.c_str());
                getline(GT_InputFile, String, ';');       State.angular_snap[1]           = atof(String.c_str());
                getline(GT_InputFile, String, ';');       State.angular_snap[2]           = atof(String.c_str());
                // Time_from_start
                getline(GT_InputFile, String, GT_InputFile.widen('\n')); 
                State.time_from_start = ros::Duration(atof(String.c_str()));



                // Append state to global trajectory
                m_GT_States.push_back(State);
            }

            GT_InputFile.close();

            ROS_INFO(
                "[%s] Global trajectory loaded from %s.",
                ros::this_node::getName().c_str(),
                GT_Filename.c_str()
                );
        } 
        else 
        {
            ROS_WARN( 
                "[%s] Failed to load global trajectory from %s.",
                ros::this_node::getName().c_str(),
                GT_Filename.c_str()
                );
        }
    }



    void ForgetfulDrones::computeGloTrajForExpert()
    {
        ROS_INFO (
            "[%s] Computing global trajectory through %d gates."
            "\t\tMinimization weights: [%f, %f, %f, %f]"
            "\t\tPolynomial order: %d"
            "\t\tContinuity order: %d"
            "\t\tMaximum thrust: %f"
            "\t\tMaximum roll-pitch rate: %f"
            "\t\tSampling Time: %f ...",
            ros::this_node::getName().c_str(),
            m_Gates_n,
            m_GT_Settings.minimization_weights[0],
            m_GT_Settings.minimization_weights[1],
            m_GT_Settings.minimization_weights[2],
            m_GT_Settings.minimization_weights[3],
            m_GT_Settings.polynomial_order,
            m_GT_Settings.continuity_order,
            m_GT_MaxThrust,
            m_GT_MaxRollPitchRate,
            m_GT_SamplingTime
            );

        
        m_GT_Settings.way_points = { 
            std::make_move_iterator( m_Gates_WaypointPos_Temp.begin() ),
            std::make_move_iterator( m_Gates_WaypointPos_Temp.end() ),
            };

        // Calculate segment times, first element relates to segment from last to first waypoint.
        Eigen::VectorXd GT_InitialSegmentTimes = Eigen::VectorXd::Ones( m_Gates_n );
        Eigen::Vector3d SegmentStart = m_Gates_WaypointPos_Temp.back();
        for ( int i = 0; i < m_Gates_n; i++ ) 
        {
            GT_InitialSegmentTimes[i] 
                = ( m_Gates_WaypointPos_Temp[i] - SegmentStart ).norm() / p_GT_MaxVel;

            SegmentStart = m_Gates_WaypointPos_Temp[i];
        }

        quadrotor_common::Trajectory GT_Sampled =
        trajectory_generation_helper::polynomials::generateMinimumSnapRingTrajectoryWithSegmentRefinement(
            GT_InitialSegmentTimes,
            m_GT_Settings,
            p_GT_MaxVel,
            m_GT_MaxThrust,
            m_GT_MaxRollPitchRate,
            m_GT_SamplingTime
            );

        m_GT_States.clear();
        m_GT_States = { 
            std::make_move_iterator( GT_Sampled.points.begin() ),
            std::make_move_iterator( GT_Sampled.points.end() ),
            };
        
        unsigned int GT_States_Number = m_GT_States.size();
        double GT_Duration = GT_States_Number * m_GT_SamplingTime;
        ROS_INFO(
            "[%s] Global trajectory successfully computed."
            "\t\tTotal flight time: %1.1f."
            "\t\tNumber of States: %d",
            ros::this_node::getName().c_str(),
            GT_Duration,
            GT_States_Number
            );
    }



    void ForgetfulDrones::saveGlobalTrajectory() 
    {
        ROS_INFO(
            "[%s] Saving global trajectory...",
            ros::this_node::getName().c_str()
            );

        // ??? Delete file global_trajectory.txt first ??? 

        std::string GT_Filename = p_GT_DirPath + "/global_trajectory.txt";
        std::ofstream GT_OutFile;

        GT_OutFile.open( GT_Filename, std::ios_base::app );
        for ( const quadrotor_common::TrajectoryPoint& State : m_GT_States ) 
        {
            GT_OutFile  
                << std::to_string(  State.position.x()              ) << ";"
                << std::to_string(  State.position.y()              ) << ";"
                << std::to_string(  State.position.z()              ) << ";"
                << std::to_string(  State.velocity.x()              ) << ";"
                << std::to_string(  State.velocity.y()              ) << ";"
                << std::to_string(  State.velocity.z()              ) << ";"
                << std::to_string(  State.acceleration.x()          ) << ";"
                << std::to_string(  State.acceleration.y()          ) << ";"
                << std::to_string(  State.acceleration.z()          ) << ";"
                << std::to_string(  State.jerk.x()                  ) << ";"
                << std::to_string(  State.jerk.y()                  ) << ";"
                << std::to_string(  State.jerk.z()                  ) << ";"
                << std::to_string(  State.snap.x()                  ) << ";"
                << std::to_string(  State.snap.y()                  ) << ";"
                << std::to_string(  State.snap.z()                  ) << ";"
                << std::to_string(  State.heading                   ) << ";"
                << std::to_string(  State.heading_rate              ) << ";"
                << std::to_string(  State.heading_acceleration      ) << ";"
                << std::to_string(  State.orientation.w()           ) << ";"
                << std::to_string(  State.orientation.x()           ) << ";"
                << std::to_string(  State.orientation.y()           ) << ";"
                << std::to_string(  State.orientation.z()           ) << ";"
                << std::to_string(  State.bodyrates.x()             ) << ";"
                << std::to_string(  State.bodyrates.y()             ) << ";"
                << std::to_string(  State.bodyrates.z()             ) << ";"
                << std::to_string(  State.angular_acceleration.x()  ) << ";"
                << std::to_string(  State.angular_acceleration.y()  ) << ";"
                << std::to_string(  State.angular_acceleration.z()  ) << ";"
                << std::to_string(  State.angular_jerk.x()          ) << ";"
                << std::to_string(  State.angular_jerk.y()          ) << ";"
                << std::to_string(  State.angular_jerk.z()          ) << ";"
                << std::to_string(  State.angular_snap.x()          ) << ";"
                << std::to_string(  State.angular_snap.y()          ) << ";"
                << std::to_string(  State.angular_snap.z()          ) << ";"
                << std::to_string(  State.time_from_start.toSec()   ) << "\n";
        }
        GT_OutFile.close();
        
        ROS_INFO(
            "[%s] Global trajectory saved to %s.",
            ros::this_node::getName().c_str(),
            GT_Filename.c_str()
            );
    }







    void ForgetfulDrones::setIndexOfGTProjStateToGTStateWithMinDist2( const Eigen::Vector3d& Position )
    {
        ROS_INFO(
            "[%s] Scanning global trajectory for state with minimum distance to (%f m, %f m, %f m)...",
            ros::this_node::getName().c_str(),
            Position[0], Position[1], Position[2]
            );

        double MinDistance2GT = std::numeric_limits< double >::max();
        double IndexedDistance;
        
        for ( size_t State_i = 0; State_i < m_GT_States.size(); State_i++ )
        {
            // Find index of state of global trajectory with minimum distance to position of drone.
            IndexedDistance = ( m_GT_States[ State_i ].position - Position ).norm();
            if ( IndexedDistance < MinDistance2GT )
            {
                MinDistance2GT = IndexedDistance;
                m_GT_ProjectionState_i = State_i;
            }
        }

        ROS_INFO(
            "[%s] Results:\n\tState at (%f m, %f m, %f m) with minimum distance of %f m.", 
            ros::this_node::getName().c_str(),
            m_GT_States[ m_GT_ProjectionState_i ].position[ 0 ],
            m_GT_States[ m_GT_ProjectionState_i ].position[ 1 ],
            m_GT_States[ m_GT_ProjectionState_i ].position[ 2 ],
            MinDistance2GT
            );
    }


    void ForgetfulDrones::setMinAndMaxSpeedAfterScanningGlobalTrajectory()
    {
        ROS_INFO(
            "[%s] Scanning global trajectory for minimum and maximum velocity...",
            ros::this_node::getName().c_str()
            );

        double IndexedVelocity;
        m_GT_MaxSpeed = 0.0;
        m_GT_MinSpeed = std::numeric_limits< double >::max();
        
        for ( size_t i = 0; i < m_GT_States.size(); i++ )
        {
            // Find the maximum and minimum velocity along the global trajectory.
            IndexedVelocity = ( m_GT_States[i].velocity ).norm();
            m_GT_MaxSpeed = std::max( m_GT_MaxSpeed, IndexedVelocity );
            m_GT_MinSpeed = std::min( m_GT_MinSpeed, IndexedVelocity );
        }

        ROS_INFO(
            "[%s] Results:\n\tMaximum Velocity: %f m/s\n\tMinimum Velocity: %f m/s", 
            ros::this_node::getName().c_str(),
            m_GT_MaxSpeed, 
            m_GT_MinSpeed
            );
    }  


    void ForgetfulDrones::setIndexOfGlobalTrajectoryProjectionState()
    {
        Eigen::Vector3d GT_ProjState_Direction;
        double GT_ProjectionState_SpeedLike;
        Eigen::Vector3d Direction_Proj2RefState;
        double ProjLevel; // of Direction_Proj2RefState onto GT_ProjState_Direction

        do
        {
            GT_ProjState_Direction
                = m_GT_States[ m_GT_ProjectionState_i     ].position 
                - m_GT_States[ m_GT_ProjectionState_i - 1 ].position;
            GT_ProjectionState_SpeedLike = GT_ProjState_Direction.norm();
            GT_ProjState_Direction /= GT_ProjectionState_SpeedLike;

            Direction_Proj2RefState
                = m_ReferenceState_WRF.position 
                - m_GT_States [ m_GT_ProjectionState_i - 1 ].position;
            
            ProjLevel = Direction_Proj2RefState.dot( GT_ProjState_Direction );

            m_GT_ProjectionState_i++;
            m_GT_ProjectionState_i %= m_GT_States.size();
        
        } while ( ProjLevel > GT_ProjectionState_SpeedLike );


        // If after above projection 
        // distance from GT Projection state to reference state is longer than 1 m,
        // set index to GT state with min distance to reference state.
        if ( 
            ( m_ReferenceState_WRF.position 
            - m_GT_States[ m_GT_ProjectionState_i ].position ).norm() > 1.0 
            ) 
            setIndexOfGTProjStateToGTStateWithMinDist2( m_ReferenceState_WRF.position );
    }





    void ForgetfulDrones::loadGatesXYZYawFromROSParameterServer() 
    {
        p_Gates_OriginPosYaw.clear();
        m_Gates_OriginPosYaw_Temp.clear();

        
        XmlRpc::XmlRpcValue Gates_XYZ_Unmodified, Gates_Yaw_Unmodified;
        if ( !ROS_PNH.getParam( "goal_positions", Gates_XYZ_Unmodified ) ||
             !ROS_PNH.getParam( "goal_orientations", Gates_Yaw_Unmodified) ) 
            ROS_ERROR (
                "[%s] Failed to load gate positions and/or orientations from ROS parameter server.", 
                ROS_PNH.getNamespace().c_str()
                );
        

        for ( int i = 0; i < Gates_XYZ_Unmodified.size(); i++ ) 
        {
            if ( static_cast<double>( Gates_XYZ_Unmodified[i]["gate"] ) > 0.5 ) 
            {
                // this is a real gate and not only a waypoint

                ROS_INFO(
                    "Gate x, y, z, yaw: [%f, %f, %f][%f]", 
                    static_cast<double>( Gates_XYZ_Unmodified   [i]["x"]  ),
                    static_cast<double>( Gates_XYZ_Unmodified   [i]["y"]  ),
                    static_cast<double>( Gates_XYZ_Unmodified   [i]["z"]  ),
                    static_cast<double>( Gates_Yaw_Unmodified   [i]["yaw"])
                    );
                
                double yaw = static_cast<double>( Gates_Yaw_Unmodified[i]["yaw"] );

                Eigen::Vector2d pos_no_shift = Eigen::Vector2d (
                    static_cast<double>( Gates_XYZ_Unmodified[i]["x"] ),
                    static_cast<double>( Gates_XYZ_Unmodified[i]["y"] )
                    );

                double offset = static_cast<double>( Gates_Yaw_Unmodified[i]["offset"] );
                Eigen::Vector2d offset_vec = Eigen::Vector2d (
                    offset * std::cos(yaw + 1.57), 
                    offset * std::sin(yaw + 1.57)
                    );

                p_Gates_OriginPosYaw.emplace_back(
                    pos_no_shift.x() + offset_vec.x(),
                    pos_no_shift.y() + offset_vec.y(),
                    static_cast<double>( Gates_XYZ_Unmodified[i]["z"] ),
                    yaw
                    );
            }
        }

        m_Gates_OriginPosYaw_Temp = p_Gates_OriginPosYaw;
        m_Gates_n = p_Gates_OriginPosYaw.size();
    }


    void ForgetfulDrones::respawnGazeboGateModelsWithRandomAxialShifts() 
    {
        ROS_INFO( 
            "[%s] Respawning gates in Gazebo randomly with axial shifts that are subject to uniform real distribution in range +/- %f m around unmodified gate positions...", 
            ros::this_node::getName().c_str(),
            p_Gates_Respawn_MaxAxialShift
            );


        m_Gates_WaypointPos_Temp.clear();


        // Create ModelState and set members shared by all gate models
        gazebo_msgs::ModelState Gate_ModelState;
            Gate_ModelState.reference_frame = "world";

        
        std::uniform_real_distribution<double> UniformRealDistribution_PlusMinus1( -1.0, 1.0 );
        // Random engine seeded with current wall time.
        std::default_random_engine REngine; REngine.seed( ros::WallTime::now().toNSec() );
        // Lambda function for convenience
        auto getRandDouble_PlusMinus1 = [ &UniformRealDistribution_PlusMinus1, &REngine ]() { return UniformRealDistribution_PlusMinus1( REngine ); };


        size_t Gate_i = 0;
        for ( const std::string& ModelName : m_Gazebo_ModelStates.name ) 
        {
            if ( ModelName.find( "gate" ) != std::string::npos )
            {   
                // -> m_Gates_OriginPosYaw_Temp
                m_Gates_OriginPosYaw_Temp[ Gate_i ][ 0 ] 
                    = p_Gates_OriginPosYaw[ Gate_i ][ 0 ] + p_Gates_Respawn_MaxAxialShift * getRandDouble_PlusMinus1();
                m_Gates_OriginPosYaw_Temp[ Gate_i ][ 1 ] 
                    = p_Gates_OriginPosYaw[ Gate_i ][ 1 ] + p_Gates_Respawn_MaxAxialShift * getRandDouble_PlusMinus1();
                m_Gates_OriginPosYaw_Temp[ Gate_i ][ 2 ] 
                    = std::max(
                        p_Gates_Respawn_MinZ,
                        p_Gates_OriginPosYaw[ Gate_i ][ 2 ] + p_Gates_Respawn_MaxAxialShift * getRandDouble_PlusMinus1()
                        );

                // -> m_Gates_WaypointPos_Temp
                //m_Gates_WaypointPos_Temp[ Gate_i ] = {
                //    m_Gates_OriginPosYaw_Temp[ Gate_i ][ 0 ],
                //    m_Gates_OriginPosYaw_Temp[ Gate_i ][ 1 ],
                //    m_Gates_OriginPosYaw_Temp[ Gate_i ][ 2 ] + p_Gates_HalfHeight
                //    };
                m_Gates_WaypointPos_Temp[ Gate_i ] = Eigen::Vector3d(
                    m_Gates_OriginPosYaw_Temp[ Gate_i ][ 0 ],
                    m_Gates_OriginPosYaw_Temp[ Gate_i ][ 1 ],
                    m_Gates_OriginPosYaw_Temp[ Gate_i ][ 2 ] + p_Gates_HalfHeight
                    );


                // Set members that are individual to gates
                Gate_ModelState.model_name = ModelName;
                Gate_ModelState.pose.position.x = m_Gates_OriginPosYaw_Temp[ Gate_i ][ 0 ];
                Gate_ModelState.pose.position.y = m_Gates_OriginPosYaw_Temp[ Gate_i ][ 1 ];
                Gate_ModelState.pose.position.z = m_Gates_OriginPosYaw_Temp[ Gate_i ][ 2 ];
                Gate_ModelState.pose.orientation.w = std::cos( m_Gates_OriginPosYaw_Temp[ Gate_i ][ 3 ] / 2.0 );
                Gate_ModelState.pose.orientation.z = std::sin( m_Gates_OriginPosYaw_Temp[ Gate_i ][ 3 ] / 2.0 );
                
                // Publish to ROS topic "/gazebo/set_model_state"
                ROS_Pub_GazeboSetModelState.publish( Gate_ModelState );
                
                Gate_i++;
            }
        }

        ROS_INFO( 
            "[%s] All gates respawned.",
            ros::this_node::getName().c_str()
            );
    }


    void ForgetfulDrones::moveGazeboGateModels()
    {
        double Run_Time = ( ros::WallTime::now() - m_Run_StartTime ).toSec();
        double Gates_Dynamic_DeterministicPhase = p_Gates_Dynamic_RadFreq * Run_Time;

        gazebo_msgs::ModelState Gazebo_GateModelState;
        Gazebo_GateModelState.reference_frame = "world";

        int Gate_i = 0;    
        for ( const std::string& ModelName : m_Gazebo_ModelStates.name ) 
        {
            if ( ModelName.find("gate") != std::string::npos ) 
            {
                Gazebo_GateModelState.model_name = ModelName;
                    
                m_Gates_OriginPosYaw_Temp[ Gate_i ][ 0 ] = p_Gates_OriginPosYaw[ Gate_i ][ 0 ] 
                    + p_Gates_Dynamic_AxialAmp * std::sin( Gates_Dynamic_DeterministicPhase + m_Gates_Dynamic_RandomPhases[ Gate_i ][ 0 ] );
                m_Gates_OriginPosYaw_Temp[ Gate_i ][ 1 ] = p_Gates_OriginPosYaw[ Gate_i ][ 1 ] 
                    + p_Gates_Dynamic_AxialAmp * std::sin( Gates_Dynamic_DeterministicPhase + m_Gates_Dynamic_RandomPhases[ Gate_i ][ 1 ] );
                m_Gates_OriginPosYaw_Temp[ Gate_i ][ 2 ] = p_Gates_OriginPosYaw[ Gate_i ][ 2 ] 
                    + 0.5 * p_Gates_Dynamic_AxialAmp * std::sin( Gates_Dynamic_DeterministicPhase + m_Gates_Dynamic_RandomPhases[ Gate_i ][ 2 ] );

                Gazebo_GateModelState.pose.position.x = m_Gates_OriginPosYaw_Temp[ Gate_i ][ 0 ];
                Gazebo_GateModelState.pose.position.y = m_Gates_OriginPosYaw_Temp[ Gate_i ][ 1 ];
                Gazebo_GateModelState.pose.position.z = m_Gates_OriginPosYaw_Temp[ Gate_i ][ 2 ];
                Gazebo_GateModelState.pose.orientation.w = std::cos( m_Gates_OriginPosYaw_Temp[ Gate_i ][ 3 ] / 2.0 );
                Gazebo_GateModelState.pose.orientation.z = std::sin( m_Gates_OriginPosYaw_Temp[ Gate_i ][ 3 ] / 2.0 );

                ROS_Pub_GazeboSetModelState.publish( Gazebo_GateModelState );

                Gate_i++;
            }
        }
    }


    void ForgetfulDrones::visualizeGazeboGateModelsInRVIZ()
    {
        visualization_msgs::MarkerArray GateMarkerArray;
            visualization_msgs::Marker GateMarker;
                GateMarker.header.frame_id = "world";
                GateMarker.scale.x = GateMarker.scale.y = GateMarker.scale.z = 1.0;
                GateMarker.action = visualization_msgs::Marker::ADD;
                GateMarker.type = visualization_msgs::Marker::MESH_RESOURCE;
                GateMarker.mesh_use_embedded_materials = true;
                GateMarker.color.a = 1.0;
                GateMarker.color.r = 217.0 / 255.0;
                GateMarker.color.g = 100.0 / 255.0;
                GateMarker.color.b = 30.0 / 255.0;
                GateMarker.frame_locked = true;
                GateMarker.mesh_resource = m_Gates_COLLADA_Filename;

        std::size_t Gate_j = 0;
        for ( std::size_t Model_i = 0; Model_i < m_Gazebo_ModelStates.name.size(); Model_i++ ) 
        {
            if ( m_Gazebo_ModelStates.name[ Model_i ].find("gate") != std::string::npos ) 
            {
                GateMarker.id = Gate_j;
                GateMarker.pose.position = m_Gazebo_ModelStates.pose[ Model_i ].position;
                GateMarker.pose.orientation = m_Gazebo_ModelStates.pose[ Model_i ].orientation;
                GateMarkerArray.markers.push_back( GateMarker );
                
                Gate_j++;
            }
        }

        ROS_Pub_RvizGates.publish( GateMarkerArray );
    }


    void ForgetfulDrones::setReferenceStateFromLocalTrajectory()
    {
        const double LT_ElapsedTime 
            = (ros::Time::now() - m_LT_StartTime).toSec() 
            + m_ROS_Param_MainLoop_NominalDuration;

        /// ??? WRF -> DRF
        m_ReferenceState_WRF.position        = EigenVector3d_From_Vec3( m_LocTraj.GetPosition     (LT_ElapsedTime) );
        m_ReferenceState_WRF.velocity        = EigenVector3d_From_Vec3( m_LocTraj.GetVelocity     (LT_ElapsedTime) );
        m_ReferenceState_WRF.acceleration    = EigenVector3d_From_Vec3( m_LocTraj.GetAcceleration (LT_ElapsedTime) );
        m_ReferenceState_WRF.heading = std::atan2(
            m_ReferenceState_WRF.velocity.y(), 
            m_ReferenceState_WRF.velocity.x()
            );
        
        // transform from world reference frame to state estimate reference frame
        const Eigen::Quaterniond q_W_O = m_T_WRF_ARF.getEigenQuaternion();
        const Eigen::Vector3d t_W_O = m_T_WRF_ARF.getPosition();

        m_ReferenceState_ARF.position        = q_W_O.inverse() * (m_ReferenceState_WRF.position - t_W_O);
        m_ReferenceState_ARF.velocity        = q_W_O.inverse() * m_ReferenceState_WRF.velocity;
        m_ReferenceState_ARF.acceleration    = q_W_O.inverse() * m_ReferenceState_WRF.acceleration;
        m_ReferenceState_ARF.heading = quadrotor_common::wrapMinusPiToPi(
            m_ReferenceState_WRF.heading 
            - quadrotor_common::quaternionToEulerAnglesZYX(q_W_O).z()
            );        
    }


    void ForgetfulDrones::visualizeLocalTrajectory( visualization_msgs::Marker& Vis_LT_Marker )
    {
        Vis_LT_Marker.id = p_RVIZ_AllLTs_On? ++m_RVIZ_LT_Count : 1.0;

        setColorRGBOfVisMarker(
            VisualizationColors::GREEN,
            Vis_LT_Marker
            );

        double Vis_LT_Duration = p_RVIZ_CompleteLT_On? 
            m_LocTraj.GetEndTime() : 
            std::min( m_LocTraj.GetEndTime(), p_RVIZ_LT_Duration );


        std::vector<quadrotor_common::TrajectoryPoint> Vis_LT;
        resampleLocalTrajectory(
                Vis_LT_Duration, 
                1 / p_RVIZ_LT_SamplingFreq,
                Vis_LT
                );
        
        setPointsOfTrajectoryVisMarker(
            Vis_LT,
            Vis_LT_Marker
            );
        
        ROS_Pub_TrajectoriesCNN.publish(Vis_LT_Marker);
    }

    void ForgetfulDrones::resampleLocalTrajectory(
        const double& IN_Duration,
        const double& IN_SamplingTime,
        std::vector<quadrotor_common::TrajectoryPoint>& OUT_LT_Resampled
        )
    {
        for ( double TimePoint = 0.0; TimePoint <= IN_Duration; TimePoint += IN_SamplingTime )
        {
            OUT_LT_Resampled.push_back(
                getStateOfLocalTrajectoryAtTimePoint( TimePoint )
                );
        }
    }

    quadrotor_common::TrajectoryPoint
    ForgetfulDrones::getStateOfLocalTrajectoryAtTimePoint( const double& IN_TimePoint ) 
    {
        quadrotor_common::TrajectoryPoint OUT_State;

        OUT_State.position = EigenVector3d_From_Vec3( m_LocTraj.GetPosition(IN_TimePoint) );
        OUT_State.velocity = EigenVector3d_From_Vec3( m_LocTraj.GetVelocity(IN_TimePoint) );
        OUT_State.acceleration = EigenVector3d_From_Vec3( m_LocTraj.GetAcceleration(IN_TimePoint) );
        OUT_State.heading = std::atan2( OUT_State.velocity.y(), OUT_State.velocity.x() );

        return OUT_State;
    }






    void ForgetfulDrones::updateAutopilotStateTo( const AutopilotStates& NextAutopilotState )
    {
        std::string NameOfLastAutopilotState;
        switch ( m_AutopilotState )
        {
            case AutopilotStates::OFF:      NameOfLastAutopilotState = "OFF"; break;
            case AutopilotStates::HOVER:    NameOfLastAutopilotState = "HOVER"; break;
            case AutopilotStates::RACING:   NameOfLastAutopilotState = "RACING"; break;
        }

        if ( m_AutopilotState == NextAutopilotState) 
            updateAutopilotStateToCurrentState( NameOfLastAutopilotState );
        else
        {
            switch ( m_AutopilotState )
            {
                case AutopilotStates::OFF: exitAutopilotStateOFF();
                    switch ( NextAutopilotState )
                    {
                        case AutopilotStates::HOVER: {enterAutopilotStateHOVER( NameOfLastAutopilotState ); break;}
                        case AutopilotStates::RACING: {enterAutopilotStateRACING( NameOfLastAutopilotState ); break;}
                    }
                    break;
                case AutopilotStates::HOVER: exitAutopilotStateHOVER();
                    switch ( NextAutopilotState )
                    {
                        case AutopilotStates::OFF: {enterAutopilotStateOFF( NameOfLastAutopilotState ); break;}
                        case AutopilotStates::RACING: {enterAutopilotStateRACING( NameOfLastAutopilotState ); break;}
                    }
                    break;
                case AutopilotStates::RACING: exitAutopilotStateRACING();
                    switch ( NextAutopilotState )
                    {
                        case AutopilotStates::OFF: {enterAutopilotStateOFF( NameOfLastAutopilotState ); break;}
                        case AutopilotStates::HOVER: {enterAutopilotStateHOVER( NameOfLastAutopilotState ); break;}
                    }
                    break;
            }
        }
    }

    void ForgetfulDrones::updateAutopilotStateToCurrentState( const std::string& NameOfLastAutopilotState )
    {
        ROS_WARN ( 
            "[%s] Attempt to update autopilot to current state: %s was carried out. Nothing happens.",
            ros::this_node::getName().c_str(),
            NameOfLastAutopilotState.c_str()
            );
    }

    void ForgetfulDrones::exitAutopilotStateOFF()
    {
        ROS_DEBUG( "[%s] No implementation for exiting autopilot state: OFF.", ros::this_node::getName().c_str() );
        ROS_INFO ( "[%s] Exited autopilot state: OFF.", ros::this_node::getName().c_str() );
    }

    void ForgetfulDrones::exitAutopilotStateHOVER()
    {
        ROS_DEBUG( "[%s] No implementation for exiting autopilot state: HOVER.", ros::this_node::getName().c_str() );
        ROS_INFO ( "[%s] Exited autopilot state: HOVER.", ros::this_node::getName().c_str() );
    }

    void ForgetfulDrones::exitAutopilotStateRACING()
    {
        ROS_DEBUG( "[%s] No implementation for exiting autopilot state: RACING.", ros::this_node::getName().c_str() );
        ROS_INFO ( "[%s] Exited autopilot state: RACING.", ros::this_node::getName().c_str() );
    }

    void ForgetfulDrones::enterAutopilotStateOFF( const std::string& NameOfLastAutopilotState )
    {
        ROS_INFO ( 
            "[%s] Updating autopilot state: %s -> OFF ...", 
            ros::this_node::getName().c_str(),
            NameOfLastAutopilotState.c_str()
            );
        ROS_WARN( "[%s] No implementation for entering autopilot state: OFF.", ros::this_node::getName().c_str() );
        
        m_AutopilotState = AutopilotStates::OFF;
        ROS_INFO ( "[%s] Entered autopilot state: OFF.", ros::this_node::getName().c_str() );
    }

    void ForgetfulDrones::enterAutopilotStateHOVER( const std::string& NameOfLastAutopilotState )
    {
        ROS_INFO ( 
            "[%s] Updating autopilot state: %s -> HOVER ...", 
            ros::this_node::getName().c_str(),
            NameOfLastAutopilotState.c_str()
            );

        p_DataRecording_On = false;
        setReferenceStateInWRFToDronePosYawInWRF();
        m_ReferenceState_ARF = QCTrajectoryPoint_From_KMQuatTransformation( m_T_ARF_DRF );

        // Publish the current state estimation as reference state to the autopilot in order that the drone hovers.
        ROS_Pub_AutopilotReferenceState.publish( m_ReferenceState_ARF.toRosMessage() );
        
        m_AutopilotState = AutopilotStates::HOVER;
        ROS_INFO ( "[%s] Entered autopilot state: HOVER.", ros::this_node::getName().c_str() );
    }

    void ForgetfulDrones::enterAutopilotStateRACING( const std::string& NameOfLastAutopilotState )
    {
        ROS_INFO ( 
            "[%s] Updating autopilot state: %s -> RACING ...", 
            ros::this_node::getName().c_str(),
            NameOfLastAutopilotState.c_str()
            );

        Eigen::Vector3d DronePos_WRF = m_T_WRF_DRF.getPosition();
        displayCubeInRVIZ (
            DronePos_WRF.x(), DronePos_WRF.y(), DronePos_WRF.z(),
            4, VisualizationColors::GREEN );

        setMinAndMaxSpeedAfterScanningGlobalTrajectory();
        setIndexOfGTProjStateToGTStateWithMinDist2( DronePos_WRF );

        
        m_AutopilotState = AutopilotStates::RACING;
        ROS_INFO ( "[%s] Entered autopilot state: RACING.", ros::this_node::getName().c_str() );
    }






















////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////













































    


/*
    

    void ForgetfulDrones::startDataCollection() 
    // use gflags to either test or collect data. gflags instead of rosparam
    {
        /////////////////
        // PARAMETERS //
        /////////////// !!! Add to rosparameters or gflags
        size_t DataCollection_Run_MaxCount = 50;
        ros::WallDuration DataCollection_Run_MaxDuration( 40.0 );


        ///////////////////////////
        // DATA COLLECTION LOOP //
        /////////////////////////

        unsigned int DataCollection_Run_Count = 0;        
        while ( DataCollection_Run_Count < DataCollection_Run_MaxCount )
        {
            
            prepareRandomizedGazeboSimulation();

            /////////////////////////////////////

            //subprocess.call("roslaunch drone_racing simulation_no_quad_gui.launch &", shell=True)
            //time.sleep(10)

            /////////////////////////////////////

            // pub run_i to /hummingbird/run_idx
                m_Run_Count = DataCollection_Run_Count;
                m_Run_DirectoryExists = false;
                ROS_INFO(
                    "[%s] Start Run #%d.", 
                    ros::this_node::getName().c_str(), m_Run_Count
                    );
            // pub empty to /hummingbird/autopilot/off
                ROS_Pub_AutopilotOff.publish( std_msgs::Empty() );
            // drone to initial pos: os.system("rosservice call /gazebo/set_model_state '{model_state: { model_name: hummingbird, pose: { position: { x: -0.5, y: 22.0 ,z: 0.2 }, orientation: {x: 0, y: 0, z: -0.707, w: 0.707 } },
            // twist:{ linear: {x: 0.0 , y: 0 ,z: 0 } , angular: { x: 0.0 , y: 0 , z: 0.0 } } , reference_frame: world } }'")
                gazebo_msgs::ModelState Drone_InitialPos;
                    Drone_InitialPos.model_name = "hummingbird";
                    Drone_InitialPos.pose.position.x = -0.5;
                    Drone_InitialPos.pose.position.y = 22.0;
                    Drone_InitialPos.pose.position.z = 0.2;
                    Drone_InitialPos.pose.orientation.z = -0.707;
                    Drone_InitialPos.pose.orientation.w = 0.707;
                    Drone_InitialPos.reference_frame = "world";


                if ( ROS_SrvClient_GazeboSetModelState.call( Drone_InitialPos ) )
                    ROS_INFO(
                        "[%s] Succesfully set drone to initial pose in world reference frame:"
                        "\tx: [%f] m"
                        "\ty: [%f] m"
                        "\tz: [%f] m"
                        "\theading: [%f] degree.", 
                        Drone_InitialPos.pose.position.x, Drone_InitialPos.pose.position.y, Drone_InitialPos.pose.position.z,
                        quadrotor_common::quaternionToEulerAnglesZYX( 
                            quadrotor_common::geometryToEigen( Drone_InitialPos.pose.orientation ) ).z() * 180/M_PI
                        );  
                else
                {
                    ROS_FATAL(
                        "[%s] Failed to call ROS service [%s] in order to set drone to initial pose."
                        "Shutting down ROS...",
                        ros::this_node::getName().c_str(),
                        ROS_SrvClient_GazeboSetModelState.getService().c_str()
                        );
                    ros::shutdown();
                }
            // start drone: pub true to /hummingbird/bridge/arm and empty to /hummingbird/autopilot/start, sleep 3 s
                std_msgs::Bool BridgeArm; BridgeArm.data = true;
                m_ROSPub_BridgeArm.publish( BridgeArm );

                m_ROSPub_AutopilotStart.publish( std_msgs::Empty() );
                ros::Duration( 3.0 ).sleep();
            // setup environment: pub empty to /hummingbird/setup_environment
                ROS_INFO(
                    "[%s] Resetting environment...",
                    ros::this_node::getName().c_str()
                    );
                
                m_Run_SimEnvIsReady = false;
                p_DataRecording_On = false;

                m_Gates_Curr_i = p_Gates_Start_i;
                m_Gates_Last_i = ( m_Gates_Curr_i + m_Gates_n - 1 ) % m_Gates_n;

                respawnGazeboGateModelsWithRandomAxialShifts();
                ros::Duration(2.0).sleep();
            
                visualizeGazeboGateModelsInRVIZ();

                initializeGlobalTrajectory();
                ros::Duration(1.0).sleep();

                Eigen::Vector3d DronePos_WRF = m_T_WRF_DRF.getPosition();
                displayCubeInRVIZ ( DronePos_WRF.x(), DronePos_WRF.y(), DronePos_WRF.z(),
                    4, VisualizationColors::GREEN );
                
                if ( m_ORCFlightMission == FlightMissions::DATACOLLECTION )
                {
                    setMinAndMaxSpeedAfterScanningGlobalTrajectory();
                    setIndexOfGTProjStateToGTStateWithMinDist2( DronePos_WRF );
                }
                        
                m_Run_SimEnvIsReady = true;

                updateAutopilotStateTo( AutopilotStates::RACING );

                m_Run_StartTime = ros::WallTime::now();
                setReferenceStateInWRFToDronePosYawInWRF();

                // Publish the current state estimation as reference state to the autopilot in order that the drone hovers.
                ROS_Pub_AutopilotReferenceState.publish( 
                    QCTrajectoryPoint_From_KMQuatTransformation( 
                        m_T_ARF_DRF 
                        ).toRosMessage()
                    );

                std_msgs::Bool feedthrough_msg;
                feedthrough_msg.data = true;
                ROS_Pub_CopilotFeedthrough.publish( feedthrough_msg );

                ROS_INFO(
                    "[%s] Environment successfully reset.",
                    ros::this_node::getName().c_str()
                    );
            // setup environment: finished

    

            ros::WallTime DataCollection_Run_StartTime = ros::WallTime::now();
            ros::WallDuration DataCollection_Run_Duration;
            do
            {
                DataCollection_Run_Duration = ros::WallTime::now() - DataCollection_Run_StartTime;
                ROS_INFO(
                    "[%s] Scheduled termination of current run in %d s.",
                    ros::this_node::getName().c_str(),
                    (DataCollection_Run_MaxDuration - DataCollection_Run_Duration).toNSec()
                    );
                ros::Duration(1.0).sleep();
            } while ( DataCollection_Run_Duration < DataCollection_Run_MaxDuration );
            
            
            //pub empty to /hummingbird/hard_stop
                m_Run_CameraFrame_Count = 0;
                updateAutopilotStateTo( AutopilotStates::HOVER );
            //finished: pub empty to /hummingbird/hard_stop

            DataCollection_Run_Count++;
        }
    }


    */

    


}

