#include "forgetful_drones/forgetful_simulator.h"
#include "forgetful_drones/forgetful_helpers.h"






namespace forgetful_drone{


ForgetfulSimulator::ForgetfulSimulator( const ros::NodeHandle& RNH, const ros::NodeHandle& PNH )
    :
    m_ROSRNH( RNH ), 
    m_ROSPNH( PNH ),

    m_UniRealDistri_0To1{0.0, 1.0},
    m_SimulationReady(false),

    
    // --- Paths ---
    m_Models_DirPath( ros::package::getPath("forgetful_drones") + "/gazebo/models" ),
    
    m_Gate_NamePrefix( "Gate#" ),
    m_Gate_TexDirPath ( m_Models_DirPath + "/gate/materials/textures/resources" ),
    m_Gate_TexFileNames( getFileNamesInDir(m_Gate_TexDirPath) ),
    m_Gate_DAEDirPath ( m_Models_DirPath + "/gate/meshes/resources/dae" ),
    m_Gate_DAEFilePath( "file://" + m_Models_DirPath + "/gate/meshes/gate.dae" ),
    m_Gate_DAEFileNames( getFileNamesInDir(m_Gate_DAEDirPath) ),
    m_Gate_STLDirPath( m_Models_DirPath + "/gate/meshes/resources/stl" ),
    m_Gate_STLFileNames( getFileNamesInDir(m_Gate_STLDirPath) ),
    m_Gate_RandIllumScriptFilePath( ros::package::getPath("forgetful_drones") + "/src/adjust_ambient_and_emission_tag_of_XML_file.py" ),
    m_Gate_TmpDirPath( m_Models_DirPath + "/gate/meshes/.tmp" ),
    
    m_Ground_Name( "GroundPlane" ),
    m_Ground_TexDirPath( m_Models_DirPath + "/ground_plane/materials/textures/resources" ),
    m_Ground_TexFileNames( getFileNamesInDir(m_Ground_TexDirPath) ),
    m_Ground_TmpDirPath( m_Models_DirPath + "/ground_plane/materials/scripts/.tmp" ),
    
    m_Wall_NamePrefix( "Wall#" ),
    m_Wall_TexDirPath( m_Models_DirPath + "/wall/materials/textures/resources" ),
    m_Wall_TexFileNames( getFileNamesInDir( m_Wall_TexDirPath ) ),
    m_Wall_TmpDirPath( m_Models_DirPath + "/wall/materials/scripts/.tmp" ),

    m_Drone_Name( "hummingbird" ),
    m_Drone_FrameID( "hummingbird/base_link" ),
    m_DroneSpawned( false ),
    // ---

    // --- ROS ---
    m_ROSSub_DynamicGatesSwitch( m_ROSRNH.subscribe("simulator/dynamic_gates_switch", 1, &ForgetfulSimulator::ROSCB_DynamicGatesSwitch, this) ),
    m_ROSSrvSv_BuildDroneRacingSimulation( m_ROSRNH.advertiseService("simulator/build_drone_racing_simulation", &ForgetfulSimulator::ROSSrvFunc_buildDroneRacingSimulation, this) ),
    m_ROSTimer_SimulatorLoop( m_ROSRNH.createTimer(ros::Duration(m_SimulatorLoopTime), &ForgetfulSimulator::ROSTimerFunc_SimulatorLoop, this, false, false) ),

    m_ROSPub_RvizGates( m_ROSRNH.advertise<visualization_msgs::MarkerArray>("rviz/gates", 1, true) ),
    m_ROSPub_RvizDrone( m_ROSRNH.advertise<visualization_msgs::MarkerArray>("rviz/drone", 1, true) ),

    m_ROSSub_GazeboModelStates( m_ROSRNH.subscribe("/gazebo/model_states", 1, &ForgetfulSimulator::ROS_CB_GazeboModelStates, this ) ),
    m_ROSPub_GazeboSetModelState( m_ROSRNH.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state", 0) ),
    m_ROSSrvCl_GazeboDeleteModel( m_ROSRNH.serviceClient<gazebo_msgs::DeleteModel>("/gazebo/delete_model") ),
    m_ROSSrvCl_GazeboResetSimulation( m_ROSRNH.serviceClient<std_srvs::Empty>("/gazebo/reset_simulation") ),
    m_ROSSrvCl_GazeboSpawnSDFModel( m_ROSRNH.serviceClient<gazebo_msgs::SpawnModel>("/gazebo/spawn_sdf_model") ),
    m_ROSSrvCl_GazeboSpawnURDFModel( m_ROSRNH.serviceClient<gazebo_msgs::SpawnModel>("/gazebo/spawn_urdf_model") ),
    // ---
    
    // --- Const Members ---
    m_GateWaypointHeight(),
    m_GateBaseMinAltitude(),

    m_RandFig8_MaxAxShift(),
    m_RandFig8_MinScale(),
    m_RandFig8_MaxScale(),
    
    m_RandSprint_GateN(),
    m_RandSprint_GateMinN(),
    m_RandSprint_GateMaxN(),    
    m_RandSprint_SectionGateMinN(),
    m_RandSprint_SectionGateMaxN(),
    m_RandSprint_MinSpacing(),
    m_RandSprint_UnidirStdDevSpacing(),
    m_RandSprint_LRMeanYaw(),
    m_RandSprint_LRStdDevYaw(),
    m_RandSprint_MeanPitch(),
    m_RandSprint_StdDevPitch(),
    
    m_DynGates_MaxAxAmp(),
    m_DynGates_MaxAxFreq(),

    m_WallBufferDistance(),
    m_WallHeight(),
    m_WallThickness(),

    m_Drone_InitX(),
    m_Drone_InitY(),
    m_Drone_InitZ(),
    m_Drone_InitPosition(),
    m_Drone_InitYaw(),
    
    m_Drone_RotorN(),
    m_Drone_ArmLength(),
    m_Drone_BodyWidth(),
    m_Drone_BodyHeight(),
    m_Drone_Scale(),
    m_Drone_URDF(),

    m_SimulatorLoopTime()
{
    m_RandEngine.seed( ros::WallTime::now().toNSec() );

    bool InitializationSuccessful = true;

        std::vector< std::pair<const char*, const bool*> > KeysAndBoolOutputs
        {
            //
        };
        std::vector< std::pair<const char*, const int*> > KeysAndIntOutputs
        {
            {"randomized_sprint_number_of_gates" , &m_RandSprint_GateN},
            {"randomized_sprint_min_number_of_gates", &m_RandSprint_GateMinN},
            {"randomized_sprint_max_number_of_gates", &m_RandSprint_GateMaxN},
            {"randomized_sprint_min_number_of_gates_in_section", &m_RandSprint_SectionGateMinN},
            {"randomized_sprint_max_number_of_gates_in_section", &m_RandSprint_SectionGateMaxN},
            {"drone_marker_number_of_rotors", &m_Drone_RotorN},            
        };
        std::vector< std::pair<const char*, const double*> > KeysAndDoubleOutputs
        {
            {"gate_waypoint_height_above_base", &m_GateWaypointHeight},
            {"gate_base_min_altitude", &m_GateBaseMinAltitude},
            {"randomized_figure8_max_axial_shift", &m_RandFig8_MaxAxShift},
            {"randomized_figure8_min_scale", &m_RandFig8_MinScale},
            {"randomized_figure8_max_scale", &m_RandFig8_MaxScale},
            {"randomized_sprint_min_gate_spacing", &m_RandSprint_MinSpacing},
            {"randomized_sprint_unidirectional_standard_deviation_of_gate_spacing", &m_RandSprint_UnidirStdDevSpacing},
            {"randomized_sprint_left_right_mean_yaw_between_gates", &m_RandSprint_LRMeanYaw},
            {"randomized_sprint_left_right_standard_deviation_yaw_between_gates", &m_RandSprint_LRStdDevYaw},
            {"randomized_sprint_mean_pitch_between_gates", &m_RandSprint_MeanPitch},
            {"randomized_sprint_standard_deviation_pitch_between_gates", &m_RandSprint_StdDevPitch},
            {"dynamic_gates_max_axial_amplitude", &m_DynGates_MaxAxAmp},
            {"dynamic_gates_max_axial_frequency", &m_DynGates_MaxAxFreq},
            {"wall_buffer_distance_to_gate", &m_WallBufferDistance},
            {"wall_height", &m_WallHeight},
            {"wall_thickness", &m_WallThickness},
            {"drone_initial_position_x", &m_Drone_InitX},            
            {"drone_initial_position_y", &m_Drone_InitY},
            {"drone_initial_position_z", &m_Drone_InitZ},
            {"drone_initial_position_yaw", &m_Drone_InitYaw},
            {"drone_marker_arm_length", &m_Drone_ArmLength},
            {"drone_marker_body_width", &m_Drone_BodyWidth},
            {"drone_marker_body_height", &m_Drone_BodyHeight},
            {"drone_marker_scale", &m_Drone_Scale},
            {"simulator_loop_nominal_period_time", &m_SimulatorLoopTime}         
        };
        std::vector< std::pair<const char*, const std::string*> > KeysAndStringOutputs
        {
            {"drone_model_description", &m_Drone_URDF}
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

        const_cast< Eigen::Vector3d& >( m_Drone_InitPosition ) = { m_Drone_InitX, m_Drone_InitY, m_Drone_InitZ };

        if ( ! AllPathsExists() ) InitializationSuccessful = false;

        if ( ! AllResourcesValid() ) InitializationSuccessful = false;

        if ( ! generateGateTmpFiles() ) InitializationSuccessful = false;

        generateGroundTmpFiles();

        generateWallTmpFiles();

    if ( ! InitializationSuccessful )
    {
        ROS_FATAL( "[%s]\n  >> Initialization failed. Shutting down ROS node...", 
            ros::this_node::getName().c_str() );
        ros::shutdown();
    }
    else ROS_INFO( "[%s]\n  >> Initialization successful.", ros::this_node::getName().c_str() );




    // --- Marker for gates in RVIZ ---
    m_GateMarker.header.frame_id = "world";
    m_GateMarker.scale.x = m_GateMarker.scale.y = m_GateMarker.scale.z = 1.0;
    m_GateMarker.action = visualization_msgs::Marker::ADD;
    m_GateMarker.type = visualization_msgs::Marker::MESH_RESOURCE;
    m_GateMarker.mesh_use_embedded_materials = true;
    //m_GateMarker.color.a = 1.0;
    //m_GateMarker.color.r = 217.0 / 255.0;
    //m_GateMarker.color.g = 100.0 / 255.0;
    //m_GateMarker.color.b = 30.0 / 255.0;
    //m_GateMarker.frame_locked = true;



}



ForgetfulSimulator::~ForgetfulSimulator()
{
    deleteDirectoryContents( m_Gate_TmpDirPath );
    deleteDirectoryContents( m_Ground_TmpDirPath );
    deleteDirectoryContents( m_Wall_TmpDirPath );
}




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





///////////////////////////////////////////////////////////////////////
/// \brief Check if all paths initialized in constructor are valid. //
/// \return True or false.                                         //
////////////////////////////////////////////////////////////////////
bool ForgetfulSimulator::AllPathsExists()
{
    bool ReturnBool = true;


    const std::array< const std::string* const, 9> AllPaths{
        &m_Gate_RandIllumScriptFilePath,
        &m_Gate_TexDirPath,
        &m_Gate_DAEDirPath,
        &m_Gate_STLDirPath,
        &m_Ground_TexDirPath,
        &m_Wall_TexDirPath,
        &m_Gate_TmpDirPath,
        &m_Ground_TmpDirPath,
        &m_Wall_TmpDirPath
        };


    for ( const std::string* const FilePath : AllPaths )
        if ( ! std::filesystem::exists(*FilePath) )
        {
            ROS_ERROR( "[%s]\n  >> Path \"%s\" does not exist.", 
                ros::this_node::getName().c_str(), FilePath->c_str() );

            ReturnBool = false;
        }

    
    return ReturnBool;
}

///////////////////////////////////////////////////////////////////
/// \brief Check if resources for randomizing models are valid. //
/// \return True or false.                                     //
////////////////////////////////////////////////////////////////
bool ForgetfulSimulator::AllResourcesValid()
{
    bool ReturnBool = true;


    const std::array< const std::vector<std::string>* const, 5 > FileNames{
        &m_Gate_TexFileNames, 
        &m_Gate_DAEFileNames, 
        &m_Gate_STLFileNames, 
        &m_Ground_TexFileNames, 
        &m_Wall_TexFileNames
        };
    const std::array< const std::string* const, 5>& DirPaths{
        &m_Gate_TexDirPath,
        &m_Gate_DAEDirPath,
        &m_Gate_STLDirPath,
        &m_Ground_TexDirPath,
        &m_Wall_TexDirPath
        };
    const std::array< const std::string, 5 > FileExts{
        ".jpg", 
        ".dae", 
        ".stl", 
        ".jpg", 
        ".jpg"
        };


    // Check if not empty, i.e., there are files in corresponding directories.
    for ( int i = 0; i < FileNames.size(); i++ )
        if ( (*FileNames[ i ]).size() == 0 ) 
        {
            ROS_FATAL( "[%s] Resources loaded from <%s> are empty.",
                ros::this_node::getName().c_str(), DirPaths[ i ]->c_str() );
            
            ReturnBool = false;
        }


    // Correct File extension
    for ( int i = 0; i < FileNames.size(); i++ )
        for ( int j = 0; j < (*FileNames[ i ]).size(); j++ )
        {
            const std::string& Resource = (*FileNames[ i ])[ j ];
            size_t LastDot = Resource.find_last_of( '.' );
            if ( LastDot == std::string::npos || Resource.substr( LastDot ) != FileExts[ i ] )
            {
                ROS_FATAL( "[%s] File <%s> is not of type <%s>.",
                    ros::this_node::getName().c_str(),
                    (*DirPaths[ i ] + Resource).c_str(), FileExts[ i ].c_str() );
                
                ReturnBool = false;
            }
        }


    // Check if one .dae and .stl file for one gate model ...
    if ( m_Gate_DAEFileNames.size() != m_Gate_STLFileNames.size() )
    {
        ROS_FATAL( "[%s] Directories <%s> and <%s> do not have the same number of files.",
            ros::this_node::getName().c_str(),
            m_Gate_DAEDirPath.c_str(), m_Gate_STLDirPath.c_str() );
        
        ReturnBool = false;
    }
    // ... with the same name
    for ( int i = 0; i < m_Gate_DAEFileNames.size(); i++ )
    {
        const std::string& DAE_Filename = m_Gate_DAEFileNames[ i ];
        const std::string& STL_Filename = m_Gate_STLFileNames[ i ];

        int DAE_LastDot = DAE_Filename.find_last_of(".");
        int STL_LastDot = STL_Filename.find_last_of(".");

        if ( DAE_Filename.substr( 0, DAE_LastDot ) != STL_Filename.substr( 0, STL_LastDot) )
        {
            ROS_FATAL( "[%s] Files <%s> and <%s>, "
                "which are at the same alphabetical position <%d> in their directory, do not have the same stem.",
                ros::this_node::getName().c_str(), m_Gate_DAEFileNames[ i ].c_str(), 
                m_Gate_STLFileNames[ i ].c_str(), i );
            
            ReturnBool = false;
        }
    }

    if (ReturnBool)
    {
        ROS_INFO( "[%s] All resources successfully loaded.", 
            ros::this_node::getName().c_str() );
        /*for ( int i = 0; i < FileNames.size(); i++ )
        {
            std::cout << "\tFrom \"" <<  *DirPaths[ i ] << "\":\n\t\t";
            
            for ( int j = 0; j < (*FileNames[ i ]).size() -1; j++ )
            {
                std::cout << "\"" <<  (*FileNames[ i ])[ j ] << "\", ";
            }
            std::cout << "\"" <<  (*FileNames[ i ])[ (*FileNames[ i ]).size() ] << "\".";

            std::cout << std::endl;
        }*/
    }

    return ReturnBool;
}





bool ForgetfulSimulator::generateGateTmpFiles()
{
    bool ReturnBool = true;


    deleteDirectoryContents( m_Gate_TmpDirPath );

    
    for ( size_t Mesh_Idx = 0; Mesh_Idx < m_Gate_DAEFileNames.size(); Mesh_Idx++ )
        for ( size_t Tex_Idx = 0; Tex_Idx < m_Gate_TexFileNames.size(); Tex_Idx++ )
        {
            size_t Model_Idx = m_Gate_TexFileNames.size() * Mesh_Idx + Tex_Idx;
            std::string DirPath_Idx = m_Gate_TmpDirPath + "/" + std::to_string( Model_Idx );
            if ( ! std::filesystem::create_directory( DirPath_Idx ) )
            {
                ROS_FATAL( "[%s] Failed to create directory \"%s\"", 
                    ros::this_node::getName().c_str(), DirPath_Idx.c_str() );
                ReturnBool = false; 
                break;
            }

            std::string GateDAE_Src = m_Gate_DAEDirPath + "/" + m_Gate_DAEFileNames[ Mesh_Idx ];
            std::string GateDAE_Dst = DirPath_Idx + "/gate.dae";
            std::filesystem::copy( GateDAE_Src, GateDAE_Dst );

            std::string GateSTL_Src = m_Gate_STLDirPath + "/" + m_Gate_STLFileNames[ Mesh_Idx ];
            std::string GateSTL_Dst = DirPath_Idx + "/gate.stl";
            std::filesystem::copy( GateSTL_Src, GateSTL_Dst );

            std::string GateTex_Src = m_Gate_TexDirPath + "/" + m_Gate_TexFileNames[ Tex_Idx ];
            std::string GateTex_Dst = DirPath_Idx + "/gate.jpg";
            std::filesystem::copy( GateTex_Src, GateTex_Dst );

            // Random illumination of gates
            double GateEmission = 0.1 * m_UniRealDistri_0To1( m_RandEngine );
            double GateAmbient = m_UniRealDistri_0To1( m_RandEngine );
            auto x = system(
                ( "python3 "    + m_Gate_RandIllumScriptFilePath
                + " -xml_file " + GateDAE_Dst
                + " -emission " + std::to_string( GateEmission )
                + " -ambient "  + std::to_string( GateAmbient )         ).c_str() );
        }


    return ReturnBool;
}

void ForgetfulSimulator::generateGroundTmpFiles()
{
    deleteDirectoryContents( m_Ground_TmpDirPath );


    std::string Dst;
    for ( int TexRes_i = 0; TexRes_i < m_Ground_TexFileNames.size(); TexRes_i++ )
    {    
        Dst = m_Ground_TmpDirPath + "/ground_plane_" + std::to_string( TexRes_i ) + ".material";
        writeStringToFile( getGroundMatScript( TexRes_i ), Dst );
    }
}

std::string ForgetfulSimulator::getGroundMatScript( const int& TexRes_i )
{
    return 
    R"(material ground_plane_)" + std::to_string( TexRes_i ) + R"(
    {
        technique
        {
            pass
            {
                ambient 0.5 0.5 0.5 1.0
                diffuse 0.2 0.2 0.2 1.0
                specular 0.0 0.0 0.0 1.0 12.5

                texture_unit
                {
                    texture ../../textures/resources/)" + m_Ground_TexFileNames[ TexRes_i ] + R"(
                    filtering anistropic
                    max_anisotropy 16
                    scale 0.1 0.1
                }
            }
        }
    })";
}

void ForgetfulSimulator::generateWallTmpFiles()
{
    deleteDirectoryContents( m_Wall_TmpDirPath );

    std::string Dst;
    for ( int TexRes_i = 0; TexRes_i < m_Wall_TexFileNames.size(); TexRes_i++ )
    {    
        Dst = m_Wall_TmpDirPath + "/wall_" + std::to_string( TexRes_i ) + ".material";
        writeStringToFile( getWallMatScript( TexRes_i ), Dst );
    }
}

std::string ForgetfulSimulator::getWallMatScript( const int& TexRes_i )
{
    return 
    R"(material wall_)" + std::to_string( TexRes_i ) + R"(
    {
        technique
        {
            pass
            {
                ambient 0.5 0.5 0.5 1.0
                diffuse 0.5 0.5 0.5 1.0
                emissive 0.0 0.0 0.0 1

                texture_unit
                {
                    texture ../../textures/resources/)" + m_Wall_TexFileNames[ TexRes_i ] + R"(
                    filtering anistropic
                    max_anisotropy 16
                    scale 1.0 1.0
                }
            }
        }
    })";
}





void ForgetfulSimulator::ROS_CB_GazeboModelStates( const gazebo_msgs::ModelStates::ConstPtr &msg ) 
{ 
    m_GazeboModelStates = *msg;
}


void ForgetfulSimulator::ROSCB_DynamicGatesSwitch( const std_msgs::Bool::ConstPtr& msg )
{
    ROS_INFO( 
        "[%s]\n  >> Dynamic gates %s.", 
        ros::this_node::getName().c_str(),
        msg->data? "enabled" : "disabled"
        );

    msg->data? m_ROSTimer_SimulatorLoop.start() : m_ROSTimer_SimulatorLoop.stop();
}





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
        case req.FIG8_DET: computeGatePoses_Fig8Det(); break;
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
    
    setGateModelNames();

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


    /*
    // Reset Simulation //
    /////////////////////
    std_srvs::Empty srv;
    if ( m_ROSSrvCl_GazeboResetSimulation.call(srv) )
    {
        ROS_INFO( "[%s] Gazebo Simulation successfully reset.", 
            ros::this_node::getName().c_str() );
    }
    else return false;
    */


    // --- Set service response ---
        res.Drone_InitPose.position = GeometryMsgsPoint_From_EigenVector3d( m_Drone_InitPosition );
        res.Drone_InitPose.orientation = tf::createQuaternionMsgFromYaw( m_Drone_InitYaw );

        res.Gates_WaypointPose.clear();
        geometry_msgs::Pose WaypointPose;
        for ( size_t GateIdx = 0; GateIdx < m_Gates_BasePoint.size(); GateIdx++ )
        {
            WaypointPose.position 
                = GeometryMsgsPoint_From_EigenVector3d( m_Gates_BasePoint[GateIdx] );
            WaypointPose.position.z += m_GateWaypointHeight;
            WaypointPose.orientation = tf::createQuaternionMsgFromYaw( m_Gates_Yaw[GateIdx] );

            res.Gates_WaypointPose.push_back( WaypointPose );
        }
        


    m_SimulationReady = true;
    return true;
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

    for ( size_t GateIdx = 0; GateIdx < m_Gates_BasePoint.size(); GateIdx++ )
    {
        ModelState.model_name = m_Gates_ModelName[ GateIdx ];

        AxialPhases
            = m_DynGates_InitAxPhases[ GateIdx ]
            + 2*M_PI*m_DynGates_AxFreqs[ GateIdx ] * SimTime;

        ModelState.pose.position = GeometryMsgsPoint_From_EigenVector3d(
            m_Gates_BasePoint[ GateIdx ] + m_DynGates_AxAmps[ GateIdx ].cwiseProduct(
                Eigen::Vector3d{ 
                    std::sin(AxialPhases.x()), 
                    std::sin(AxialPhases.y()), 
                    std::sin(AxialPhases.z()) 
                    }
                )
            );
        
        ModelState.pose.orientation 
            = tf::createQuaternionMsgFromYaw( m_Gates_Yaw[ GateIdx ] );

        
        //m_GateMarker.id = GateIdx;
        //m_GateMarker.pose.position = ModelState.pose.position;
        //m_GateMarker.pose.orientation = ModelState.pose.orientation;
        //m_GateMarkerArray.markers.push_back( m_GateMarker );

        m_GateMarkerArray.markers[ GateIdx ].pose = ModelState.pose;


        m_ROSPub_GazeboSetModelState.publish( ModelState );
    }

    m_ROSPub_RvizGates.publish( m_GateMarkerArray );
    
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
        if ( ModelName == m_Drone_Name ) continue;

        srv.request.model_name = ModelName;

        if ( m_ROSSrvCl_GazeboDeleteModel.call( srv ) )
            ROS_INFO( "[%s]\n  >> Model \"%s\" successfully deleted from Gazebo", 
                ros::this_node::getName().c_str(), ModelName.c_str() );
        else
            ROS_ERROR( "[%s]\n  >> Failed to delete model \"%s\" from Gazebo.",
                ros::this_node::getName().c_str(), ModelName.c_str() );
    }
}






void ForgetfulSimulator::computeGatePoses_Fig8Det()
{
    size_t GateN = 14;
    m_Gates_BasePoint.resize( GateN );
    m_Gates_Yaw.resize( GateN );

    m_Gates_BasePoint = {
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

    m_Gates_Yaw = {
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


        geometry_msgs::Pose GateBasePos_RaceTrackRF;
        kindr::minimal::QuatTransformation T_RaceTrackRF_GateRF;
        kindr::minimal::QuatTransformation T_WRF_GateRF;    
    for ( size_t GateIdx = 0; GateIdx < GateN; GateIdx++ )
    {
        GateBasePos_RaceTrackRF.position 
            = GeometryMsgsPoint_From_EigenVector3d( m_Gates_BasePoint[ GateIdx ] );
        GateBasePos_RaceTrackRF.orientation 
            = tf::createQuaternionMsgFromYaw( m_Gates_Yaw[ GateIdx ] );
        tf::poseMsgToKindr (
            GateBasePos_RaceTrackRF, 
            &T_RaceTrackRF_GateRF
            );
        T_WRF_GateRF = T_WRF_RaceTrackRF * T_RaceTrackRF_GateRF;

        m_Gates_BasePoint[ GateIdx ] = T_WRF_GateRF.getPosition();
        m_Gates_Yaw[ GateIdx ] 
            = T_WRF_GateRF.getEigenQuaternion().toRotationMatrix().eulerAngles(0, 1, 2).z();
    }
}


void ForgetfulSimulator::computeGatePoses_Fig8Rand()
{
    computeGatePoses_Fig8Det();


    std::uniform_real_distribution<double> URDistri_M1ToP1( -1.0, 1.0 );
    std::uniform_real_distribution<double> URDistri_Scale( m_RandFig8_MinScale, m_RandFig8_MaxScale );
    double Scale = URDistri_Scale( m_RandEngine );

    for ( size_t GateIdx = 0; GateIdx < m_Gates_BasePoint.size(); GateIdx++ )
    {
        m_Gates_BasePoint[ GateIdx ].x() 
            += m_RandFig8_MaxAxShift * URDistri_M1ToP1( m_RandEngine );
        
        m_Gates_BasePoint[ GateIdx ].y() 
            += m_RandFig8_MaxAxShift * URDistri_M1ToP1( m_RandEngine );

        m_Gates_BasePoint[ GateIdx ].z() 
            += m_RandFig8_MaxAxShift * URDistri_M1ToP1( m_RandEngine );

        m_Gates_BasePoint[ GateIdx ].z() = std::max( m_GateBaseMinAltitude, m_Gates_BasePoint[ GateIdx ].z() );

        m_Gates_BasePoint[ GateIdx ].x() *= Scale;
        m_Gates_BasePoint[ GateIdx ].y() *= Scale;
    }

    const_cast< Eigen::Vector3d& >( m_Drone_InitPosition ) *= Scale;
}


///////////////////////////////////////////////////////////////////////////////////////////////////////
/// \brief [ Set member variables: ´m_Gates_BasePoint´ and ´m_Gates_Yaw´ ]
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
    m_Gates_BasePoint.resize( Gates_N );
    m_Gates_Yaw.resize( Gates_N );
    
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

        m_Gates_BasePoint[ Gate_i ]
            = PrevGate_Pos 
            + Eigen::Vector3d{
                getRand_Spacing() * cos( Yaw ) * cos( Pitch ),
                getRand_Spacing() * sin( Yaw ) * cos( Pitch ),
                getRand_Spacing()              * sin( Pitch )
                };

        m_Gates_BasePoint[ Gate_i ].z() = std::max( m_GateBaseMinAltitude, m_Gates_BasePoint[ Gate_i ].z() );

        PrevGate_Pos = m_Gates_BasePoint[ Gate_i ];
    }
    // Yaw
    PrevGate_Pos = m_Drone_InitPosition + Eigen::Vector3d{ 0.0, 0.0, m_GateBaseMinAltitude };
    for ( size_t Gates_i = 0; Gates_i < Gates_N - 1; Gates_i++ )
    {
        Eigen::Vector3d Prev2Next = m_Gates_BasePoint[ Gates_i + 1 ] - PrevGate_Pos;
        
        m_Gates_Yaw[ Gates_i ] = atan2( Prev2Next.y(), Prev2Next.x() );

        PrevGate_Pos = m_Gates_BasePoint[ Gates_i ];
    }
    Eigen::Vector3d SecondLast2Last = m_Gates_BasePoint[ Gates_N - 1 ] - m_Gates_BasePoint[ Gates_N - 2 ];
    m_Gates_Yaw[ Gates_N - 1 ] = atan2( SecondLast2Last.y(), SecondLast2Last.x() );


    ROS_INFO( "[%s] Position and yaw of gates for randomized sprint race successfully computed:\n"
        "\tNumber of Gates:           <%d>\n"
        "\tPosition of first Gate:    <%f, %f, %f>\n"
        "\tPosition of last Gate:     <%f, %f, %f>\n",
        ros::this_node::getName().c_str(), Gates_N, 
        m_Gates_BasePoint[ 0 ].x(), m_Gates_BasePoint[ 0 ].y(), m_Gates_BasePoint[ 0 ].z(),
        m_Gates_BasePoint[ m_Gates_BasePoint.size() - 1 ].x(), m_Gates_BasePoint[ m_Gates_BasePoint.size() - 1 ].y(), m_Gates_BasePoint[ m_Gates_BasePoint.size() - 1 ].z()
        );
}




void ForgetfulSimulator::setGateModelNames()
{
    m_Gates_ModelName.resize( m_Gates_BasePoint.size() );
    std::stringstream GateIdx_String;
    int NameWidth = static_cast<int>( log10(m_Gates_BasePoint.size()) +1 );

    for ( size_t GateIdx = 0; GateIdx < m_Gates_BasePoint.size(); GateIdx++ )
    {
        GateIdx_String.str( "" );
        GateIdx_String 
            << std::setw( NameWidth ) 
            << std::setfill('0') 
            << GateIdx;

        m_Gates_ModelName[ GateIdx ] = m_Gate_NamePrefix + GateIdx_String.str();
    }
}





void ForgetfulSimulator::initRandParamsOfDynGates()
{
    m_DynGates_AxAmps.resize( m_Gates_BasePoint.size() );
    m_DynGates_AxFreqs.resize( m_Gates_BasePoint.size() );
    m_DynGates_InitAxPhases.resize( m_Gates_BasePoint.size() );

    for ( size_t GateIdx = 0; GateIdx < m_Gates_BasePoint.size(); GateIdx++ )
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




void ForgetfulSimulator::computePositionAndSizeOfGroundAndWalls()
{

    double MinX = m_Drone_InitPosition.x(), MaxX = MinX;
    double MinY = m_Drone_InitPosition.y(), MaxY = MinY;

    for ( size_t GateIdx = 0; GateIdx < m_Gates_BasePoint.size(); GateIdx++ )
    {
        MinX = std::min( MinX, m_Gates_BasePoint[ GateIdx ].x() );
        MaxX = std::max( MaxX, m_Gates_BasePoint[ GateIdx ].x() );
        MinY = std::min( MinY, m_Gates_BasePoint[ GateIdx ].y() );
        MaxY = std::max( MaxY, m_Gates_BasePoint[ GateIdx ].y() );
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



/// \brief Spawning gates (with pose from ´m_Gates_BasePoint´ and ´m_Gates_Yaw´) in Gazebo.
/// Each gates is randomized, i.e., the texture and mesh is randomly chosen from resources
/// and the illumination is randomly set.
/// \return True if all gates successfully randomized and spawned, false otherwise.
void ForgetfulSimulator::spawnRandGatesInGazeboAndRVIZ()
{
    gazebo_msgs::SpawnModel srv;
        srv.request.reference_frame = "world";
        srv.request.robot_namespace = "RaceTrack";
    
    m_GateMarkerArray.markers.clear();
    m_GateMarkerArray.markers.reserve( m_Gates_BasePoint.size() );
    size_t TmpGateIdx;
    for ( size_t GateIdx = 0; GateIdx < m_Gates_BasePoint.size(); GateIdx++ )
    {   
        // --- Gazebo ---
        srv.request.model_name = m_Gates_ModelName[ GateIdx ];
        srv.request.model_xml = getRandGateSDF( TmpGateIdx );
        srv.request.initial_pose.position 
            = GeometryMsgsPoint_From_EigenVector3d( m_Gates_BasePoint[GateIdx] );
        srv.request.initial_pose.orientation
            = tf::createQuaternionMsgFromYaw( m_Gates_Yaw[GateIdx] );
        
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

std::string ForgetfulSimulator::getRandGateSDF( size_t& OUT_TmpGateIdx )
{
    std::uniform_int_distribution<size_t> GateTex_UIDistri{0, m_Gate_TexFileNames.size() -1};
    std::uniform_int_distribution<size_t> GateMesh_UIDistri{0, m_Gate_DAEFileNames.size() -1};

    size_t Tex_Idx = GateTex_UIDistri( m_RandEngine );
    size_t Mesh_Idx = GateMesh_UIDistri( m_RandEngine );
    OUT_TmpGateIdx = m_Gate_TexFileNames.size() * Mesh_Idx + Tex_Idx;

    return getGateSDF( OUT_TmpGateIdx );
}

std::string ForgetfulSimulator::getGateSDF( const size_t& Gate_i )
{
    return 
    R"(<?xml version="1.0"?>
    <sdf version="1.5">
        <model name="Gate_)" + std::to_string( Gate_i ) + R"(">
            <pose>0 0 0 0 0 0</pose>
            <static>true</static>
            <link name="link_)" + std::to_string( Gate_i ) + R"(">
                <collision name="collision">
                    <geometry>
                        <mesh>
                            <uri>model://gate/meshes/.tmp/)" + std::to_string( Gate_i ) + R"(/gate.stl</uri>
                        </mesh>
                    </geometry>
                    <max_contacts>10</max_contacts>
                    <surface>
                        <contact>
                            <ode/>
                        </contact>
                        <bounce/>
                        <friction>
                            <torsional>
                                <ode/>
                            </torsional>
                            <ode/>
                        </friction>
                    </surface>
                </collision>
                <visual name="visual">
                    <geometry>
                        <mesh>
                            <uri>model://gate/meshes/.tmp/)" + std::to_string( Gate_i ) + R"(/gate.dae</uri>
                        </mesh>
                    </geometry>
                </visual>
                <self_collide>0</self_collide>
                <enable_wind>0</enable_wind>
                <kinematic>0</kinematic>
            </link>
        </model>
    </sdf>)";
}



/// \brief Spawning the ground plane (with pose from !!!!´???´ and ´???´ !!!!) in Gazebo.
/// It is randomized, i.e., the texture is randomly chosen from resources.
/// \return True if ground plane successfully spawned, false otherwise.
void ForgetfulSimulator::spawnRandGroundInGazebo()
{   
    gazebo_msgs::SpawnModel srv;
        
        srv.request.reference_frame = "world";
        srv.request.robot_namespace = "Environment";
        srv.request.model_name = m_Ground_Name;
        srv.request.model_xml = getRandGroundSDF();
        srv.request.initial_pose.position.x = m_Ground_BasePoint.x();
        srv.request.initial_pose.position.y = m_Ground_BasePoint.y();

        
    if ( m_ROSSrvCl_GazeboSpawnSDFModel.call( srv ) )
        ROS_INFO( "[%s]\n  >> Model \"%s\" successfully spawned in Gazebo.", 
                ros::this_node::getName().c_str(), srv.request.model_name.c_str() );
    else
        ROS_ERROR( "[%s]\n  >> Failed to spawn model \"%s\" in Gazebo.",
            ros::this_node::getName().c_str(), srv.request.model_name.c_str() );
}

std::string ForgetfulSimulator::getRandGroundSDF()
{
    std::uniform_int_distribution<size_t> GroundTex_UIDistri{0, m_Ground_TexFileNames.size() -1};

    return getGroundSDF( GroundTex_UIDistri(m_RandEngine) );
}

std::string ForgetfulSimulator::getGroundSDF( const size_t& TexRes_i )
{
    return 
    R"(<?xml version="1.0"?>
    <sdf version="1.6">
        <model name='Ground_Plane_)" + std::to_string( TexRes_i ) + R"('>
            <pose frame='world'>0 0 0 0 0 0</pose>
            <static>1</static>
            <link name='link'>
                <collision name='collision'>
                    <geometry>
                        <plane>
                            <normal>0 0 1</normal>
                            <size>)" 
                                + std::to_string( m_Ground_Size.x() ) 
                                + " " 
                                + std::to_string( m_Ground_Size.y() ) 
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
                                + std::to_string( m_Ground_Size.x() ) 
                                + " " 
                                + std::to_string( m_Ground_Size.y() ) 
                                + R"(</size>
                        </plane>
                    </geometry>
                    <material>
                        <script>
                            <uri>model://ground_plane/materials/scripts/.tmp/ground_plane_)" 
                            + std::to_string( TexRes_i ) + R"(.material</uri>
                            <name>ground_plane_)" + std::to_string( TexRes_i ) + R"(</name>
                        </script>
                    </material>
                </visual>
                <self_collide>0</self_collide>
                <enable_wind>0</enable_wind>
                <kinematic>0</kinematic>
            </link>
        </model>
    </sdf>)";
}



/// \brief Spawning walls (with pose from !!!!´???´ and ´???´ !!!!) in Gazebo.
/// Each wall is randomized, i.e., the texture is randomly chosen from resources.
/// \return True if all walls successfully randomized and spawned, false otherwise.
void ForgetfulSimulator::spawnRandWallsInGazebo()
{
    gazebo_msgs::SpawnModel srv;
        srv.request.reference_frame = "world";
        srv.request.robot_namespace = "Environment";
    
    std::stringstream WallIdx_String;
    size_t WallN = 4; 
    for ( size_t WallIdx = 0; WallIdx < WallN; WallIdx++ )
    {
        // Set members of the srv
        WallIdx_String.str("");
        WallIdx_String 
            << std::setw( static_cast<int>( log10(WallN) + 1 ) ) 
            << std::setfill('0') 
            << WallIdx;
        
        srv.request.model_name = m_Wall_NamePrefix + WallIdx_String.str();
        srv.request.model_xml = getRandWallSDF( WallIdx );
        srv.request.initial_pose.position 
            = GeometryMsgsPoint_From_EigenVector3d( m_Walls_BasePoint[ WallIdx ] );
        

        if ( m_ROSSrvCl_GazeboSpawnSDFModel.call( srv ) )
            ROS_INFO( "[%s]\n  >> Model \"%s\" successfully spawned in Gazebo.", 
                ros::this_node::getName().c_str(), srv.request.model_name.c_str() );
        else
            ROS_ERROR( "[%s]\n  >> Failed to spawn model \"%s\" in Gazebo.",
                ros::this_node::getName().c_str(), srv.request.model_name.c_str() );
    }
}

std::string ForgetfulSimulator::getRandWallSDF( const size_t& Wall_i )
{
    std::uniform_int_distribution<size_t> WallTex_UIDistri{0, m_Wall_TexFileNames.size() -1};

    return getWallSDF( Wall_i, WallTex_UIDistri(m_RandEngine) );
}

std::string ForgetfulSimulator::getWallSDF( const size_t& Wall_i, const size_t& TexRes_i )
{
    return 
    R"(<?xml version="1.0"?>
    <sdf version="1.6">
        <model name='Wall_)" + std::to_string( TexRes_i ) + R"('>
            <pose frame='world'>0 0 0 0 0 0</pose>
            <static>1</static>
            <link name='link'>
                <inertial>
                    <mass>1</mass>
                </inertial>
                <collision name='collision'>
                    <geometry>
                        <box>
                            <size>)" 
                                + std::to_string( m_Walls_Size[ Wall_i ].x() ) + R"( )" 
                                + std::to_string( m_Walls_Size[ Wall_i ].y() ) + R"( )" 
                                + std::to_string( m_Walls_Size[ Wall_i ].z() ) + R"(</size>
                        </box>
                    </geometry>
                    <max_contacts>10</max_contacts>
                    <surface>
                        <contact>
                            <ode/>
                        </contact>
                        <bounce/>
                        <friction>
                            <torsional>
                                <ode/>
                            </torsional>
                            <ode/>
                        </friction>
                    </surface>
                </collision>
                <visual name='visual'>
                    <geometry>
                        <box>
                            <size>)" 
                                + std::to_string( m_Walls_Size[ Wall_i ].x() ) + R"( )" 
                                + std::to_string( m_Walls_Size[ Wall_i ].y() ) + R"( )" 
                                + std::to_string( m_Walls_Size[ Wall_i ].z() ) + R"(</size>
                        </box>
                    </geometry>
                    <material>
                        <script>
                        <uri>model://wall/materials/scripts/.tmp/wall_)" 
                            + std::to_string( TexRes_i ) + R"(.material</uri>
                        <name>wall_)" + std::to_string( TexRes_i ) + R"(</name>
                        </script>
                    </material>
                </visual>
                <self_collide>false</self_collide>
                <enable_wind>false</enable_wind>
                <kinematic>false</kinematic>
            </link>
        </model>
    </sdf>)";
}



void ForgetfulSimulator::spawnDroneInGazebo()
{
    gazebo_msgs::SpawnModel srv;
        srv.request.model_name = m_Drone_Name;
        srv.request.reference_frame = "world";
        srv.request.initial_pose.position 
            = GeometryMsgsPoint_From_EigenVector3d( m_Drone_InitPosition );
        srv.request.initial_pose.orientation 
            = tf::createQuaternionMsgFromYaw( m_Drone_InitYaw );
        srv.request.model_xml = m_Drone_URDF;

    m_ROSSrvCl_GazeboSpawnURDFModel.call( srv );
}



void ForgetfulSimulator::resetDroneModelState()
{
    gazebo_msgs::ModelState ModelState;
    ModelState.reference_frame = "world";
    ModelState.model_name = m_Drone_Name;
    ModelState.pose.position = GeometryMsgsPoint_From_EigenVector3d (m_Drone_InitPosition);
    ModelState.pose.orientation = tf::createQuaternionMsgFromYaw( m_Drone_InitYaw );
    m_ROSPub_GazeboSetModelState.publish( ModelState );
}



}



