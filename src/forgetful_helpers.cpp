#include "forgetful_drones/forgetful_helpers.hpp"




namespace forgetful_drone
{


geometry_msgs::Point Pose::position_as_geometry_msg() const
{
    geometry_msgs::Point Point;
    Point.x = position.x();
    Point.y = position.y();
    Point.z = position.z();
    return Point;
}
geometry_msgs::Quaternion Pose::orientation_as_geometry_msg() const
{
    geometry_msgs::Quaternion Quaternion;
    Quaternion.w = orientation.w();
    Quaternion.x = orientation.x();
    Quaternion.y = orientation.y();
    Quaternion.z = orientation.z();
    return Quaternion;
}
geometry_msgs::Pose Pose::as_geometry_msg() const
{
    geometry_msgs::Pose Pose;
    Pose.position = position_as_geometry_msg();
    Pose.orientation = orientation_as_geometry_msg();
    return Pose;
}
double Pose::yaw() const
{
    return orientation.toRotationMatrix().eulerAngles(0, 1, 2).z();
}




Eigen::Vector3d 
EigenVector3d_From_Vec3
( const Vec3& IN )
{
    return { IN.x, IN.y, IN.z };
}


Vec3 
Vec3_From_EigenVector3d
( const Eigen::Vector3d& IN )
{
    return { IN.x(), IN.y(), IN.z() };
}


geometry_msgs::Pose 
GeometryMsgsPose_From_NavMsgsOdometry
( const nav_msgs::Odometry& IN )
{
    geometry_msgs::Pose OUT;
    OUT.position = IN.pose.pose.position;
    OUT.orientation = IN.pose.pose.orientation;
    return OUT;
}


quadrotor_common::TrajectoryPoint 
QCTrajectoryPoint_From_KMQuatTransformation
( const kindr::minimal::QuatTransformation& IN )
{
    quadrotor_common::TrajectoryPoint OUT;
    OUT.position = IN.getPosition();
    OUT.heading = quadrotor_common::quaternionToEulerAnglesZYX( IN.getEigenQuaternion() ).z();
    return OUT;
}


geometry_msgs::Point
GeometryMsgsPoint_From_EigenVector3d
( const Eigen::Vector3d& IN )
{
    geometry_msgs::Point OUT;
    OUT.x = IN.x();
    OUT.y = IN.y();
    OUT.z = IN.z();
    return OUT;
}


Eigen::Vector3d
EigenVector3d_From_GeometryMsgsPoint
( const geometry_msgs::Point& IN )
{
    return {IN.x, IN.y, IN.z };
}




Eigen::Quaterniond
EigenQuaterniond_From_GeometryMsgsQuaternion
( const geometry_msgs::Quaternion& IN )
{
    return { IN.w, IN.x, IN.y, IN.z };
}

geometry_msgs::Quaternion
GeometryMsgsQuaternion_From_EigenQuaterniond
( const Eigen::Quaterniond& IN )
{
    geometry_msgs::Quaternion OUT;
    OUT.w = IN.w(); OUT.x = IN.x(); OUT.y = IN.y(); OUT.z = IN.z();
    return OUT;
}








std::string 
getCurrUTCDateTimeAsString()
{
    std::chrono::system_clock::time_point now = std::chrono::system_clock::now();
    time_t tt = std::chrono::system_clock::to_time_t(now);
    tm gm_tm = *std::gmtime(&tt);
    std::string DateTimeString 
        = "UTC-"
        + std::to_string( gm_tm.tm_year + 1900 )    + "-"
        + std::to_string( gm_tm.tm_mon + 1 )        + "-"
        + std::to_string( gm_tm.tm_mday )           + "-"
        + std::to_string( gm_tm.tm_hour )           + "-"
        + std::to_string( gm_tm.tm_min )            + "-"
        + std::to_string( gm_tm.tm_sec );
    
    return DateTimeString;
}








double 
Saturation
( const double& InputVal, const double& LowerLimit, const double& UpperLimit )
{
    return std::min( 
        UpperLimit, std::max(LowerLimit, InputVal) 
        );
}






std::vector< std::string > 
getFileNamesInDir
( const std::string& IN_DirPath )
{
    std::vector<std::string> OUT_Filenames;

    struct dirent* entry;
    DIR* dir = opendir( IN_DirPath.c_str() );
    
    if ( dir == nullptr ) 
    {
        ROS_FATAL( "[%s] Failed to open directory [%s]. Shutting down ROS...",
            ros::this_node::getName().c_str(), IN_DirPath.c_str() );
        ros::shutdown();
    }

    while ( (entry = readdir( dir )) != nullptr ) 
        if ( strcmp(entry->d_name, ".") && strcmp(entry->d_name, "..") )
            OUT_Filenames.push_back( entry->d_name );

    closedir( dir );

    std::sort( OUT_Filenames.begin(), OUT_Filenames.end() );

    return OUT_Filenames;
}

void 
writeStringToFile
( const std::string& Cont, const std::string& Dest )
{
    std::ofstream( Dest.c_str() ) << Cont;
}

void 
deleteDirectoryContents
( const std::string& dir_path )
{
    for (const auto& entry : std::filesystem::directory_iterator(dir_path)) 
        std::filesystem::remove_all( entry.path() );
}






// Vis

visualization_msgs::Marker
getTrajMarker
()
{
    visualization_msgs::Marker Marker;
    Marker.header.frame_id = "world";
    Marker.pose.orientation.w = 1.0;
    Marker.scale.x = 0.05;
    Marker.color.a = 1.0;
    Marker.type = visualization_msgs::Marker::LINE_LIST;

    return Marker;
}


visualization_msgs::MarkerArray
getDroneMarker
( unsigned int& Rotors_N, double& ArmLength, double& BodyWidth, double& BodyHeight, double& Scale )
{
    visualization_msgs::MarkerArray DroneMarkerArray;
    DroneMarkerArray.markers.reserve( 2 * Rotors_N + 1 );

    visualization_msgs::Marker RotorMarker;
        // RotorMarker.header.stamp = ros::Time();
        // RotorMarker.header.frame_id = p_RVIZ_Drone_Childs_FrameID;
        // RotorMarker.ns = "vehicle_rotor";
        RotorMarker.action = visualization_msgs::Marker::ADD;
        RotorMarker.type = visualization_msgs::Marker::CYLINDER;
        RotorMarker.scale.x = 0.2 * Scale;
        RotorMarker.scale.y = 0.2 * Scale;
        RotorMarker.scale.z = 0.01 * Scale;
        setRGBOfVisMarker( VisualizationColors::GREY, RotorMarker );
        RotorMarker.color.a = 0.7;
        RotorMarker.pose.position.z = 0;

    visualization_msgs::Marker ArmMarker;
        //RVIZ_Drone_Arm.header.stamp = ros::Time();
        //RVIZ_Drone_Arm.header.frame_id = p_RVIZ_Drone_Childs_FrameID;
        //RVIZ_Drone_Arm.ns = "vehicle_arm";
        ArmMarker.action = visualization_msgs::Marker::ADD;
        ArmMarker.type = visualization_msgs::Marker::CUBE;
        ArmMarker.scale.x = ArmLength * Scale;
        ArmMarker.scale.y = 0.02 * Scale;
        ArmMarker.scale.z = 0.01 * Scale;
        setRGBOfVisMarker( VisualizationColors::BLUE, ArmMarker );
        ArmMarker.color.a = 1.0;
        ArmMarker.pose.position.z = -0.015 * Scale;

            const float RotorAngleIncrement = 2 * M_PI / Rotors_N;
            for (
                float Angle = RotorAngleIncrement / 2; 
                Angle <= 2 * M_PI; 
                Angle += RotorAngleIncrement
                ) 
            {
                RotorMarker.pose.position.x = ArmLength * cos( Angle ) * Scale;
                RotorMarker.pose.position.y = ArmLength * sin( Angle ) * Scale;
                RotorMarker.id++;

                ArmMarker.pose.position.x = RotorMarker.pose.position.x / 2;
                ArmMarker.pose.position.y = RotorMarker.pose.position.y / 2;
                ArmMarker.pose.orientation = tf::createQuaternionMsgFromYaw( Angle );
                ArmMarker.id++;

                DroneMarkerArray.markers.push_back( RotorMarker );
                DroneMarkerArray.markers.push_back( ArmMarker );
            }

    visualization_msgs::Marker BodyMarker;
        //BodyMarker.header.stamp = ros::Time();
        //BodyMarker.header.frame_id = p_RVIZ_Drone_Childs_FrameID;
        //BodyMarker.ns = "vehicle_body";
        BodyMarker.action = visualization_msgs::Marker::ADD;
        BodyMarker.type = visualization_msgs::Marker::CUBE;
        BodyMarker.scale.x = BodyMarker.scale.y = BodyMarker.scale.z = BodyHeight * Scale;
        setRGBOfVisMarker( VisualizationColors::GREEN, BodyMarker );
        BodyMarker.color.a = 0.8;

        DroneMarkerArray.markers.push_back( BodyMarker );

    return DroneMarkerArray;
}


void 
setRGBOfVisMarker
( const VisualizationColors& IN_Color, visualization_msgs::Marker& OUT_VisMarker )
{
    switch ( IN_Color ) 
    {
        case VisualizationColors::RED: 
            OUT_VisMarker.color.r = 1.0;
            OUT_VisMarker.color.g = 0.0;
            OUT_VisMarker.color.b = 0.0; break;

        case VisualizationColors::BLUE: 
            OUT_VisMarker.color.r = 0.0;
            OUT_VisMarker.color.g = 0.0;
            OUT_VisMarker.color.b = 1.0; break;
        
        case VisualizationColors::BLACK: 
            OUT_VisMarker.color.r = 0.0;
            OUT_VisMarker.color.g = 0.0;
            OUT_VisMarker.color.b = 0.0; break;
        
        case VisualizationColors::GREEN: 
            OUT_VisMarker.color.r = 0.0;
            OUT_VisMarker.color.g = 1.0;
            OUT_VisMarker.color.b = 0.0; break;
        
        case VisualizationColors::PURPLE: 
            OUT_VisMarker.color.r = 1.0;
            OUT_VisMarker.color.g = 0.0;
            OUT_VisMarker.color.b = 1.0; break;
        
        case VisualizationColors::WHITE: 
            OUT_VisMarker.color.r = 1.0;
            OUT_VisMarker.color.g = 1.0;
            OUT_VisMarker.color.b = 1.0; break;
        
        case VisualizationColors::YELLOW: 
            OUT_VisMarker.color.r = 1.0;
            OUT_VisMarker.color.g = 1.0;
            OUT_VisMarker.color.b = 0.0; break;

        case VisualizationColors::GREY: 
            OUT_VisMarker.color.r = 0.5;
            OUT_VisMarker.color.g = 0.5;
            OUT_VisMarker.color.b = 0.5; break;           
    }
}



void
rvizPosition
(
    const Eigen::Vector3d& Pos,
    const VisPosTypes& Type,
    const ros::Publisher& ROSPub
)
{
    visualization_msgs::Marker Marker;
        Marker.header.frame_id = "world";
        Marker.header.stamp = ros::Time();
        Marker.type = visualization_msgs::Marker::SPHERE;
        Marker.action = visualization_msgs::Marker::ADD;
        Marker.pose.orientation.x = 0.0;
        Marker.pose.orientation.y = 0.0;
        Marker.pose.orientation.z = 0.0;
        Marker.pose.orientation.w = 1.0;
        Marker.scale.x = 0.2;
        Marker.scale.y = 0.2;
        Marker.scale.z = 0.2;
        Marker.color.a = 1.0;

        Marker.pose.position = GeometryMsgsPoint_From_EigenVector3d( Pos );
        switch (Type)
        {
        case VisPosTypes::CURRGATE :
            Marker.id = 1;
            setRGBOfVisMarker( VisualizationColors::BLACK, Marker );
            Marker.ns = "current_gate";
            break;

        case VisPosTypes::EXPERT :
            Marker.id = 2;
            setRGBOfVisMarker( VisualizationColors::YELLOW, Marker );
            Marker.ns = "expert_state";
            break;

        case VisPosTypes::HORIZON :
            Marker.id = 3;
            setRGBOfVisMarker( VisualizationColors::BLUE, Marker );
            Marker.ns = "horizon_state";
            break;

        case VisPosTypes::REFERENCE :
            Marker.id = 4;
            setRGBOfVisMarker( VisualizationColors::RED, Marker );
            Marker.ns = "reference_state";
            break;
        
        case VisPosTypes::LT_END :
            Marker.id = 5;
            setRGBOfVisMarker( VisualizationColors::GREEN, Marker );
            Marker.ns = "local_trajectory_end_state";
            break;
        
        default:
            break;
        }

    ROSPub.publish( Marker );
}





bool
fetchROSParameters
(
    const ros::NodeHandle& ROSNH,
    const std::vector<std::pair<const char*, const bool*>>& KeysAndBoolOutputs,
    const std::vector<std::pair<const char*, const int*>>& KeysAndIntOutputs,
    const std::vector<std::pair<const char*, const double*>>& KeysAndDoubleOutputs,
    const std::vector<std::pair<const char*, const std::string*>>& KeysAndStringOutputs
)
{
    bool ReturnBool = true;
    XmlRpc::XmlRpcValue Output;


    // --- Parameters of type bool ---
    for ( size_t i = 0; i < KeysAndBoolOutputs.size(); i++ )
    {
        if ( ROSNH.getParam(std::get<0>(KeysAndBoolOutputs[ i ]), Output) )
        {
            const_cast< bool& >( *std::get<1>(KeysAndBoolOutputs[ i ]) ) 
                = static_cast<bool>( Output );
            
            ROS_INFO( "[%s]\n  >> Fetched \"%s: %s\" from ROS parameter server.", 
                ros::this_node::getName().c_str(), 
                std::get<0>(KeysAndBoolOutputs[ i ]),  (*std::get<1>(KeysAndBoolOutputs[ i ]))? "true" : "false" );
        }
        else
        {
            ROS_FATAL( "[%s]\n  >> Failed to fetch \"%s\" from ROS parameter server.", 
                ros::this_node::getName().c_str(), std::get<0>(KeysAndBoolOutputs[ i ]) );
            ReturnBool = false;
        }
    }

    // --- Parameters of type int ---
    for ( size_t i = 0; i < KeysAndIntOutputs.size(); i++ )
    {
        if ( ROSNH.getParam(std::get<0>(KeysAndIntOutputs[ i ]), Output) )
        {
            const_cast< int& >( *std::get<1>(KeysAndIntOutputs[ i ]) ) 
                = static_cast<int>( Output );
            
            ROS_INFO( "[%s]\n  >> Fetched \"%s: %d\" from ROS parameter server.", 
                ros::this_node::getName().c_str(), 
                std::get<0>(KeysAndIntOutputs[ i ]),  *std::get<1>(KeysAndIntOutputs[ i ]));
        }
        else
        {
            ROS_FATAL( "[%s]\n  >> Failed to fetch \"%s\" from ROS parameter server.", 
                ros::this_node::getName().c_str(), std::get<0>(KeysAndIntOutputs[ i ]) );
            ReturnBool = false;
        }
    }

    // --- Parameters of type double ---
    for ( size_t i = 0; i < KeysAndDoubleOutputs.size(); i++ )
    {
        if ( ROSNH.getParam(std::get<0>(KeysAndDoubleOutputs[ i ]), Output) )
        {
            const_cast< double& >( *std::get<1>(KeysAndDoubleOutputs[ i ]) ) 
                = static_cast<double>( Output );

            ROS_INFO( "[%s]\n  >> Fetched \"%s: %f\" from ROS parameter server.", 
                ros::this_node::getName().c_str(), 
                std::get<0>(KeysAndDoubleOutputs[ i ]),  *std::get<1>(KeysAndDoubleOutputs[ i ]));
        }
        else
        {
            ROS_FATAL( "[%s]\n  >> Failed to fetch \"%s\" from ROS parameter server.", 
                ros::this_node::getName().c_str(), std::get<0>(KeysAndDoubleOutputs[ i ]) );
            ReturnBool = false;
        }
    }

    // --- Parameters of type std::string ---
    for ( size_t i = 0; i < KeysAndStringOutputs.size(); i++ )
    {
        if ( ROSNH.getParam(std::get<0>(KeysAndStringOutputs[ i ]), Output) )
        {
            const_cast< std::string& >( *std::get<1>(KeysAndStringOutputs[ i ]) ) 
                = static_cast<std::string>( Output );
            
            const std::string& Value = *std::get<1>(KeysAndStringOutputs[ i ]);

            ROS_INFO( "[%s]\n  >> Fetched \"%s: %s\" from ROS parameter server.", 
                ros::this_node::getName().c_str(), 
                std::get<0>(KeysAndStringOutputs[ i ]),
                Value.length() > 30? (Value.substr(0, 30) + "...").c_str() : Value.c_str()
                );
        }
        else
        {
            ROS_FATAL( "[%s]\n  >> Failed to fetch \"%s\" from ROS parameter server.", 
                ros::this_node::getName().c_str(), std::get<0>(KeysAndStringOutputs[ i ]) );
            ReturnBool = false;
        }
    }


    //________________
    return ReturnBool;
}



void runMultiThreadedSpinner()
{
    ros::MultiThreadedSpinner Spinner;
    Spinner.spin();
}

}