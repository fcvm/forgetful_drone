#include "forgetful_drones/forgetful_helpers.hpp"








namespace forgetful_drone {


Pose::Pose (const Eigen::Vector3f pos, const Eigen::Quaternionf ori)
    : 
    position {pos}, 
    orientation {ori}
    {}

Pose::Pose ()
    : 
    position {0.0, 0.0, 0.0}, 
    orientation {1.0, 0.0, 0.0, 0.0}
    {}

Pose::Pose (const geometry_msgs::Pose& pose)
    : 
    position {EV3f___GMP (pose.position)}, 
    orientation {EQf___GMQ (pose.orientation)}
    {}



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
    return Yaw_From_EQf(orientation);
}



double Yaw_From_EQd(const Eigen::Quaterniond& in) {
    return in.toRotationMatrix().eulerAngles(0, 1, 2).z();
}

double Yaw_From_EQf (const Eigen::Quaternionf& in) {
    return in.toRotationMatrix().eulerAngles(0, 1, 2).z();
}



Eigen::Vector3d 
EV3d___Vec3
( const Vec3& IN )
{
    return { IN.x, IN.y, IN.z };
}


Eigen::Quaternionf EQf_from_Yaw(
    const double& IN, 
    const bool& in_degree
) {
    float yaw = in_degree? (IN/180.0*M_PI) : IN;
    
    return Eigen::Quaternionf{
        Eigen::AngleAxisf{
            yaw, 
            Eigen::Vector3f{0, 0, 1}
        }
    };
}

Vec3 
Vec3_from_EV3d
( const Eigen::Vector3d& IN )
{
    return { IN.x(), IN.y(), IN.z() };
}


geometry_msgs::Pose GMPose_From_NMO (const nav_msgs::Odometry& in) {
    geometry_msgs::Pose out;
    out.position = in.pose.pose.position;
    out.orientation = in.pose.pose.orientation;
    
    return out;
}


geometry_msgs::Pose GMPose___EV3d (const Eigen::Vector3d& v) {
    geometry_msgs::Pose p;
        p.position = GMPoint__from__EV3d (v);
        p.orientation.w = 1.0; p.orientation.x = 0.0; p.orientation.y = 0.0; p.orientation.z = 0.0;
    return p;
}


geometry_msgs::Pose GMPose___EV3d_EQd (const Eigen::Vector3d& in_ev3d, const Eigen::Quaterniond& in_eqd) {
    geometry_msgs::Pose out;
    out.position = GMPoint__from__EV3d(in_ev3d);
    out.orientation = GMQ_From_EQd(in_eqd);
    
    return out;
}





void playAudioFile (const std::string fpath) {
    std::string cmd = "ffplay -nodisp -autoexit " + fpath + " >/dev/null 2>&1";
    system(cmd.c_str());
}

void playAudio (const std::string txt) {
    std::string cmd = "spd-say \"" + txt + "\"";
    system(cmd.c_str());
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


geometry_msgs::Point GMPoint__from__EV3d (const Eigen::Vector3d& v) {
    geometry_msgs::Point p; 
        p.x = v.x(); p.y = v.y(); p.z = v.z();
    return p;
}

Eigen::Vector3d EV3d_From_GMV3 (const geometry_msgs::Vector3& IN) {return {IN.x, IN.y, IN.z };}

Eigen::Vector3d EV3d___GMP (const geometry_msgs::Point& p) {return {p.x, p.y, p.z};}
Eigen::Vector3f EV3f___GMP (const geometry_msgs::Point& p) {return {p.x, p.y, p.z};}
Eigen::Quaterniond EQd___GMQ (const geometry_msgs::Quaternion& q) {return {q.w, q.x, q.y, q.z};}
Eigen::Quaternionf EQf___GMQ (const geometry_msgs::Quaternion& q) {return {q.w, q.x, q.y, q.z};}


geometry_msgs::Quaternion
GMQ_From_EQd
( const Eigen::Quaterniond& IN )
{
    geometry_msgs::Quaternion OUT;
    OUT.w = IN.w(); OUT.x = IN.x(); OUT.y = IN.y(); OUT.z = IN.z();
    return OUT;
}



std::vector<double>
StdVector_From_EigenVector
( const Eigen::VectorXd& IN )
{
    std::vector<double> OUT;
    for (int i = 0; i < IN.size(); i++)
        OUT.push_back(IN(i));
    return OUT;
}

Eigen::VectorXd
EigenVector_From_StdVector
( const std::vector<double>& IN )
{
    Eigen::VectorXd OUT = Eigen::VectorXd::Zero(IN.size());
    for (std::size_t i = 0; i < IN.size(); i++)
        OUT(i) = IN[i];
    return OUT;
}







std::string UTCDateTime () {

    std::chrono::system_clock::time_point now = std::chrono::system_clock::now ();
    time_t tt = std::chrono::system_clock::to_time_t (now);
    tm gm_tm = *std::gmtime (&tt);
    
    char dlmtr = '_';
    std::ostringstream dt; dt
        << "UTC"                            << dlmtr
        << gm_tm.tm_year + 1900             << dlmtr
        << asSeqNo (2, gm_tm.tm_mon + 1)    << dlmtr
        << asSeqNo (2, gm_tm.tm_mday)       << dlmtr
        << asSeqNo (2, gm_tm.tm_hour)       << dlmtr
        << asSeqNo (2, gm_tm.tm_min)        << dlmtr
        << asSeqNo (2, gm_tm.tm_sec)
        ;

    return dt.str ();
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
    for (const auto& entry : std::experimental::filesystem::directory_iterator(dir_path)) 
        std::experimental::filesystem::remove_all( entry.path() );
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

        Marker.pose.position = GMPoint__from__EV3d( Pos );
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
            setRGBOfVisMarker( VisualizationColors::BLACK, Marker );
            Marker.ns = "local_trajectory_end_state";
            break;
        
        default:
            break;
        }

    ROSPub.publish( Marker );
}



void
rvizState
(
    const Eigen::Vector3d& Pos,
    const Eigen::Vector3d& Vel,
    const VisPosTypes& Type,
    const ros::Publisher& ROSPub
)
{
    visualization_msgs::Marker Marker;
        Marker.header.frame_id = "world";
        Marker.header.stamp = ros::Time();
        Marker.type = visualization_msgs::Marker::ARROW;
        Marker.action = visualization_msgs::Marker::ADD;
        Marker.scale.x = 0.1; // shaft diameter
        Marker.scale.y = 0.2; // head diameter
        Marker.scale.z = 0.0; // if not zero: head length
        Marker.color.a = 1.0;

        Marker.points.push_back(GMPoint__from__EV3d(Pos));
        Marker.points.push_back(GMPoint__from__EV3d(Pos + Vel.normalized()));


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
            setRGBOfVisMarker( VisualizationColors::BLACK, Marker );
            Marker.ns = "local_trajectory_end_state";
            break;
        
        default:
            break;
        }

    ROSPub.publish( Marker );
}







bool fetchROSParameters (
    const ros::NodeHandle& rosNH,
    const std::vector<std::pair<const char*, const bool*>>& keys_bools,
    const std::vector<std::pair<const char*, const int*>>& keys_ints,
    const std::vector<std::pair<const char*, const double*>>& keys_doubles,
    const std::vector<std::pair<const char*, const std::string*>>& keys_strings,
    const bool& log_enabled
){
    bool AllParamsFound {true};

    for (const auto& key_dest : keys_bools)
        if (!fetchROSParameter<bool>(rosNH, std::get<0>(key_dest), 
                const_cast<bool&>(*std::get<1>(key_dest)), log_enabled)) 
            AllParamsFound = false;

    for (const auto& key_dest : keys_ints)
        if (!fetchROSParameter<int>(rosNH, std::get<0>(key_dest), 
                const_cast<int&>(*std::get<1>(key_dest)), log_enabled)) 
            AllParamsFound = false;

    for (const auto& key_dest : keys_doubles)
        if (!fetchROSParameter<double>(rosNH, std::get<0>(key_dest), 
                const_cast<double&>(*std::get<1>(key_dest)), log_enabled)) 
            AllParamsFound = false;

    for (const auto& key_dest : keys_strings)
        if (!fetchROSParameter<std::string>(rosNH, std::get<0>(key_dest), 
                const_cast<std::string&>(*std::get<1>(key_dest)), log_enabled)) 
            AllParamsFound = false;

    return AllParamsFound;
}



void runMultiThreadedSpinner () {
    ros::MultiThreadedSpinner s;
    s.spin();
}

void runForgetfulSimulator () {
    system("rosrun forgetful_drones forgetful_simulator __name:/hummingbird/forgetful_simulator");
}


void checkTimerPeriod (
    const std::string& tag,
    const ros::TimerEvent& te,
    const double& period
) {
    const double duration = te.profile.last_duration.toSec();
    if (duration > period) {
        ROS_WARN_STREAM(tag << "ROS timer with period of " << period << " s took " << duration << " s.");
    }
}


bool createDir (const std::string& tag, const std::string& path) {
    if (std::experimental::filesystem::create_directories (path)) {
        ROS_INFO_STREAM (tag << "Created directory \"" << path << "\"");
        return true;
    } else {
        ROS_ERROR_STREAM(tag << "Failed to create directory \"" << path << "\"");
        return false;
    }
}

bool isDir (const std::string& p) {
    return std::experimental::filesystem::is_directory (p);
}

bool isFile (const std::string& p) {
    return std::experimental::filesystem::exists (p);
}




bool copyFile (
    const std::string& tag,
    const std::string& src,
    const std::string& dst
) {
    if (std::experimental::filesystem::copy_file (src, dst)) {
        ROS_INFO_STREAM(tag << "Copied file from \"" << src << "\" to \"" << dst << "\"");
        return true;
    } else {
        ROS_INFO_STREAM(tag << "Failed to copy file from \"" << src << "\" to \"" << dst << "\"");
        return false;
    }
}


void saveCVMat (
    const std::string& tag,
    const std::string& file_path,
    const cv::Mat& cv_mat
) {
    if (!cv::imwrite(file_path, cv_mat)) {
        ROS_ERROR_STREAM(tag << "Failed to save image \"" << file_path << "\"");
    }
}


bool isEmpty (const std::string& tag, const cv::Mat& cv_mat) {
    if (cv_mat.cols * cv_mat.rows == 0) {
        ROS_WARN_STREAM(tag << "Detected empty image");
        return true;
    }
    return false;
}




std::string asSeqNo (const int& width, const int& no) {
    std::ostringstream oss; 
    oss 
        << std::setw (width) 
        << std::setfill ('0') 
        << no;
    return oss.str ();
}

std::string asFixedFloat (const int& width, const int& prec, const double& no) {
    std::ostringstream oss; 
    oss 
        << std::setfill ('0') 
        << std::setw (width) 
        << std::fixed 
        << std::setprecision (prec) 
        << no;
    return oss.str ();
}


}