#include <gflags/gflags.h>
#include <glog/logging.h>
#include "forgetful_drones/forgetful_drone.h"





int runForgetfulDrone( int argc, char** argv );

int main( int argc, char** argv )
{
  //google::InitGoogleLogging( argv[0] );
  //google::ParseCommandLineFlags( &argc, &argv, true );
  //google::InstallFailureSignalHandler();
  //FLAGS_alsologtostderr = true;
  //FLAGS_colorlogtostderr = true;

  return runForgetfulDrone(argc, argv);
}

int runForgetfulDrone(int argc, char** argv) 
{
  ros::init( argc, argv, "forgetful_drone" );

  forgetful_drone::ForgetfulDrone ForgetfulDrone;
  ros::MultiThreadedSpinner Spinner;
  Spinner.spin();
  
  return 0;
}