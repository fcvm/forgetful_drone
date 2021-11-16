#include <gflags/gflags.h>
#include <glog/logging.h>
#include "forgetful_drones/forgetful_simulator.h"


int runForgetfulSimulator( int argc, char** argv );



int main( int argc, char** argv )
{
  //google::InitGoogleLogging( argv[0] );
  //google::ParseCommandLineFlags( &argc, &argv, true );
  //google::InstallFailureSignalHandler();
  //FLAGS_alsologtostderr = true;
  //FLAGS_colorlogtostderr = true;

  return runForgetfulSimulator( argc, argv );
}

int runForgetfulSimulator( int argc, char** argv ) 
{
  ros::init(argc, argv, "forgetful_simulator");

  forgetful_drone::ForgetfulSimulator ForgetfulSimulator;
  ros::MultiThreadedSpinner Spinner;
  Spinner.spin();
  
  return 0;
}