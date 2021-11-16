#include <gflags/gflags.h>
#include <glog/logging.h>

#include "forgetful_drones/forgetful_orchestrator.h"





int main_ForgetfulOrchestrator(int argc, char** argv);




int main(int argc, char** argv)
{
  //google::InitGoogleLogging(argv[0]);
  //google::ParseCommandLineFlags(&argc, &argv, true);
  //google::InstallFailureSignalHandler();
  //FLAGS_alsologtostderr = true;
  //FLAGS_colorlogtostderr = true;
  return main_ForgetfulOrchestrator( argc, argv );
}


int main_ForgetfulOrchestrator(int argc, char** argv) 
{
  ros::init(argc, argv, "forgetful_orchestrator");

  forgetful_drone::ForgetfulOrchestrator forgetful_orchestrator;

  ros::MultiThreadedSpinner spinner;
  spinner.spin();
  return 0;
}