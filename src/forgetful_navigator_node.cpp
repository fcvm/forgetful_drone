#include <gflags/gflags.h>
#include <glog/logging.h>

#include "forgetful_drones/forgetful_navigator.h"





int main_ForgetfulNavigator(int argc, char** argv);




int main(int argc, char** argv)
{
  //google::InitGoogleLogging(argv[0]);
  //google::ParseCommandLineFlags(&argc, &argv, true);
  //google::InstallFailureSignalHandler();
  //FLAGS_alsologtostderr = true;
  //FLAGS_colorlogtostderr = true;
  return main_ForgetfulNavigator( argc, argv );
}


int main_ForgetfulNavigator(int argc, char** argv) 
{
  ros::init(argc, argv, "forgetful_navigator");

  forgetful_drone::ForgetfulNavigator forgetful_navigator;

  ros::MultiThreadedSpinner spinner;
  spinner.spin();
  return 0;
}