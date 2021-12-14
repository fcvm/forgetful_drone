#include <gflags/gflags.h>
#include <glog/logging.h>

#include "forgetful_drones/forgetful_expert.h"





int main_ForgetfulExpert(int argc, char** argv);




int main(int argc, char** argv)
{
  //google::InitGoogleLogging(argv[0]);
  //google::ParseCommandLineFlags(&argc, &argv, true);
  //google::InstallFailureSignalHandler();
  //FLAGS_alsologtostderr = true;
  //FLAGS_colorlogtostderr = true;
  return main_ForgetfulExpert( argc, argv );
}


int main_ForgetfulExpert(int argc, char** argv) 
{
  ros::init(argc, argv, "forgetful_expert");

  forgetful_drone::ForgetfulExpert forgetful_expert;

  ros::MultiThreadedSpinner spinner;
  spinner.spin();
  return 0;
}