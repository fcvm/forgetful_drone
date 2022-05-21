#include "forgetful_drones/forgetful_simulator.hpp"


int main (int argc, char** argv) {
  
  ros::init(argc, argv, "forgetful_simulator");
  forgetful_drone::ForgetfulSimulator fs (/*as_ros_node*/true);
  ros::MultiThreadedSpinner spinner;
  spinner.spin();
  return 0;

}