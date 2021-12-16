#include "forgetful_drones/forgetful_simulator.hpp"


int main( int argc, char** argv )
{
  ros::init(argc, argv, "forgetful_simulator");
  
  forgetful_drone::ForgetfulSimulator ForgetfulSimulator;
  
  ros::MultiThreadedSpinner Spinner;
  Spinner.spin();
  
  return 0;
}