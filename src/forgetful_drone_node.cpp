#include "forgetful_drones/forgetful_drone.hpp"

int main (int argc, char** argv) {
    ros::init (argc, argv, "forgetful_drone");
    forgetful_drone::ForgetfulDrone fd;
    ros::MultiThreadedSpinner s;
    s.spin();

    return 0;
}