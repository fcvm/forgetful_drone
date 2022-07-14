#!/bin/bash
roslaunch forgetful_drones test_dagger.launch 2> >(grep -v -E "TF_REPEATED_DATA|/tmp/binarydeb/ros-noetic-tf2-0.7.5/src/buffer_core.cpp")