#!/bin/bash
export OGRE_RTT_MODE=Copy #"Copy", "PBuffer", or "FBO", https://wiki.ros.org/rviz/Troubleshooting#Segfault_during_startup

roslaunch forgetful_drones test_simulator.launch unity_scene_index:=4 2> >(grep -v -E "TF_REPEATED_DATA|/tmp/binarydeb/ros-noetic-tf2-0.7.5/src/buffer_core.cpp")
