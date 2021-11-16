cmake_minimum_required(VERSION 3.1.0)
#project(forgetful_drones)
PROJECT(forgetful_drones VERSION 1.1.0 LANGUAGES CXX)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
set(CMAKE_BUILD_TYPE Release)
add_compile_options(-std=c++11)
add_compile_options(-std=c++14)
add_compile_options(-std=c++17)


find_package(catkin_simple REQUIRED)
find_package(OpenCV REQUIRED)

# ???Find out if it is part of catkin_simple()???
#find_package(catkin REQUIRED COMPONENTS
#  geometry_msgs
#)
#
#catkin_package(
#  CATKIN_DEPENDS geometry_msgs
#)

catkin_simple()

cs_add_library(forgetful_helpers src/forgetful_helpers.cpp)
cs_add_library(forgetful_drone_base src/forgetful_drone_base.cpp)
cs_add_library(forgetful_simulator_base src/forgetful_simulator_base.cpp)

#cs_add_library(forgetful_orchestrator_base src/forgetful_orchestrator_base.cpp)
#cs_add_library(forgetful_navigator_base src/forgetful_navigator_base.cpp)
#cs_add_library(forgetful_expert_base src/forgetful_expert_base.cpp)

cs_add_executable(forgetful_drone src/forgetful_drone_node.cpp)
cs_add_executable(forgetful_simulator src/forgetful_simulator_node.cpp)

#cs_add_executable(forgetful_orchestrator_node src/forgetful_orchestrator_node.cpp)
#cs_add_executable(forgetful_navigator_node src/forgetful_navigator_node.cpp)
#cs_add_executable(forgetful_expert_node src/forgetful_expert_node.cpp)

target_link_libraries(forgetful_drone forgetful_drone_base)
target_link_libraries(forgetful_drone ${OpenCV_LIBS})
target_link_libraries(forgetful_drone forgetful_helpers)

target_link_libraries(forgetful_simulator forgetful_simulator_base)
target_link_libraries(forgetful_simulator forgetful_helpers)

#target_link_libraries(forgetful_orchestrator_node forgetful_orchestrator_base)
#target_link_libraries(forgetful_orchestrator_node forgetful_helpers)

#target_link_libraries(forgetful_navigator_node forgetful_navigator_base)
#target_link_libraries(forgetful_navigator_node forgetful_helpers)

#target_link_libraries(forgetful_expert_node forgetful_expert_base)
#target_link_libraries(forgetful_expert_node forgetful_helpers)

cs_export()

# CMake Indexing
FILE(GLOB_RECURSE LibFiles "include/*")
add_custom_target(headers SOURCES ${LibFiles})
FILE(GLOB_RECURSE ProtoFiles "proto/*")
add_custom_target(protos SOURCES ${ProtoFiles})
