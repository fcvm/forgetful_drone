project(forgetful_drones)

cmake_minimum_required(VERSION 3.0.0)

find_package(catkin_simple REQUIRED)
find_package(OpenCV REQUIRED)

catkin_simple()

# Setup Default Build Type as Release
if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "" FORCE)
endif ()

# Setup Architcture-specific Flags
if ("${CMAKE_HOST_SYSTEM_PROCESSOR}" STREQUAL "armv7l")
    message(STATUS "Using ARMv7 optimized flags!")
    set(CMAKE_CXX_ARCH_FLAGS " -Wno-psabi -march=armv7-a -mfpu=neon -mfloat-abi=hard -funsafe-math-optimizations")
elseif ("${CMAKE_HOST_SYSTEM_PROCESSOR}" STREQUAL "aarch64")
    message(STATUS "Using ARM aarch64 optimized flags!")
    set(CMAKE_CXX_ARCH_FLAGS " -Wno-psabi -march=armv8-a+crypto -mcpu=cortex-a57+crypto")
else ()
    set(CMAKE_CXX_ARCH_FLAGS " -march=native")
endif ()

# Setup General C++ Flags
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DEIGEN_STACK_ALLOCATION_LIMIT=1048576")
# otherwise double free or corruption (out) error when running racing or motion_planning example
add_compile_options(-O3)

# Setup Release and Debug flags
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS} ${CMAKE_CXX_ARCH_FLAGS} -Wall -DNDEBUG -fPIC")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS} -Wall -g")




#get_filename_component(matplotlib_cpp_CMAKE_DIR "${CMAKE_CURRENT_LIST_FILE}" "/home/fm/git/matplotlib-cpp")
#
#if(NOT TARGET matplotlib_cpp::matplotlib_cpp)
#  find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
#  find_package(Python3 COMPONENTS NumPy)
#  include("${matplotlib_cpp_CMAKE_DIR}/matplotlib_cppTargets.cmake")
#
#  get_target_property(matplotlib_cpp_INCLUDE_DIRS matplotlib_cpp::matplotlib_cpp INTERFACE_INCLUDE_DIRECTORIES)
#
#endif()
find_package(PythonLibs REQUIRED)
include_directories(${PYTHON_INCLUDE_DIRS})



set(GUROBI_INCLUDE_DIRECTORY "/opt/gurobi950/linux64/include")
include_directories(${GUROBI_INCLUDE_DIRECTORY})
set(GUROBI_LIB_DIRECTORY "/opt/gurobi950/linux64/lib")
link_directories(${GUROBI_LIB_DIRECTORY})




# CPP Units -> Libraries
cs_add_library(forgetful_helpers src/forgetful_helpers.cpp)
cs_add_library(forgetful_drone_base src/forgetful_drone_base.cpp)
cs_add_library(forgetful_simulator_base src/forgetful_simulator_base.cpp)

target_link_libraries(forgetful_helpers ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} stdc++fs)
target_link_libraries(forgetful_drone_base ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} stdc++fs)
target_link_libraries(forgetful_drone_base ${PYTHON_LIBRARIES})
target_link_libraries(forgetful_drone_base libgurobi_g++5.2.a gurobi95)

cs_add_executable(forgetful_drone src/forgetful_drone_node.cpp)
cs_add_executable(forgetful_simulator src/forgetful_simulator_node.cpp)

target_link_libraries(forgetful_drone forgetful_drone_base forgetful_helpers ${OpenCV_LIBRARIES} stdc++fs)
target_link_libraries(forgetful_simulator forgetful_simulator_base forgetful_helpers ${OpenCV_LIBRARIES} stdc++fs)

cs_install()
cs_export()

