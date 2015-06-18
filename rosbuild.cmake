cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
rosbuild_add_library(${PROJECT_NAME} SHARED 
  src/CachedAction.cpp
  src/SBPLMain.cpp 
  src/SBPLBasePlanner.cpp 
  src/SBPLBasePlannerEnvironment.cpp
  src/SBPLBasePlannerTypes.cpp
  src/TwistAction.cpp)
target_link_libraries(${PROJECT_NAME} sbpl yaml-cpp)

rosbuild_add_executable(yamltest test/YamlTest.cpp src/CachedAction.cpp src/SBPLBasePlannerTypes.cpp)

#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)

target_link_libraries(yamltest yaml-cpp)
