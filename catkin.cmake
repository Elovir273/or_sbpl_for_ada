cmake_minimum_required(VERSION 2.8.3)
project(or_sbpl_for_ada)

find_package(catkin REQUIRED COMPONENTS 
  openrave_catkin
  roscpp
  rospy
  std_msgs
  )

catkin_package(
    INCLUDE_DIRS include/
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS roscpp rospy std_msgs
)

find_package(OpenRAVE REQUIRED)

include(FindPkgConfig)
pkg_check_modules(SBPL REQUIRED sbpl)
pkg_check_modules(YamlCpp REQUIRED yaml-cpp)

if (${YamlCpp_VERSION} VERSION_LESS 0.5.0)
    message(STATUS "Using the old-style yaml-cpp (< 0.5.0) API.")
else ()
    add_definitions(-DYAMLCPP_NEWAPI)
    message(STATUS "Using the new-style yaml-cpp (>= 0.5.0) API.")
endif ()
include_directories(
  include
  ${OpenRAVE_INCLUDE_DIRS}
  ${SBPL_INCLUDE_DIRS}
  ${Yaml_INCLUDE_DIRS})

link_directories(
  ${OpenRAVE_LIBRARY_DIRS} 
  ${SBPL_LIBRARY_DIRS} 
  ${Yaml_LIBRARY_DIRS})
 
add_library(${PROJECT_NAME} 
  src/SBPLBasePlanner7d.cpp 
  src/SBPLBasePlannerEnvironment7d.cpp 
  src/SBPLBasePlannerTypes7d.cpp
 # src/CachedAction.cpp
 # src/TwistAction.cpp
 # src/start_pos_listener.cpp
  src/Action7d.cpp)

target_link_libraries(${PROJECT_NAME}
    ${SBPL_LIBRARIES} 
    ${OpenRAVE_LIBRARIES} 
    ${Yaml_LIBRARIES})

openrave_plugin(${PROJECT_NAME}_plugin src/SBPLMain.cpp)
target_link_libraries(${PROJECT_NAME}_plugin
    ${PROJECT_NAME} 
    yaml-cpp 
    ${SBPL_LIBRARIES} 
    ${OpenRAVE_LIBRARIES})

add_executable(yamltest2
  test/YamlTest2.cpp
  src/Action7d.cpp
  src/SBPLBasePlannerTypes7d.cpp
)

target_link_libraries(yamltest2
   yaml-cpp ${OpenRAVE_LIBRARIES} boost_system)

install(TARGETS or_sbpl_for_ada
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    PATTERN ".svn" EXCLUDE)
