cmake_minimum_required(VERSION 3.0.2)
project(mavproxy_ros)

## Compile as C++17 (required by event_callback headers: std::any, if constexpr)
add_compile_options(-std=c++17)
catkin_python_setup()
###################################
## Find catkin macros and libraries
###################################
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  mavros_msgs
  visualization_msgs
  std_srvs
  quadrotor_msgs
  rsos_msgs
  event_callback
  message_generation
)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate services in the 'srv' folder
add_service_files(
  FILES
  ProcessRequest.srv
  Register.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS
    roscpp rospy std_msgs
    geometry_msgs nav_msgs sensor_msgs
    mavros_msgs visualization_msgs std_srvs
    quadrotor_msgs rsos_msgs event_callback
    message_runtime
)

###########
## Build ##
###########
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Executable: test
add_executable(${PROJECT_NAME}_test src/test.cpp)
add_dependencies(${PROJECT_NAME}_test
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(${PROJECT_NAME}_test
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

## Install C++ executables
install(TARGETS ${PROJECT_NAME}_test
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Install Python executable scripts
catkin_install_python(PROGRAMS
  scripts/connection.py
  scripts/control.py
  scripts/pland.py
  scripts/param.py
  scripts/other.py
  scripts/dummy_service.py
  scripts/key_contrl.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Install launch files and config
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)
