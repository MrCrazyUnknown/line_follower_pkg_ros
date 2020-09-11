cmake_minimum_required(VERSION 3.0.2)
project(line_follower)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  dynamic_reconfigure
)

generate_dynamic_reconfigure_options(
  cfg/PIDControl.cfg
)

catkin_package(
  DEPENDS
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)


catkin_install_python(PROGRAMS scripts/teleop.py scripts/CamRcv.py scripts/threshold.py scripts/AutoMove.py scripts/Movement.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
