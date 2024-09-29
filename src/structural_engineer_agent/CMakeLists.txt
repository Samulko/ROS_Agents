cmake_minimum_required(VERSION 3.0.2)
project(structural_engineer_agent)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  multi_agent_system
)

find_package(Python3 REQUIRED)

catkin_package(
  CATKIN_DEPENDS rospy std_msgs multi_agent_system
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/structural_engineer_agent.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

