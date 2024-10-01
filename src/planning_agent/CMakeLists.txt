cmake_minimum_required(VERSION 3.0.2)
project(planning_agent)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  multi_agent_system
  message_generation
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS rospy std_msgs multi_agent_system message_runtime
)

catkin_install_python(PROGRAMS
  scripts/planning_agent.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)