# Install script for directory: /home/samko/Documents/GitHub/ros_noetic_311/src/multi_agent_system

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/samko/Documents/GitHub/ros_noetic_311/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/multi_agent_system/msg" TYPE FILE FILES
    "/home/samko/Documents/GitHub/ros_noetic_311/src/multi_agent_system/msg/UserCommand.msg"
    "/home/samko/Documents/GitHub/ros_noetic_311/src/multi_agent_system/msg/AgentResponse.msg"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/multi_agent_system/srv" TYPE FILE FILES
    "/home/samko/Documents/GitHub/ros_noetic_311/src/multi_agent_system/srv/ValidateRequest.srv"
    "/home/samko/Documents/GitHub/ros_noetic_311/src/multi_agent_system/srv/PlanExecution.srv"
    "/home/samko/Documents/GitHub/ros_noetic_311/src/multi_agent_system/srv/StabilityAnalysis.srv"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/multi_agent_system/cmake" TYPE FILE FILES "/home/samko/Documents/GitHub/ros_noetic_311/build/multi_agent_system/catkin_generated/installspace/multi_agent_system-msg-paths.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/samko/Documents/GitHub/ros_noetic_311/devel/include/multi_agent_system")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/samko/Documents/GitHub/ros_noetic_311/devel/share/roseus/ros/multi_agent_system")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/samko/Documents/GitHub/ros_noetic_311/devel/share/common-lisp/ros/multi_agent_system")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/samko/Documents/GitHub/ros_noetic_311/devel/share/gennodejs/ros/multi_agent_system")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/usr/bin/python3" -m compileall "/home/samko/Documents/GitHub/ros_noetic_311/devel/lib/python3/dist-packages/multi_agent_system")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3/dist-packages" TYPE DIRECTORY FILES "/home/samko/Documents/GitHub/ros_noetic_311/devel/lib/python3/dist-packages/multi_agent_system")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/samko/Documents/GitHub/ros_noetic_311/build/multi_agent_system/catkin_generated/installspace/multi_agent_system.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/multi_agent_system/cmake" TYPE FILE FILES "/home/samko/Documents/GitHub/ros_noetic_311/build/multi_agent_system/catkin_generated/installspace/multi_agent_system-msg-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/multi_agent_system/cmake" TYPE FILE FILES
    "/home/samko/Documents/GitHub/ros_noetic_311/build/multi_agent_system/catkin_generated/installspace/multi_agent_systemConfig.cmake"
    "/home/samko/Documents/GitHub/ros_noetic_311/build/multi_agent_system/catkin_generated/installspace/multi_agent_systemConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/multi_agent_system" TYPE FILE FILES "/home/samko/Documents/GitHub/ros_noetic_311/src/multi_agent_system/package.xml")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/multi_agent_system" TYPE PROGRAM FILES "/home/samko/Documents/GitHub/ros_noetic_311/build/multi_agent_system/catkin_generated/installspace/agent_responses_logger.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/multi_agent_system" TYPE PROGRAM FILES "/home/samko/Documents/GitHub/ros_noetic_311/build/multi_agent_system/catkin_generated/installspace/user_input.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/multi_agent_system" TYPE PROGRAM FILES "/home/samko/Documents/GitHub/ros_noetic_311/src/multi_agent_system/scripts/agent_responses_logger.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/multi_agent_system" TYPE DIRECTORY FILES "/home/samko/Documents/GitHub/ros_noetic_311/src/multi_agent_system/launch")
endif()

