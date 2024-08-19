# Install script for directory: /home/cn/NeuralSymbol_AI_for_task2/src/Universal_Robots_ROS_Driver/ur_msgs

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/cn/NeuralSymbol_AI_for_task2/install")
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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ur_msgs/msg" TYPE FILE FILES
    "/home/cn/NeuralSymbol_AI_for_task2/src/Universal_Robots_ROS_Driver/ur_msgs/msg/Analog.msg"
    "/home/cn/NeuralSymbol_AI_for_task2/src/Universal_Robots_ROS_Driver/ur_msgs/msg/Digital.msg"
    "/home/cn/NeuralSymbol_AI_for_task2/src/Universal_Robots_ROS_Driver/ur_msgs/msg/IOStates.msg"
    "/home/cn/NeuralSymbol_AI_for_task2/src/Universal_Robots_ROS_Driver/ur_msgs/msg/RobotStateRTMsg.msg"
    "/home/cn/NeuralSymbol_AI_for_task2/src/Universal_Robots_ROS_Driver/ur_msgs/msg/MasterboardDataMsg.msg"
    "/home/cn/NeuralSymbol_AI_for_task2/src/Universal_Robots_ROS_Driver/ur_msgs/msg/RobotModeDataMsg.msg"
    "/home/cn/NeuralSymbol_AI_for_task2/src/Universal_Robots_ROS_Driver/ur_msgs/msg/ToolDataMsg.msg"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ur_msgs/srv" TYPE FILE FILES
    "/home/cn/NeuralSymbol_AI_for_task2/src/Universal_Robots_ROS_Driver/ur_msgs/srv/SetPayload.srv"
    "/home/cn/NeuralSymbol_AI_for_task2/src/Universal_Robots_ROS_Driver/ur_msgs/srv/SetSpeedSliderFraction.srv"
    "/home/cn/NeuralSymbol_AI_for_task2/src/Universal_Robots_ROS_Driver/ur_msgs/srv/SetIO.srv"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ur_msgs/cmake" TYPE FILE FILES "/home/cn/NeuralSymbol_AI_for_task2/build/Universal_Robots_ROS_Driver/ur_msgs/catkin_generated/installspace/ur_msgs-msg-paths.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/cn/NeuralSymbol_AI_for_task2/devel/include/ur_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/cn/NeuralSymbol_AI_for_task2/devel/share/roseus/ros/ur_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/cn/NeuralSymbol_AI_for_task2/devel/share/common-lisp/ros/ur_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/cn/NeuralSymbol_AI_for_task2/devel/share/gennodejs/ros/ur_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/home/cn/anaconda3/envs/NSAI/bin/python3" -m compileall "/home/cn/NeuralSymbol_AI_for_task2/devel/lib/python3/dist-packages/ur_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3/dist-packages" TYPE DIRECTORY FILES "/home/cn/NeuralSymbol_AI_for_task2/devel/lib/python3/dist-packages/ur_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/cn/NeuralSymbol_AI_for_task2/build/Universal_Robots_ROS_Driver/ur_msgs/catkin_generated/installspace/ur_msgs.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ur_msgs/cmake" TYPE FILE FILES "/home/cn/NeuralSymbol_AI_for_task2/build/Universal_Robots_ROS_Driver/ur_msgs/catkin_generated/installspace/ur_msgs-msg-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ur_msgs/cmake" TYPE FILE FILES
    "/home/cn/NeuralSymbol_AI_for_task2/build/Universal_Robots_ROS_Driver/ur_msgs/catkin_generated/installspace/ur_msgsConfig.cmake"
    "/home/cn/NeuralSymbol_AI_for_task2/build/Universal_Robots_ROS_Driver/ur_msgs/catkin_generated/installspace/ur_msgsConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ur_msgs" TYPE FILE FILES "/home/cn/NeuralSymbol_AI_for_task2/src/Universal_Robots_ROS_Driver/ur_msgs/package.xml")
endif()

