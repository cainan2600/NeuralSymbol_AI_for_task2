# Install script for directory: /home/cn/NeuralSymbol_AI_for_task2/src/fmauch_universal_robot/ur_real_robot/end_effector

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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/cn/NeuralSymbol_AI_for_task2/build/fmauch_universal_robot/ur_real_robot/end_effector/catkin_generated/installspace/end_effector.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/end_effector/cmake" TYPE FILE FILES
    "/home/cn/NeuralSymbol_AI_for_task2/build/fmauch_universal_robot/ur_real_robot/end_effector/catkin_generated/installspace/end_effectorConfig.cmake"
    "/home/cn/NeuralSymbol_AI_for_task2/build/fmauch_universal_robot/ur_real_robot/end_effector/catkin_generated/installspace/end_effectorConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/end_effector" TYPE FILE FILES "/home/cn/NeuralSymbol_AI_for_task2/src/fmauch_universal_robot/ur_real_robot/end_effector/package.xml")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/end_effector/config" TYPE DIRECTORY FILES "/home/cn/NeuralSymbol_AI_for_task2/src/fmauch_universal_robot/ur_real_robot/end_effector/config/")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/end_effector/launch" TYPE DIRECTORY FILES "/home/cn/NeuralSymbol_AI_for_task2/src/fmauch_universal_robot/ur_real_robot/end_effector/launch/")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/end_effector/meshes" TYPE DIRECTORY FILES "/home/cn/NeuralSymbol_AI_for_task2/src/fmauch_universal_robot/ur_real_robot/end_effector/meshes/")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/end_effector/urdf" TYPE DIRECTORY FILES "/home/cn/NeuralSymbol_AI_for_task2/src/fmauch_universal_robot/ur_real_robot/end_effector/urdf/")
endif()

