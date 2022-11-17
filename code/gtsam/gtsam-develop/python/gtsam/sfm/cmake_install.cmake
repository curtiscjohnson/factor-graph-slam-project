# Install script for directory: /home/kody/SLAM/factor-graph-slam-project/code/gtsam/gtsam-develop/gtsam/sfm

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/gtsam/sfm" TYPE FILE FILES
    "/home/kody/SLAM/factor-graph-slam-project/code/gtsam/gtsam-develop/gtsam/sfm/BinaryMeasurement.h"
    "/home/kody/SLAM/factor-graph-slam-project/code/gtsam/gtsam-develop/gtsam/sfm/DsfTrackGenerator.h"
    "/home/kody/SLAM/factor-graph-slam-project/code/gtsam/gtsam-develop/gtsam/sfm/MFAS.h"
    "/home/kody/SLAM/factor-graph-slam-project/code/gtsam/gtsam-develop/gtsam/sfm/SfmData.h"
    "/home/kody/SLAM/factor-graph-slam-project/code/gtsam/gtsam-develop/gtsam/sfm/SfmTrack.h"
    "/home/kody/SLAM/factor-graph-slam-project/code/gtsam/gtsam-develop/gtsam/sfm/ShonanAveraging.h"
    "/home/kody/SLAM/factor-graph-slam-project/code/gtsam/gtsam-develop/gtsam/sfm/ShonanFactor.h"
    "/home/kody/SLAM/factor-graph-slam-project/code/gtsam/gtsam-develop/gtsam/sfm/ShonanGaugeFactor.h"
    "/home/kody/SLAM/factor-graph-slam-project/code/gtsam/gtsam-develop/gtsam/sfm/TranslationFactor.h"
    "/home/kody/SLAM/factor-graph-slam-project/code/gtsam/gtsam-develop/gtsam/sfm/TranslationRecovery.h"
    )
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/kody/SLAM/factor-graph-slam-project/code/gtsam/gtsam-develop/python/gtsam/sfm/tests/cmake_install.cmake")

endif()
