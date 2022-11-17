# Install script for directory: /home/kody/SLAM/factor-graph-slam-project/code/gtsam/gtsam-develop/cmake

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
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/usr/local/lib/cmake/GTSAMCMakeTools/GTSAMCMakeToolsConfig.cmake;/usr/local/lib/cmake/GTSAMCMakeTools/Config.cmake.in;/usr/local/lib/cmake/GTSAMCMakeTools/dllexport.h.in;/usr/local/lib/cmake/GTSAMCMakeTools/GtsamBuildTypes.cmake;/usr/local/lib/cmake/GTSAMCMakeTools/GtsamMakeConfigFile.cmake;/usr/local/lib/cmake/GTSAMCMakeTools/GtsamTesting.cmake;/usr/local/lib/cmake/GTSAMCMakeTools/GtsamPrinting.cmake;/usr/local/lib/cmake/GTSAMCMakeTools/FindNumPy.cmake;/usr/local/lib/cmake/GTSAMCMakeTools/README.html")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/usr/local/lib/cmake/GTSAMCMakeTools" TYPE FILE FILES
    "/home/kody/SLAM/factor-graph-slam-project/code/gtsam/gtsam-develop/cmake/GTSAMCMakeToolsConfig.cmake"
    "/home/kody/SLAM/factor-graph-slam-project/code/gtsam/gtsam-develop/cmake/Config.cmake.in"
    "/home/kody/SLAM/factor-graph-slam-project/code/gtsam/gtsam-develop/cmake/dllexport.h.in"
    "/home/kody/SLAM/factor-graph-slam-project/code/gtsam/gtsam-develop/cmake/GtsamBuildTypes.cmake"
    "/home/kody/SLAM/factor-graph-slam-project/code/gtsam/gtsam-develop/cmake/GtsamMakeConfigFile.cmake"
    "/home/kody/SLAM/factor-graph-slam-project/code/gtsam/gtsam-develop/cmake/GtsamTesting.cmake"
    "/home/kody/SLAM/factor-graph-slam-project/code/gtsam/gtsam-develop/cmake/GtsamPrinting.cmake"
    "/home/kody/SLAM/factor-graph-slam-project/code/gtsam/gtsam-develop/cmake/FindNumPy.cmake"
    "/home/kody/SLAM/factor-graph-slam-project/code/gtsam/gtsam-develop/cmake/README.html"
    )
endif()

