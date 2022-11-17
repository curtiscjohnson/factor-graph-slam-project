# Install script for directory: /home/kody/SLAM/factor-graph-slam-project/code/gtsam/gtsam-develop/gtsam

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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/gtsam" TYPE FILE FILES
    "/home/kody/SLAM/factor-graph-slam-project/code/gtsam/gtsam-develop/gtsam/global_includes.h"
    "/home/kody/SLAM/factor-graph-slam-project/code/gtsam/gtsam-develop/gtsam/precompiled_header.h"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/gtsam" TYPE FILE FILES
    "/home/kody/SLAM/factor-graph-slam-project/code/gtsam/gtsam-develop/python/gtsam/config.h"
    "/home/kody/SLAM/factor-graph-slam-project/code/gtsam/gtsam-develop/python/gtsam/dllexport.h"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  foreach(file
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libgtsam.so.4.2.0"
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libgtsam.so.4"
      )
    if(EXISTS "${file}" AND
       NOT IS_SYMLINK "${file}")
      file(RPATH_CHECK
           FILE "${file}"
           RPATH "")
    endif()
  endforeach()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES
    "/home/kody/SLAM/factor-graph-slam-project/code/gtsam/gtsam-develop/python/gtsam/libgtsam.so.4.2.0"
    "/home/kody/SLAM/factor-graph-slam-project/code/gtsam/gtsam-develop/python/gtsam/libgtsam.so.4"
    )
  foreach(file
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libgtsam.so.4.2.0"
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libgtsam.so.4"
      )
    if(EXISTS "${file}" AND
       NOT IS_SYMLINK "${file}")
      file(RPATH_CHANGE
           FILE "${file}"
           OLD_RPATH "/home/kody/SLAM/factor-graph-slam-project/code/gtsam/gtsam-develop/python/gtsam/3rdparty/metis/libmetis:"
           NEW_RPATH "")
      if(CMAKE_INSTALL_DO_STRIP)
        execute_process(COMMAND "/usr/bin/strip" "${file}")
      endif()
    endif()
  endforeach()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libgtsam.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libgtsam.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libgtsam.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/kody/SLAM/factor-graph-slam-project/code/gtsam/gtsam-develop/python/gtsam/libgtsam.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libgtsam.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libgtsam.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libgtsam.so"
         OLD_RPATH "/home/kody/SLAM/factor-graph-slam-project/code/gtsam/gtsam-develop/python/gtsam/3rdparty/metis/libmetis:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libgtsam.so")
    endif()
  endif()
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/kody/SLAM/factor-graph-slam-project/code/gtsam/gtsam-develop/python/gtsam/3rdparty/cmake_install.cmake")
  include("/home/kody/SLAM/factor-graph-slam-project/code/gtsam/gtsam-develop/python/gtsam/base/cmake_install.cmake")
  include("/home/kody/SLAM/factor-graph-slam-project/code/gtsam/gtsam-develop/python/gtsam/basis/cmake_install.cmake")
  include("/home/kody/SLAM/factor-graph-slam-project/code/gtsam/gtsam-develop/python/gtsam/geometry/cmake_install.cmake")
  include("/home/kody/SLAM/factor-graph-slam-project/code/gtsam/gtsam-develop/python/gtsam/inference/cmake_install.cmake")
  include("/home/kody/SLAM/factor-graph-slam-project/code/gtsam/gtsam-develop/python/gtsam/symbolic/cmake_install.cmake")
  include("/home/kody/SLAM/factor-graph-slam-project/code/gtsam/gtsam-develop/python/gtsam/discrete/cmake_install.cmake")
  include("/home/kody/SLAM/factor-graph-slam-project/code/gtsam/gtsam-develop/python/gtsam/hybrid/cmake_install.cmake")
  include("/home/kody/SLAM/factor-graph-slam-project/code/gtsam/gtsam-develop/python/gtsam/linear/cmake_install.cmake")
  include("/home/kody/SLAM/factor-graph-slam-project/code/gtsam/gtsam-develop/python/gtsam/nonlinear/cmake_install.cmake")
  include("/home/kody/SLAM/factor-graph-slam-project/code/gtsam/gtsam-develop/python/gtsam/sam/cmake_install.cmake")
  include("/home/kody/SLAM/factor-graph-slam-project/code/gtsam/gtsam-develop/python/gtsam/sfm/cmake_install.cmake")
  include("/home/kody/SLAM/factor-graph-slam-project/code/gtsam/gtsam-develop/python/gtsam/slam/cmake_install.cmake")
  include("/home/kody/SLAM/factor-graph-slam-project/code/gtsam/gtsam-develop/python/gtsam/navigation/cmake_install.cmake")

endif()
