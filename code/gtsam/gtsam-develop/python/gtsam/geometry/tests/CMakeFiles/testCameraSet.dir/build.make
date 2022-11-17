# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/kody/SLAM/factor-graph-slam-project/code/gtsam/gtsam-develop

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/kody/SLAM/factor-graph-slam-project/code/gtsam/gtsam-develop/python

# Include any dependencies generated for this target.
include gtsam/geometry/tests/CMakeFiles/testCameraSet.dir/depend.make

# Include the progress variables for this target.
include gtsam/geometry/tests/CMakeFiles/testCameraSet.dir/progress.make

# Include the compile flags for this target's objects.
include gtsam/geometry/tests/CMakeFiles/testCameraSet.dir/flags.make

gtsam/geometry/tests/CMakeFiles/testCameraSet.dir/testCameraSet.cpp.o: gtsam/geometry/tests/CMakeFiles/testCameraSet.dir/flags.make
gtsam/geometry/tests/CMakeFiles/testCameraSet.dir/testCameraSet.cpp.o: ../gtsam/geometry/tests/testCameraSet.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/kody/SLAM/factor-graph-slam-project/code/gtsam/gtsam-develop/python/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object gtsam/geometry/tests/CMakeFiles/testCameraSet.dir/testCameraSet.cpp.o"
	cd /home/kody/SLAM/factor-graph-slam-project/code/gtsam/gtsam-develop/python/gtsam/geometry/tests && /usr/bin/c++  $(CXX_DEFINES) -DTOPSRCDIR=\"/home/kody/SLAM/factor-graph-slam-project/code/gtsam/gtsam-develop\" $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/testCameraSet.dir/testCameraSet.cpp.o -c /home/kody/SLAM/factor-graph-slam-project/code/gtsam/gtsam-develop/gtsam/geometry/tests/testCameraSet.cpp

gtsam/geometry/tests/CMakeFiles/testCameraSet.dir/testCameraSet.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/testCameraSet.dir/testCameraSet.cpp.i"
	cd /home/kody/SLAM/factor-graph-slam-project/code/gtsam/gtsam-develop/python/gtsam/geometry/tests && /usr/bin/c++ $(CXX_DEFINES) -DTOPSRCDIR=\"/home/kody/SLAM/factor-graph-slam-project/code/gtsam/gtsam-develop\" $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/kody/SLAM/factor-graph-slam-project/code/gtsam/gtsam-develop/gtsam/geometry/tests/testCameraSet.cpp > CMakeFiles/testCameraSet.dir/testCameraSet.cpp.i

gtsam/geometry/tests/CMakeFiles/testCameraSet.dir/testCameraSet.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/testCameraSet.dir/testCameraSet.cpp.s"
	cd /home/kody/SLAM/factor-graph-slam-project/code/gtsam/gtsam-develop/python/gtsam/geometry/tests && /usr/bin/c++ $(CXX_DEFINES) -DTOPSRCDIR=\"/home/kody/SLAM/factor-graph-slam-project/code/gtsam/gtsam-develop\" $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/kody/SLAM/factor-graph-slam-project/code/gtsam/gtsam-develop/gtsam/geometry/tests/testCameraSet.cpp -o CMakeFiles/testCameraSet.dir/testCameraSet.cpp.s

# Object files for target testCameraSet
testCameraSet_OBJECTS = \
"CMakeFiles/testCameraSet.dir/testCameraSet.cpp.o"

# External object files for target testCameraSet
testCameraSet_EXTERNAL_OBJECTS =

gtsam/geometry/tests/testCameraSet: gtsam/geometry/tests/CMakeFiles/testCameraSet.dir/testCameraSet.cpp.o
gtsam/geometry/tests/testCameraSet: gtsam/geometry/tests/CMakeFiles/testCameraSet.dir/build.make
gtsam/geometry/tests/testCameraSet: CppUnitLite/libCppUnitLite.a
gtsam/geometry/tests/testCameraSet: gtsam/libgtsam.so.4.2.0
gtsam/geometry/tests/testCameraSet: /usr/lib/x86_64-linux-gnu/libboost_serialization.so.1.71.0
gtsam/geometry/tests/testCameraSet: /usr/lib/x86_64-linux-gnu/libboost_system.so.1.71.0
gtsam/geometry/tests/testCameraSet: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so.1.71.0
gtsam/geometry/tests/testCameraSet: /usr/lib/x86_64-linux-gnu/libboost_thread.so.1.71.0
gtsam/geometry/tests/testCameraSet: /usr/lib/x86_64-linux-gnu/libboost_atomic.so.1.71.0
gtsam/geometry/tests/testCameraSet: /usr/lib/x86_64-linux-gnu/libboost_date_time.so.1.71.0
gtsam/geometry/tests/testCameraSet: /usr/lib/x86_64-linux-gnu/libboost_regex.so.1.71.0
gtsam/geometry/tests/testCameraSet: /usr/lib/x86_64-linux-gnu/libboost_timer.so.1.71.0
gtsam/geometry/tests/testCameraSet: /usr/lib/x86_64-linux-gnu/libboost_chrono.so.1.71.0
gtsam/geometry/tests/testCameraSet: gtsam/3rdparty/metis/libmetis/libmetis-gtsam.so
gtsam/geometry/tests/testCameraSet: gtsam/geometry/tests/CMakeFiles/testCameraSet.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/kody/SLAM/factor-graph-slam-project/code/gtsam/gtsam-develop/python/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable testCameraSet"
	cd /home/kody/SLAM/factor-graph-slam-project/code/gtsam/gtsam-develop/python/gtsam/geometry/tests && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/testCameraSet.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
gtsam/geometry/tests/CMakeFiles/testCameraSet.dir/build: gtsam/geometry/tests/testCameraSet

.PHONY : gtsam/geometry/tests/CMakeFiles/testCameraSet.dir/build

gtsam/geometry/tests/CMakeFiles/testCameraSet.dir/clean:
	cd /home/kody/SLAM/factor-graph-slam-project/code/gtsam/gtsam-develop/python/gtsam/geometry/tests && $(CMAKE_COMMAND) -P CMakeFiles/testCameraSet.dir/cmake_clean.cmake
.PHONY : gtsam/geometry/tests/CMakeFiles/testCameraSet.dir/clean

gtsam/geometry/tests/CMakeFiles/testCameraSet.dir/depend:
	cd /home/kody/SLAM/factor-graph-slam-project/code/gtsam/gtsam-develop/python && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/kody/SLAM/factor-graph-slam-project/code/gtsam/gtsam-develop /home/kody/SLAM/factor-graph-slam-project/code/gtsam/gtsam-develop/gtsam/geometry/tests /home/kody/SLAM/factor-graph-slam-project/code/gtsam/gtsam-develop/python /home/kody/SLAM/factor-graph-slam-project/code/gtsam/gtsam-develop/python/gtsam/geometry/tests /home/kody/SLAM/factor-graph-slam-project/code/gtsam/gtsam-develop/python/gtsam/geometry/tests/CMakeFiles/testCameraSet.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : gtsam/geometry/tests/CMakeFiles/testCameraSet.dir/depend

