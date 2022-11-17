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
include examples/CMakeFiles/SelfCalibrationExample.dir/depend.make

# Include the progress variables for this target.
include examples/CMakeFiles/SelfCalibrationExample.dir/progress.make

# Include the compile flags for this target's objects.
include examples/CMakeFiles/SelfCalibrationExample.dir/flags.make

examples/CMakeFiles/SelfCalibrationExample.dir/SelfCalibrationExample.cpp.o: examples/CMakeFiles/SelfCalibrationExample.dir/flags.make
examples/CMakeFiles/SelfCalibrationExample.dir/SelfCalibrationExample.cpp.o: ../examples/SelfCalibrationExample.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/kody/SLAM/factor-graph-slam-project/code/gtsam/gtsam-develop/python/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object examples/CMakeFiles/SelfCalibrationExample.dir/SelfCalibrationExample.cpp.o"
	cd /home/kody/SLAM/factor-graph-slam-project/code/gtsam/gtsam-develop/python/examples && /usr/bin/c++  $(CXX_DEFINES) -DTOPSRCDIR=\"/home/kody/SLAM/factor-graph-slam-project/code/gtsam/gtsam-develop\" $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/SelfCalibrationExample.dir/SelfCalibrationExample.cpp.o -c /home/kody/SLAM/factor-graph-slam-project/code/gtsam/gtsam-develop/examples/SelfCalibrationExample.cpp

examples/CMakeFiles/SelfCalibrationExample.dir/SelfCalibrationExample.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/SelfCalibrationExample.dir/SelfCalibrationExample.cpp.i"
	cd /home/kody/SLAM/factor-graph-slam-project/code/gtsam/gtsam-develop/python/examples && /usr/bin/c++ $(CXX_DEFINES) -DTOPSRCDIR=\"/home/kody/SLAM/factor-graph-slam-project/code/gtsam/gtsam-develop\" $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/kody/SLAM/factor-graph-slam-project/code/gtsam/gtsam-develop/examples/SelfCalibrationExample.cpp > CMakeFiles/SelfCalibrationExample.dir/SelfCalibrationExample.cpp.i

examples/CMakeFiles/SelfCalibrationExample.dir/SelfCalibrationExample.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/SelfCalibrationExample.dir/SelfCalibrationExample.cpp.s"
	cd /home/kody/SLAM/factor-graph-slam-project/code/gtsam/gtsam-develop/python/examples && /usr/bin/c++ $(CXX_DEFINES) -DTOPSRCDIR=\"/home/kody/SLAM/factor-graph-slam-project/code/gtsam/gtsam-develop\" $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/kody/SLAM/factor-graph-slam-project/code/gtsam/gtsam-develop/examples/SelfCalibrationExample.cpp -o CMakeFiles/SelfCalibrationExample.dir/SelfCalibrationExample.cpp.s

# Object files for target SelfCalibrationExample
SelfCalibrationExample_OBJECTS = \
"CMakeFiles/SelfCalibrationExample.dir/SelfCalibrationExample.cpp.o"

# External object files for target SelfCalibrationExample
SelfCalibrationExample_EXTERNAL_OBJECTS =

examples/SelfCalibrationExample: examples/CMakeFiles/SelfCalibrationExample.dir/SelfCalibrationExample.cpp.o
examples/SelfCalibrationExample: examples/CMakeFiles/SelfCalibrationExample.dir/build.make
examples/SelfCalibrationExample: gtsam_unstable/libgtsam_unstable.so.4.2.0
examples/SelfCalibrationExample: /usr/lib/x86_64-linux-gnu/libboost_program_options.so.1.71.0
examples/SelfCalibrationExample: gtsam/libgtsam.so.4.2.0
examples/SelfCalibrationExample: /usr/lib/x86_64-linux-gnu/libboost_serialization.so.1.71.0
examples/SelfCalibrationExample: /usr/lib/x86_64-linux-gnu/libboost_system.so.1.71.0
examples/SelfCalibrationExample: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so.1.71.0
examples/SelfCalibrationExample: /usr/lib/x86_64-linux-gnu/libboost_thread.so.1.71.0
examples/SelfCalibrationExample: /usr/lib/x86_64-linux-gnu/libboost_atomic.so.1.71.0
examples/SelfCalibrationExample: /usr/lib/x86_64-linux-gnu/libboost_date_time.so.1.71.0
examples/SelfCalibrationExample: /usr/lib/x86_64-linux-gnu/libboost_regex.so.1.71.0
examples/SelfCalibrationExample: /usr/lib/x86_64-linux-gnu/libboost_timer.so.1.71.0
examples/SelfCalibrationExample: /usr/lib/x86_64-linux-gnu/libboost_chrono.so.1.71.0
examples/SelfCalibrationExample: gtsam/3rdparty/metis/libmetis/libmetis-gtsam.so
examples/SelfCalibrationExample: examples/CMakeFiles/SelfCalibrationExample.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/kody/SLAM/factor-graph-slam-project/code/gtsam/gtsam-develop/python/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable SelfCalibrationExample"
	cd /home/kody/SLAM/factor-graph-slam-project/code/gtsam/gtsam-develop/python/examples && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/SelfCalibrationExample.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
examples/CMakeFiles/SelfCalibrationExample.dir/build: examples/SelfCalibrationExample

.PHONY : examples/CMakeFiles/SelfCalibrationExample.dir/build

examples/CMakeFiles/SelfCalibrationExample.dir/clean:
	cd /home/kody/SLAM/factor-graph-slam-project/code/gtsam/gtsam-develop/python/examples && $(CMAKE_COMMAND) -P CMakeFiles/SelfCalibrationExample.dir/cmake_clean.cmake
.PHONY : examples/CMakeFiles/SelfCalibrationExample.dir/clean

examples/CMakeFiles/SelfCalibrationExample.dir/depend:
	cd /home/kody/SLAM/factor-graph-slam-project/code/gtsam/gtsam-develop/python && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/kody/SLAM/factor-graph-slam-project/code/gtsam/gtsam-develop /home/kody/SLAM/factor-graph-slam-project/code/gtsam/gtsam-develop/examples /home/kody/SLAM/factor-graph-slam-project/code/gtsam/gtsam-develop/python /home/kody/SLAM/factor-graph-slam-project/code/gtsam/gtsam-develop/python/examples /home/kody/SLAM/factor-graph-slam-project/code/gtsam/gtsam-develop/python/examples/CMakeFiles/SelfCalibrationExample.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : examples/CMakeFiles/SelfCalibrationExample.dir/depend

