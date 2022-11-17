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
include examples/CMakeFiles/HMMExample.dir/depend.make

# Include the progress variables for this target.
include examples/CMakeFiles/HMMExample.dir/progress.make

# Include the compile flags for this target's objects.
include examples/CMakeFiles/HMMExample.dir/flags.make

examples/CMakeFiles/HMMExample.dir/HMMExample.cpp.o: examples/CMakeFiles/HMMExample.dir/flags.make
examples/CMakeFiles/HMMExample.dir/HMMExample.cpp.o: ../examples/HMMExample.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/kody/SLAM/factor-graph-slam-project/code/gtsam/gtsam-develop/python/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object examples/CMakeFiles/HMMExample.dir/HMMExample.cpp.o"
	cd /home/kody/SLAM/factor-graph-slam-project/code/gtsam/gtsam-develop/python/examples && /usr/bin/c++  $(CXX_DEFINES) -DTOPSRCDIR=\"/home/kody/SLAM/factor-graph-slam-project/code/gtsam/gtsam-develop\" $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/HMMExample.dir/HMMExample.cpp.o -c /home/kody/SLAM/factor-graph-slam-project/code/gtsam/gtsam-develop/examples/HMMExample.cpp

examples/CMakeFiles/HMMExample.dir/HMMExample.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/HMMExample.dir/HMMExample.cpp.i"
	cd /home/kody/SLAM/factor-graph-slam-project/code/gtsam/gtsam-develop/python/examples && /usr/bin/c++ $(CXX_DEFINES) -DTOPSRCDIR=\"/home/kody/SLAM/factor-graph-slam-project/code/gtsam/gtsam-develop\" $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/kody/SLAM/factor-graph-slam-project/code/gtsam/gtsam-develop/examples/HMMExample.cpp > CMakeFiles/HMMExample.dir/HMMExample.cpp.i

examples/CMakeFiles/HMMExample.dir/HMMExample.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/HMMExample.dir/HMMExample.cpp.s"
	cd /home/kody/SLAM/factor-graph-slam-project/code/gtsam/gtsam-develop/python/examples && /usr/bin/c++ $(CXX_DEFINES) -DTOPSRCDIR=\"/home/kody/SLAM/factor-graph-slam-project/code/gtsam/gtsam-develop\" $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/kody/SLAM/factor-graph-slam-project/code/gtsam/gtsam-develop/examples/HMMExample.cpp -o CMakeFiles/HMMExample.dir/HMMExample.cpp.s

# Object files for target HMMExample
HMMExample_OBJECTS = \
"CMakeFiles/HMMExample.dir/HMMExample.cpp.o"

# External object files for target HMMExample
HMMExample_EXTERNAL_OBJECTS =

examples/HMMExample: examples/CMakeFiles/HMMExample.dir/HMMExample.cpp.o
examples/HMMExample: examples/CMakeFiles/HMMExample.dir/build.make
examples/HMMExample: gtsam_unstable/libgtsam_unstable.so.4.2.0
examples/HMMExample: /usr/lib/x86_64-linux-gnu/libboost_program_options.so.1.71.0
examples/HMMExample: gtsam/libgtsam.so.4.2.0
examples/HMMExample: /usr/lib/x86_64-linux-gnu/libboost_serialization.so.1.71.0
examples/HMMExample: /usr/lib/x86_64-linux-gnu/libboost_system.so.1.71.0
examples/HMMExample: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so.1.71.0
examples/HMMExample: /usr/lib/x86_64-linux-gnu/libboost_thread.so.1.71.0
examples/HMMExample: /usr/lib/x86_64-linux-gnu/libboost_atomic.so.1.71.0
examples/HMMExample: /usr/lib/x86_64-linux-gnu/libboost_date_time.so.1.71.0
examples/HMMExample: /usr/lib/x86_64-linux-gnu/libboost_regex.so.1.71.0
examples/HMMExample: /usr/lib/x86_64-linux-gnu/libboost_timer.so.1.71.0
examples/HMMExample: /usr/lib/x86_64-linux-gnu/libboost_chrono.so.1.71.0
examples/HMMExample: gtsam/3rdparty/metis/libmetis/libmetis-gtsam.so
examples/HMMExample: examples/CMakeFiles/HMMExample.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/kody/SLAM/factor-graph-slam-project/code/gtsam/gtsam-develop/python/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable HMMExample"
	cd /home/kody/SLAM/factor-graph-slam-project/code/gtsam/gtsam-develop/python/examples && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/HMMExample.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
examples/CMakeFiles/HMMExample.dir/build: examples/HMMExample

.PHONY : examples/CMakeFiles/HMMExample.dir/build

examples/CMakeFiles/HMMExample.dir/clean:
	cd /home/kody/SLAM/factor-graph-slam-project/code/gtsam/gtsam-develop/python/examples && $(CMAKE_COMMAND) -P CMakeFiles/HMMExample.dir/cmake_clean.cmake
.PHONY : examples/CMakeFiles/HMMExample.dir/clean

examples/CMakeFiles/HMMExample.dir/depend:
	cd /home/kody/SLAM/factor-graph-slam-project/code/gtsam/gtsam-develop/python && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/kody/SLAM/factor-graph-slam-project/code/gtsam/gtsam-develop /home/kody/SLAM/factor-graph-slam-project/code/gtsam/gtsam-develop/examples /home/kody/SLAM/factor-graph-slam-project/code/gtsam/gtsam-develop/python /home/kody/SLAM/factor-graph-slam-project/code/gtsam/gtsam-develop/python/examples /home/kody/SLAM/factor-graph-slam-project/code/gtsam/gtsam-develop/python/examples/CMakeFiles/HMMExample.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : examples/CMakeFiles/HMMExample.dir/depend

