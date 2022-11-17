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
include tests/CMakeFiles/testSubgraphSolver.dir/depend.make

# Include the progress variables for this target.
include tests/CMakeFiles/testSubgraphSolver.dir/progress.make

# Include the compile flags for this target's objects.
include tests/CMakeFiles/testSubgraphSolver.dir/flags.make

tests/CMakeFiles/testSubgraphSolver.dir/testSubgraphSolver.cpp.o: tests/CMakeFiles/testSubgraphSolver.dir/flags.make
tests/CMakeFiles/testSubgraphSolver.dir/testSubgraphSolver.cpp.o: ../tests/testSubgraphSolver.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/kody/SLAM/factor-graph-slam-project/code/gtsam/gtsam-develop/python/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object tests/CMakeFiles/testSubgraphSolver.dir/testSubgraphSolver.cpp.o"
	cd /home/kody/SLAM/factor-graph-slam-project/code/gtsam/gtsam-develop/python/tests && /usr/bin/c++  $(CXX_DEFINES) -DTOPSRCDIR=\"/home/kody/SLAM/factor-graph-slam-project/code/gtsam/gtsam-develop\" $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/testSubgraphSolver.dir/testSubgraphSolver.cpp.o -c /home/kody/SLAM/factor-graph-slam-project/code/gtsam/gtsam-develop/tests/testSubgraphSolver.cpp

tests/CMakeFiles/testSubgraphSolver.dir/testSubgraphSolver.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/testSubgraphSolver.dir/testSubgraphSolver.cpp.i"
	cd /home/kody/SLAM/factor-graph-slam-project/code/gtsam/gtsam-develop/python/tests && /usr/bin/c++ $(CXX_DEFINES) -DTOPSRCDIR=\"/home/kody/SLAM/factor-graph-slam-project/code/gtsam/gtsam-develop\" $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/kody/SLAM/factor-graph-slam-project/code/gtsam/gtsam-develop/tests/testSubgraphSolver.cpp > CMakeFiles/testSubgraphSolver.dir/testSubgraphSolver.cpp.i

tests/CMakeFiles/testSubgraphSolver.dir/testSubgraphSolver.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/testSubgraphSolver.dir/testSubgraphSolver.cpp.s"
	cd /home/kody/SLAM/factor-graph-slam-project/code/gtsam/gtsam-develop/python/tests && /usr/bin/c++ $(CXX_DEFINES) -DTOPSRCDIR=\"/home/kody/SLAM/factor-graph-slam-project/code/gtsam/gtsam-develop\" $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/kody/SLAM/factor-graph-slam-project/code/gtsam/gtsam-develop/tests/testSubgraphSolver.cpp -o CMakeFiles/testSubgraphSolver.dir/testSubgraphSolver.cpp.s

# Object files for target testSubgraphSolver
testSubgraphSolver_OBJECTS = \
"CMakeFiles/testSubgraphSolver.dir/testSubgraphSolver.cpp.o"

# External object files for target testSubgraphSolver
testSubgraphSolver_EXTERNAL_OBJECTS =

tests/testSubgraphSolver: tests/CMakeFiles/testSubgraphSolver.dir/testSubgraphSolver.cpp.o
tests/testSubgraphSolver: tests/CMakeFiles/testSubgraphSolver.dir/build.make
tests/testSubgraphSolver: CppUnitLite/libCppUnitLite.a
tests/testSubgraphSolver: gtsam/libgtsam.so.4.2.0
tests/testSubgraphSolver: /usr/lib/x86_64-linux-gnu/libboost_serialization.so.1.71.0
tests/testSubgraphSolver: /usr/lib/x86_64-linux-gnu/libboost_system.so.1.71.0
tests/testSubgraphSolver: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so.1.71.0
tests/testSubgraphSolver: /usr/lib/x86_64-linux-gnu/libboost_thread.so.1.71.0
tests/testSubgraphSolver: /usr/lib/x86_64-linux-gnu/libboost_atomic.so.1.71.0
tests/testSubgraphSolver: /usr/lib/x86_64-linux-gnu/libboost_date_time.so.1.71.0
tests/testSubgraphSolver: /usr/lib/x86_64-linux-gnu/libboost_regex.so.1.71.0
tests/testSubgraphSolver: /usr/lib/x86_64-linux-gnu/libboost_timer.so.1.71.0
tests/testSubgraphSolver: /usr/lib/x86_64-linux-gnu/libboost_chrono.so.1.71.0
tests/testSubgraphSolver: gtsam/3rdparty/metis/libmetis/libmetis-gtsam.so
tests/testSubgraphSolver: tests/CMakeFiles/testSubgraphSolver.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/kody/SLAM/factor-graph-slam-project/code/gtsam/gtsam-develop/python/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable testSubgraphSolver"
	cd /home/kody/SLAM/factor-graph-slam-project/code/gtsam/gtsam-develop/python/tests && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/testSubgraphSolver.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
tests/CMakeFiles/testSubgraphSolver.dir/build: tests/testSubgraphSolver

.PHONY : tests/CMakeFiles/testSubgraphSolver.dir/build

tests/CMakeFiles/testSubgraphSolver.dir/clean:
	cd /home/kody/SLAM/factor-graph-slam-project/code/gtsam/gtsam-develop/python/tests && $(CMAKE_COMMAND) -P CMakeFiles/testSubgraphSolver.dir/cmake_clean.cmake
.PHONY : tests/CMakeFiles/testSubgraphSolver.dir/clean

tests/CMakeFiles/testSubgraphSolver.dir/depend:
	cd /home/kody/SLAM/factor-graph-slam-project/code/gtsam/gtsam-develop/python && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/kody/SLAM/factor-graph-slam-project/code/gtsam/gtsam-develop /home/kody/SLAM/factor-graph-slam-project/code/gtsam/gtsam-develop/tests /home/kody/SLAM/factor-graph-slam-project/code/gtsam/gtsam-develop/python /home/kody/SLAM/factor-graph-slam-project/code/gtsam/gtsam-develop/python/tests /home/kody/SLAM/factor-graph-slam-project/code/gtsam/gtsam-develop/python/tests/CMakeFiles/testSubgraphSolver.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : tests/CMakeFiles/testSubgraphSolver.dir/depend

