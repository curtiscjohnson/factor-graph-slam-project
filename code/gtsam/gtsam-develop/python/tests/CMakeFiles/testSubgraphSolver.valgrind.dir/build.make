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

# Utility rule file for testSubgraphSolver.valgrind.

# Include the progress variables for this target.
include tests/CMakeFiles/testSubgraphSolver.valgrind.dir/progress.make

tests/CMakeFiles/testSubgraphSolver.valgrind: tests/testSubgraphSolver
	cd /home/kody/SLAM/factor-graph-slam-project/code/gtsam/gtsam-develop/python/tests && valgrind --error-exitcode=1 /home/kody/SLAM/factor-graph-slam-project/code/gtsam/gtsam-develop/python/tests/testSubgraphSolver

testSubgraphSolver.valgrind: tests/CMakeFiles/testSubgraphSolver.valgrind
testSubgraphSolver.valgrind: tests/CMakeFiles/testSubgraphSolver.valgrind.dir/build.make

.PHONY : testSubgraphSolver.valgrind

# Rule to build all files generated by this target.
tests/CMakeFiles/testSubgraphSolver.valgrind.dir/build: testSubgraphSolver.valgrind

.PHONY : tests/CMakeFiles/testSubgraphSolver.valgrind.dir/build

tests/CMakeFiles/testSubgraphSolver.valgrind.dir/clean:
	cd /home/kody/SLAM/factor-graph-slam-project/code/gtsam/gtsam-develop/python/tests && $(CMAKE_COMMAND) -P CMakeFiles/testSubgraphSolver.valgrind.dir/cmake_clean.cmake
.PHONY : tests/CMakeFiles/testSubgraphSolver.valgrind.dir/clean

tests/CMakeFiles/testSubgraphSolver.valgrind.dir/depend:
	cd /home/kody/SLAM/factor-graph-slam-project/code/gtsam/gtsam-develop/python && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/kody/SLAM/factor-graph-slam-project/code/gtsam/gtsam-develop /home/kody/SLAM/factor-graph-slam-project/code/gtsam/gtsam-develop/tests /home/kody/SLAM/factor-graph-slam-project/code/gtsam/gtsam-develop/python /home/kody/SLAM/factor-graph-slam-project/code/gtsam/gtsam-develop/python/tests /home/kody/SLAM/factor-graph-slam-project/code/gtsam/gtsam-develop/python/tests/CMakeFiles/testSubgraphSolver.valgrind.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : tests/CMakeFiles/testSubgraphSolver.valgrind.dir/depend
