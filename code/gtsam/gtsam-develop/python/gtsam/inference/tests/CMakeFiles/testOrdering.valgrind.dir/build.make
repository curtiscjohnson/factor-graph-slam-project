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

# Utility rule file for testOrdering.valgrind.

# Include the progress variables for this target.
include gtsam/inference/tests/CMakeFiles/testOrdering.valgrind.dir/progress.make

gtsam/inference/tests/CMakeFiles/testOrdering.valgrind: gtsam/inference/tests/testOrdering
	cd /home/kody/SLAM/factor-graph-slam-project/code/gtsam/gtsam-develop/python/gtsam/inference/tests && valgrind --error-exitcode=1 /home/kody/SLAM/factor-graph-slam-project/code/gtsam/gtsam-develop/python/gtsam/inference/tests/testOrdering

testOrdering.valgrind: gtsam/inference/tests/CMakeFiles/testOrdering.valgrind
testOrdering.valgrind: gtsam/inference/tests/CMakeFiles/testOrdering.valgrind.dir/build.make

.PHONY : testOrdering.valgrind

# Rule to build all files generated by this target.
gtsam/inference/tests/CMakeFiles/testOrdering.valgrind.dir/build: testOrdering.valgrind

.PHONY : gtsam/inference/tests/CMakeFiles/testOrdering.valgrind.dir/build

gtsam/inference/tests/CMakeFiles/testOrdering.valgrind.dir/clean:
	cd /home/kody/SLAM/factor-graph-slam-project/code/gtsam/gtsam-develop/python/gtsam/inference/tests && $(CMAKE_COMMAND) -P CMakeFiles/testOrdering.valgrind.dir/cmake_clean.cmake
.PHONY : gtsam/inference/tests/CMakeFiles/testOrdering.valgrind.dir/clean

gtsam/inference/tests/CMakeFiles/testOrdering.valgrind.dir/depend:
	cd /home/kody/SLAM/factor-graph-slam-project/code/gtsam/gtsam-develop/python && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/kody/SLAM/factor-graph-slam-project/code/gtsam/gtsam-develop /home/kody/SLAM/factor-graph-slam-project/code/gtsam/gtsam-develop/gtsam/inference/tests /home/kody/SLAM/factor-graph-slam-project/code/gtsam/gtsam-develop/python /home/kody/SLAM/factor-graph-slam-project/code/gtsam/gtsam-develop/python/gtsam/inference/tests /home/kody/SLAM/factor-graph-slam-project/code/gtsam/gtsam-develop/python/gtsam/inference/tests/CMakeFiles/testOrdering.valgrind.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : gtsam/inference/tests/CMakeFiles/testOrdering.valgrind.dir/depend

