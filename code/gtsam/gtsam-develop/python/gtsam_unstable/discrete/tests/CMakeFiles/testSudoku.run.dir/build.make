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

# Utility rule file for testSudoku.run.

# Include the progress variables for this target.
include gtsam_unstable/discrete/tests/CMakeFiles/testSudoku.run.dir/progress.make

gtsam_unstable/discrete/tests/CMakeFiles/testSudoku.run: gtsam_unstable/discrete/tests/testSudoku
	cd /home/kody/SLAM/factor-graph-slam-project/code/gtsam/gtsam-develop/python/gtsam_unstable/discrete/tests && ./testSudoku

testSudoku.run: gtsam_unstable/discrete/tests/CMakeFiles/testSudoku.run
testSudoku.run: gtsam_unstable/discrete/tests/CMakeFiles/testSudoku.run.dir/build.make

.PHONY : testSudoku.run

# Rule to build all files generated by this target.
gtsam_unstable/discrete/tests/CMakeFiles/testSudoku.run.dir/build: testSudoku.run

.PHONY : gtsam_unstable/discrete/tests/CMakeFiles/testSudoku.run.dir/build

gtsam_unstable/discrete/tests/CMakeFiles/testSudoku.run.dir/clean:
	cd /home/kody/SLAM/factor-graph-slam-project/code/gtsam/gtsam-develop/python/gtsam_unstable/discrete/tests && $(CMAKE_COMMAND) -P CMakeFiles/testSudoku.run.dir/cmake_clean.cmake
.PHONY : gtsam_unstable/discrete/tests/CMakeFiles/testSudoku.run.dir/clean

gtsam_unstable/discrete/tests/CMakeFiles/testSudoku.run.dir/depend:
	cd /home/kody/SLAM/factor-graph-slam-project/code/gtsam/gtsam-develop/python && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/kody/SLAM/factor-graph-slam-project/code/gtsam/gtsam-develop /home/kody/SLAM/factor-graph-slam-project/code/gtsam/gtsam-develop/gtsam_unstable/discrete/tests /home/kody/SLAM/factor-graph-slam-project/code/gtsam/gtsam-develop/python /home/kody/SLAM/factor-graph-slam-project/code/gtsam/gtsam-develop/python/gtsam_unstable/discrete/tests /home/kody/SLAM/factor-graph-slam-project/code/gtsam/gtsam-develop/python/gtsam_unstable/discrete/tests/CMakeFiles/testSudoku.run.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : gtsam_unstable/discrete/tests/CMakeFiles/testSudoku.run.dir/depend

