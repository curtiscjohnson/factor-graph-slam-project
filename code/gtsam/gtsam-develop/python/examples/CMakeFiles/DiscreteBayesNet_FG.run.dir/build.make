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

# Utility rule file for DiscreteBayesNet_FG.run.

# Include the progress variables for this target.
include examples/CMakeFiles/DiscreteBayesNet_FG.run.dir/progress.make

examples/CMakeFiles/DiscreteBayesNet_FG.run: examples/DiscreteBayesNet_FG
	cd /home/kody/SLAM/factor-graph-slam-project/code/gtsam/gtsam-develop/python/examples && ./DiscreteBayesNet_FG

DiscreteBayesNet_FG.run: examples/CMakeFiles/DiscreteBayesNet_FG.run
DiscreteBayesNet_FG.run: examples/CMakeFiles/DiscreteBayesNet_FG.run.dir/build.make

.PHONY : DiscreteBayesNet_FG.run

# Rule to build all files generated by this target.
examples/CMakeFiles/DiscreteBayesNet_FG.run.dir/build: DiscreteBayesNet_FG.run

.PHONY : examples/CMakeFiles/DiscreteBayesNet_FG.run.dir/build

examples/CMakeFiles/DiscreteBayesNet_FG.run.dir/clean:
	cd /home/kody/SLAM/factor-graph-slam-project/code/gtsam/gtsam-develop/python/examples && $(CMAKE_COMMAND) -P CMakeFiles/DiscreteBayesNet_FG.run.dir/cmake_clean.cmake
.PHONY : examples/CMakeFiles/DiscreteBayesNet_FG.run.dir/clean

examples/CMakeFiles/DiscreteBayesNet_FG.run.dir/depend:
	cd /home/kody/SLAM/factor-graph-slam-project/code/gtsam/gtsam-develop/python && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/kody/SLAM/factor-graph-slam-project/code/gtsam/gtsam-develop /home/kody/SLAM/factor-graph-slam-project/code/gtsam/gtsam-develop/examples /home/kody/SLAM/factor-graph-slam-project/code/gtsam/gtsam-develop/python /home/kody/SLAM/factor-graph-slam-project/code/gtsam/gtsam-develop/python/examples /home/kody/SLAM/factor-graph-slam-project/code/gtsam/gtsam-develop/python/examples/CMakeFiles/DiscreteBayesNet_FG.run.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : examples/CMakeFiles/DiscreteBayesNet_FG.run.dir/depend
