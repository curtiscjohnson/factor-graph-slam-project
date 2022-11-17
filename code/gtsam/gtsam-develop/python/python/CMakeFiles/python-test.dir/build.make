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

# Utility rule file for python-test.

# Include the progress variables for this target.
include python/CMakeFiles/python-test.dir/progress.make

python/CMakeFiles/python-test: python/gtsam/gtsam.cpython-38-x86_64-linux-gnu.so
python/CMakeFiles/python-test: python/gtsam_unstable/gtsam_unstable.cpython-38-x86_64-linux-gnu.so
python/CMakeFiles/python-test: gtsam/tests/testEssentialMatrixConstraint.py
python/CMakeFiles/python-test: gtsam/tests/testScenarioRunner.py
python/CMakeFiles/python-test: gtsam/tests/test_Cal3Fisheye.py
python/CMakeFiles/python-test: gtsam/tests/test_Cal3Unified.py
python/CMakeFiles/python-test: gtsam/tests/test_DSFMap.py
python/CMakeFiles/python-test: gtsam/tests/test_DecisionTreeFactor.py
python/CMakeFiles/python-test: gtsam/tests/test_DiscreteBayesNet.py
python/CMakeFiles/python-test: gtsam/tests/test_DiscreteBayesTree.py
python/CMakeFiles/python-test: gtsam/tests/test_DiscreteConditional.py
python/CMakeFiles/python-test: gtsam/tests/test_DiscreteDistribution.py
python/CMakeFiles/python-test: gtsam/tests/test_DiscreteFactorGraph.py
python/CMakeFiles/python-test: gtsam/tests/test_DsfTrackGenerator.py
python/CMakeFiles/python-test: gtsam/tests/test_Factors.py
python/CMakeFiles/python-test: gtsam/tests/test_FrobeniusFactor.py
python/CMakeFiles/python-test: gtsam/tests/test_GaussianBayesNet.py
python/CMakeFiles/python-test: gtsam/tests/test_GaussianFactorGraph.py
python/CMakeFiles/python-test: gtsam/tests/test_GraphvizFormatting.py
python/CMakeFiles/python-test: gtsam/tests/test_HybridFactorGraph.py
python/CMakeFiles/python-test: gtsam/tests/test_HybridNonlinearFactorGraph.py
python/CMakeFiles/python-test: gtsam/tests/test_HybridValues.py
python/CMakeFiles/python-test: gtsam/tests/test_JacobianFactor.py
python/CMakeFiles/python-test: gtsam/tests/test_KalmanFilter.py
python/CMakeFiles/python-test: gtsam/tests/test_KarcherMeanFactor.py
python/CMakeFiles/python-test: gtsam/tests/test_LocalizationExample.py
python/CMakeFiles/python-test: gtsam/tests/test_NonlinearOptimizer.py
python/CMakeFiles/python-test: gtsam/tests/test_OdometryExample.py
python/CMakeFiles/python-test: gtsam/tests/test_PinholeCamera.py
python/CMakeFiles/python-test: gtsam/tests/test_PlanarSLAMExample.py
python/CMakeFiles/python-test: gtsam/tests/test_Point2.py
python/CMakeFiles/python-test: gtsam/tests/test_Point3.py
python/CMakeFiles/python-test: gtsam/tests/test_Pose2.py
python/CMakeFiles/python-test: gtsam/tests/test_Pose2SLAMExample.py
python/CMakeFiles/python-test: gtsam/tests/test_Pose3.py
python/CMakeFiles/python-test: gtsam/tests/test_Pose3SLAMExample.py
python/CMakeFiles/python-test: gtsam/tests/test_PriorFactor.py
python/CMakeFiles/python-test: gtsam/tests/test_Rot3.py
python/CMakeFiles/python-test: gtsam/tests/test_SFMExample.py
python/CMakeFiles/python-test: gtsam/tests/test_SO4.py
python/CMakeFiles/python-test: gtsam/tests/test_SOn.py
python/CMakeFiles/python-test: gtsam/tests/test_Scenario.py
python/CMakeFiles/python-test: gtsam/tests/test_SfmData.py
python/CMakeFiles/python-test: gtsam/tests/test_ShonanAveraging.py
python/CMakeFiles/python-test: gtsam/tests/test_Sim2.py
python/CMakeFiles/python-test: gtsam/tests/test_Sim3.py
python/CMakeFiles/python-test: gtsam/tests/test_SimpleCamera.py
python/CMakeFiles/python-test: gtsam/tests/test_StereoVOExample.py
python/CMakeFiles/python-test: gtsam/tests/test_TranslationRecovery.py
python/CMakeFiles/python-test: gtsam/tests/test_Triangulation.py
python/CMakeFiles/python-test: gtsam/tests/test_Utilities.py
python/CMakeFiles/python-test: gtsam/tests/test_Values.py
python/CMakeFiles/python-test: gtsam/tests/test_VisualISAMExample.py
python/CMakeFiles/python-test: gtsam/tests/test_basis.py
python/CMakeFiles/python-test: gtsam/tests/test_custom_factor.py
python/CMakeFiles/python-test: gtsam/tests/test_dataset.py
python/CMakeFiles/python-test: gtsam/tests/test_initialize_pose3.py
python/CMakeFiles/python-test: gtsam/tests/test_lago.py
python/CMakeFiles/python-test: gtsam/tests/test_logging_optimizer.py
python/CMakeFiles/python-test: gtsam/tests/test_pickle.py
python/CMakeFiles/python-test: gtsam/tests/test_sam.py
	cd /home/kody/SLAM/factor-graph-slam-project/code/gtsam/gtsam-develop/python/python/gtsam/tests && /usr/bin/cmake -E env PYTHONPATH=/home/kody/SLAM/factor-graph-slam-project/code/gtsam/gtsam-develop/python/python/ /usr/bin/python3.8 -m unittest discover -v -s .

python-test: python/CMakeFiles/python-test
python-test: python/CMakeFiles/python-test.dir/build.make

.PHONY : python-test

# Rule to build all files generated by this target.
python/CMakeFiles/python-test.dir/build: python-test

.PHONY : python/CMakeFiles/python-test.dir/build

python/CMakeFiles/python-test.dir/clean:
	cd /home/kody/SLAM/factor-graph-slam-project/code/gtsam/gtsam-develop/python/python && $(CMAKE_COMMAND) -P CMakeFiles/python-test.dir/cmake_clean.cmake
.PHONY : python/CMakeFiles/python-test.dir/clean

python/CMakeFiles/python-test.dir/depend:
	cd /home/kody/SLAM/factor-graph-slam-project/code/gtsam/gtsam-develop/python && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/kody/SLAM/factor-graph-slam-project/code/gtsam/gtsam-develop /home/kody/SLAM/factor-graph-slam-project/code/gtsam/gtsam-develop/python /home/kody/SLAM/factor-graph-slam-project/code/gtsam/gtsam-develop/python /home/kody/SLAM/factor-graph-slam-project/code/gtsam/gtsam-develop/python/python /home/kody/SLAM/factor-graph-slam-project/code/gtsam/gtsam-develop/python/python/CMakeFiles/python-test.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : python/CMakeFiles/python-test.dir/depend
