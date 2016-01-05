# GRID PLAN

Chris Paxton

This is the GRID-PLAN project (Grounded Robot Instruction from Demonstrations-Planning). The package has code for learning and planning based on human user demonstrations.

Initially this project relies on segemented and labeled human demonstrations.

## Installation

This code requires:
  * Boost
  * ROS (tested on Indigo)
  * Orocos KDL (download a new version)
  * ``scikit-learn`` for different mixture models
  * ``pyhsmm`` for HDP-HMM learning

To run the experiments, you'll need my ```grid_experiments``` ROS packages, as well as a working version of the ```lcsr_barrett``` simulation.

## Guide to Files

### Executables

The ***trajectory_test*** exectable generates a set of random trajectories into the pose of the "node" object. So of course this will only work if you are running the assembly simulation.

### Directory Structure

#### Config

RVIZ configuration files for debugging.

#### Data

Stores files containing demonstrated trajectories in YAML and Rosbag formats. Subfolders contain demonstrations recorded on different platforms.

#### Include

Contains the C++ headers for the produced library.

#### Python

This directory contains grid-planning python code, for collecting demonstrations and for prototyping LfD.
Python dependencies are changing, but may include:
  * ``scikit-learn`` for different mixture models
  * ``pyhsmm`` for HDP-HMM learning

#### Scripts

This directory contains Python scripts.

Scripts will collect data based on examples, segment, and do other things. These let you more or less directly run the different demos.

Some of the most important of these are:
  * ``working_cpp.py`` and ``run_ur5.py`` for running simulation and UR5 experiments
  * ``demonstrate.py`` and ``demonstrate_ur5.py`` for recording data
  * ``create_skills.py`` and ``create_skills_ur5.py`` for creating skill models using the experimental (python) code
  * ``convert_yml_to_rosbag.py`` for converting yml of examples into rosbag format to read via C++. I decided to go with rosbags for my example data mostly to reduce the number of external dependencies for the c++ code; I already am requiring ROS be installed as it is.

#### Skills

This directory contains skill files in different formats, in particular as YAML and skill (custom) files.

#### SRC

This directory contains C++ code for planning.

C++ code includes/will include modifications to DMPs, possibly C++ cross-entropy code.
