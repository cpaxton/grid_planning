# GRID PLAN

Chris Paxton

This is the GRID-PLAN project (Grounded Robot Instruction from Demonstrations-Planning). The package has code for learning and planning based on human user demonstrations.

Initially this project relies on segemented and labeled human demonstrations.

## Installation

This code requires:
  * Boost
  * ROS (tested on Indigo)
  * Orocos KDL (download a new version)
  * ``scikit-learn`` for different mixture models (python only!)
  * ``pyhsmm`` for HDP-HMM learning (used in some experiments; may not be necessray for most things!)

To run the experiments, you'll need my ```grid_experiments``` ROS packages, as well as a working version of the ```lcsr_barrett``` simulation.

## Experiments

### Running Tests

There are a lot of ways you can run the different experiments provided.

The Execution test will let you connect two different skills. An example is this:

```
rosrun grid_plan execution_test _step_size:=0.65 _iter:=10 _ntrajs:=50 _noise:=1e-10 trajectory:=/gazebo/traj_rml/joint_traj_cmd _skill:=approach
```

This command runs with the Barrett simulation. It'll tell the arm to approach, with the goal being to grasp.

```
rosrun grid_plan execution_test _step_size:=0.75 _iter:=10 _ntrajs:=55 _noise:=1e-10 trajectory:=/gazebo/traj_rml/joint_traj_cmd _skill:=disengage
```

This one does something a little different. It tries to disengage from a link, and will move to a position a bit farther back away from it.

Note that in both of these examples I set a large number of different parameters. You really don't need to worry about these. They matter a lot more depending on how many samples you're making. As my code is not heavily parallelized, it can be inefficient to create task plans for lots of different examples at once.

Running the DMP tests is a little different. The basic DMP is actually in joint space, not cartesian space. This means that the trajectories you find will deal with joint limits, singularities, etc., but the configuration space is huge and unintuitive. So we need to search more slowly and draw a lot more samples. Here's an example:

```
rosrun grid_plan dmp_execution_test _step_size:=0.25 _iter:=50 _ntrajs:=100 _noise:=1e-10 trajectory:=/gazebo/traj_rml/joint_traj_cmd _skill:=approach
```

Results, for example:

```
[45] >>>> AVG P = 0.020318
[46] >>>> AVG P = 0.0203147
[47] >>>> AVG P = 0.0229661
[48] >>>> AVG P = 0.0238406
[49] >>>> AVG P = 0.0240259
```

As of 2016-01-15, some that worked:

```
rosrun grid_plan dmp_execution_test _step_size:=0.75 _iter:=20 _ntrajs:=50 _noise:=1e-10 trajectory:=/gazebo/traj_rml/joint_traj_cmd _skill:=disengage
rosrun grid_plan dmp_execution_test _step_size:=0.75 _iter:=20 _ntrajs:=50 _noise:=1e-10 trajectory:=/gazebo/traj_rml/joint_traj_cmd _skill:=approach
rosrun grid_plan dmp_execution_test _step_size:=0.75 _iter:=20 _ntrajs:=50 _noise:=1e-10 trajectory:=/gazebo/traj_rml/joint_traj_cmd _skill:=prepare
```

The __prepare__ action is intended to move to the beginning of an approach.

### Enabling Collision Detection

Collision detection is disabled by default for now, but you can enable it in any particular node by setting the __detect_collisions__ flag to true. For example:

```
rosrun grid_plan dmp_execution_test _step_size:=0.75 _iter:=10 _ntrajs:=20 _noise:=1e-10 trajectory:=/gazebo/traj_rml/joint_traj_cmd _skill:=approach _wait:=0.25 _detect_collisions:=1
```

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

#### Launch

Launch files are used to either (1) bring up tests for different things, or (2) set up the configuration for skills at run-time.

##### Setup

These launch files set up the different skills that we want to look at during runtime.

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
