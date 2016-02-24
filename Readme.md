# GRID PLAN

Chris Paxton

This is the GRID-PLAN project (Grounded Robot Instruction from Demonstrations-Planning). The package has code for learning and planning based on human user demonstrations.

Initially this project relies on segmented and labeled human demonstrations.

Start the UR5 with:

```
roslaunch simple_ur_driver simple_ur_pd.launch 
```


## Installation

This code requires:
  * Boost
  * ROS (tested on Indigo)
  * Orocos KDL (download a new version)
  * ``scikit-learn`` for different mixture models (python only!)
  * ``pyhsmm`` for HDP-HMM learning (used in some sexperiments; may not be necessray for most things!)
  * Dynamic Movement Primitives ROS package
  * Gazebo
  * The ``lcsr_assembly`` stack

To run the experiments, you'll need my ```grid_experiments``` ROS packages, as well as a working version of the ```lcsr_barrett``` simulation.

## Experiments

### Collecting Data

Data collection requires a proper configuration of the environment. You're grounding all of the different actions for particular objects, right?

Examples are configured to collect gripper data for the Barrett WAM arms. So just start with ``hydra:=true`` and then run:
```
rosrun grid_plan demonstrate.py myaction1.yml
```

You can then either use the python (prototype) code or the C++ code.
The C++ code is an attempt to build a more fully integrated version of the pipeline, faster and with more features. Jury is out on whether that worked, but it seems ideal because of the large numbers of loops and such. The C++ code reads in rosbag files, which you can create from these demonstration YAML files with:

```
rosrun grid_plan convert_yml_to_rosbag.py /path/to/training/data/
```

Of course you replace ``/path/to/training/data/`` with a path to your examples.

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

The __task_test__ node actually executes task planning, and has even more options to consider:

```
rosrun grid_plan task_test _step_size:=0.75 _iter:=70 _ntrajs:=75 _verbosity:=0 _starting_horizon:=4 _max_horizon:=5 _update_horizon:=0.1
```

We may want to set the number of actions we want to start thinking about (root, approach, grasp, and align with ```_starting_horizon:=4``` as above). Or we might want to adjust how confident we need to be to add another action to the skill tree.

### Enabling Collision Detection

Collision detection is disabled by default for now, but you can enable it in any particular node by setting the __detect_collisions__ flag to true. For example:

```
rosrun grid_plan dmp_execution_test _step_size:=0.75 _iter:=10 _ntrajs:=20 _noise:=1e-10 trajectory:=/gazebo/traj_rml/joint_traj_cmd _skill:=approach _wait:=0.25 _detect_collisions:=1
```

### Other Notes

The model normalization term is really important to getting strong results. Setting it to a nonzero value is one of the several ways we prevent the distribution over trajectories from converging too quickly: "bad" examples in early iterations may still be helpful for guiding the distribution towards an optimal goal.

For example, here's a set of parameters that seems to work pretty well as of 2016-02-07 for the assembly task:

```
rosrun grid_plan task_test _step_size:=0.75 _iter:=50 _ntrajs:=50 _verbosity:=0 _starting_horizon:=5 _max_horizon:=5 _update_horizon:=0.001 _detect_collisions:=true _wait:=0 _base_model_norm:=0.1
```

As of 2016-02-09, I have been removing/testing the removal of a lot of the random noise parameters I have included in this file. These were things like the "base model norm" and "model norm step." Go ahead and remove those in future executions of the task test:

```
rosrun grid_plan task_test _step_size:=0.5 _iter:=20 _ntrajs:=100  _starting_horizon:=5 _max_horizon:=5 _update_horizon:=0.01 _detect_collisions:=true _base_model_norm:=0.01 _model_norm_step:=1
```

As of 2016-02-11, more changes have made the configuration slightly different. We have options to enable things like verbosity for the collision checker.

One thing that makes a big difference is the model normalization. See here:

```
rosrun grid_plan task_test _step_size:=0.5 _iter:=25 _ntrajs:=200 _starting_horizon:=5 _max_horizon:=5 _detect_collisions:=false _wait:=0 _collisions_verbose:=false _base_model_norm:=0.1 _model_norm_step=1 _update_horizon:=0.01
```

Setting the normalization higher makes convergence harder; setting it lower makes convergence easier (if you find examples). This is actually bad, though: fast convergence means you have a higher chance to end up in a local minimum.

As of 2016-02-17, this was the best set of commands with the 1-Gaussian code:

```
rosrun grid_plan task_test _step_size:=0.5 _iter:=25 _ntrajs:=200 _starting_horizon:=5 _max_horizon:=5 _detect_collisions:=false _wait:=0 _collisions_verbose:=false _base_model_norm:=0.1 _model_norm_step=1 _update_horizon:=0.01
rosrun grid_plan task_test _step_size:=0.5 _iter:=25 _ntrajs:=200 _starting_horizon:=5 _max_horizon:=5 _detect_collisions:=true _wait:=0 _collisions_verbose:=false _base_model_norm:=0.1 _model_norm_step=1 _update_horizon:=0.01
```

You should be able to see a roughly exponential improvement:

```
[5] align >>>> AVG P = 1.2851
[4] grasp >>>> AVG P = 0.00642552
[5] align >>>> AVG P = 268.966
[4] grasp >>>> AVG P = 1.34483
[5] align >>>> AVG P = 3533.92
[4] grasp >>>> AVG P = 17.6696
[5] align >>>> AVG P = 6778.76
[4] grasp >>>> AVG P = 33.8938
[5] align >>>> AVG P = 62099.1
[4] grasp >>>> AVG P = 310.496
[5] align >>>> AVG P = 88452.6
[4] grasp >>>> AVG P = 442.263
[5] align >>>> AVG P = 335451
[4] grasp >>>> AVG P = 1677.25
[5] align >>>> AVG P = 561695
[4] grasp >>>> AVG P = 2808.47
[5] align >>>> AVG P = 807009
[4] grasp >>>> AVG P = 4035.04
[5] align >>>> AVG P = 1.13222e+06
[4] grasp >>>> AVG P = 5661.09
[5] align >>>> AVG P = 1.35106e+06
[4] grasp >>>> AVG P = 6755.32
[5] align >>>> AVG P = 1.49062e+06
[4] grasp >>>> AVG P = 7453.1
[5] align >>>> AVG P = 1.56917e+06
[4] grasp >>>> AVG P = 7845.86
[5] align >>>> AVG P = 1.6093e+06
[4] grasp >>>> AVG P = 8046.48
[5] align >>>> AVG P = 1.63996e+06
[4] grasp >>>> AVG P = 8199.78

```

### Current Commands

Good places to start experiments:

```
rosrun grid_plan task_test _step_size:=0.5 _iter:=25 _ntrajs:=200 _starting_horizon:=5 _max_horizon:=5 _detect_collisions:=true _wait:=0 _collisions_verbose:=false _base_model_norm:=0.001 _model_norm_step=1 _update_horizon:=0.01
rosrun grid_plan task_test _step_size:=0.5 _iter:=15 _ntrajs:=200 _starting_horizon:=5 _max_horizon:=5 _detect_collisions:=true _wait:=0 _collisions_verbose:=false _base_model_norm:=0.0001 _model_norm_step=1 _update_horizon:=0.0001
rosrun grid_plan task_test _step_size:=0.5 _iter:=15 _ntrajs:=100 _starting_horizon:=5 _max_horizon:=5 _detect_collisions:=true _wait:=0 _collisions_verbose:=false _base_model_norm:=0.01 _model_norm_step=1 _update_horizon:=0.0001 _replan_depth:=0 _execute_depth:=5 _collision_detection_step:=2 _fixed_distribution_noise:=true _test:=0
```

Note the decreased model norm from the previous version. I changed back to adding a higher norm after switching to k=3 components.

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
