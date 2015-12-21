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

## Directory Structure

### PYTHON

This directory contains grid-planning python code, for collecting demonstrations and for prototyping LfD.
Python dependencies are changing, but may include:
  * ``scikit-learn`` for different mixture models
  * ``pyhsmm`` for HDP-HMM learning

### SCRIPTS

This directory contains Python scripts.

Scripts will collect data based on examples, segment, and do other things. These let you more or less directly run the GP demos.

### SRC

This directory contains C++ code for planning.

C++ code includes/will include modifications to DMPs, possibly C++ cross-entropy code.
