#!/usr/bin/env python

" IO "
import sys
import yaml
try:
    from yaml import CLoader as Loader, CDumper as Dumper
except ImportError:
    from yaml import Loader, Dumper

" ROS "
import rospy

" machine learning "
from sklearn.mixture import DPGMM

'''
Cluster
Loads in demonstrations from specified files and attempts to cluster them.
May also do visualizations.

The key features for clustering can be:
    - actions taken (gripper_cmd, velocities)
    - features (at current time point)

We are primarily interested in change points, but we also care about the Gaussian gating functions that determine our "states".

Goals:
    - print out list of poses from world frame, to verify that we have something worth using
        - sent as a pose array that we can visualize in RVIZ
    - convert KDL transforms into useful features (in the gripper frame of reference)
    - group and cluster
'''

if __name__ == '__main__':
    rospy.init_node('demonstration_clustering')

    if len(sys.argv) > 1:
        filename = sys.argv[1]
    else:
        filename = "demo.yml"

    stream = file(filename,'r');
    demo = yaml.load(stream,Loader=Loader);

    fx,x,u = demo.get_features([('ee','link'),('ee','node'),('link','node')])
