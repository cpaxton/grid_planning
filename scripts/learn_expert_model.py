#!/usr/bin/env python

import grid

" IO "
import sys
import yaml
try:
    from yaml import CLoader as Loader, CDumper as Dumper
except ImportError:
    from yaml import Loader, Dumper

" python tools "
import numpy as np

" machine learning "
from sklearn.mixture import GMM

" ROS "
import rospy
from geometry_msgs.msg import PoseArray

'''
Learns GMM of expert positions (in task space) as a GMM
This is used to search for trajectories that may satisfy a given action
'''
if __name__ == "__main__":
    rospy.init_node('learn_expert_models')

    if len(sys.argv) > 1:
        outfile = sys.argv[1]
        filenames = sys.argv[2:len(sys.argv)]
    else:
        outfile = ["model.yml"]
        filenames = ["app1.yml"]

    rate = rospy.Rate(10)

    data = []
    for filename in filenames:

        demo = grid.LoadRobotFeatures(filename);

        print "Loaded data from '%s', computing features..."%(filename)
        fx = demo.GetTrainingFeatures()

        print "Done computing features. %d total time steps."%(len(fx))
        data.append(np.array(fx))

    # loop over all data and create a GMM
    training_data = data[0]
    for i in range(1,len(data)):
        training_data = np.concatenate((training_data,data[i]))

    expert = GMM(n_components=5)
    expert = expert.fit(training_data)

    grid.SaveYaml(outfile,expert)
