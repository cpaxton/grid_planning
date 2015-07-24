#!/usr/bin/env python

from grid import *

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
        filenames = sys.argv[1:len(sys.argv)]
    else:
        filenames = ["app1.yml"]

    rate = rospy.Rate(10)

    data = []
    for filename in filenames:

        stream = file(filename,'r');
        demo = yaml.load(stream,Loader=Loader);

        print "Loaded data from '%s', computing features..."%(filename)
        #fx,x,u,t = demo.get_features([('ee','link'),('ee','node'),('link','node')])
        fx,x,u,t = demo.get_features([('ee','link')])

        print "Done computing features. %d total time steps."%(len(fx))
        data.append(np.array(fx))

    # loop over all data and create a GMM
    training_data = data[0]
    for i in range(1,len(data)):
        training_data = np.concatenate((training_data,data[i]))

    expert = GMM(n_components=5)
    expert = expert.fit(training_data)


