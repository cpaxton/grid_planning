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

" math "
import numpy as np

" Fitting DMPs "
from dmp.srv import *
from dmp.msg import *
from grid import RequestDMP

'''
fit_dmps
This function fits a set of dynamic movement primitives and saves them to a file.
Based off of the DMP code from the ROS wiki.

DMPs are saved for each individual motion, so this computes DMPs and features for each DMP point.
'''

if __name__ == '__main__':
    rospy.init_node('demonstration_fitting')

    if len(sys.argv) > 1:
        filename = sys.argv[1]
    else:
        filename = "demo.yml"

    stream = file(filename,'r');
    demo = yaml.load(stream,Loader=Loader);

    print "Loaded data, computing features..."
    fx,x,u,t = demo.get_features([('ee','link'),('ee','node'),('link','node')])

    # DMP parameters
    dims = len(u[0])
    dt = 1.0
    K = 100
    D = 2.0 * np.sqrt(K)
    num_bases = 4

    dmp = RequestDMP(u,0.1,K,D,5)
    print dmp
    


