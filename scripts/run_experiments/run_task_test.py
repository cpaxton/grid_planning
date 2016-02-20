#!/usr/bin/env python

import rospy
from subprocess import call

for i in range(1,11):
    name = 'double%d:=true'%(i)
    print name

    launch_cmd = ['roslaunch','grid_experiments','ascent_experiments.launch',name]

    proc = call(launch_cmd)

    rospy.sleep(2.0)
    
