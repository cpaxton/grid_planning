#!/usr/bin/env python

import grid
import rospy
import os
import sys

'''
convert
allows us to convert data between different formats
provides tools for doing so
'''

def convert_dir(cur_dir):
    if os.path.isdir(cur_dir):
        files = os.listdir(cur_dir);
        for f in files:
            new_dir = os.path.join(cur_dir,f)
            convert_dir(new_dir)

    else:
        data = grid.LoadRobotFeatures(cur_dir)
        filename = cur_dir.rsplit('.',1)[0] + ".bag"
        print 'Saving to "%s"'%(filename)
        data.ToRosBag(filename)

if __name__ == '__main__':
    rospy.init_node('converter_node')

    if len(sys.argv) < 2:
        rospy.logerr('Could not convert to rosbags! No directory provided!')

    root_dir = sys.argv[1];
    convert_dir(root_dir);

