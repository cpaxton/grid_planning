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
        tokens = cur_dir.rsplit('.',1)
        if tokens[1] == "yml" or tokens[1] == "yaml":
            data = grid.LoadRobotFeatures(cur_dir)
            filename = tokens[0] + ".bag"
            print 'Saving to "%s"'%(filename)
            data.ToRosBag(filename)
        else:
            print 'Skipping %s: not a ".yml" or ".yaml" file!'%cur_dir

if __name__ == '__main__':
    rospy.init_node('converter_node')

    if len(sys.argv) < 2:
        rospy.logerr('Could not convert to rosbags! No directory provided!')

    root_dir = sys.argv[1];
    convert_dir(root_dir);

