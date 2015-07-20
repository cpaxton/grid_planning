#!/usr/bin/env python

from grid import *

" IO "
import sys
import yaml
try:
    from yaml import CLoader as Loader, CDumper as Dumper
except ImportError:
    from yaml import Loader, Dumper

" ROS "
import rospy
from geometry_msgs.msg import PoseArray

if __name__ == "__main__":
    rospy.init_node('segment_demonstrations')

    if len(sys.argv) > 1:
        filename = sys.argv[1]
    else:
        filename = "demo.yml"

    rate = rospy.Rate(10)

    stream = file(filename,'r');
    demo = yaml.load(stream,Loader=Loader);

    print "Loaded data, computing features..."
    fx,x,u,t = demo.get_features([('ee','link'),('ee','node'),('link','node')])

    print "Done computing features. %d total time steps."%(len(fx))
    print "Segmenting data based on gripper commands..."

    gripping = abs(fx[0][0]) > 0
    action = 0;
    labels = [];
    for i in range(len(t)):
        if fx[i][0] > 0 and not gripping:
            action += 1
            gripping = True
        elif not fx[i][0] > 0 and gripping:
            action += 1
            gripping = False
        labels += [action]

    print labels

    [lfx, lx, lu] = GetSegment(fx,x,u,labels,0)

    print "%d samples selected for segment"%(len(lfx))

    #these are definitely wrong
    #dbg_ee_poses = GetPoseMessage(fx,4,'/wam/hand/bhand_palm_link') #"/gbeam_link_1/gbeam_link")
    #dbg_ee_poses2 = GetPoseMessage(fx,11,'/wam/hand/bhand_palm_link')#"/gbeam_node_1/gbeam_node")

    dbg_ee_poses = GetPoseMessage(fx,4,"/gbeam_link_1/gbeam_link")
    dbg_ee_poses2 = GetPoseMessage(fx,12,"/gbeam_node_1/gbeam_node")
    dbg_ee = demo.get_world_pose_msg("ee")
    dbg_link = demo.get_world_pose_msg("link")
    
    pa_pub = rospy.Publisher('/dbg_ee_link',PoseArray)
    pa2_pub = rospy.Publisher('/dbg_ee_node',PoseArray)
    pa_ee_pub = rospy.Publisher('/dbg_ee',PoseArray)
    pa_link_pub = rospy.Publisher('/dbg_link',PoseArray)
    while not rospy.is_shutdown():
        pa_pub.publish(dbg_ee_poses)
        pa2_pub.publish(dbg_ee_poses2)
        pa_ee_pub.publish(dbg_ee)
        pa_link_pub.publish(dbg_link)
        rate.sleep()


