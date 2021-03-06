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

" ROS "
import rospy
from geometry_msgs.msg import PoseArray

if __name__ == "__main__":
    rospy.init_node('segment_demonstrations')

    if len(sys.argv) > 1:
        filenames = sys.argv[1:len(sys.argv)]
    else:
        filenames = ["app1.yml"]

    rate = rospy.Rate(10)

    dbg_ee_poses = PoseArray()
    dbg_ee_poses2 = PoseArray()
    dbg_ee = PoseArray()
    dbg_link = PoseArray()
    dbg_node = PoseArray()
    dbg_ee_poses.header.frame_id = "/gbeam_link_1/gbeam_link"
    dbg_ee_poses2.header.frame_id = "/gbeam_node_1/gbeam_node"
    dbg_ee.header.frame_id = "/world"
    dbg_link.header.frame_id = "/world"
    dbg_node.header.frame_id = "/world"

    data = []
    for filename in filenames:

        #stream = file(filename,'r');
        #demo = yaml.load(stream,Loader=Loader);
        demo = grid.LoadRobotFeatures(filename);

        print "Loaded data from '%s', computing features..."%(filename)
        #fx,x,u,t = demo.get_features([('ee','link'),('ee','node'),('link','node')])
        fx,goal = demo.GetTrainingFeatures()

        print "Done computing features. %d total time steps."%(len(fx))
        data.append((demo, np.array(fx)))

    for (demo, fx) in data:
        print "Segmenting data based on gripper commands..."

        gripping = abs(fx[0,0]) > 0
        action = 0;
        labels = [];
        for i in range(fx.shape[0]):
            if fx[i,0] > 0 and not gripping:
                action += 1
                gripping = True
            elif not fx[i,0] > 0 and gripping:
                action += 1
                gripping = False
            labels += [action]

        print labels

        #[lfx, lx, lu] = grid.GetSegment(fx,x,u,labels,0)
        #print "%d samples selected for segment"%(len(lfx))

        #these are definitely wrong
        #dbg_ee_poses = GetPoseMessage(fx,4,'/wam/hand/bhand_palm_link') #"/gbeam_link_1/gbeam_link")
        #dbg_ee_poses2 = GetPoseMessage(fx,12,'/wam/hand/bhand_palm_link')#"/gbeam_node_1/gbeam_node")

        dbg_ee_poses_ = grid.GetPoseMessage(fx,8,"/gbeam_link_1/gbeam_link")
        dbg_ee_poses2_ = grid.GetPoseMessage(fx,0,"/gbeam_node_1/gbeam_node")
        #dbg_ee_ = demo.get_world_pose_msg("ee")
        dbg_ee_ = demo.GetFwdPoseMsg()
        #dbg_link_ = demo.get_world_pose_msg("link")
        dbg_link_ = demo.GetWorldPoseMsg("link")
        #dbg_node_ = demo.get_world_pose_msg("node")
        dbg_node_ = demo.GetWorldPoseMsg("node")

        dbg_ee_poses.poses += dbg_ee_poses_.poses
        dbg_ee_poses2.poses += dbg_ee_poses2_.poses
        dbg_ee.poses += dbg_ee_.poses
        dbg_link.poses += dbg_link_.poses
        dbg_node.poses += dbg_node_.poses

    pa_pub = rospy.Publisher('/dbg_ee_link',PoseArray)
    pa2_pub = rospy.Publisher('/dbg_ee_node',PoseArray)
    pa_ee_pub = rospy.Publisher('/dbg_ee',PoseArray)
    pa_link_pub = rospy.Publisher('/dbg_link',PoseArray)
    pa_node_pub = rospy.Publisher('/dbg_node',PoseArray)

    try:
        while not rospy.is_shutdown():
            pa_pub.publish(dbg_ee_poses)
            pa2_pub.publish(dbg_ee_poses2)
            pa_ee_pub.publish(dbg_ee)
            pa_link_pub.publish(dbg_link)
            pa_node_pub.publish(dbg_node)
            rate.sleep()
    except rospy.ROSInterruptException, e:
        pass

