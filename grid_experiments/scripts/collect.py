#!/usr/bin/env python

import rospy
import sys
import trajectory_msgs.msg as tm
import sensor_msgs.msg as sm
import tf

joints_file = open('joints.csv','w')
pose_file = open('pose.csv','w')
obj_file = open('obj.csv','w')

header = 'time,x,y,z,ax,ay,az,aw\n'
pose_file.write(header)
obj_file.write(header)

global cols
cols = False

def js_cb(msg):
    if not cols:
        global cols
        joints_file.write(','.join(i for i in msg.name) + "\n")
        cols = True
    joints_file.write(','.join(str(i) for i in msg.position) + "\n")

if __name__=='__main__':

    rospy.init_node('data_collector');
    
    rospy.Subscriber(sys.argv[1], sm.JointState, js_cb)
    frame = sys.argv[2]
    obj = sys.argv[3]

    if len(sys.argv) < 5:
        world = "/world"
    else:
        world = sys.argv[3]
    
    rate = rospy.Rate(10);

    tl = tf.TransformListener();
    try:
        while not rospy.is_shutdown():
            try:
                (trans,rot) = tl.lookupTransform(world, frame, rospy.Time(0))
                pose_file.write(str(rospy.Time.now()) + ',' + (','.join(str(i) for i in trans)) + ',' + (','.join(str(i) for i in rot)) + "\n")
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
            try:
                (trans,rot) = tl.lookupTransform(world, obj, rospy.Time(0))
                obj_file.write(str(rospy.Time.now()) + ',' + (','.join(str(i) for i in trans)) + ',' + (','.join(str(i) for i in rot)) + "\n")
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
            rate.sleep()
    except rospy.ROSInterruptException, ex:
        pass
