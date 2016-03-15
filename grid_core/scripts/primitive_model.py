#!/usr/bin/env python

import rospy;
from grid import *;

if __name__=="__main__":

    actor_frame = 'wam/hand/bhand_grasp_link'
    world_frame = 'world'

    gbeam_nodes = ['gbeam_node_1','gbeam_node_2']
    gbeam_links = ['gbeam_link_1','gbeam_link_2']
    node_frame = 'gbeam_node'
    link_frame = 'link_node'

    gbeam_frames = ['gbeam_node_1/gbeam_node','gbeam_node_2/gbeam_node','gbeam_link_1','gbeam_link_2']

    # ROS setup
    rospy.init_node('primitive_model_test')
    rate = rospy.Rate(10)

    hand_effort = EffortListener(rate=10, frame=actor_frame, world=world_frame)
    active_mates = ActiveMatesListener(rate=10, assignments=gbeam_nodes+gbeam_links,sub_topic='/active_mates')
    structure = StructureListener(rate=10, nodes=gbeam_nodes, links=gbeam_links)

    '''
    frame_listeners = []
    for frame_id in gbeam_frames:
        fl = FrameListener(rate=10,frame=frame_id,actor=actor_frame)
        frame_listeners.append(fl)
    '''
    try:

        while ~rospy.is_shutdown():
            hand_effort.tick()
            structure.tick()
            rate.sleep()


    except rospy.ROSInterruptException, e:
        pass

