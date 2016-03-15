#!/usr/bin/env python

import rospy;
from conman_msgs.msg import *;
import actionlib;

rospy.init_node('start_jtns_node');

getc = actionlib.SimpleActionClient('/gazebo/scheme/get_blocks_action', GetBlocksAction)
setc = actionlib.SimpleActionClient('/gazebo/scheme/set_blocks_action', SetBlocksAction)

delay = rospy.Duration(1.0);
rospy.sleep(5.0);

try:

    for i in range(1,10):
        
        getc.wait_for_server();
        get_goal = GetBlocksGoal();
        
        getc.send_goal(get_goal);
        getc.wait_for_result();
        msg = getc.get_result()
        
        members = []
        try:
            for group in msg.groups:
                if group.name == 'cart_imp_control':
                    members = group.members
                    break

            set_goal = SetBlocksGoal(enable=members,diff=True,force=True,strict=False)
            setc.send_goal(set_goal);
            setc.wait_for_result();
            
            print setc.get_result();
        except AttributeError:
            pass
        
        rospy.sleep(delay);

except rospy.ROSInterruptException:
    pass
