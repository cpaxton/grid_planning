#!/usr/bin/env python

import rospy;
from std_srvs.srv import Empty;

rospy.init_node('gz_publish_planning_scene_node');

rospy.wait_for_service('/gazebo/publish_planning_scene');

delay = rospy.Duration(1.0);

for i in range(1,10):
    try:
        pub = rospy.ServiceProxy('/gazebo/publish_planning_scene',Empty);
        pub();
    except rospy.ServiceException, e:
        print "Publish planning scene service call failed: %s"%e;

    rospy.sleep(delay);

