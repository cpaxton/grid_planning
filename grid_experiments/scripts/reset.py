#!/usr/bin/env python

import rospy;
from trajectory_msgs.msg import JointTrajectoryPoint
from oro_barrett_msgs.msg import BHandCmd

'''
Simulation joint state resetter for testing reinforcement learning
'''
if __name__ == '__main__':
    rospy.init_node('arm_reset_node')
    pub = rospy.Publisher('/gazebo/traj_rml/joint_traj_point_cmd',JointTrajectoryPoint,queue_size=1000);
    hand_pub = rospy.Publisher('/gazebo/barrett_manager/hand/cmd',BHandCmd,queue_size=1000);

    reset_hand = BHandCmd()
    reset_hand.cmd = [-1.5,-1.5,-1.5,0]
    reset_hand.mode = [4,4,4,4]

    reset_pt = JointTrajectoryPoint();
    reset_pt.positions = [0.7481122637193378, -1.5298081414047147, -0.5061065498285455, 1.583484578044616, -0.7458254398633741, 1.3371837624599392, 1.0944201703273189];
    #velocity: [0.0018592202345833557, -0.0011402271328998879, 0.0036258963947593953, -5.600707407910563e-05, -0.002814494174852991, 0.004306146351973827, 0.002040219994002552];

    rate = rospy.Rate(10)
    
    for i in range(5):
        pub.publish(reset_pt);
        hand_pub.publish(reset_hand);
        rate.sleep()
