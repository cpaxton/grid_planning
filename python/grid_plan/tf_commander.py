import rospy
import actionlib
import trajectory_msgs
import control_msgs.msg
import std_msgs
import tf
import tf_conversions.posemath as pm
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from trajectory_msgs.msg import JointTrajectoryPoint

'''
Trajectory Commander
sends points to the robot, one at a time.
This relies on us having a particular positon/velocity goal.
The reason we do this is to let us sync up with time commands.

This time of Commander just publishes on topics, rather than using fancy ActionLib calls.
'''

class PubCommander:

    def __init__(self,robot,output_topic,action,step=0.01):

        self.robot = robot
        self.output_topic = output_topic
        self.action = action
        self.step = abs(step);

        self.pub = rospy.Publisher(output_topic,std_msgs.msg.Float64)
	
	#self.br = tf.TransformBroadcaster()
	if not action is None:
	    #self.action_pub = rospy.Publisher(action,PoseStamped)
	    self.action_pub = rospy.Publisher(action,JointTrajectoryPoint)

    '''
    instead of messing around with actions, just publish progress messages to synchronize other nodes
    '''
    def play(self,traj=None,rate=0.5):

        t = 0
	if not traj is None:
            for pt in traj.points:
                t +=  1.0 / len(traj.points)
                print "t = %f ... "%(t)
	        #print pt.positions
	        #print self.robot.kdl_kin.forward(pt.positions)
	        #f = self.robot.kdl_kin.forward(pt.positions)
	        #print pm.toTf(pm.fromMatrix(f))
	        #pose = pm.toMsg(pm.fromMatrix(f))
		#print pose
		#msg = PoseStamped()
		#msg.pose = pose
	        #self.pub.publish(t)
		print pt.positions
		self.action_pub.publish(pt)
	        #self.br.sendTransform(trans,rot,rospy.Time.now(),self.robot.base_link,self.action)

                rospy.sleep(rate)
	else:
	    while t <= 1.0:
                t +=  1.0 / len(self.traj.points)
                print "t = %f ... "%(t)
	        self.pub.publish(t)
                rospy.sleep(rate)

        self.pub.publish(1.01) # make sure it's clearly done

	#while not rospy.is_shutdown():
	#    #self.br.sendTransform(trans,rot,rospy.Time.now(),self.robot.base_link,self.action)
        #    #rospy.sleep(rate)

    def cmd_cb(self,msg):
	print msg
        self.traj = msg
