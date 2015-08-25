import rospy
import actionlib
import trajectory_msgs
import control_msgs

'''
Trajectory Commander
sends points to the robot, one at a time.
This relies on us having a particular positon/velocity goal.
The reason we do this is to let us sync up with time commands.
'''

class TrajectoryCommander:

    def __init__(self,robot,command_topic,output_topic):

        self.robot = robot

        self.sub = rospy.Subscriber(command_topic,trajectory_msgs.msg.JointTrajectory,self.cmd_cb)
        self.pub = rospy.Publisher(output_topic)

    def cmd_cb(self,msg):
        for pt in msg.points:
            client = actionlib.SimpleActionClient(self.output_topic, control_msgs.msg.FollowJointTrajectoryAction)

            # make actionlib call
