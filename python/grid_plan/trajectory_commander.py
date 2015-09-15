import rospy
import actionlib
import trajectory_msgs
import control_msgs.msg
import std_msgs

'''
Trajectory Commander
sends points to the robot, one at a time.
This relies on us having a particular positon/velocity goal.
The reason we do this is to let us sync up with time commands.
'''

class TrajectoryCommander:

    def __init__(self,robot,command_topic,output_topic,action,step=0.01):

        self.robot = robot
        self.output_topic = output_topic
        self.action = action
        self.command_topic = command_topic
        self.step = abs(step);

        self.pub = rospy.Publisher(output_topic,std_msgs.msg.Float64)
        if not command_topic is None:
            self.sub = rospy.Subscriber(command_topic,trajectory_msgs.msg.JointTrajectory,self.cmd_cb)

    '''
    instead of messing around with actions, just publish progress messages to synchronize other nodes
    '''
    def play(self,rate=0.05):
        t = 0
        while t <= 1:
            t = t + self.step
            print "t = %f"%(t)
            self.pub.publish(t)
            rospy.sleep(rate)
            

    def cmd_cb(self,msg):
        client = actionlib.SimpleActionClient(self.action, control_msgs.msg.FollowJointTrajectoryAction)
        progress = 0
        for pt in msg.points:

            progress += 1.0 / len(msg.points)

            goal = control_msgs.msg.FollowJointTrajectoryGoal()
            goal.trajectory = trajectory_msgs.msg.JointTrajectory()
            goal.trajectory.joint_names = msg.joint_names
            goal.trajectory.points = [pt]

            client.send_goal(goal)
            client.wait_for_result(rospy.Duration(1.0))

            self.pub.publish(progress)

        self.pub.publish(1.01) # make sure it's clearly done

