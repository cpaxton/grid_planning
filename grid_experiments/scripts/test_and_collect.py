#!/usr/bin/env python

import rospy
import sys
import trajectory_msgs.msg as tm
import sensor_msgs.msg as sm
import tf
import thread

" necessary for moveit stuff "
import actionlib

" import joint/position messages "
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose

" import moveit messages "
from moveit_msgs.msg import *
from moveit_msgs.srv import *


joints_file = open('joints.csv','w')
pose_file = open('pose.csv','w')
obj_file = open('obj.csv','w')

header = 'time,x,y,z,ax,ay,az,aw\n'
pose_file.write(header)
obj_file.write(header)

global cols
cols = False

global js
js = None

def js_cb(msg):
    global cols
    global js

    if not cols:
        joints_file.write(','.join(i for i in msg.name) + "\n")
        cols = True
    js = msg
    joints_file.write(','.join(str(i) for i in msg.position) + "\n")

'''
actually compute goal for each pose
'''
def getConstraints(robot_ns,pose,group):

    srv = rospy.ServiceProxy(robot_ns + "/compute_ik", moveit_msgs.srv.GetPositionIK)

    global js
    ik_req = moveit_msgs.msg.PositionIKRequest()
    ik_req.robot_state.joint_state = js
    ik_req.avoid_collisions = True
    ik_req.timeout = rospy.Duration(3.0)
    ik_req.attempts = 5
    ik_req.group_name = group
    ik_req.pose_stamped = pose

    print "Getting IK position..."
    ik_resp = srv(ik_req)

    print "IK RESULT ERROR CODE = %d"%(ik_resp.error_code.val)

    ###############################
    # now create the goal based on inverse kinematics

    goal = Constraints()

    for i in range(0,len(ik_resp.solution.joint_state.name)):
        print ik_resp.solution.joint_state.name[i]
        print ik_resp.solution.joint_state.position[i]
        joint = JointConstraint()
        joint.joint_name = ik_resp.solution.joint_state.name[i]
        joint.position = ik_resp.solution.joint_state.position[i] 
        joint.tolerance_below = 0.005
        joint.tolerance_above = 0.005
        joint.weight = 1.0
        goal.joint_constraints.append(joint)

    return goal


def write_thread(world,frame,obj):
    rate = rospy.Rate(10)
    tl = tf.TransformListener();
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

'''
generate a moveit plan from current position to next position
'''
def plan(robot_ns,client,pose,group):

    # set up options
    planning_options = PlanningOptions()
    planning_options.plan_only = False
    planning_options.replan = False
    planning_options.replan_attempts = 0
    planning_options.replan_delay = 2.0
    planning_options.planning_scene_diff.is_diff = True
    planning_options.planning_scene_diff.robot_state.is_diff = True

    # set up the motion request
    motion_req = MotionPlanRequest()

    global js
    motion_req.start_state.joint_state = js
    motion_req.workspace_parameters.header.frame_id = "world"
    motion_req.workspace_parameters.max_corner.x = 2.0
    motion_req.workspace_parameters.max_corner.y = 2.0
    motion_req.workspace_parameters.max_corner.z = 2.0
    motion_req.workspace_parameters.min_corner.x = -2.0
    motion_req.workspace_parameters.min_corner.y = -2.0
    motion_req.workspace_parameters.min_corner.z = -2.0

    # create the goal constraints
    motion_req.goal_constraints.append(getConstraints(robot_ns,pose,group))
    motion_req.group_name = group
    motion_req.num_planning_attempts = 10
    motion_req.allowed_planning_time = 5.0
    motion_req.planner_id = "RRTConnectkConfigDefault"

    if len(motion_req.goal_constraints[0].joint_constraints) == 0:
        return 'ik_error'

    goal = MoveGroupGoal()
    goal.planning_options = planning_options
    goal.request = motion_req

    print "Sending request..."

    client.send_goal(goal)
    client.wait_for_result()
    res = client.get_result()

    print "Done: " + str(res.error_code.val)

    if res.error_code.val == moveit_msgs.msg.MoveItErrorCodes.SUCCESS:
        return 'success'
    elif res.error_code.val >= -4:
        return 'moveit_error'
    else:
        return 'failure'

'''
MAIN FUNCTION GOES HERE
'''
if __name__=='__main__':

    rospy.init_node('data_collector');
    
    if sys.argv[1] == 'wam':
        rospy.Subscriber('/gazebo/barrett_manager/wam/joint_states', sm.JointState, js_cb)
        frame = '/wam/wrist_palm_link'
        namespace = '/gazebo'
        group = 'arm'
    elif sys.argv[1] == 'ur5':
        rospy.Subscriber('/joint_states', sm.JointState, js_cb)
        namespace = ''
        frame = '/ee_link'
        group = 'manipulator'

    if len(sys.argv) > 2:
        obj = sys.argv[2]
    else:
        obj = '/gbeam_link_1/gbeam_link'

    if len(sys.argv) < 4:
        world = "/world"
    else:
        world = sys.argv[3]
    
    rate = rospy.Rate(10);

    start_pose = geometry_msgs.msg.PoseStamped()
    start_pose.pose.position.x = 0.633
    start_pose.pose.position.y = 0.131
    start_pose.pose.position.z = 0.644
    start_pose.pose.orientation.x = 0.991
    start_pose.pose.orientation.y = 0.134
    start_pose.pose.orientation.z = -0.001
    start_pose.pose.orientation.w = -0.000
    start_pose.header.frame_id = "/world"

    mid_pose = geometry_msgs.msg.PoseStamped()
    mid_pose.pose.position.x = 0.553
    mid_pose.pose.position.y = 0.493
    mid_pose.pose.position.z = 0.445
    mid_pose.pose.orientation.x = 0.818
    mid_pose.pose.orientation.y = 0.576
    mid_pose.pose.orientation.z = -0.001
    mid_pose.pose.orientation.w = -0.001
    mid_pose.header.frame_id = "/world"

    end_pose = geometry_msgs.msg.PoseStamped()
    end_pose.pose.position.x = 0.347
    end_pose.pose.position.y = 0.790
    end_pose.pose.position.z = 0.468
    end_pose.pose.orientation.x = 0.503
    end_pose.pose.orientation.y = 0.864
    end_pose.pose.orientation.z = -0.001
    end_pose.pose.orientation.w = -0.001
    end_pose.header.frame_id = "/world"

    client = actionlib.SimpleActionClient(namespace+"/move_group", MoveGroupAction)

    while not rospy.is_shutdown():
        rate.sleep()
        if not js == None:
            break

    plan(namespace,client,start_pose,group)

    print "Done moving to start!"

    rospy.sleep(rospy.Duration(1.0))

    thread.start_new_thread(write_thread, (world, frame, obj))

    plan(namespace,client,mid_pose,group)
    plan(namespace,client,end_pose,group)

    try:
        while not rospy.is_shutdown():
            rate.sleep()
    except rospy.ROSInterruptException, ex:
        pass
