#!/usr/bin/env python

import rospy

import actionlib

from grid_plan.msg import *
import grid
from grid_plan import PyPlanner
from grid_plan import GripperRegressor
from grid_plan import TrajectoryCommander

from trajectory_msgs.msg import JointTrajectory

import actionlib
import control_msgs

class CommandActionExecutor(object):
  # create messages that are used to publish feedback/result
  _feedback = CommandFeedback()
  _result   = CommandResult()
  _reg = None
  _traj_pub = rospy.Publisher("/gazebo/traj_rml/joint_traj_cmd", JointTrajectory, queue_size=100)
  _client = actionlib.SimpleActionClient("/gazebo/traj_rml/action",control_msgs.msg.FollowJointTrajectoryAction)
  _record = False
  #_record = True

  def __init__(self, name, robot, skills, gripper_topic):

    try:
        _record = rospy.get_param("~record")
        if not _record is None:
            self._record = bool(_record)
        else:
            self._record = False
    except KeyError, e:
        print "KeyError: " + str(e)
        self._record = False

    rospy.loginfo("Record set to: %s"%(str(self._record)))

    self._action_name = name
    self._as = actionlib.SimpleActionServer(self._action_name, CommandAction, execute_cb=self.execute_cb, auto_start = False)
    self._as.start()
    self._reg = GripperRegressor(robot,gripper_topic,None,None)

    self._robot = robot;
    self._robot.TfUpdateWorld()
    self._robot.sync_gripper = False # it does not make sense to record this at all

    for skill in skills:
      self._reg.addSkill(skill)

    self._traj_pub = rospy.Publisher("/gazebo/traj_rml/joint_traj_cmd", JointTrajectory, queue_size=100)
    self._client = actionlib.SimpleActionClient("/gazebo/traj_rml/action",control_msgs.msg.FollowJointTrajectoryAction)
    
  def execute_cb(self, goal):
    
    self._robot.TfUpdateWorld()
    if self._robot is None:
        print "[EXECUTE] Error: no robot!"

    print goal
    self._client.wait_for_server()
    rospy.loginfo("Server ready!")

    goal.traj.points[-1].velocities = [0]*len(goal.traj.points[-1].positions)
    actionlib_goal = control_msgs.msg.FollowJointTrajectoryGoal(trajectory=goal.traj)
    self._client.send_goal(actionlib_goal)

    for i in range(len(goal.keys)):
        self._robot.AddObject(goal.keys[i],goal.values[i])
        print " ... adding %s with frame=%s"%(goal.keys[i],goal.values[i])

    print self._robot

    if self._record:
        self._robot.StartRecording()

    self._client.wait_for_result()
    rospy.loginfo("Server done executing trajectory!")
    if self._record:
        self._robot.save(goal.name + ".yml")
        rospy.loginfo(("Saving now as %s")%(goal.name + ".yml"));
        self._robot.StopRecording()
        rospy.loginfo("Recording stopped!")

    config = []
    for i in range(len(goal.keys)):
        config.append((goal.keys[i], goal.values[i]))

    #print config
    if len(config) > 0:
        self._reg.configure(config)
        if self._reg.set_active_skill(goal.name):
            rospy.logwarn("Regression!")
            self._reg.regress(0.025,0.2)

        #wait = rospy.Duration(len(goal.traj.points)*2.0/30)
        #rospy.sleep(wait)
      
    success = True
    if success:
      #self._result.sequence = self._feedback.sequence
      rospy.loginfo('%s: Succeeded for \"%s\"' % (self._action_name,goal.name))
      self._as.set_succeeded(self._result)
      
if __name__ == '__main__':
  rospy.init_node('execute_server')

  skills = []
  for i in range(1,len(sys.argv)):

    if sys.argv[i][0] == '_':
        continue

	skill_filename = 'skills/sim/%s_skill.yml'%(sys.argv[i])
	skills.append(grid.RobotSkill(filename=skill_filename))
	print "Loaded skill '%s'"%(skills[-1].name)


  skill_topic = "current_skill"
  config = [('link','/gbeam_link_1/gbeam_link'),('node','/gbeam_node_1/gbeam_node')]
  joint_states_topic="/gazebo/barrett_manager/wam/joint_states"
  planning_scene_topic="/gazebo/planning_scene"
  gripper_topic='/gazebo/barrett_manager/hand/cmd'

  preset = "wam7_sim"
  if preset == "wam7_sim":
    base_link = 'wam/base_link'
    end_link = 'wam/wrist_palm_link'
    robot_description="robot_description"
    joint_states_topic="/gazebo/barrett_manager/wam/joint_states"
    planning_scene_topic="/gazebo/planning_scene"
    gripper_topic="/gazebo/barrett_manager/hand/cmd"
    command_topic="/gazebo/traj_rml/joint_traj_cmd"
    dof = 7

  elif preset == "ur5":
    base_link = '/base_link'
    end_link = '/ee_link'
    robot_description="/robot_description"
    joint_states_topic="/joint_states"
    planning_scene_topic="/planning_scene"
    command_topic='/arm_controller/command'
    dof = 6

  robot = grid.RobotFeatures(
        base_link=base_link,
        end_link=end_link,
        js_topic=joint_states_topic,
        gripper_topic=gripper_topic,
        dof=dof,
        preset=preset)

  print "... created RobotFeatures\n"


  CommandActionExecutor('command',robot,skills,gripper_topic)

  print "\n\nReady to receive commands!"
  rospy.spin()

