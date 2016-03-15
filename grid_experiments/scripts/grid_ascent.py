#!/usr/bin/python
import yappi

# Ros imports
import rospy
import copy

import PyKDL as kdl
from tf_conversions.posemath import fromTf, toTf, toMsg, fromMsg
import tf

from urdf_parser_py.urdf import URDF

# Messages
import std_msgs.msg as std_msgs
import geometry_msgs.msg as geometry_msgs

import grid_msgs.msg as grid_msgs
from grid_experiments.msg import AscentBridge

# ASCENT generic APIs
from ascent_augmenter.augmenter import Augmenter
from ascent_augmenter.observer import Observer
from ascent_augmenter.augmenter import GraspClassifier, ApproachClassifier, GrippedObjectPoseObserver
from ascent_augmenter.manipulator import Manipulator

# Barrett Wam manipulator
from ascent_lcsr_barrett.barrett_manipulator import BarrettManipulator


def flatten_frame(frame):
    f = []
    for i in range(3):
        for j in range(3):
            f.append(frame[i,j])
        f.append(frame.p[i])
    f.extend([0.0,0.0,0.0,1.0])

    return f


class MateObserver(Observer):
    def __init__(self, manipulator):
        super(MateObserver, self).__init__(manipulator)

    def observe(self, models, telemanip_history, percepts, predicates, interventions):
        """
        """

        MAX_LIN_MATE_ERROR = 0.1 # [m]
        MAX_ROT_MATE_ERROR = 0.25 # [rad] along major axis

        gripper_occupied = predicates.get('gripper_occupied',False)
        gripped_object_class = predicates.get('gripped_object_class',None)

        if not gripped_object_class or not gripper_occupied:
            return {}

        for percept in percepts:
            for mate in percept['mates'].get(gripped_object_class,[]):
                for mate_pose in mate['poses']:
                    mated_pose = mate['pose']*mate_pose

                    for mated_percept in percepts:
                        if mated_percept['label'] == gripped_object_class:
                            mate_err = kdl.diff(mated_pose, mated_percept['pose'])
                            lin_mate_err = mate_err.vel.Norm()
                            rot_mate_err = mate_err.rot.Norm()

                            if lin_mate_err < MAX_LIN_MATE_ERROR and rot_mate_err < MAX_ROT_MATE_ERROR:
                                return {'gripped_object_mated': True}

        return {}

class StateCollector(Observer):

    """Collect features and predicates and publish them over ROS"""

    def __init__(self, manipulator):
        super(StateCollector, self).__init__(manipulator)

        # get robot model
        # TODO: get robot model urdf and use it for FK
        #self.urdf_xml_strng = rospy.get_param('robot_description')
        #self.robot = URDF.from_xml_string(self.urdf_xml_string)

        # tf listner
        self.listener = tf.TransformListener(True, rospy.Duration(40.0))

        self.bridge_publisher = rospy.Publisher('/ascent_bridge',AscentBridge)

    def observe(self, models, telemanip_history, percepts, predicates, interventions):

        percepts = copy.copy(percepts)

        msg = AscentBridge()

        # Set header (time is a lie)
        msg.header.frame_id = '/world'
        msg.header.stamp = rospy.Time.now()

        # Robot State
        try:
            self.listener.waitForTransform('/world', self.manipulator.tip_link, msg.header.stamp, rospy.Duration(0.1))
            gripper_tform = self.listener.lookupTransform('/world', self.manipulator.tip_link, msg.header.stamp)
            prev_gripper_tform = self.listener.lookupTransform('/world', self.manipulator.tip_link, msg.header.stamp - rospy.Duration(0.1))
        except (Exception, tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as exc:
            rospy.logerr("Couldn't get gripper pose: "+str(exc))
            return {}

        gripper_frame = fromTf(gripper_tform)
        prev_gripper_frame = fromTf(prev_gripper_tform)
        gripper_twist = kdl.diff(gripper_frame, prev_gripper_frame)

        msg.gripper_pose = flatten_frame(gripper_frame)
        msg.gripper_twist = list(gripper_twist.vel) + list(gripper_twist.rot)
        msg.gripper_pose_cmd = flatten_frame(fromMsg(telemanip_history[0].posetwist.pose)) if len(telemanip_history) > 0 else self.last_msg.gripper_pose_cmd
        msg.gripper_cmd = telemanip_history[0].grasp_opening if len(telemanip_history) > 0 else self.last_msg.gripper_cmd

        msg.joint_pos = self.manipulator.get_joint_state().position
        msg.joint_vel = self.manipulator.get_joint_state().velocity
        msg.joint_eff = self.manipulator.get_joint_state().effort

        robot_percept = {
            'id': 0,
            'label': 'bhand',
            'stamp': msg.header.stamp,
            'pose': gripper_frame,
            'grasps': [],
            'mates': {} }

        percepts.append(robot_percept)

        # Features
        # TODO: add robot to the list of percepts, 'class' : 'robot'
        # get center-center twists for all models
        for i in range(len(percepts)):

            eid = percepts[i]['id']
            label = percepts[i]['label']
            key = "%s_%03d" % (label, eid)
            pose = percepts[i]['pose']

            msg.entity_keys.append(key)
            msg.entity_ids.append(eid)
            msg.entity_labels.append(label)
            msg.entity_poses.extend(flatten_frame(pose))

            for j in range(0, i):
                rel_key = msg.entity_keys[j]
                rel_label = msg.entity_labels[j]
                rel_eid = msg.entity_ids[j]
                rel_pose = percepts[j]['pose']

                rel_twist = kdl.diff(pose, rel_pose)
                rel_twist_msg = list(rel_twist.vel) + list(rel_twist.rot)
                rel_twist_rev_msg = list(-rel_twist.vel) + list(-rel_twist.rot)

                msg.twist_from.append(key)
                msg.twist_to.append(rel_key)
                msg.twists.extend(rel_twist_msg)

                msg.twist_from.append(rel_key)
                msg.twist_to.append(key)
                msg.twists.extend(rel_twist_rev_msg)

        # Gripped object information
        msg.gripped_object_label = predicates.get('gripped_object_label', '')
        msg.gripped_object_pose = flatten_frame(fromMsg((predicates.get('gripped_object_pose', None) or geometry_msgs.PoseStamped(std_msgs.Header(0,rospy.Time(0),''),toMsg(kdl.Frame()))).pose))

        # Predicates
        used_keys = [
            'gripper_open',
            'gripper_occupied',
            'gripped_object_mated'
        ]

        for k in used_keys:
            msg.predicate_names.append(k)
            msg.predicate_values.append(predicates.get(k,False))

        msg.predicate_names.append('gripper_active')
        msg.predicate_values.append(any(['gripper' in i.resources for i in interventions]))

        # Publish bridge message
        #print msg
        self.bridge_publisher.publish(msg)

        # No predicates to update for ascent
        return {}

    def observe2(self, models, telemanip_history, percepts, predicates):

        percepts = copy.copy(percepts)

        msg = AscentBridge()

        # Set header (time is a lie)
        msg.header.frame_id = '/world'
        msg.header.stamp = rospy.Time.now()

        # Robot State
        msg.joint_state = self.manipulator.get_joint_state()

        try:
            self.listener.waitForTransform(self.manipulator.tip_link, '/world', msg.joint_state.header.stamp, rospy.Duration(0.1))
            gripper_tform = self.listener.lookupTransform(self.manipulator.tip_link, '/world', msg.joint_state.header.stamp)
            prev_gripper_tform = self.listener.lookupTransform(self.manipulator.tip_link, '/world', msg.joint_state.header.stamp - rospy.Duration(0.1))
        except (Exception, tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as exc:
            rospy.logerr("Couldn't get gripper pose: "+str(exc))
            return {}

        gripper_frame = fromTf(gripper_tform)
        prev_gripper_frame = fromTf(prev_gripper_tform)

        gripper_twist = kdl.diff(gripper_frame, prev_gripper_frame)

        msg.gripper_pose = toMsg(gripper_frame)
        msg.gripper_twist = geometry_msgs.Twist(
            geometry_msgs.Vector3(*list(gripper_twist.vel)),
            geometry_msgs.Vector3(*list(gripper_twist.rot)))
        msg.gripper_pose_cmd = telemanip_history[0].posetwist.pose if len(telemanip_history) > 0 else self.last_msg.gripper_pose_cmd
        msg.gripper_cmd = telemanip_history[0].grasp_opening if len(telemanip_history) > 0 else self.last_msg.gripper_cmd

        robot_percept = {
            'id': 0,
            'label': 'bhand',
            'stamp': msg.header.stamp,
            'pose': fromMsg(msg.gripper_pose),
            'grasps': [],
            'mates': {} }

        percepts.append(robot_percept)

        # Features
        # TODO: add robot to the list of percepts, 'class' : 'robot'
        # get center-center twists for all models
        for i in range(len(percepts)):

            eid = percepts[i]['id']
            label = percepts[i]['label']
            key = "%s_%03d" % (label, eid)
            pose = percepts[i]['pose']

            msg.entity_keys.append(key)
            msg.entity_ids.append(eid)
            msg.entity_labels.append(label)
            msg.entity_poses.append(toMsg(pose))

            for j in range(0, i):
                rel_key = msg.entity_keys[j]
                rel_label = msg.entity_labels[j]
                rel_eid = msg.entity_ids[j]
                rel_pose = percepts[j]['pose']

                rel_twist = kdl.diff(pose, rel_pose)
                rel_twist_msg = geometry_msgs.Twist(
                    geometry_msgs.Vector3(*list(rel_twist.vel)),
                    geometry_msgs.Vector3(*list(rel_twist.rot)))
                rel_twist_rev_msg = geometry_msgs.Twist(
                    geometry_msgs.Vector3(*list(-rel_twist.vel)),
                    geometry_msgs.Vector3(*list(-rel_twist.rot)))

                msg.twist_from.append(key)
                msg.twist_to.append(rel_key)
                msg.twists.append(rel_twist_msg)

                msg.twist_from.append(rel_key)
                msg.twist_to.append(key)
                msg.twists.append(rel_twist_rev_msg)

        # Gripped object information
        msg.gripped_object_label = predicates.get('gripped_object_label', '')
        msg.gripped_object_pose = (predicates.get('gripped_object_pose', None) or geometry_msgs.PoseStamped(std_msgs.Header(0,rospy.Time(0),''),toMsg(kdl.Frame()))).pose

        # Predicates
        used_keys = [
            'gripper_open',
            'gripper_occupied',
            'gripped_object_mated'
        ]

        for k in used_keys:
            msg.predicate_names.append(k)
            msg.predicate_values.append(predicates.get(k,False))

        # Publish bridge message
        #print msg
        self.bridge_publisher.publish(msg)

        # No predicates to update for ascent
        return {}

class ProximityObserver(Observer):

    """Determine the pose of a grasped object in the gripper."""

    def __init__(self, manipulator):
        super(GrippedObjectPoseObserver, self).__init__(manipulator)

        self.proximity_sub = rospy.Subscriber('object_comparison', grid_msgs.ObjectComparison, proximity_cb)

    def proximity_cb(self, msg):
        """listen for mesh proximity"""
        pass

    def observe(self, models, telemanip_history, percepts, predicates, interventions):
        """
        """
        pass


def main():
    rospy.init_node('augmenter',log_level=rospy.INFO,disable_signals=False)

    # Construct barrett manipulator
    barrett_manipulator = BarrettManipulator()
    barrett_manipulator.connect()

    predicates = {}
    predicates['gripper_open'] = True
    #predicates['gripped_object_class'] = 'gbeam_link'
    #predicates['gripped_object_pose'] = geometry_msgs.PoseStamped(
        #std_msgs.Header(0,rospy.Time.now(),barrett_manipulator.tip_link),toMsg(kdl.Frame()))

    # Construct augmenter to contrl the WAM
    augmenter = Augmenter(barrett_manipulator, predicates)

    augmenter.classifiers = [
        GraspClassifier(barrett_manipulator)]

    augmenter.observers = [
        StateCollector(barrett_manipulator),
        MateObserver(barrett_manipulator),
        GrippedObjectPoseObserver(barrett_manipulator)]

    #yappi.start()

    augmenter.start()

    rospy.spin()

    #yappi.get_func_stats().print_all()

if __name__=='__main__':
    main()
