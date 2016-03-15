#!/usr/bin/env python

import rospy
import tf
import tf_conversions as tfc
import PyKDL as pk

from predicator_msgs.msg import * 

verbose = False
publish_predicator = False

pub = rospy.Publisher("predicator/input", predicator_msgs.msg.PredicateList)
vpub = rospy.Publisher("predicator/valid_input", predicator_msgs.msg.ValidPredicates)

flipped_frames_allowed = False

def get_frames(num, step, trans, rot, frame=None, off=0, suffix=""):

    f = tfc.fromTf((trans, rot))

    bc = tf.TransformBroadcaster()

    names = []

    for i in range(1, num+1):
        r = tfc.Rotation(f.M)

        (roll, pitch, yaw) = r.GetRPY()
        r = tfc.Rotation.RPY(roll, pitch, yaw + step)

        rf = tfc.Frame(tfc.Rotation.RotZ(step))
        rfz = tfc.Frame(tfc.Rotation.RPY(0,3.14159,0))

        p = rf * tfc.Vector(f.p)
        p2 = tfc.Vector(-1*p.x(), -1*p.y(), -1*p.z())

        r2 = tfc.Rotation.RPY(roll + 3.14159, pitch, (yaw + step))

        f = tfc.Frame(r, p)
        f2 = tfc.Frame(r2, p2)

        (trans2, rot2) = tfc.toTf(f)
        (trans3, rot3) = tfc.toTf(f2)

        if verbose:
            print (trans2, rot2)
            if flipped_frames_allowed:
                print (trans3, rot3)

        if not frame == None:
            bc.sendTransform(trans2, rot2, rospy.Time.now(), frame2 + suffix + "_gen" + str(off+i), frame)
            names.append(frame2 + suffix + "_gen" + str(off+i))

            if flipped_frames_allowed:
                bc.sendTransform(trans3, rot3, rospy.Time.now(), frame2  + suffix + "_flip" + str(off+i), frame)
                names.append(frame2 + suffix + "_flip" + str(off+i))

        if verbose:
            print "---" + str(i) + "---"

    return names

if __name__ == "__main__":

    rospy.init_node("ring_rotation_helper_node")

    frame1 = rospy.get_param("~frame1")
    frame2 = rospy.get_param("~frame2")
    from_frame = rospy.get_param("~from_frame", frame1)
    suffix = rospy.get_param("~suffix", "")
    num = rospy.get_param("~num", 4)
    step = rospy.get_param("~step", 0.314159*4)
    name = rospy.get_param("~name", "ring1/gen_grasp")
    parent = rospy.get_param("~parent", "ring1")
    spin_rate = rospy.get_param("~rate", 10)
    flipped_frames_allowed = rospy.get_param("~flipped_frames", 0) == 1
    predicate_name = rospy.get_param("~predicate_name", "grasp_point")

    verbose = rospy.get_param("~verbose", 1) == 1
    publish_predicator = rospy.get_param("~publish_predicates", 1) == 1

    rate = rospy.Rate(spin_rate)

    start_idx_offset = 1

    listener = tf.TransformListener()

    done_tf = False
    while not done_tf:
        try:
            (trans, rot) = listener.lookupTransform(frame1, frame2, rospy.Time(0))
            done_tf = True
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

    while not rospy.is_shutdown():
        gen_frames = get_frames(num, step, trans, rot, from_frame, start_idx_offset, suffix)

        if publish_predicator:
            msg = PredicateList()
            msg.pheader.source = rospy.get_name()

            vp = ValidPredicates()
            vp.pheader.source = rospy.get_name()
            vp.assignments.append(parent)
            vp.predicates.append(predicate_name)
            vp.predicates.append(predicate_name + "_of")

            for gen in gen_frames:

                ps = PredicateStatement()
                ps.predicate = predicate_name + "_of"
                ps.value = PredicateStatement.TRUE
                ps.confidence = PredicateStatement.TRUE
                ps.params[0] = gen
                ps.params[1] = parent
                ps.param_classes.append("location")
                ps.param_classes.append("object")
                ps.num_params = 2
                msg.statements.append(ps)

                vp.assignments.append(gen)

                ps = PredicateStatement()
                ps.predicate = predicate_name
                ps.value = PredicateStatement.TRUE
                ps.confidence = PredicateStatement.TRUE
                ps.params[0] = gen
                ps.param_classes.append("location")
                ps.num_params = 1
                msg.statements.append(ps)

            pub.publish(msg)
            vpub.publish(vp)

            rate.sleep()
