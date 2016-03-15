import rospy
import PyKDL
import tf
import tf_conversions as tfc
import geometry_msgs.msg as geometry_msgs
import numpy as np

'''
EffortListener
This class aggregates information from one frame over time to produce features indicating effort and rotation
'''
class EffortListener:

    '''
    init
    creates a listener for only a single frame
    '''
    def __init__(self,world,frame,rate):
        self.frame = frame
        self.world = world
        self.rate = rate
        self.listener = tf.TransformListener() # might not actually want this in each thread

        # initialize all the important variables we will be using later on
        self.t0 = None
        self.t1 = rospy.Time.now()

        self.frame0 = None
        self.frame1 = None

        self.rot_speed = None
        self.trans_speed = None

    def tick(self):
        t = rospy.Time.now()
        (trans, rot) = (None, None)
        while (rospy.Time.now() - t).to_sec() < 1.0:
            try:
                (trans,rot) = self.listener.lookupTransform(self.world,self.frame,rospy.Time(0))
                break
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException), e:
                continue

        if trans != None:
            new_frame = PyKDL.Frame(PyKDL.Rotation.Quaternion(*rot),PyKDL.Vector(*trans))

            if self.frame1 == None:
                self.frame1 = new_frame
                self.t1 = rospy.Time.now()
            else:
                # move everything down
                self.frame0 = self.frame1
                self.t0 = self.t1
                self.frame1 = new_frame
                self.t1 = rospy.Time.now()

        if self.frame0 != None:
            # compute the magnitude of the rotation/translation
            diff = self.frame0.Inverse() * self.frame1
            pos_diff = diff.p
            rot_diff = PyKDL.Vector(*diff.M.GetRPY())

            print "Pos speed=%f, rot speed=%f"%(pos_diff.Norm(),rot_diff.Norm())

            #return (pos_diff.Norm(), rot_diff.Norm())
            trans_speed = pos_diff.Norm()
            rot_speed = rot_diff.Norm()

        else:
            trans_speed = None
            rot_speed = None

        return (trans_speed, rot_speed)

    '''
    get_values
    returns the last values for this feature creator
    '''
    def get_values(self):
        return (trans_speed, rot_speed)
