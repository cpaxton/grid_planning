" IO "
import sys
from yaml import load, dump
try:
    from yaml import CLoader as Loader, CDumper as Dumper
except ImportError:
    from yaml import Loader, Dumper

" ROS "
import rospy
import tf

" Message types "
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState

" Conversions "
import tf_conversions.posemath as pm
import PyKDL as kdl

'''
Demonstration
Listen for information from the WAM arm to collect a demonstration
Save the whole joint state trajectory as well as the whole feature space trajectory.

Feature space:
    - stored as a set of KDL transforms between bhand_wrist and objects

Joint space:
    - stored as arrays of joint states
    - also has a variable for the closure of the gripper
'''

class Demonstration:
    def __init__(self):
        self.joint_p = []
        self.joint_v = []
        self.joint_t = []
        self.tform = {}
        self.world_t = []

demo = Demonstration()

def js_cb(msg):
    demo.joint_p.append([i for i in msg.position])
    demo.joint_v.append([i for i in msg.velocity])
    demo.joint_t.append(rospy.Time.now())

if __name__ == '__main__':
    rospy.init_node('demonstration_observer')

    filename = 'demo.yml'
    if len(sys.argv) > 1:
        filename = sys.argv[1]

    # set up parameters
    world='/world'
    frame='/wam/hand/bhand_palm_link'
    obj1='/gbeam_link_1/gbeam_link'
    obj2='/gbeam_node_1/gbeam_node'

    # set up ros things
    rate = rospy.Rate(10)
    tl = tf.TransformListener();
    sub = rospy.Subscriber('/gazebo/barrett_manager/wam/joint_states',JointState, js_cb)

    # collect data
    try:

        demo.tform['ee'] = []
        demo.tform['link'] = []
        demo.tform['node'] = []

        while not rospy.is_shutdown():
            try:
                (trans0,rot0) = tl.lookupTransform(world, frame, rospy.Time(0))
                (trans1,rot1) = tl.lookupTransform(world, obj1, rospy.Time(0))
                (trans2,rot2) = tl.lookupTransform(world, obj2, rospy.Time(0))

                ee_tf = pm.fromTf((trans0,rot0))
                obj1_tf = pm.fromTf((trans1,rot1))
                obj2_tf = pm.fromTf((trans2,rot2))

                demo.tform['ee'].append(ee_tf) 
                demo.tform['link'].append(obj1_tf) 
                demo.tform['node'].append(obj2_tf) 
                demo.world_t.append(rospy.Time.now())

            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException): 
                continue
            rate.sleep()

    except rospy.ROSInterruptException:
        if len(demo.joint_p) == 0:
            print "List of joint states was empty!"
        #for (t,pt) in zip(position_times,positions):
        #    print (t, pt)
