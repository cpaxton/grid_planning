from features import *

import numpy as np

# marker data types
from visualization_msgs.msg import MarkerArray
from visualization_msgs.msg import Marker

# conversion
import tf_conversions.posemath as pm

r = [1.,0.,0.,1.,1.,0.,0.5,1.0]
g = [0.,1.,0.,0.,1.,1.,0.5,1.0]
b = [0.,0.,1.,1.,0.,1.,0.5,1.0]

def GetLabeledArray(demo,labels,used=None):
    
    msg = MarkerArray()

    #if not used == None:
    #    r = np.random.rand(len(used))
    #    g = np.random.rand(len(used))
    #    b = np.random.rand(len(used))

    for i in range(len(labels)):
        marker = Marker()
        marker.header.frame_id = demo.base_link
        pt = demo.GetForward(demo.joint_states[i].position) * PyKDL.Frame(PyKDL.Rotation.RotY(-1*np.pi/2))
        marker.pose = pm.toMsg(pt)

        marker.color.a = 1.0
        if used == None:
            marker.color.r = marker.color.g = marker.color.b = 1.0
        else:
            idx = used.index(labels[i])
            marker.color.r = r[idx]
            marker.color.g = g[idx]
            marker.color.b = b[idx]

        marker.id = i
        marker.scale.x = 0.015
        marker.scale.y = 0.0025
        marker.scale.z = 0.0025
        marker.type = Marker.ARROW

        msg.markers.append(marker)

    return msg

