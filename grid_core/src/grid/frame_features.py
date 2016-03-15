import rospy

'''
FrameListener
This class processes input from a target object frame relative to the manipulation frame.
It is configured by providing both a target frame and a relative "actor" frame.
'''
class FrameListener:

    '''
    init
    creates ROS transform listeners
    processing in KDL creates the final set of features
    '''
    def __init__(self,actor,target):
        self.actor = actor
        self.target = target
        pass

    def tick(self):
        pass


