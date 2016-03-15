import rospy
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

'''
ToyEnvironment
Defines a simple "toy" environment, handles interactions and generates task-relevant predicates.
'''
class ToyEnvironment:



    '''
    tick()
    Main update function for the toy interactive environment
    '''
    def tick():


        markerArray = MarkerArray()
        markerArray.markers = zones + targets + balls

        marker_pub.publish(markerArray)


    '''
    Initialize based on one of a few presets
    Other ways of initialization will be added later
    '''
    def __init__(self,manip_frame,preset=0):
        self.manip_frame = manip_frame # what frame are we paying attention to
        self.zones=[] # you might move differently in zones
        self.targets=[] # move balls to targets
        self.balls=[] # balls
        self.marker_pub = rospy.Publisher('environment/markers',MarkerArray,1000)

        if preset == 0:
            zone1 = Marker()
            zone1.scale.x = 0.5
            zone1.scale.y = 0.5
            zone1.scale.z = 0.5
            zone1.pose.position.x = 0.5
            zone1.pose.position.y = 0.5
            zone1.pose.position.z = 0.75
            zone1.color.r = 1.0
            zone1.color.g = 0.2
            zone1.color.b = 0.1
            zone1.color.a = 0.25
            
