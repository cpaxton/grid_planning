" ros utils "
import rospy

" numpy "
import numpy as np

" dmp message types "
from dmp.msg import *
from dmp.srv import *

"""
DMP UTILITIES
These are based on the sample code from http://wiki.ros.org/dmp
"""


'''
Put together a DMP request
'''
def RequestDMP(u,dt,k_gain,d_gain,num_basis_functions):

    ndims = len(u[0])
    k_gains = [k_gain]*ndims
    d_gains = [d_gain]*ndims
    
    ex = DMPTraj()
    
    for i in range(len(u)):
        pt = DMPPoint()
        pt.positions = u[i] # always sends positions regardless of actual content
        ex.points.append(pt)
        ex.times.append(dt * i) # make sure times are reasonable

    print "Waiting for DMP service..."
    rospy.wait_for_service('learn_dmp_from_demo')

    print "Sending DMP learning request..."
    try:
        lfd = rospy.ServiceProxy('learn_dmp_from_demo', LearnDMPFromDemo)
        resp = lfd(ex, k_gains, d_gains, num_basis_functions)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
    print "LfD done"    
            
    return resp;

def PlanDMP(x_0, x_dot_0, t_0, goal, goal_thresh, 
                    seg_length, tau, dt, integrate_iter):
    print "Starting DMP planning..."
    rospy.wait_for_service('get_dmp_plan')
    try:
        gdp = rospy.ServiceProxy('get_dmp_plan', GetDMPPlan)
        resp = gdp(x_0, x_dot_0, t_0, goal, goal_thresh, 
                   seg_length, tau, dt, integrate_iter)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
    print "DMP planning done"   
            
    return resp;
