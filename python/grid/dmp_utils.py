" grid "
from features import LoadRobotFeatures

" ros utils "
import rospy

" numpy "
import numpy as np

" dmp message types "
from dmp.msg import *
from dmp.srv import *

"""
DMP UTILITIES
===============================================================
These are based on the sample code from http://wiki.ros.org/dmp
(sample code by Scott Niekum)
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

def RequestActiveDMP(dmps):
    try:
        sad = rospy.ServiceProxy('set_active_dmp', SetActiveDMP)
        sad(dmps)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

"""
Other Utilities
================================================================
These are by Chris Paxton, used for loading data and searching
for trajectories.
"""

'''
LoadDataDMP
Go through a list of filenames and load all of them into memory
Also learn a whole set of DMPs
'''
def LoadDataDMP(filenames):
    params = []
    data = []
    for filename in filenames:
        print 'Loading demonstration from "%s"'%(filename)
        demo = LoadRobotFeatures(filename)
        
        print "Loaded data, computing features..."
        #fx,x,u,t = demo.get_features([('ee','link'),('ee','node'),('link','node')])
        fx = demo.GetTrainingFeatures()
        x = demo.GetJointPositions()

        print "Fitting DMP for this trajectory..."
        # DMP parameters
        dims = len(x[0])
        dt = 0.1
        K = 100
        D = 2.0 * np.sqrt(K)
        num_bases = 4

        resp = RequestDMP(x,0.1,K,D,5)
        dmp = resp.dmp_list

        dmp_weights = []
        for idmp in dmp:
            dmp_weights += idmp.weights
            num_weights = len(idmp.weights)

        params += [[i for i in x[-1]] + dmp_weights]

        data.append((demo, fx, resp))
    return (data, params, num_weights)
