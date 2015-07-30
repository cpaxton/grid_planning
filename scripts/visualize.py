#!/usr/bin/env python

import rospy
import sys
import grid
import matplotlib.pyplot as plt
import numpy as np

from visualization_msgs.msg import MarkerArray

'''
PLOT_DATA
This script goes through some data, shows some python plots, and experiments a bit with HDP-HSMM clustering.
The idea is that we can use this sort of clustering to detect repeated segments.
Each segment 
'''

if __name__ == "__main__":
    rospy.init_node('visualize_labels')

    if len(sys.argv) > 1:
        filenames = sys.argv[1:len(sys.argv)]
    else:
        filenames = ["app1.yml"]

    data = []
    for filename in filenames:
        demo = grid.LoadRobotFeatures(filename)
        fx = demo.GetTrainingFeatures()
        
        data.append((demo, np.array(fx)))


    ndata = data[0][1]
    xx = np.array(range(ndata.shape[0]))

    for i in range(ndata.shape[1]):
        yy = ndata[:,i]
        plt.plot(xx,yy)

    #yy = ndata[:,10]
    #plt.plot(xx,yy)

    plt.show()


    '''
    This section does some simple HDP-HSMM clustering.
    Features are Gaussian-distributed and durations are Poisson distributed.
    '''

    print "clustering..."

    import copy
    import pyhsmm
    import pyhsmm.basic.distributions as distributions
    from pybasicbayes.util.text import progprint_xrange

    obs_dim = ndata.shape[1]
    Nmax = 25

    obs_hypparams = {'mu_0':np.zeros(obs_dim),
                    'sigma_0':np.eye(obs_dim),
                    'kappa_0':0.3,
                    'nu_0':obs_dim+5}
    dur_hypparams = {'alpha_0':2*30,
                     'beta_0':2}

    obs_distns = [distributions.Gaussian(**obs_hypparams) for state in range(Nmax)]
    dur_distns = [distributions.PoissonDuration(**dur_hypparams) for state in range(Nmax)]

    posteriormodel = pyhsmm.models.WeakLimitHDPHSMM(
    #posteriormodel = pyhsmm.models.WeakLimitHDPHMM(
            alpha=6.,gamma=6., # better to sample over these; see concentration-resampling.py
            init_state_concentration=6., # pretty inconsequential
            obs_distns=obs_distns,
            #obs_distns=obs_distns)
            dur_distns=dur_distns)

    for (demo, fx) in data:
        ndata = np.diff(fx,axis=0)
        posteriormodel.add_data(ndata)

    models = []
    for idx in progprint_xrange(150):
        posteriormodel.resample_model()
        if (idx+1) % 10 == 0:
            models.append(copy.deepcopy(posteriormodel))

    print posteriormodel.used_states
    print posteriormodel.stateseqs

    rate = rospy.Rate(10)
    pub = rospy.Publisher("/segmented_trajectory",MarkerArray)

    #msg = MarkerArray()
    msg = grid.GetLabeledArray(data[0][0],posteriormodel.stateseqs[0],posteriormodel.used_states)

    try:
        while not rospy.is_shutdown():
            pub.publish(msg)
            rate.sleep()
    except rospy.ROSInterruptException, e:
        pass
