#!/usr/bin/env python

import grid
import matplotlib.pyplot as plt
import numpy as np

'''
PLOT_DATA
This script goes through some data, shows some python plots, and experiments a bit with HDP-HSMM clustering.
The idea is that we can use this sort of clustering to detect repeated segments.
Each segment 
'''

frames=[('ee','link'),('ee','node')]
data = grid.LoadRobotFeatures('app3.yml')
fx = data.GetTrainingFeatures(frames)
ndata = np.array(fx)

#xx = np.array(range(ndata.shape[0]-1))
xx = np.array(range(ndata.shape[0]))

labels = data.GetFeatureLabels()
for i in range(ndata.shape[1]):
    #yy = np.diff(ndata[:,i])
    yy = ndata[:,i]
    lab, = plt.plot(xx,yy)
    labels.append(lab)
plt.legend(labels=labels)

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


data2 = grid.LoadRobotFeatures('app2.yml')
fx = data2.GetTrainingFeatures(frames)
ndata2 = np.array(fx)
data1 = grid.LoadRobotFeatures('app1.yml')
fx = data1.GetTrainingFeatures(frames)
ndata1 = np.array(fx)

ndata = np.diff(ndata,axis=0)
ndata1 = np.diff(ndata1,axis=0)
ndata2 = np.diff(ndata2,axis=0)

#posteriormodel.add_data(ndata,trunc=60)
#posteriormodel.add_data(ndata1,trunc=60)
#posteriormodel.add_data(ndata2,trunc=60)

posteriormodel.add_data(ndata)
posteriormodel.add_data(ndata1)
posteriormodel.add_data(ndata2)

models = []
for idx in progprint_xrange(150):
    posteriormodel.resample_model()
    if (idx+1) % 10 == 0:
        models.append(copy.deepcopy(posteriormodel))


