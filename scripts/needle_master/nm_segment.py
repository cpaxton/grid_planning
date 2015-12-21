#!/usr/bin/env python

import needle_master as nm
import numpy as np

import os

import matplotlib.pyplot as plt

files = os.listdir('./needle_master_trials')

start_idx = 1
end_idx = 11
envs = [0]*(end_idx-start_idx)
ncols = 5

for i in range(start_idx,end_idx):
    filename = "needle_master_trials/environment_%d.txt"%(i)

    # process as an environment
    env = nm.Environment(filename)
    envs[i-1] = env
    plt.subplot(2,ncols,i)
    env.Draw()

for file in files:
    if file[:5] == 'trial':
        # process as a trial
        filename = os.path.join('needle_master_trials',file)
        (env,t) = nm.ParseDemoName(filename)

        # draw
        if env < end_idx and env >= start_idx:
            demo = nm.Demo(env_height=envs[env-1].height,env_width=envs[env-1].width,filename=filename)
            plt.subplot(2,ncols,env)
            demo.Draw()

            envs[env-1].InGate(demo)

plt.show()
