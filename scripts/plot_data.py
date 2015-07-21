import grid
import matplotlib.pyplot as plt
import numpy as np

data = grid.LoadYaml('demo3.yml')
fx,x,u,t = data.get_features([('ee','link'),('ee','node')])
ndata = np.array(fx)

xx = np.array(range(ndata.shape[0]))

for i in range(ndata.shape[1]):
    yy = ndata[:,i]
    plt.plot(xx,yy)

#yy = ndata[:,10]
#plt.plot(xx,yy)

plt.show()
