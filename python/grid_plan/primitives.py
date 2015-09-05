import PyKDL
import numpy as np
from sklearn.mixture import GMM
import tf_conversions.posemath as pm

def Sample(gmm):
    idx = int(np.floor((np.random.rand() * gmm.n_components)))
    A = np.linalg.cholesky(gmm.covars_[idx,:,:])
    ndim = gmm.means_.shape[1]

    n = np.random.normal(0,1,ndim)
    
    # cholesky returns the transpose of the MATLAB function chol()
    return gmm.means_[idx,:] + A.dot(n)

def SamplePrimitives(ee,Z,kdl_kin,q):
    z = Sample(Z)
    nvars = Z.means_.shape[1]
    nframes = nvars / 6
    traj = []
    for i in range(nframes):
        frame = i * 6
        p = PyKDL.Vector(z[frame],z[frame+1],z[frame+2])
        M = PyKDL.Rotation.RPY(z[frame+3],z[frame+4],z[frame+5])
        traj.append(kdl_kin.inverse(pm.toMatrix(ee*PyKDL.Frame(M,p)),q))

    return z,traj

def InitSearch(npts,guess):
    Z = GMM(n_components=1,covariance_type="full")
    Z.means_ = np.zeros((1,6*npts))
    Z.covars_ = np.zeros((1,6*npts,6*npts))
    Z.covars_[0] = 0.005*np.eye(6*npts)
    for i in range(npts):
        p = (i+1) * guess / float(npts)
        ii = 6*i
        Z.means_[0,ii:ii+3] = [x for x in p]

    return Z

