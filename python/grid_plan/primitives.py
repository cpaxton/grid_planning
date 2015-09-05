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
        traj.append(kdl_kin.inverse(pm.toMatrix(M),q))

def InitSearch(npts):
    Z = GMM(n_components=1,covariance_type="full")
    Z.means_ = np.zeros((1,6*npts))
    Z.covars_ = np.zeros((1,6*npts,6*npts))
    Z.covars_[0] = np.eye(6*npts)

