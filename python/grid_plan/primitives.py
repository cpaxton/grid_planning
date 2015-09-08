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

def SamplePrimitives(ee,Z,kdl_kin,q,num_interp=4):
    z = Sample(Z)
    nvars = Z.means_.shape[1]
    nframes = nvars / 6
    traj = []
    #rpy = ee.M.GetRPY()
    #last_param = np.array([ee.p.x(),ee.p.y(),ee.p.z(),rpy[0],rpy[1],rpy[2]])
    last_param = np.zeros((6))
    last_frame = ee
    for i in range(nframes):
        frame = i * 6
        param = np.array(z[frame:frame+6])
        d_param = last_param - param
        for j in range(num_interp):
            interp_param = last_param - ((float(j+1)/num_interp)*d_param)
            p = PyKDL.Vector(interp_param[0],interp_param[1],interp_param[2])/10
            M = PyKDL.Rotation.RPY(interp_param[3]/5,interp_param[4]/5,interp_param[5]/5)
            frame = last_frame*PyKDL.Frame(M,p)
            res = kdl_kin.inverse(pm.toMatrix(frame),q)
            if not res is None:
                traj.append(res.tolist())
            else:
                traj.append(None)
        last_param = param
        #last_frame = frame

    return z,traj

def InitSearch(npts,guess):
    Z = GMM(n_components=1,covariance_type="full")
    Z.means_ = np.zeros((1,6*npts))
    Z.covars_ = np.zeros((1,6*npts,6*npts))
    Z.covars_[0] = np.eye(6*npts)
    for i in range(npts):
        p = (i+1) * guess / float(npts+1) * 10
        ii = 6*i
        Z.means_[0,ii:ii+3] = [x for x in p]
        #print p
    #raw_input()

    return Z

