import numpy as np
from pypr.clustering import gmm

'''
Wraps some PyPr functions for easy grouping of different GMMs.
Uses a couple different functions.
'''
class GMM:
    
    def __init__(self,k=1,data=None):
        self.k = 1
        if not data == None:
            self.fit(data)
        self.mu = None
        self.sigma = None
        self.pi = None

    '''
    fit gmm from data
    '''
    def fit(self,data):
        self.mu,self.sigma,self.pi,ll = gmm.em_gm(data,self.k)

    '''
    return log likelihoods
    '''
    def score(self,data):
        return gmm.gm_log_likelihood(data,self.mu,self.sigma,self.pi)

    '''
    predict results by filling in NaNs
    '''
    def predict(self,data):
        p = gmm.predict(data,self.mu,self.sigma,self.pi)
        return data

